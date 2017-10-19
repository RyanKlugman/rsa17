/*
 * explorer.cpp
 *
 *  Created on: 22/02/2013
 *      Author: mmcgill
 */

#include <ros/ros.h>
#include <rsa17/explorer.hpp>
#include <crosbot/utils.hpp>

#include <unistd.h>

namespace crosbot {

#define DEFAULT_RATE            10
#define DEFAULT_SLEEP_TIME		50000
#define DEFAULT_SLEEP_TIME_MAX	10000000
#define LOG_START               "CrosbotExplorer ::"

#define ACCEPTABLE_DRIFT_IN_WALL_ANGLE		DEG2RAD(10)
#define CELL_IS_VALID(C,V)                  ((C).x >= 0 && (C).y >= 0 && (C).x < (V).width && (C).y < (V).height)

Explorer::Explorer() :
    driveThread_(*this),
    planThread_(*this),
    voronoiThread_(*this),
	rate_planThread(DEFAULT_RATE),
	rate_driveThread(DEFAULT_RATE),
	rate_voronoiThread(DEFAULT_RATE),
	operating(true),
	paused(true),
	stopSent(false),
	searchWhilePaused(true),
	debugMsgs(false),
	hysteresisParams()
{}

Explorer::~Explorer() {
    shutdown();
}

void Explorer::planThread() {
    ros::Rate rate(rate_planThread);
	while (operating) {
	    { // Lock scope
	        Lock planningLock(planningThreadMutex);
			if (debugMsgs) ROS_INFO("%s planThread()", LOG_START); 
            if (!paused || searchWhilePaused) {
                // Load the latest pose for this loop here
                loadLatestPose();

                VoronoiGridPtr voronoi = getLatestVoronoi();
                Pose robot = getLatestPose();
                ROS_INFO("%s Robot position: %s", LOG_START, robot.position.toString().c_str());
                if (voronoi != NULL && robot.isFinite()) {
                    // search voronoi grid
                    if (debugMsgs) ROS_INFO("%s find drive target", LOG_START);
                    Pose foundTarget = findDriveTarget(*voronoi, robot);
                    ROS_INFO("%s Drive target: %s", LOG_START, foundTarget.position.toString().c_str());

                    // TODO: I think this is a race condition, and is not needed anyway
                    // TODO: see change in paused() method
                    //if (!paused) {
                        driveParams.setDriveTarget(foundTarget);
                    //} else {
                        //drive.setDriveTarget(Pose(INFINITY, INFINITY, INFINITY));
                    //}
                } else {
                    if (voronoi != NULL) {
                        ROS_WARN("%s No Map to Navigate with", LOG_START);
                    }
                    if (robot.isFinite()) {
                        ROS_WARN("%s Robot position is infinite", LOG_START);
                    }
                    driveParams.setDriveTarget(Pose(INFINITY, INFINITY, INFINITY));
                }
            } else {
                driveParams.setDriveTarget(Pose(INFINITY, INFINITY, INFINITY));
            }
	    }

		rate.sleep();
	}
}

void Explorer::driveThread() {
    ros::Rate rate(rate_driveThread);
	while (operating) {
        { // Lock scope
        	if (debugMsgs) ROS_INFO("%s driveThread()", LOG_START); 
            Lock drivingLock(drivingThreadMutex);
			
            // Ensure latest data is loaded
            loadLatestPose();

            Pose target = driveParams.getDriveTarget();
            Pose pose = getLatestPose();

            if (!paused &&
                target.position.isFinite() &&
                pose.position.isFinite()) {
                // send motion
                Pose relative = pose.toTF().inverse() * target.toTF();
                driveTo(relative);
            } else if (!stopSent) {
                // send stop
                stopSent = stopMotors();
            }
	    }

		rate.sleep();
	}

	if (!stopSent) {
		// send stop
		stopSent = stopMotors();
	}
}

void Explorer::voronoiThread() {
    ros::Rate rate(rate_voronoiThread);
    while (operating) {
        { // Lock scope
            // Ensure latest data is loaded
            if(debugMsgs) ROS_INFO("%s voronoi thread updating", LOG_START);
            loadLatestPose();

            // Load voronoi
            loadLatestVoronoi();
        }

        rate.sleep();
    }
}

void Explorer::configure() {
}

void Explorer::startup() {
	if (debugMsgs) ROS_INFO("%s startup()", LOG_START); 
    voronoiThread_.start();
    planThread_.start();
    driveThread_.start();
}

void Explorer::shutdown() {
	if (debugMsgs) ROS_INFO("%s shutdown()", LOG_START); 
	operating = false;

	// Cleanup data structures
	hysteresis.reset();

	// Wait for plan/drive threads to stop
	uint64_t slept = 0;
	while (slept < DEFAULT_SLEEP_TIME_MAX
	        && planThread_.isAlive()
	        && driveThread_.isAlive()
	        && voronoiThread_.isAlive()) {
		usleep(DEFAULT_SLEEP_TIME);
		slept += DEFAULT_SLEEP_TIME;
	}
}

void Explorer::pause() {
    paused = true;
    statusChanged(rsa17::ExplorerStatus::Status::STATUS_PAUSED);
    hysteresis.reset();
}

void Explorer::resume() {
	if (debugMsgs) ROS_INFO("%s resume()", LOG_START); 
    // TODO: Again, another stupid race condition
    // TODO: Replace this with some flag to indicate to the drive thread that a new position has been planned
    //drive.setDriveTarget(Pose(INFINITY, INFINITY, INFINITY));
    hysteresis.reset();

    paused = false;
    stopSent = false;
    statusChanged(rsa17::ExplorerStatus::Status::STATUS_RUNNING);
}

Pose Explorer::findDriveTarget(const VoronoiGrid& voronoi, const Pose& robot) {
	if (debugMsgs) ROS_INFO("%s findDriveTarget()", LOG_START); 
	Pose targetPose(INFINITY, INFINITY, INFINITY);
    if (searchParams.strategy == SearchParameters::WallFollow) {
        targetPose = findWallFollowTarget(voronoi, robot);
	} else if (searchParams.strategy == SearchParameters::Waypoint) {
	    targetPose = findWaypointTarget(voronoi, robot);
	}

    if (targetPose.isFinite() && hysteresisParams.enabled) {
        targetPose = findHysteresisTarget(voronoi, robot, targetPose);
    }

	return targetPose;
}

Pose Explorer::findWallFollowTarget(const VoronoiGrid& voronoi, const Pose& robot) {
    ROS_INFO("%s Finding %s wallfollow target, robot at: %s", LOG_START, (searchParams.side == SearchParameters::Left) ? "left" : "right", robot.toString().c_str());
    // Update previous wall target
    searchParams.previousWallTarget = searchParams.wallTarget;

	double Y, P, R;
	robot.orientation.getYPR(Y,P,R);

	// Calculate angle to start finding wall from (relative to robot)
	double startAngle = (searchParams.side==SearchParameters::Left) ? searchParams.wallSearchAngle : -searchParams.wallSearchAngle;
	if (searchParams.previousWallTarget.isFinite()) {
		double prevStartAngle = atan2(searchParams.previousWallTarget.y - robot.position.y,
				                      searchParams.previousWallTarget.x - robot.position.x) - Y;
		if (fabs(prevStartAngle - startAngle) <= ACCEPTABLE_DRIFT_IN_WALL_ANGLE) {
			startAngle = prevStartAngle;
		} else {
		    //ROS_WARN("%s wall drifted too far - resetting starting angle", LOG_START);
		}
	}

	// Find wall target
	searchParams.wallTarget = findWall(voronoi, robot, startAngle);

	if (!searchParams.wallTarget.isFinite()) {
		ROS_WARN("%s findWallFollowTarget(): No wall found", LOG_START);
		return Pose(INFINITY, INFINITY, INFINITY);
	}
	double wallAngle = atan2(searchParams.wallTarget.y - robot.position.y, searchParams.wallTarget.x - robot.position.x);

	Index2D currentCell = findFirstSkeletonCell(voronoi, robot, wallAngle);
	Index2D previousCell(-1,-1);

	if (!CELL_IS_VALID(currentCell, voronoi)) {
		WARN("Explorer::findWallFollowTarget(): Unable to find a skeleton cell in range.\n");
		return Pose(INFINITY, INFINITY, INFINITY);
	}

	Index2D robotCell = voronoi.findCell(robot.position);
	uint32_t n = 0;
	for (; n < searchParams.maxIterations; ++n) {
		Index2D nextCell = findNextSkeletonCell(voronoi, currentCell, previousCell);

		if (!CELL_IS_VALID(nextCell, voronoi)) {
			break;
		}

		if (((nextCell.distanceTo(robotCell) * voronoi.resolution) > searchParams.minTravelDistance) ||
				!cellIsDirectlyTraversible(voronoi, nextCell, robotCell)) {
			break;
		}

		previousCell = currentCell;
		currentCell = nextCell;
	}

	if (n >= searchParams.maxIterations) {
		WARN("Explorer::findWallFollowTarget(): Skeleton found is a probably a small closed loop.\n");
		return Pose(INFINITY, INFINITY, INFINITY);
	}

	Index2D saferCell = findSaferSkeletonCell(voronoi, currentCell, robotCell);

	return Pose(voronoi.getPosition(saferCell), Quaternion());
}

Point Explorer::findWall(const VoronoiGrid& voronoi, const Pose& robot, double startAngle) {
	if (debugMsgs) ROS_INFO("%s findWall()", LOG_START); 
	double Y,P,R, cosRay, sinRay;
	robot.orientation.getYPR(Y,P,R);
	Index2D robotCell = voronoi.findCell(robot.position), cell;
	Index2D wallCell(-1,-1);

	double rayAngle = 0;
	for (; rayAngle < 2*M_PIl && !CELL_IS_VALID(wallCell, voronoi); rayAngle += searchParams.angleStep) {
		if (searchParams.side == SearchParameters::Left) {
			cosRay = cos(Y + startAngle - rayAngle);
			sinRay = sin(Y + startAngle - rayAngle);
		} else {
			cosRay = cos(Y + startAngle + rayAngle);
			sinRay = sin(Y + startAngle + rayAngle);
		}

		for (double ray = 0; ray <= (searchParams.wallSearchDistance / voronoi.resolution); ray += searchParams.rayStep) {
			cell = Index2D(robotCell.x + ray * cosRay, robotCell.y + ray * sinRay);

			if (!CELL_IS_VALID(cell, voronoi)) {
				continue;
			}

			const VoronoiGrid::VoronoiCell& vCell = voronoi.getVoronoiCell(cell);
			if (vCell.status & VoronoiGrid::VoronoiCell::Wall) {
				wallCell = cell;
				return voronoi.getPosition(wallCell);
			} else if (vCell.status & VoronoiGrid::VoronoiCell::Restricted) {
				wallCell = vCell.nearestWallCell;
				return voronoi.getPosition(wallCell);
			} else if ((vCell.status & VoronoiGrid::VoronoiCell::Restricted) ||
					(vCell.status & VoronoiGrid::VoronoiCell::PatiallyRestricted) ||
					(vCell.status & VoronoiGrid::VoronoiCell::Expansion)) {
				wallCell = vCell.nearestWallCell;
			}
		}
	}

	for (; rayAngle < 2*M_PIl; rayAngle += searchParams.angleStep) {
		if (searchParams.side == SearchParameters::Left) {
			cosRay = cos(Y + startAngle - rayAngle);
			sinRay = sin(Y + startAngle - rayAngle);
		} else {
			cosRay = cos(Y + startAngle + rayAngle);
			sinRay = sin(Y + startAngle + rayAngle);
		}

		for (double ray = 0; ray <= (searchParams.wallSearchDistance / voronoi.resolution); ray += searchParams.rayStep) {
			cell = Index2D(robotCell.x + ray * cosRay, robotCell.y + ray * sinRay);

			if (!CELL_IS_VALID(cell, voronoi)) {
				continue;
			}

			const VoronoiGrid::VoronoiCell& vCell = voronoi.getVoronoiCell(cell);
			if (vCell.status & VoronoiGrid::VoronoiCell::Wall) {
				wallCell = cell;
				return voronoi.getPosition(wallCell);
			} else if (vCell.status & VoronoiGrid::VoronoiCell::Restricted) {
				wallCell = vCell.nearestWallCell;
				return voronoi.getPosition(wallCell);
			}
		}
	}

	if (CELL_IS_VALID(wallCell, voronoi)) {
		return voronoi.getPosition(wallCell);
	}
	return Point(INFINITY, INFINITY, INFINITY);
}

Index2D Explorer::findFirstSkeletonCell(const VoronoiGrid& voronoi, const Pose& robot, double startAngle) {
	if (debugMsgs) ROS_INFO("%s findFirstSkeletonCell()", LOG_START); 
	double cosRay, sinRay;
	searchParams.skeleton1st = searchParams.skeleton2nd = Index2D(-1,-1);
	searchParams.skeletonSearch.clear();

	Index2D robotCell = voronoi.findCell(robot.position), cell;

	for (double rayAngle = 0; rayAngle < 2*M_PIl; rayAngle += searchParams.angleStep) {
		if (searchParams.side == SearchParameters::Left) {
			cosRay = cos(startAngle - rayAngle);
			sinRay = sin(startAngle - rayAngle);
		} else {
			cosRay = cos(startAngle + rayAngle);
			sinRay = sin(startAngle + rayAngle);
		}

		for (double ray = 0; ray <= (searchParams.skeletonSearchDistance / voronoi.resolution); ray += searchParams.rayStep) {
			cell = Index2D(robotCell.x + ray * cosRay, robotCell.y + ray * sinRay);

			if (!CELL_IS_VALID(cell, voronoi)) {
				continue;
			}

			const VoronoiGrid::VoronoiCell& vCell = voronoi.getVoronoiCell(cell);
			if ((vCell.status & VoronoiGrid::VoronoiCell::Wall) ||
					(vCell.status & VoronoiGrid::VoronoiCell::Restricted)) {
				break;
			} else if (vCell.status & VoronoiGrid::VoronoiCell::Skeleton) {
				searchParams.skeleton1st = cell;
				searchParams.skeletonSearch.push_back(cell);
				return cell;
			}
		}
	}
	return Index2D(-1,-1);
}

Index2D Explorer::findNextSkeletonCell(const VoronoiGrid& voronoi, const Index2D currentCell, const Index2D previous) {
	if (debugMsgs) ROS_INFO("%s findNextSkeletonCell()", LOG_START); 
	Index2D prev = previous;
	bool lookingFor2ndCell = false;
	if (!CELL_IS_VALID(previous, voronoi)) {
		// Use wall point to find a virtual previous
		lookingFor2ndCell = true;
		Index2D wallCell = voronoi.findCell(searchParams.wallTarget);

		int dx = wallCell.x - currentCell.x;
		int dy = wallCell.y - currentCell.y;
		if (dx == 0 && dy == 0) {
			return Index2D(-1, -1);
		}

		if (searchParams.side == SearchParameters::Left) {
			if (dx > 0 && dy >= 0)
				prev = Index2D(currentCell.x, currentCell.y+1);
			else if (dy > 0) {
				prev = Index2D(currentCell.x-1, currentCell.y);
			} else if (dx < 0) {
				prev = Index2D(currentCell.x, currentCell.y-1);
			} else {
				prev = Index2D(currentCell.x+1, currentCell.y);
			}
		} else {
			if (dx >= 0 && dy > 0)
				prev = Index2D(currentCell.x+1, currentCell.y);
			else if (dx > 0) {
				prev = Index2D(currentCell.x, currentCell.y-1);
			} else if (dy < 0) {
				prev = Index2D(currentCell.x-1, currentCell.y);
			} else {
				prev = Index2D(currentCell.x, currentCell.y+1);
			}
		}
	}

	int dx = currentCell.x - prev.x;
	int dy = currentCell.y - prev.y;

	Index2D searchOrder[4];
	if (searchParams.side == SearchParameters::Left) {
		searchOrder[0] = Index2D(-dy, dx);
		searchOrder[1] = Index2D(dx, dy);
		searchOrder[2] = Index2D(dy, -dx);
		searchOrder[3] = Index2D(-dx, -dy);
	} else {
		searchOrder[0] = Index2D(dy, -dx);
		searchOrder[1] = Index2D(dx, dy);
		searchOrder[2] = Index2D(-dy, dx);
		searchOrder[3] = Index2D(-dx, -dy);
	}

	for (unsigned int i = 0; i < 4; i++) {
		Index2D next = currentCell + searchOrder[i];

		if (!CELL_IS_VALID(next, voronoi)) {
			continue;
		}

		if (voronoi.cells[next.y * voronoi.width + next.x].status & VoronoiGrid::VoronoiCell::Skeleton) {
			if (lookingFor2ndCell) {
				searchParams.skeleton2nd = next;
			}
			searchParams.skeletonSearch.push_back(next);

			return next;
		}
	}

	return Index2D(-1,-1);
}

Index2D Explorer::findSaferSkeletonCell(const VoronoiGrid& voronoi, const Index2D currentCell, const Index2D robot) {
	if (debugMsgs) ROS_INFO("%s findSaferSkeletonCell()", LOG_START); 
	uint32_t n = 0;
	Index2D safeCell = currentCell;
	if (currentCell == robot) {
		return currentCell;
	}

//	double currentRisk = cellTraversibleRisk(voronoi, currentCell, robot);
//	Index2D searchOrder[4];
//	searchOrder[0] = Index2D(0,1);
//	searchOrder[1] = Index2D(0,-1);
//	searchOrder[2] = Index2D(1,0);
//	searchOrder[3] = Index2D(-1,0);
//
//	bool riskDecreasing = true;
//	for (; n < search.maxIterations && riskDecreasing; ++n) {
//		riskDecreasing = false;
//		for (int ne = 0; ne < 4; ++ne) {
//			Index2D neighbour = safeCell + searchOrder[ne];
//
//			if (!CELL_IS_VALID(neighbour, voronoi))
//				continue;
//
//			const VoronoiGrid::VoronoiCell& cell = voronoi.cells[neighbour.y * voronoi.width + neighbour.x];
//			if (!(cell.status & VoronoiGrid::VoronoiCell::Skeleton))
//				continue;
//
//			double neighbourRisk = cellTraversibleRisk(voronoi, neighbour, robot);
//			neighbourRisk *= 1 + neighbour.distanceTo(currentCell) / currentCell.distanceTo(robot);
//			if (neighbourRisk < currentRisk) {
//				safeCell = neighbour;
//				currentRisk = neighbourRisk;
//				riskDecreasing = true;
//			}
//		}
//	}

	int32_t windowSize = ceil(searchParams.skeletonSearchDistance / voronoi.resolution) + 1;
	double restrictedC = (voronoiConstraints.restricted / voronoi.resolution);
	double maxExpandC = ((voronoiConstraints.partial + voronoiConstraints.expand) / voronoi.resolution);

	double maxScore = 0;
	double currentScore = -INFINITY;
	double currentD = currentCell.distanceTo(robot);
	for (int y = -windowSize; y <= windowSize; ++y) {
		for (int x = -windowSize; x <= windowSize; ++x) {
			Index2D cell2Check = currentCell + Index2D(x,y);

			if (!CELL_IS_VALID(cell2Check, voronoi)) {
				continue;
			}

			const VoronoiGrid::VoronoiCell& vCell = voronoi.getVoronoiCell(cell2Check);
			if (!(vCell.status & VoronoiGrid::VoronoiCell::Skeleton)) {
				continue;
			}
			double d = cell2Check.distanceTo(robot);
			double score = 0;
			double dx = (cell2Check.x - robot.x) / d;
			double dy = (cell2Check.y - robot.y) / d;

			int count = 0;
			for (double r = 0; r <= d; r += searchParams.rayStep) {
				Index2D rayCell(robot.x + dx*r, robot.y + dy*r);

				if (!CELL_IS_VALID(rayCell, voronoi)) {
					continue;
				}

				const VoronoiGrid::VoronoiCell& rCell = voronoi.getVoronoiCell(rayCell);
				if ((rCell.status & VoronoiGrid::VoronoiCell::Wall) ||
						(rCell.status & VoronoiGrid::VoronoiCell::Restricted)) {
					score = 0;
					break;
				}

				// The score for a raytraced cell is as 0..1 value representing distance from wall
				double d = (rCell.distanceFromWall - restrictedC) / maxExpandC;
				if (d > 1) {
					d = 1;
				}
				double cellScore;
//				if (rCell.status & VoronoiGrid::VoronoiCell::Skeleton) {
//					cellScore = 1.2 * d;
//				} else
				if (rCell.status & VoronoiGrid::VoronoiCell::Vacant) {
					cellScore = 1;
				} else if (rCell.status & VoronoiGrid::VoronoiCell::PatiallyRestricted) {
					cellScore = 0.5 * d;
				} else {
					cellScore = d;
				}

				score += cellScore;
				++count;
//				ROS_INFO("Score Sum (%d, %d)(%.3lf) - %.2lf, %.2lf - %.2lf, %.2lf", cell2Check.x, cell2Check.y, score,
//						rCell.distanceFromWall, restrictedC);
			}

			if (count == 0) {
				continue;
			}

			// Score for a cell is the mean of the intermediate ray traced cells
			score = score / count;

			// Adjust score based on distance from the goal
			double adjustment = 1 - pow((currentCell.distanceTo(cell2Check) / currentD), 2);
			score *= adjustment;

//			ROS_INFO("Score (%d, %d)(%.3lf)", cell2Check.x, cell2Check.y, score);
			if (currentCell == cell2Check) {
				currentScore = score;
			}

			if (score > maxScore) {
				safeCell = cell2Check;
				maxScore = score;
			}
		}
	}

//	ROS_WARN("Preferring (%d, %d)(%.3lf) over (%d, %d)(%.3lf)",
//			safeCell.x, safeCell.y, maxScore,
//			currentCell.x, currentCell.y, currentScore);
	return safeCell;
}

/*
Index2D Explorer::findSaferCell(const VoronoiGrid& voronoi, const Index2D currentCell, const Index2D robot) {
	double voronoiDistance = currentCell.distanceTo(robot) * voronoi.resolution;
	double score, maxScore = 0, meanScore, sint, cost, xf, yf, cosA;
	int c, count, xi, yi;

	double vdSq = voronoiDistance*voronoiDistance, vd2cosA;
	Index2D vectorCurrent = currentCell - robot;

	Index2D saferCell = currentCell;
	int32_t windowSize = ceil(search.wallSearchDistance / voronoi.resolution) + 1;

	double restrictedC = (voronoiConstraints.restricted / voronoi.resolution),
			maxExpandC = ((voronoiConstraints.partial + voronoiConstraints.expand) / voronoi.resolution);

	for (int y = -windowSize; y <= windowSize; ++y) {
		for (int x = -windowSize; x <= windowSize; ++x) {
			Index2D cell2Check = currentCell + Index2D(x,y);

			if (!CELL_IS_VALID(cell2Check, voronoi))
				continue;

			const VoronoiGrid::VoronoiCell& vCell = voronoi.getVoronoiCell(cell2Check);
			if ((vCell.status & VoronoiGrid::VoronoiCell::Wall) ||
					(vCell.status & VoronoiGrid::VoronoiCell::Restricted))
				continue;

			double d = cell2Check.distanceTo(robot);
			double dx = (cell2Check.x - robot.x) / d,
					dy = (cell2Check.y - robot.y) / d;

			score = count = 0;
			for (double r = 0; r <= d; r += search.rayStep) {
				Index2D rayCell(robot.x + dx*r, robot.y + dy*r);

				if (!CELL_IS_VALID(rayCell, voronoi))
					continue;

				const VoronoiGrid::VoronoiCell& rCell = voronoi.getVoronoiCell(rayCell);
				if ((rCell.status & VoronoiGrid::VoronoiCell::Wall) ||
									(rCell.status & VoronoiGrid::VoronoiCell::Restricted)) {
					score = 0; break;
				}
				double d = (rCell.distanceFromWall - restrictedC) / maxExpandC;
				if (d > 1)
					d = 1;
				double cellScore;
				if (rCell.status & VoronoiGrid::VoronoiCell::Skeleton) {
					cellScore = 1.2 * d;
				} else if (rCell.status & VoronoiGrid::VoronoiCell::Vacant) {
					cellScore = 1;
				} else if (rCell.status & VoronoiGrid::VoronoiCell::PatiallyRestricted) {
					cellScore = 0.5 * d;
				} else {
					cellScore = d;
				}

				score += cellScore;
				++count;
			}

			if (count == 0)
				continue;

			meanScore = score / count;

			// adjust meanScore based on distance to goal

			Index2D vectorCheck = cell2Check - robot;
			double theta = atan2(currentCell.y - robot.y, currentCell.x - robot.x) -
					atan2(cell2Check.y - robot.y, cell2Check.x - robot.x);
			cosA = cos(theta);
			vd2cosA = voronoiDistance*cosA*2;
			double r = currentCell.distanceTo(cell2Check);

			d = sqrt(vdSq + r*r - r*vd2cosA);
			meanScore = (meanScore / exp2(2*d*search.wallSearchDistance/voronoiDistance));

			if (meanScore > maxScore) {
				saferCell = cell2Check;
				maxScore = meanScore;
			}
		}
	}

	return saferCell;
}
*/

//void Explorer::findSaferTarget(double& voronoiAngle, double& voronoiDistance, const Pose& robot) {
//	double preferredAngle = voronoiAngle, preferredDist = voronoiDistance;
//	double score, maxScore = 0, meanScore, sint, cost, xf, yf, cosA;
//
//	double vdSq = voronoiDistance*voronoiDistance, vd2cosA;
//
//	int c, count, xi, yi;
//	VoronoiCell *cell = NULL;
//
//	for (double theta = -angleAcceptDiff; theta <= angleAcceptDiff; theta += searchRes) {
//		sint = sin(theta + voronoiAngle + localMap->robotPose.o.yaw),
//		cost = cos(theta + voronoiAngle + localMap->robotPose.o.yaw),
//		cosA = cos(theta);
//
//		vd2cosA = voronoiDistance*cosA*2;
//
//		score = count = 0;
//
//		for (double ray = searchStep; ray <= searchDistance; ray += searchStep) {
//			GET_CELL_FOR_RAY(ray, c);
//			if (c == -1 || cell->status == VoronoiCell::Occupied || cell->status == VoronoiCell::Obstacle) {
//				break;
//			}
//			double d = (cell->voronoiDistance - localMap->restrictedC) / localMap->maxExpansionC;
//			if (d > 1)
//				d = 1;
//			double cellScore;
//			if (cell->status == VoronoiCell::Horizon) {
//				cellScore = 1.2 * d;
//			} else if (cell->status == VoronoiCell::Vacant) {
//				cellScore = 1;
//			} else {
//				cellScore = d;
//			}
//
//			score += cellScore;
//			++count;
//
//			meanScore = score / count;
//
//			// adjust meanScore based on distance to goal
//			d = sqrt(vdSq + ray*ray - ray*vd2cosA);
//			meanScore = (meanScore / exp2(2*d*searchDistance/voronoiDistance));
//
//			if (meanScore > maxScore) {
//				preferredAngle = theta + voronoiAngle; preferredDist = ray;
//				maxScore = meanScore;
//			}
//		}
//	}
//
//	NORMALISE_ANGLE(preferredAngle);
//	voronoiAngle = preferredAngle; voronoiDistance = preferredDist;
//}

Pose Explorer::findWaypointTarget(const VoronoiGrid& voronoi, const Pose& robot) {
    // Selected 'drive-to' target, default to no path
    Pose driveToTarget(INFINITY, INFINITY, INFINITY);
    ROS_INFO("%s Finding waypoint target, robot at: %s", LOG_START, robot.toString().c_str());

    // Choose next waypoint to drive to
	{{
	    Lock lock(searchParams.waypointLock, true);

	    // Find next waypoint based on distance to robot
	    // Note: Selected waypoint may be zero here, and waypoints empty
	    int selectedWaypoint = searchParams.currentWaypoint;
	    while (selectedWaypoint < searchParams.waypoints.size() &&
               searchParams.waypoints[selectedWaypoint].position.distanceTo(robot.position) < searchParams.proximityDistance) {
			++selectedWaypoint;
		}
	    searchParams.currentWaypoint = selectedWaypoint;

	    // Find potentially better waypoint in front of where the robot is going
	    //    helps to remove waypoints behind the robot when the path was calculated and the robot moved
	    /*while (selectedWaypoint < searchParams.waypoints.size() &&
	           (robot.position.distanceTo(searchParams.waypoints[selectedWaypoint].position) < searchParams.skeletonSearchDistance ||
               fabs(robot.headingToPointXY(searchParams.waypoints[selectedWaypoint].position)) > searchParams.inFrontAngle)) {
            ++selectedWaypoint;
        }
	    if (selectedWaypoint < searchParams.waypoints.size()) {
	        Index2D robotCell = voronoi.findCell(robot.position);
	        Index2D waypointCell = voronoi.findCell(searchParams.waypoints[selectedWaypoint].position);
	        if (cellIsDirectlyTraversible(voronoi, waypointCell, robotCell)) {
                ROS_INFO("%s found better point in direction robot is facing (d: %.2lf a: %.1lf)", LOG_START, searchParams.skeletonSearchDistance, RAD2DEG(searchParams.inFrontAngle));
                searchParams.currentWaypoint = selectedWaypoint;
	        }
	    }*/

	    // Check if "at destination" (and rotation desired)
	    if (searchParams.waypoints.size() > 0) {
	        // If out of waypoints - at final destination
            if (searchParams.currentWaypoint >= searchParams.waypoints.size()) {
                // Check that the robot is facing the right direction
                Pose& lastWaypoint = searchParams.waypoints.back();
                double poseDiff = poseYawDifference(robot, lastWaypoint);
                double withinAngle = fabs(poseDiff) <= searchParams.proximityAngle;

                // If not within angle - set waypoint to last, and direct search to rotate
                if (searchParams.targetOrientation && !withinAngle) {
                    //ROS_INFO("%s waypoint: at goal, but rotating to desired (poseDiff: %.2lf)", LOG_START, RAD2DEG(poseDiff));
                    searchParams.rotateToPose = true;
                    searchParams.currentWaypoint = searchParams.waypoints.size() - 1;
                    searchParams.waypoint = lastWaypoint;
                    driveToTarget = lastWaypoint;
                } else {
                    // If so - the arrived
                    if (!searchParams.atDestination) {
                        // Update searchParams.atDestination on first occurrence of finishing navigation
                        Point destination = searchParams.waypoints[searchParams.waypoints.size()-1].position;
                        searchParams.atDestination = true;
                        searchParams.rotateToPose = false;
                        ROS_INFO("Explorer: Arrived at destination (%.2lf, %.2lf, %.2lf)", destination.x, destination.y, destination.z);
                        statusChanged(rsa17::ExplorerStatus::Status::STATUS_ARRIVED);
                    }
                    searchParams.waypoint = Pose(INFINITY, INFINITY, INFINITY);
                    driveToTarget = Pose(INFINITY, INFINITY, INFINITY);
                }
            } else {
                // Otherwise update current waypoint target
                //ROS_INFO("%s too far from current waypoint - drive to it", LOG_START);
                searchParams.waypoint = searchParams.waypoints[searchParams.currentWaypoint];
                //ROS_INFO("%s selected point: %s dist: %.2lf, angle: %.1lf", LOG_START, searchParams.waypoints[searchParams.currentWaypoint].position.toString().c_str(), robot.position.distanceTo(searchParams.waypoints[searchParams.currentWaypoint].position), RAD2DEG(robot.headingToPointXY(searchParams.waypoints[searchParams.currentWaypoint].position)));
            }
	    } else {
	        // If no waypoints, just set everything to nil
	        searchParams.waypoint = Pose(INFINITY, INFINITY, INFINITY);
            driveToTarget = Pose(INFINITY, INFINITY, INFINITY);
	    }

	}}

	// If waypoint has been selected - find drive-able path to it
	if (!searchParams.atDestination && !searchParams.rotateToPose) {
	    //ROS_INFO("%s Find traversible path to %s", LOG_START, searchParams.waypoint.position.toString().c_str());

        // Head towards searchParams.waypoint
        double waypointHeading = atan2(searchParams.waypoint.position.y - robot.position.y, searchParams.waypoint.position.x - robot.position.x);
        double waypointDistance = searchParams.waypoint.position.distanceTo(robot.position);
        double minAcceptableDistance = searchParams.wallSearchDistance - (searchParams.rayStep * voronoi.resolution);
        if ((waypointDistance - searchParams.proximityDistance) < minAcceptableDistance) {
            minAcceptableDistance = waypointDistance - searchParams.proximityDistance;
        }

        double d = traversibleDistance(voronoi, robot.position, waypointHeading, searchParams.wallSearchDistance);

        // Head directly towards waypoint, if far enough to searchParams.waypoint
        if (d >= minAcceptableDistance) {
            ROS_INFO("%s findWaypointTarget(): Heading directly towards waypoint, heading: %.2lf, dist: %.2lf", LOG_START, waypointHeading, waypointDistance);
            searchParams.pathBlocked = false;
            Point target = robot.position + Point(d*cos(waypointHeading), d*sin(waypointHeading), 0);
            driveToTarget = Pose(target, Quaternion());
        } else {
            ROS_INFO("%s findWaypointTarget(): Finding path in direction of waypoint", LOG_START);
            // Otherwise search for a path in the direction of the waypoint, at increasing radius of angles
            bool haveTarget = false;
            double a = searchParams.angleStep;

            while (!haveTarget && a <= searchParams.obstructionAngle) {
                // Check left branch
                d = traversibleDistance(voronoi, robot.position, waypointHeading + a, searchParams.wallSearchDistance);
                if (d >= minAcceptableDistance) {
                    //LOG("Explorer::findWaypointTarget(): Taking left branch (%.1lf).\n", RAD2DEG((a)));
                    haveTarget = true;
                    searchParams.pathBlocked = false;
                    Point target = robot.position + Point(d*cos(waypointHeading+a), d*sin(waypointHeading+a), 0);
                    driveToTarget = Pose(target, Quaternion());
                }

                // Check right branch (if left is no good)
                if (!haveTarget) {
                    d = traversibleDistance(voronoi, robot.position, waypointHeading - a, searchParams.wallSearchDistance);
                    if (d >= minAcceptableDistance) {
                        //LOG("Explorer::findWaypointTarget(): Taking right branch (%.1lf).\n", RAD2DEG((-a)));
                        haveTarget = true;
                        searchParams.pathBlocked = false;
                        Point target = robot.position + Point(d*cos(waypointHeading-a), d*sin(waypointHeading-a), 0);
                        driveToTarget = Pose(target, Quaternion());
                    }
                }

                // Inc loop
                a += searchParams.angleStep;
            }

            // If no target found - path is blocked
            if (!haveTarget) {
                if (!searchParams.pathBlocked) {
                    // Give message on first recording of path being blocked
                    ROS_WARN("Explorer::findWaypointTarget(): Path to waypoint (%.2lf, %.2lf, %.2lf) is blocked.",
                            searchParams.waypoint.position.x, searchParams.waypoint.position.y, searchParams.waypoint.position.z);
                }
                searchParams.pathBlocked = true;
                statusChanged(rsa17::ExplorerStatus::Status::STATUS_BLOCKED);
                driveToTarget = Pose(INFINITY, INFINITY, INFINITY);
            }
        }
	}


	return driveToTarget;
}

bool Explorer::cellIsDirectlyTraversible(const VoronoiGrid& voronoi, const Index2D& cell, const Index2D& pov) const {
	if (debugMsgs) ROS_INFO("%s cellIsDirectlyTransversible()", LOG_START); 
    double d = cell.distanceTo(pov);
    double dx = (cell.x - pov.x) / d;
    double dy = (cell.y - pov.y) / d;

    for (double r = 0; r <= d; r += searchParams.rayStep) {
        Index2D rayCell(pov.x + dx*r, pov.y + dy*r);

        if (!CELL_IS_VALID(rayCell, voronoi)) {
            return false;
        }

        const VoronoiGrid::VoronoiCell& vCell = voronoi.getVoronoiCell(rayCell);

        if (vCell.status == VoronoiGrid::VoronoiCell::Wall ||
                vCell.status == VoronoiGrid::VoronoiCell::Restricted ||
                vCell.status == VoronoiGrid::VoronoiCell::NotVisible) {
            return false;
        }
    }
    return true;
}

double Explorer::cellTraversibleRisk(const VoronoiGrid& voronoi, const Index2D& cell, const Index2D& pov) const {
        double d = cell.distanceTo(pov);
        double dx = (cell.x - pov.x) / d,
                dy = (cell.y - pov.y) / d;

        double risk = 0; uint32_t stepCount = 0;
        double restrictScale = voronoiConstraints.restricted / voronoi.resolution,
                partialScale = (voronoiConstraints.partial - voronoiConstraints.restricted) / voronoi.resolution,
                partialMax = voronoiConstraints.partial / voronoi.resolution,
                expansionScale = voronoiConstraints.expand / voronoi.resolution,
                expansionMax = (voronoiConstraints.expand + voronoiConstraints.partial) / voronoi.resolution;
        for (double r = 0; r <= d; r += searchParams.rayStep) {
            Index2D rayCell(pov.x + dx*r, pov.y + dy*r);

            if (!CELL_IS_VALID(rayCell, voronoi))
                return INFINITY;

            const VoronoiGrid::VoronoiCell& vCell = voronoi.getVoronoiCell(rayCell);

            if (vCell.status == VoronoiGrid::VoronoiCell::Wall ||
                    vCell.status == VoronoiGrid::VoronoiCell::Restricted ||
                    vCell.status == VoronoiGrid::VoronoiCell::NotVisible)
                return INFINITY;

            ++stepCount;
            if (vCell.status & VoronoiGrid::VoronoiCell::Vacant) {
                risk += 0.0;
            } else if (vCell.status & VoronoiGrid::VoronoiCell::PatiallyRestricted) {
                if (vCell.distanceFromWall > partialMax)
                    risk += 2;
                else
                    risk += 2 + (partialMax - vCell.distanceFromWall) / partialScale;
            } else {
                if (vCell.distanceFromWall > expansionMax)
                    risk += 0.0;
                else {
                    risk += (expansionMax - vCell.distanceFromWall) / expansionScale;
                }
            }
        }
        if (stepCount == 0) {
            return INFINITY;
        }
        return risk / stepCount;
    }

double Explorer::traversibleDistance(const VoronoiGrid& voronoi, const Point position, const double angle, const double max) {
	if (debugMsgs) ROS_INFO("%s transversibleDistance()", LOG_START); 
	double sinA = sin(angle), cosA = cos(angle), rStep = searchParams.rayStep * voronoi.resolution;

	double ray = 0;
	for (; ray <= max; ray += rStep) {
		Index2D cIdx = voronoi.findCell(position + Point(ray*cosA, ray*sinA, 0));

		if (!CELL_IS_VALID(cIdx, voronoi)) {
			break;
		}

		const VoronoiGrid::VoronoiCell& vCell = voronoi.getVoronoiCell(cIdx);
		if (vCell.status == VoronoiGrid::VoronoiCell::Wall ||
				vCell.status == VoronoiGrid::VoronoiCell::Restricted) {
			break;
		}
	}
	ray -= rStep;


	return ray;
}

double Explorer::poseYawDifference(const Pose& from, const Pose& to) {
	if (debugMsgs) ROS_INFO("%s poseYawDifference()", LOG_START); 
    double fromYaw, fromPitch, fromRoll;
    double toYaw, toPitch, toRoll;
    from.getYPR(fromYaw, fromPitch, fromRoll);
    to.getYPR(toYaw, toPitch, toRoll);

    // To difference
    double diff = toYaw - fromYaw;

    // Scale difference to [-180,180] range
    while (diff < -M_PIl) {
        diff += 2 * M_PIl;
    }
    while (diff > M_PIl) {
        diff -= 2 * M_PIl;
    }

    return diff;
}


#define CELL_PXL(C)		(((uint8_t *)rval->data) + (voronoi.height - (C).y - 1) * rval->step + 3 * (C).x)
ImagePtr Explorer::getPlanImage(const VoronoiGrid& voronoi, const Pose& robot, const Pose& target) {
	if (debugMsgs) ROS_INFO("%s getPlanImage()", LOG_START); 
	ImagePtr rval = voronoi.getImage();

//	LOG("Explorer::getPlanImage(): - robot(%.2lf, %.2lf, %.2lf) (%.2lf, %.2lf, %.2lf, %.2lf).\n",
//			robot.position.x, robot.position.y, robot.position.z,
//			robot.orientation.x, robot.orientation.y, robot.orientation.z, robot.orientation.w);
	// Draw robot pose
	Index2D robotCell = voronoi.findCell(robot.position);

	if (CELL_IS_VALID(robotCell, voronoi)) {
		double roll, pitch, yaw, cosYaw, sinYaw;
		robot.orientation.getYPR(yaw, pitch, roll);
		cosYaw = cos(yaw);
		sinYaw = sin(yaw);

		/**
		 * Draw robot heading.
		 */
		for (float r = 0; r <= 20; r += 0.95) {
			Index2D rayCell = robotCell + Index2D(r * cosYaw, r * sinYaw);

			if (!CELL_IS_VALID(rayCell, voronoi))
				continue;

			uint8_t* pxl = CELL_PXL(rayCell);

			pxl[0] = 196;
			pxl[1] = 196;
			pxl[2] = 255;
		}

		/**
		 * Mark robot position
		 */
		uint8_t* pxl = CELL_PXL(robotCell);
		pxl[0] = 0;
		pxl[1] = 0;
		pxl[2] = 255;
	}

	if (target.isFinite()) {
		Index2D targetCell = voronoi.findCell(target.position);

		if (CELL_IS_VALID(targetCell, voronoi)) {
			for (uint32_t i = 0; i < 3; ++i) {

				Index2D neighbors[4];
				neighbors[0] = Index2D(0, i);
				neighbors[1] = Index2D(0, -i);
				neighbors[2] = Index2D(i, 0);
				neighbors[3] = Index2D(-i, 0);

				for (uint32_t n = 0; n < 4; ++n) {
					Index2D mark = targetCell + neighbors[n];
					if (CELL_IS_VALID(mark, voronoi)) {
						uint8_t* pxl = CELL_PXL(mark);
						pxl[0] = 255;
						pxl[1] = 191;
						pxl[2] = 0;
					}
				}
			}
		}
	}


	// Draw search results
	if (searchParams.strategy == SearchParameters::WallFollow) {
		Index2D wallC(-1,-1), prevWallC(-1,-1);
		uint8_t* pxl = NULL;
		if (searchParams.wallTarget.isFinite()) {
			wallC = voronoi.findCell(searchParams.wallTarget);
		}
		if (searchParams.previousWallTarget.isFinite()) {
			prevWallC = voronoi.findCell(searchParams.previousWallTarget);
		}

		if (CELL_IS_VALID(wallC, voronoi) || wallC == prevWallC) {
			pxl = CELL_PXL(wallC);

			pxl[0] = 127;
			pxl[1] =   0;
			pxl[2] = 255;
		} else {
			if (CELL_IS_VALID(wallC, voronoi)) {
				pxl = CELL_PXL(wallC);

				pxl[0] = 255;
				pxl[1] =   0;
				pxl[2] = 255;
			}
			if (CELL_IS_VALID(prevWallC, voronoi)) {
				pxl = CELL_PXL(prevWallC);

				pxl[0] = 255;
				pxl[1] = 127;
				pxl[2] = 255;
			}
		}

		// The section of skeleton searched
		for (size_t i = 0; i < searchParams.skeletonSearch.size(); ++i) {
			Index2D idx = searchParams.skeletonSearch[i];
			if (CELL_IS_VALID(idx, voronoi)) {
				pxl = CELL_PXL(idx);

				pxl[0] = 127;
				pxl[1] = 127;
				pxl[2] = 127;
			}
		}

		if (CELL_IS_VALID(searchParams.skeleton2nd, voronoi)) {
			pxl = CELL_PXL(searchParams.skeleton2nd);

			pxl[0] = 191;
			pxl[1] = 127;
			pxl[2] =   0;
		}

		if (CELL_IS_VALID(searchParams.skeleton1st, voronoi)) {
			pxl = CELL_PXL(searchParams.skeleton1st);

			pxl[0] = 255;
			pxl[1] = 127;
			pxl[2] =   0;
		}
	} else if (searchParams.strategy == SearchParameters::Waypoint) {
		uint8_t* pxl = NULL;
		Lock lock(searchParams.waypointLock);

		if (searchParams.waypoint.isFinite()) {
			Index2D wC = voronoi.findCell(searchParams.waypoint.position);
			if (CELL_IS_VALID(wC, voronoi)) {
				Colour c(191, 127, 0);
				if (searchParams.pathBlocked)
					c = Colour(191,0,0);
				for (uint32_t i = 0; i < 3; ++i) {

					Index2D neighbors[4];
					neighbors[0] = Index2D(i, i);
					neighbors[1] = Index2D(i, -i);
					neighbors[2] = Index2D(-i, i);
					neighbors[3] = Index2D(-i, -i);

					for (uint32_t n = 0; n < 4; ++n) {
						Index2D mark = wC + neighbors[n];
						if (CELL_IS_VALID(mark, voronoi)) {
							uint8_t* pxl = CELL_PXL(mark);
							pxl[0] = c.r;
							pxl[1] = c.g;
							pxl[2] = c.b;
						}
					}
				}
			}
		}

		for (size_t w = searchParams.currentWaypoint; w < searchParams.waypoints.size(); ++w) {
			Index2D wC = voronoi.findCell(searchParams.waypoints[w].position);

			if (!CELL_IS_VALID(wC, voronoi))
				continue;

			pxl = CELL_PXL(wC);

			pxl[0] = 255;
			pxl[1] = 127;
			pxl[2] =   0;
		}
	}

	return rval;
}

Pose Explorer::findHysteresisTarget(const VoronoiGrid& voronoi, const Pose& robot, const Pose& target) {
	if (debugMsgs) ROS_INFO("%s findHysteresisTarget()", LOG_START); 
    Time timestamp = voronoi.timestamp;
    Pose actualTarget(INFINITY, INFINITY, INFINITY);

    if (target.isFinite()) {
        if (debugMsgs) ROS_INFO("%s Hysteresis target pose: %s", LOG_START, target.position.toString().c_str());
        // Pre-process to remove old data, and re-calculate updated data, etc.
        hysteresisPreprocess(timestamp, robot);
        if (debugMsgs) ROS_INFO("%s \t Hysteresis done pre-process", LOG_START);

        // Assign the target to a cluster
        int assignedCluster = hysteresisAssignCluster(timestamp, target, robot);
        if (debugMsgs) ROS_INFO("%s \t Hysteresis assigned cluster: %d", LOG_START, assignedCluster);

        // Do hysteresis of assigned cluster index
        int hysteresisCluster = hysteresisIndex(assignedCluster, timestamp);
        if (debugMsgs) ROS_INFO("%s \t Hysteresis hysteresis cluster: %d", LOG_START, hysteresisCluster);

        // Lookup target from cluster
        if (hysteresisCluster != HysteresisData::NO_INDEX) {
            actualTarget = hysteresis.clusters[hysteresisCluster].targets.back().pose;
        }
        if (debugMsgs) ROS_INFO("%s \t Hysteresis actual target pose: %s", LOG_START, actualTarget.position.toString().c_str());
    }

    return actualTarget;
}

void Explorer::hysteresisPreprocess(const Time& timestamp, const Pose& robot) {
	if (debugMsgs) ROS_INFO("%s histeresisPreprocess()", LOG_START); 
    Time clusterTTL = timestamp - hysteresisParams.clusterTimeout;
    Time hysteresisTTL = timestamp - hysteresisParams.hysteresisTimeout;

    // Process clusters
    for (HysteresisData::Cluster& cluster : hysteresis.clusters) {
        Point3D centre(0, 0, 0);

        // Remove old poses from clusters
        // Calculate new cluster centres
        for(auto it = cluster.targets.begin(); it != cluster.targets.end(); /*inc in loop*/ ) {
            HysteresisData::TimestampedPose& element = *it;
            if (element.timestamp < clusterTTL) {
                it = cluster.targets.erase(it);
            } else {
                centre += element.pose.position;

                ++it;
            }
        }

        // Calculate new cluster centre offsets (distance & heading)
        if (cluster.isEmpty()) {
            cluster.reset();
        } else {
            centre = centre / cluster.targets.size();
            cluster.centre = centre;
            cluster.centreHeading = robot.headingToPointXY(centre);
            cluster.centreDistance = robot.position.distanceTo(centre);
        }
    }

    // Process hysteresis
    for (auto it = hysteresis.hysteresis.begin(); it != hysteresis.hysteresis.end(); /*inc in loop*/ ) {
        HysteresisData::Hysteresis& hyst = *it;
        if (hyst.timestamp < hysteresisTTL) {
            it = hysteresis.hysteresis.erase(it);
        } else {
            ++it;
        }
    }
    while (hysteresis.hysteresis.size() > 5) {
        hysteresis.hysteresis.pop_front();
    }
}

int Explorer::hysteresisAssignCluster(const Time& timestamp, const Pose& target, const Pose& robot) {
	if (debugMsgs) ROS_INFO("%s hysteresisAssignCluster()", LOG_START); 
    // Get dist/heading values
    double targetDistance = robot.position.distanceTo(target.position);
    double targetHeading = robot.headingToPointXY(target.position);
    int assignedIndex = HysteresisData::NO_CLUSTER;

    auto withinMargin = [](double& target, double& value, double& margin) {
        return (fabs(target - value) <= margin);
    };

    // Find cluster (by index)
    for (int clusterIndex = 0; clusterIndex != hysteresis.clusters.size(); ++clusterIndex) {
        HysteresisData::Cluster& cluster = hysteresis.clusters[clusterIndex];

        if (withinMargin(targetDistance, cluster.centreDistance, hysteresisParams.marginDistance) &&
            withinMargin(targetHeading, cluster.centreHeading, hysteresisParams.marginHeading) &&
            assignedIndex == HysteresisData::NO_CLUSTER) {
            assignedIndex = clusterIndex;
        }
    }
    //ROS_INFO("%s::hysteresisAssignCluster \t assignedIndex: %d (from clusters)", LOG_START, assignedIndex);

    // If no cluster, then assign to a new cluster (first empty cluster)
    { // scoping block
        int clusterIndex = 0;
        while (assignedIndex == HysteresisData::NO_CLUSTER &&
                clusterIndex != hysteresis.clusters.size()) {
            if (hysteresis.clusters[clusterIndex].targets.size() == 0) {
                assignedIndex = clusterIndex;
            }

            ++clusterIndex;
        }
        if (assignedIndex == HysteresisData::NO_CLUSTER) {
            // Create new cluster
            hysteresis.clusters.push_back(HysteresisData::Cluster());
            assignedIndex = hysteresis.clusters.size() - 1;
        }
    }
    //ROS_INFO("%s::hysteresisAssignCluster \t assignedIndex: %d (if new cluster)", LOG_START, assignedIndex);

    // Add target to assigned cluster
    HysteresisData::TimestampedPose assignedTarget(target, timestamp);
    hysteresis.clusters[assignedIndex].targets.push_back(assignedTarget);

    return assignedIndex;
}

int Explorer::hysteresisIndex(int assignedCluster, const Time& timestamp) {
	if (debugMsgs) ROS_INFO("%s hysteresisIndex()", LOG_START); 
    // Add index to hysteresis
    hysteresis.hysteresis.push_back(HysteresisData::Hysteresis(assignedCluster, timestamp));

    // Check for uniqueness
    int selectedIndex = hysteresis.index;
    int uniqueIndex = hysteresis.hysteresis.front().index;
    bool unique = true;
    for (HysteresisData::Hysteresis& index : hysteresis.hysteresis) {
        if (index.index != uniqueIndex) {
            unique = false;
        }
    }

    // If hysteresis is unique - then switch
    if (unique) {
        selectedIndex = uniqueIndex;
    }

    // Check selected index has values in the cluster
    if (selectedIndex < hysteresis.clusters.size() &&
        hysteresis.clusters[selectedIndex].targets.size() == 0) {
        // Revert back to assignedCluster
        selectedIndex = assignedCluster;
    }

    // Set selected index
    hysteresis.index = selectedIndex;

    return selectedIndex;
}

} // namespace crosbot


