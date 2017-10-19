/*
 * searchParameters.hpp
 *
 *  Created on: 11/10/2016
 *      Author: rescue
 */

#ifndef CROSBOT_EXPLORER_SEARCHPARAMETERS_HPP_
#define CROSBOT_EXPLORER_SEARCHPARAMETERS_HPP_

#include <crosbot/thread.hpp>
#include <crosbot/geometry/index2d.hpp>
#include <crosbot/geometry/poses.hpp>

#include <vector>

namespace crosbot {

struct SearchParameters {
public:
    enum Strategy {
        WallFollow, Waypoint
    };
    enum Side {
        Left, Right
    };


    /* Current search params */
    Strategy strategy;
    Side side;
    bool targetOrientation;             // Target the orientation of final goal/path/waypoints

    /* Configuration */
    double angleStep;
    double inFrontAngle;                // Robot specifying if something is "in front of the robot"
    uint32_t maxIterations;
    double minTravelDistance;           // Minimum travel distance for wall following target
    double obstructionAngle;            // Angle to deviate by to consider that a path is blocked
    double proximityAngle;              // Angle to target pose to define "at goal"
    double proximityDistance;           // Distance to target pose to define "at goal"
    double rayStep;
    double skeletonSearchDistance;      // Distance to look for skeleton cells
    double wallSearchDistance;          // Radial distance to look for a wall, during wall-following
    double wallSearchAngle;             // Angle at which to start a search for the wall


    /* Waypoint management */
    ReadWriteLock waypointLock;
    std::vector<Pose> waypoints;
    size_t currentWaypoint;
    Pose waypoint;
    bool atDestination;
    bool rotateToPose;
    bool pathBlocked;

    /* Search results */
    Point wallTarget;
    Point previousWallTarget;
    Index2D skeleton1st;
    Index2D skeleton2nd;
    std::vector<Index2D> skeletonSearch;


    SearchParameters() :
        strategy(WallFollow),
        side(Left),
        targetOrientation(false),
        angleStep(DEG2RAD(1)),
        inFrontAngle(DEG2RAD(60)),
        maxIterations(1000),
        minTravelDistance(0.1),
        obstructionAngle(DEG2RAD(45)),
        proximityAngle(DEG2RAD(20)),
        proximityDistance(0.5),
        rayStep(0.9),
        skeletonSearchDistance(1.0),
        wallSearchDistance(1.0),
        wallSearchAngle(DEG2RAD(90)),
        atDestination(false),
        rotateToPose(false),
        pathBlocked(false),
        currentWaypoint(0),
        wallTarget(INFINITY, INFINITY, INFINITY),
        previousWallTarget(INFINITY, INFINITY, INFINITY)
    {}

    void resetWaypointData() {
        Lock lock(waypointLock, true);
        atDestination = false;
        currentWaypoint = 0;
        rotateToPose = false;
        pathBlocked = false;
        waypoint = Pose(INFINITY, INFINITY, INFINITY);
    }

    void resetWallFollowData() {
        wallTarget = Point(INFINITY, INFINITY, INFINITY);
        previousWallTarget = Point(INFINITY, INFINITY, INFINITY);
        targetOrientation = false;
    }

    void setWaypoints(const std::vector<Pose>& waypoints, const Pose& pose) {
        Lock lock(waypointLock, true);
        atDestination = false;
        rotateToPose = false;
        pathBlocked = false;
        this->waypoints = waypoints;
        for (int64_t w = waypoints.size() - 1; w >= 0; --w) {
            if (waypoints[w].position.distanceTo(pose.position) <= proximityDistance) {
                currentWaypoint = w;
                return;
            }
        }
        currentWaypoint = 0;
    }


    static std::vector<Pose> createSpiral() {
        std::vector<Pose> rval;
        double a, r;
        for (int32_t i = 0; i < 100; ++i) {
            r = i * 0.02; a = i * .10;
            rval.push_back(Pose(r * cos(a), r * sin(a), 0));
        }

        return rval;
    }

    static std::vector<Pose> createStair() {
        std::vector<Pose> rval;
        double x, y;
        for (int32_t i = 0; i < 1000; ++i) {
            if ((i / 100) % 2 == 0) {
                y = i / 200;
                x = (i % 100) * 0.01 + i / 200;
            } else {
                y = (i % 100) * 0.01 + (i-100) / 200;
                x = (i + 100) / 200;
            }

            rval.push_back(Pose(x + 2, y, 0));
        }

        return rval;
    }
};

} // namespace crosbot



#endif /* CROSBOT_EXPLORER_SEARCHPARAMETERS_HPP_ */
