/*
 * explorer.hpp
 *
 *  Created on: 22/02/2013
 *      Author: mmcgill
 */

#ifndef CROSBOT_EXPLORER_HPP_
#define CROSBOT_EXPLORER_HPP_

#include <crosbot/thread.hpp>
#include <crosbot_map/voronoi.hpp>

#include <rsa17/basicClient/actionParameters.hpp>
#include <rsa17/driveParameters.hpp>
#include <rsa17/hysteresisData.hpp>
#include <rsa17/hysteresisParameters.hpp>
#include <rsa17/searchParameters.hpp>

namespace crosbot {

class Explorer {
protected:
    void driveThread();
    void planThread();
    void voronoiThread();

	class PlanThread : public Thread {
	public:
		Explorer& explorer;
		PlanThread(Explorer& explorer) : explorer(explorer) {}

		void run() {
			explorer.planThread();
		}
	};

    class DriveThread : public Thread {
    public:
        Explorer& explorer;
        DriveThread(Explorer& explorer) : explorer(explorer) {}

        void run() {
            explorer.driveThread();
        }
    };

    class VoronoiThread : public Thread {
    public:
        Explorer& explorer;
        VoronoiThread(Explorer& explorer) : explorer(explorer) {}

        void run() {
            explorer.voronoiThread();
        }
    };

	// Processing threads
    DriveThread driveThread_;
	PlanThread planThread_;
	VoronoiThread voronoiThread_;
	double rate_driveThread;
	double rate_planThread;
	double rate_voronoiThread;

	// Concurrency handling
	Mutex planningThreadMutex;
	Mutex drivingThreadMutex;

	// Configuration
	bool debugMsgs;
	DriveParameters driveParams;
	HysteresisParameters hysteresisParams;
	SearchParameters searchParams;
	bool searchWhilePaused;
	VoronoiGrid::Constraints voronoiConstraints;

	// Processing data
	bool operating;
	bool paused;
	bool stopSent;
	HysteresisData hysteresis;

	// Searching for drive targets
	virtual Pose findDriveTarget(const VoronoiGrid& voronoi, const Pose& robot);
	virtual Pose findWallFollowTarget(const VoronoiGrid& voronoi, const Pose& robot);
	virtual Pose findWaypointTarget(const VoronoiGrid& voronoi, const Pose& robot);
	virtual ImagePtr getPlanImage(const VoronoiGrid& voronoi, const Pose& robot, const Pose& target);

	// Hysteresis of finding drive target
	virtual Pose findHysteresisTarget(const VoronoiGrid& voronoi, const Pose& robot, const Pose& target);
	void hysteresisPreprocess(const Time& timestamp, const Pose& robot);
	int hysteresisAssignCluster(const Time& timestamp, const Pose& target, const Pose& robot);
	int hysteresisIndex(int assignedCluster, const Time& timestamp);

	bool cellIsDirectlyTraversible(const VoronoiGrid& voronoi, const Index2D& cell, const Index2D& pov) const;
	double cellTraversibleRisk(const VoronoiGrid& voronoi, const Index2D& cell, const Index2D& pov) const;

	// functions for the wall follow implementation
	virtual Point findWall(const VoronoiGrid& voronoi, const Pose& robot, double startAngle);
	virtual Index2D findFirstSkeletonCell(const VoronoiGrid& voronoi, const Pose& robot, double startAngle);
	virtual Index2D findNextSkeletonCell(const VoronoiGrid& voronoi, const Index2D currentCell, const Index2D previous);
	virtual Index2D findSaferSkeletonCell(const VoronoiGrid& voronoi, const Index2D currentCell, const Index2D robot);
//  virtual Index2D findSaferCell(const VoronoiGrid& voronoi, const Index2D currentCell, const Index2D robot);
//	virtual void findSaferTarget(double& voronoiAngle, double& voronoiDistance, const Pose& robot);

	// functions for the waypoint follow implementation
	virtual double traversibleDistance(const VoronoiGrid& voronoi, const Point position, const double angle, const double max = INFINITY);

	// Calculations
	virtual double poseYawDifference(const Pose& from, const Pose& to);

	// Driving execution
	/**
	 * Load the latest pose of the robot
	 */
	virtual void loadLatestPose() = 0;

	/**
     * Load the latest pose of the robot
     */
	virtual void loadLatestVoronoi() = 0;

	/**
	 * Get the most recently loaded pose - does not load the pose
	 */
	virtual Pose getLatestPose() = 0;

	/**
	 * Get the most recently loaded voronoi grid - does not load a new grid
	 */
	virtual VoronoiGridPtr getLatestVoronoi() = 0;

	/**
	 * Stop the robot driving
	 */
	virtual bool stopMotors() = 0;
	virtual void driveTo(const Pose& relativePosition)=0;

public:
	Explorer();
	virtual ~Explorer();

    virtual void configure();
    virtual void startup();
    virtual void shutdown();

	virtual void pause();
	virtual void resume();


	virtual void statusChanged(rsa17::ExplorerStatus::Status status) {}

};

} // namespace crosbot

#endif /* CROSBOT_EXPLORER_HPP_ */
