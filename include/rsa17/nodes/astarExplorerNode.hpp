/*
 * explorerNode.hpp
 *
 *  Created on: 30/08/2016
 *      Author: rescue
 */

#ifndef CROSBOT_EXPLORE_MOVE_NODE_HPP_
#define CROSBOT_EXPLORE_MOVE_NODE_HPP_

#include <rsa17/explorerROS.hpp>

#include <crosbot/handle.hpp>
#include <rsa17/astar.hpp>
#include <rsa17/astar/astarParameters.hpp>
#include <rsa17/CrosbotAStarReconfigConfig.h>

namespace rsa17 {

class MoveMode : public ExploreMode {
public:
    enum {
        MODE_ASTAR  = 5
    };
};

} // namespace rsa17

namespace crosbot {

// Typedef for A* specific reconfigure
typedef dynamic_reconfigure::Server<rsa17::CrosbotAStarReconfigConfig> AStarReconfigServer;

// Forward reference
class AStarExplorerROSNode;

class PlanningThreadMove : public crosbot::Thread, public crosbot::HandledObject {
public:

    PlanningThreadMove(AStarExplorerROSNode *moveNode);
    virtual ~PlanningThreadMove();

    virtual void run();

private:
    AStarExplorerROSNode *moveNode;
};
typedef crosbot::Handle<PlanningThreadMove> PlanningThreadMovePtr;

class AStarExplorerROSNode : public ExplorerROSNode {
public:

    AStarExplorerROSNode();
    virtual ~AStarExplorerROSNode();

    virtual void configure();
    virtual void startup();
    virtual void shutdown();

    // Planning loop for calculating updated A* path
    void planningLoop();

    // Over-rides from crosbot::Explorer
    // TODO: implement override
//    virtual void statusChanged(const std::string& status);

protected:
    // Reconfig callback
    void callback_astarOccGrid(const nav_msgs::OccupancyGridConstPtr& astarLatestMap);
    void callback_astarReconfigure(rsa17::CrosbotAStarReconfigConfig &config, uint32_t level);

    // Astar Voronoi
    virtual void loadLatestVoronoi();
    VoronoiGridPtr getLatestAstarVoronoi();

    // Over-rides from crosbot::Explorer
    virtual void handleExploreMode(rsa17::SetExplorerModeRequest &request,
            rsa17::SetExplorerModeResponse &response, bool& enableDrive);

    /**
     * Set a sequence of waypoints to follow, via an A* Plan.
     * Converts the plan to a nav_msgs::Path and calls ExplorerROSNode::setWaypoints(nav_msgs::Path)
     *  in the frame of navPathFrameId
     * @return Were the waypoints could be configured
     */
    bool setWaypoints(Plan& plan, const std::string& frameId, const std::string& navPathFrameId, const crosbot::Time time);

private:
    PlanningThreadMovePtr planningThread;

    // Configuration
    std::string astarImgPub_name;
    std::string astarGridSub_name;
    bool differentPlanningGrid;
    double pointDistance;
    double rate_astarReplan;
    double frequency_voronoiUpdate;
    VoronoiGrid::Constraints astarVoronoiConstraints;

    // ROS Publishers/Subscribers
    ros::Subscriber astarGridSub;
    ros::Publisher astarImagePub;

    // Dynamic reconfiguration - only on config request
    AStarReconfigServer *aStarReconfig_server;
    AStarReconfigServer::CallbackType aStarReconfig_callbackType;
    boost::recursive_mutex aStarMutexReconfig;

    // A* planner
    ReadWriteLock astarRosLock;
    AStarPlanner planner;
    nav_msgs::OccupancyGridConstPtr astarLatestMap;
    VoronoiGridPtr astarLatestVoronoi;
    std::deque<Pose> astarWaypoints;
    ros::Time lastAstarVoronoiUpdate;

    // Planning loop
    bool planningOperating;
    ReadWriteLock dataLock;
    Pose3DStamped currentAStarGoal;

    /**
     * Publish A* Configuration to dynamic parameter server
     */
    void publishReconfig(AStarParameters& params);

    /**
     * Store current A* goal, taking care of concurrency issues
     */
    void setCurrentAStarGoal(const Pose3DStamped& goal);

    /**
     * Get The A* goal, in the frame of reference of the voronoi map
     */
    Pose3DStamped getCurrentAStarGoal(std::string frame_id, Time timestamp);

    /**
     * Calculate entire A* path
     */
    void calculateAStarPath();

    /**
     * Revise the sequence of waypoints for the robot to follow
     */
    bool updateWaypoints();
};

} // namespace crosbot


#endif /* CROSBOT_EXPLORE_MOVE_NODE_HPP_ */
