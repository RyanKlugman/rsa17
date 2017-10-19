/*
 * explorerNode.hpp
 *
 *  Created on: 30/08/2016
 *      Author: rescue
 */

#ifndef CROSBOT_EXPLORE_EXPLORERNODE_HPP_
#define CROSBOT_EXPLORE_EXPLORERNODE_HPP_

#include <rsa17/explorer.hpp>

#include <actionlib/server/simple_action_server.h>
#include <crosbot/thread.hpp>
#include <rsa17/GetPath.h>
#include <rsa17/CrosbotExploreReconfigConfig.h>
#include <rsa17/SetExplorerMode.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/publisher.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <tf/transform_listener.h>

#include <string>

namespace crosbot {

// Typedef for reconfigure
typedef dynamic_reconfigure::Server<rsa17::CrosbotExploreReconfigConfig> ReconfigServer;

class ExplorerROSNode : public Explorer {
protected:
    // Configuration
    std::string baseFrame;
    std::string explorer_srv_name;
    std::string explorer_feedback_pub_name;
    std::string gridSub_name;
    std::string historySub_name;
    std::string imagePub_name;
    std::string velPub_name;

    // Current action/goal
    int currentGoalId;

    // ROS Topics
    ros::ServiceServer  explorer_srv;
    ros::Publisher      explorer_feedback_pub;
    ros::Subscriber     gridSub;
    ros::Subscriber     historySub;
    ros::Publisher      velPub;
    ros::Publisher      voronoiImagePub;
    tf::TransformListener tfListener;

    // Latest information
    ReadWriteLock lock_ros;
    ReadWriteLock lock_pose;
    nav_msgs::OccupancyGridConstPtr latestMap;
    nav_msgs::PathConstPtr latestHistory;
    VoronoiGridPtr latestVoronoi;
    Pose latestPose;

public:
    ExplorerROSNode();
    virtual ~ExplorerROSNode();

    virtual void configure();
    virtual void startup();
    virtual void shutdown();

    // Over-rides from crosbot::Explorer
    virtual void loadLatestPose();
    virtual void loadLatestVoronoi();
    virtual Pose getLatestPose();
    virtual VoronoiGridPtr getLatestVoronoi();
    virtual Pose findDriveTarget(const VoronoiGrid& voronoi, const Pose& robot);
    virtual bool stopMotors();
    virtual void driveTo(const Pose& relativePosition);
    virtual void statusChanged(rsa17::ExplorerStatus::Status status);

    // ROS callbacks
    void callbackOccGrid(const nav_msgs::OccupancyGridConstPtr& latestMap);
    void callbackHistory(const nav_msgs::PathConstPtr& latestHistory);

    // Reconfig callback
    void callback_reconfigure(rsa17::CrosbotExploreReconfigConfig &config, uint32_t level);

    // Service callback
    bool callbackExplorerMode(rsa17::SetExplorerModeRequest &request, rsa17::SetExplorerModeResponse &response);

protected:
    /**
     * Subscribe to grid topic
     */
    void subscribeGrid(ros::NodeHandle& nh);

    /**
     * Get the latest pose of the robot, given the frame-of-reference and timestamp, rather than pulling it from the latestMap
     */
    virtual Pose getLatestPose(std::string frame_id, Time timestamp);

    /**
     * Define the necessary parameters for processing the goal mode.
     */
    virtual void handleExploreMode(rsa17::SetExplorerModeRequest &request,
            rsa17::SetExplorerModeResponse &response, bool& enableDrive);

    /**
     * Set a sequence of waypoints to follow, via a nav_msgs::Path
     * @return Were the waypoints could be configured
     */
    bool setWaypoints(nav_msgs::Path& navPath);

    /**
     * Publish Configuration to dynamic parameter server
     */
    void publishReconfig(VoronoiGrid::Constraints& constraints);

private:
    // Allow local voronoi constraints for this explorer (this allows the constraints to be reset between different modes)
    VoronoiGrid::Constraints localVoronoiConstraints;

    // Dynamic reconfiguration - only on config request
    ReconfigServer *reconfig_server;
    ReconfigServer::CallbackType reconfig_callbackType;
    boost::recursive_mutex mutexReconfig;
    bool gotReconfig;
};

} // namespace crosbot


#endif /* CROSBOT_EXPLORE_EXPLORERNODE_HPP_ */
