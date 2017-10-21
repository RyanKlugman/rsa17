/*
 * basicClient.hpp
 *
 *  Created on: 04/07/2016
 *      Author: rescue
 */

#ifndef CROSBOT_EXPLORE_BASIC_CLIENT_HPP_
#define CROSBOT_EXPLORE_BASIC_CLIENT_HPP_

#include <crosbot/geometry/poses.hpp>
#include <crosbot/handle.hpp>
#include <crosbot/controls/command.hpp>
#include <crosbot/controls/status.hpp>
#include <rsa17/ExplorerFeedback.h>
#include <rsa17/SetExplorerMode.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/publisher.h>
#include <ros/service.h>
#include <tf/transform_listener.h>

namespace rsa17 {

class BasicExplorerClient : public crosbot::HandledObject {
public:
    BasicExplorerClient();
    virtual ~BasicExplorerClient();

    void configure();
    void startup();
    void shutdown();

    // Command callbacks
    void callback_targetPath(const nav_msgs::PathPtr& targetPose);
    void callback_receivedCommand(const crosbot_msgs::ControlCommandPtr command);
    void callback_explorerFeedback(const rsa17::ExplorerFeedbackConstPtr& feedback);

private:
    // Configuration
    std::string explorer_srv_name;
    std::string explorer_feedback_sub_name;
    std::string targetPathSub_name;
    std::string robot_frame;
    std::string postrackResetPub_name;
    std::string slamResetPub_name;
    std::string world_frame;
    nav_msgs::Path markerPath;
    int goalIndex;

    // Publishers/subscribers/services
    crosbot::CrosbotCommandPtr crosbotCommand;
    crosbot::CrosbotStatusPtr crosbotStatus;
    ros::ServiceClient explorer_srv;
    ros::Subscriber explorer_feedback_sub;
    ros::Publisher pubPostrackReset;
    ros::Publisher pubGraphSlamReset;
    ros::Subscriber subTargetPath;
    tf::TransformListener tfListener;

    /**
     * Set mode of rsa17
     */
    void setExploreMode(int mode);

    /**
     * Get command points
     */
    crosbot::Pose3D getCommandPoint(const crosbot_msgs::ControlCommandPtr command);
    crosbot::Pose3D getCommandPose(const crosbot_msgs::ControlCommandPtr command);

    /**
     * Get A* path to origin and set path
     */
    void setAStarPose(crosbot::Pose3D targetPose, bool targetOrientation);
    void setAStarPose(const geometry_msgs::PoseStamped& targetPose, bool targetOrientation);

    void command_reset(const crosbot_msgs::ControlCommandPtr command);
};
typedef crosbot::Handle<BasicExplorerClient> BasicExplorerClientPtr;

} // namespace rsa17

#endif /* CROSBOT_EXPLORE_BASIC_CLIENT_HPP_ */
