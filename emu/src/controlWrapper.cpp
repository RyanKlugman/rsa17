/*
 * controlWrapper.cpp
 *
 *  Created on: 04/07/2016
 *      Author: rescue
 */

#include <ros/ros.h>

#include <controlWrapper.hpp>

#include <crosbot/geometry/poses.hpp>
#include <crosbot_explore/FollowPath.h>
#include <crosbot_explore/GetPath.h>
#include <crosbot_explore/SetMode.h>
#include <common.hpp>

#include <std_msgs/String.h>


#define CONTROL_COMMAND_RESET_MAP       "reset_map"
#define LOG_START                       "TBControlWrapper ::"

namespace timothyw_emu {

class _CommandCallback : public crosbot::CrosbotCommandCallback {
private:
    TBControlWrapperPtr wrapper;

public:
    _CommandCallback(TBControlWrapperPtr wrapper) : wrapper(wrapper) {};
    virtual ~_CommandCallback() {};
    virtual void callback_receivedCommand(const crosbot_msgs::ControlCommandPtr command) {
        if (wrapper != NULL) wrapper->callback_receivedCommand(command);
    };
};

TBControlWrapper::TBControlWrapper() {
}

TBControlWrapper::~TBControlWrapper() {
}

void TBControlWrapper::configure() {
    // Use "local" namespace for config
    ros::NodeHandle nh("~");

    aStarSrvName = nh.param<std::string>("astar_path_topic", "astar_path");
    exploreModeSrvName = nh.param<std::string>("crosbot_explore_mode_topic", "crosbot_explore_mode");
    explorePathSrvName = nh.param<std::string>("crosbot_explore_path_topic", "crosbot_explore_path");
    robot_frame = nh.param<std::string>("robotFrame", "base_link");
    world_frame = nh.param<std::string>("worldFrame", "icp_test");
}

void TBControlWrapper::startup() {
    // Use "local" namespace for config
    ros::NodeHandle nh("~");

    crosbotCommand = new crosbot::CrosbotCommand(TURTLEBOT_CONTROL_NAMESPACE, true,
            new _CommandCallback(this));
    crosbotStatus = new crosbot::CrosbotStatus(TURTLEBOT_CONTROL_NAMESPACE);
    aStarSrv = nh.serviceClient<crosbot_explore::GetPath>(aStarSrvName);
    exploreModeSrv = nh.serviceClient<crosbot_explore::SetMode>(exploreModeSrvName);
    explorePathSrv = nh.serviceClient<crosbot_explore::FollowPath>(explorePathSrvName);
    pubPostrackReset = nh.advertise<std_msgs::String>("/postrack/resetMap", 1);
    pubGraphSlamReset = nh.advertise<std_msgs::String>("/graphslam/resetMap", 1);
}

void TBControlWrapper::shutdown() {
    crosbotCommand = NULL;
    exploreModeSrv.shutdown();
}

void TBControlWrapper::callback_receivedCommand(const crosbot_msgs::ControlCommandPtr command) {
    if (command->command == crosbot_msgs::ControlCommand::CMD_EM_STOP) {
        setExploreMode(crosbot_explore::SetMode::Request::MODE_PAUSED);
    } else if (command->command == crosbot_msgs::ControlCommand::CMD_RESET) {
        command_reset(command);
    } else if (command->command == crosbot_msgs::ControlCommand::CMD_RESUME) {
        setExploreMode(crosbot_explore::SetMode::Request::MODE_RESUME);
    } else if (command->command == crosbot_msgs::ControlCommand::CMD_START) {
        setExploreMode(crosbot_explore::SetMode::Request::MODE_RESUME);
    } else if (command->command == crosbot_msgs::ControlCommand::CMD_STOP) {
        setExploreMode(crosbot_explore::SetMode::Request::MODE_PAUSED);
    } else if (command->command == TURTLEBOT_COMMAND_LEFTWALL_FOLLOW) {
        setExploreMode(crosbot_explore::SetMode::Request::MODE_LEFT_WALL);
    } else if (command->command == TURTLEBOT_COMMAND_RIGHTWALL_FOLLOW) {
        setExploreMode(crosbot_explore::SetMode::Request::MODE_RIGHT_WALL);
    }
}

void TBControlWrapper::setExploreMode(int mode) {
    if (exploreModeSrv.waitForExistence(ros::Duration(0.1))) {
        crosbot_explore::SetModeRequest request;
        crosbot_explore::SetModeResponse response;

        request.mode = mode;
        STATUS_INFO(crosbotStatus, "%s sending explore mode: %d", LOG_START, mode);

        exploreModeSrv.call(request, response);
        STATUS_WARN(crosbotStatus, "%s SUCCEED", LOG_START);
    } else {
    	STATUS_WARN(crosbotStatus, "%s FAIL", LOG_START);
        STATUS_ERROR(crosbotStatus, "%s Explore Mode Service unavailable (%s)", LOG_START, exploreModeSrv.getService().c_str());
    }
}

void TBControlWrapper::command_reset(const crosbot_msgs::ControlCommandPtr command) {
    std_msgs::String msg;
    msg.data = CONTROL_COMMAND_RESET_MAP;
    pubPostrackReset.publish(msg);
    pubGraphSlamReset.publish(msg);
}

} // namespace timothyw_turtlebot


