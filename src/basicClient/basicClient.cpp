/*
 * controlWrapper.cpp
 *
 *  Created on: 04/07/2016
 *      Author: rescue
 */

#include <ros/ros.h>

#include <crosbot/geometry/poses.hpp>
#include <rsa17/basicClient/actionParameters.hpp>
#include <rsa17/basicClient/basicClient.hpp>
#include <rsa17/basicClient/basicClientCommands.hpp>
#include <rsa17/nodes/astarExplorerNode.hpp>

#include <std_msgs/String.h>
#include <string>

#define CONTROL_COMMAND_RESET_MAP       "reset_map"
#define DEFAULT_SERVER_WAIT             0.5
#define LOG_START                       "BasicExplorerClient ::"


#define STATUS_ERROR(STPUB, FMT, ...)                                                       \
        STPUB->sendStatus(crosbot_msgs::ControlStatus::LEVEL_ERROR, FMT, __VA_ARGS__);      \
        ROS_ERROR(FMT, __VA_ARGS__);

#define STATUS_INFO(STPUB, FMT, ...)                                                        \
        STPUB->sendStatus(crosbot_msgs::ControlStatus::LEVEL_INFO, FMT, __VA_ARGS__);       \
        ROS_INFO(FMT, __VA_ARGS__);

#define STATUS_WARN(STPUB, FMT, ...)                                                        \
        STPUB->sendStatus(crosbot_msgs::ControlStatus::LEVEL_WARNING, FMT, __VA_ARGS__);    \
        ROS_WARN(FMT, __VA_ARGS__);


namespace rsa17 {

class _CommandCallback : public crosbot::CrosbotCommandCallback {
private:
    BasicExplorerClientPtr wrapper;

public:
    _CommandCallback(BasicExplorerClientPtr wrapper) : wrapper(wrapper) {};
    virtual ~_CommandCallback() {};
    virtual void callback_receivedCommand(const crosbot_msgs::ControlCommandPtr command) {
        if (wrapper != NULL) wrapper->callback_receivedCommand(command);
    };
};

BasicExplorerClient::BasicExplorerClient()
{}

BasicExplorerClient::~BasicExplorerClient() {
}

void BasicExplorerClient::configure() {
    // Use "local" namespace for config
    ros::NodeHandle nh("~");

    explorer_srv_name = nh.param<std::string>("explore_srv", "/rsa17/explorer_service");
    explorer_feedback_sub_name = nh.param<std::string>("explorer_feedback_sub", "/rsa17/explorer_feedback");
    targetPathSub_name = nh.param<std::string>("targetPath_sub", "/targetPath");
    robot_frame = nh.param<std::string>("robotFrame", "base_link");
    postrackResetPub_name = nh.param<std::string>("postrackReset_pub", "/postrack/resetMap");
    slamResetPub_name = nh.param<std::string>("slamReset_pub", "/graphslam/resetMap");
    world_frame = nh.param<std::string>("worldFrame", "icp_test");
    goalIndex = 0;
}

void BasicExplorerClient::startup() {
    // Use "local" namespace for config
    ros::NodeHandle nh("~");

    crosbotCommand = new crosbot::CrosbotCommand(CROSBOT_EXPLORE_DEFAULT_CONTROL_NAMESPACE, true,
            new _CommandCallback(this));
    crosbotStatus = new crosbot::CrosbotStatus(CROSBOT_EXPLORE_DEFAULT_CONTROL_NAMESPACE);
    explorer_srv = nh.serviceClient<rsa17::SetExplorerMode>(explorer_srv_name);
    explorer_feedback_sub = nh.subscribe(explorer_feedback_sub_name, 1, &BasicExplorerClient::callback_explorerFeedback, this);
    pubPostrackReset = nh.advertise<std_msgs::String>(postrackResetPub_name, 1);
    pubGraphSlamReset = nh.advertise<std_msgs::String>(slamResetPub_name, 1);
    subTargetPath = nh.subscribe(targetPathSub_name, 1, &BasicExplorerClient::callback_targetPath, this);
}

void BasicExplorerClient::shutdown() {
    crosbotCommand = NULL;
    explorer_srv.shutdown();
    explorer_feedback_sub.shutdown();
}

void BasicExplorerClient::callback_targetPath(const nav_msgs::PathPtr& targetPath) {
    markerPath = *targetPath;
    std::string printMsg = LOG_START;
    printMsg = printMsg + " Received new path to follow: \n";
    for (int i = 0; i < markerPath.poses.size(); i++) {
    	printMsg = printMsg + "\t(" + std::to_string(markerPath.poses[i].pose.position.x) + ", " + std::to_string(markerPath.poses[i].pose.position.y) + ")\n";
    }
    STATUS_INFO(crosbotStatus, "%s", printMsg.c_str());
}

void BasicExplorerClient::callback_receivedCommand(const crosbot_msgs::ControlCommandPtr command) {
    if (command->command == crosbot_msgs::ControlCommand::CMD_EM_STOP) {
        setExploreMode(rsa17::ExploreMode::MODE_STOP);
    } else if (command->command == crosbot_msgs::ControlCommand::CMD_RESET) {
        command_reset(command);
    } else if (command->command == crosbot_msgs::ControlCommand::CMD_RESUME) {
        setExploreMode(rsa17::ExploreMode::MODE_RESUME);
    } else if (command->command == crosbot_msgs::ControlCommand::CMD_START) {
        setExploreMode(rsa17::ExploreMode::MODE_RESUME);
    } else if (command->command == crosbot_msgs::ControlCommand::CMD_STOP) {
        setExploreMode(rsa17::ExploreMode::MODE_STOP);
    } else if (command->command == CROSBOT_EXPLORE_COMMAND_LEFTWALL_FOLLOW) {
        setExploreMode(rsa17::ExploreMode::MODE_LEFT_WALL);
    } else if (command->command == CROSBOT_EXPLORE_COMMAND_RIGHTWALL_FOLLOW) {
        setExploreMode(rsa17::ExploreMode::MODE_RIGHT_WALL);
    } else if (command->command == CROSBOT_EXPLORE_COMMAND_GO_TO_ORIGIN) {
        STATUS_INFO(crosbotStatus, "%s Travelling to origin", LOG_START);
        setAStarPose(crosbot::Pose3D(), false);
    } else if (command->command == CROSBOT_EXPLORE_COMMAND_FOLLOW_PATH) {
    	if (goalIndex < markerPath.poses.size() && markerPath.poses.size() > 0) {
    		STATUS_INFO(crosbotStatus, "%s Travelling to goal #%d (%f, %f)", LOG_START, goalIndex, markerPath.poses[goalIndex].pose.position.x, markerPath.poses[goalIndex].pose.position.y);
    		setAStarPose(markerPath.poses[goalIndex], false);
    	} else {
    		STATUS_INFO(crosbotStatus, "%s Already at destination", LOG_START);
    	}
    } else if (command->command == CROSBOT_EXPLORE_COMMAND_CLEAR_PATH) {
        STATUS_INFO(crosbotStatus, "%s Clearing path", LOG_START);
        goalIndex = 0;
    }
}

void BasicExplorerClient::callback_explorerFeedback(const rsa17::ExplorerFeedbackConstPtr& feedback) {
    std::string searchStrategy = "";
    if (feedback->searchStrategy == rsa17::ExploreMode::MODE_LEFT_WALL) {
        searchStrategy = "left wall";
    } else if (feedback->searchStrategy == rsa17::ExploreMode::MODE_RIGHT_WALL) {
        searchStrategy = "right wall";
    } else if (feedback->searchStrategy == rsa17::MoveMode::MODE_ASTAR) {
        searchStrategy = "astar";
    } else if (feedback->searchStrategy == rsa17::ExploreMode::MODE_WAYPOINT) {
        searchStrategy = "waypoint";
    }

    rsa17::ExplorerStatus::Status status = rsa17::ExplorerStatus::statusFromInt(feedback->status);
    STATUS_INFO(crosbotStatus, "%s Explorer feedback: %s (%s)", LOG_START, ExplorerStatus::statusToString(status).c_str(), searchStrategy.c_str());
    if (status == rsa17::ExplorerStatus::Status::STATUS_ARRIVED) {
    	goalIndex++;
    	if (goalIndex < markerPath.poses.size() && markerPath.poses.size() > 0) {
    		STATUS_INFO(crosbotStatus, "%s Travelling to next goal #%d (%f, %f)", LOG_START, goalIndex, markerPath.poses[goalIndex].pose.position.x, markerPath.poses[goalIndex].pose.position.y);
    		setAStarPose(markerPath.poses[goalIndex], false);
    	} else {
    		STATUS_INFO(crosbotStatus, "%s Reached final goal", LOG_START);
    	}
    }
}

crosbot::Pose3D BasicExplorerClient::getCommandPoint(const crosbot_msgs::ControlCommandPtr command) {
    crosbot::Pose3D toPoint(INFINITY, INFINITY, INFINITY);

    if (command->args_doubles.size() >= 3) {
        toPoint.position.x = command->args_doubles[0];
        toPoint.position.y = command->args_doubles[1];
        toPoint.position.z = command->args_doubles[2];
    }

    return toPoint;
}

crosbot::Pose3D BasicExplorerClient::getCommandPose(const crosbot_msgs::ControlCommandPtr command) {
    crosbot::Pose3D toPose = getCommandPoint(command);

    if (command->args_doubles.size() >= 7) {
        toPose.orientation.x = command->args_doubles[3];
        toPose.orientation.y = command->args_doubles[4];
        toPose.orientation.z = command->args_doubles[5];
        toPose.orientation.w = command->args_doubles[6];
    }

    return toPose;
}

void BasicExplorerClient::setExploreMode(int mode) {
    // Ensure action server has started
    bool hasServer = explorer_srv.waitForExistence(ros::Duration(DEFAULT_SERVER_WAIT));
    if (hasServer) {
        STATUS_INFO(crosbotStatus, "%s sending explore mode: %d", LOG_START, mode);

        rsa17::SetExplorerModeRequest  request;
        rsa17::SetExplorerModeResponse response;
        request.id = 0;
        request.mode = mode;
        explorer_srv.call(request, response);
    } else {
        STATUS_ERROR(crosbotStatus, "%s Explorer Action Server not available", LOG_START);
    }
}

// TODO: use pose
void BasicExplorerClient::setAStarPose(crosbot::Pose3D targetPose, bool targetOrientation) {
    ros::Time tNow = ros::Time::now();

    // Get A* path - points must be in the frame of reference of the local or world map
    geometry_msgs::PointStamped fromPoint;
    geometry_msgs::PointStamped toPoint;
    fromPoint.point = crosbot::Point3D().toROS();
    toPoint.point = crosbot::Point3D().toROS();

    // Path
    crosbot::Pose3D toPose(crosbot::Point3D(toPoint.point), crosbot::Quaternion());
    geometry_msgs::Pose geomToPose = toPose.toROS();
    geometry_msgs::PoseStamped stamptedToPose;
    stamptedToPose.pose = geomToPose;
    stamptedToPose.header.stamp = tNow;
    stamptedToPose.header.frame_id = world_frame;
    setAStarPose(stamptedToPose, targetOrientation);
}

void BasicExplorerClient::setAStarPose(const geometry_msgs::PoseStamped& targetPose, bool targetOrientation) {
    // Send new action
    bool hasServer = explorer_srv.waitForExistence(ros::Duration(DEFAULT_SERVER_WAIT));
    if (hasServer) {
        int mode = rsa17::MoveMode::MODE_ASTAR;
        STATUS_INFO(crosbotStatus, "%s A* to geom pose: %s", LOG_START, crosbot::Pose3D(targetPose).position.toString().c_str());

        // Set goal
        rsa17::SetExplorerModeRequest  request;
        rsa17::SetExplorerModeResponse response;
        request.id = 0;
        request.mode = mode;
        request.targetOrientation = targetOrientation;

        // Path
        request.path.header.frame_id = targetPose.header.frame_id;
        request.path.header.stamp = targetPose.header.stamp;
        request.path.poses.push_back(targetPose);

        // Send action
        explorer_srv.call(request, response);
    } else {
        STATUS_ERROR(crosbotStatus, "%s Explorer Action Server not available", LOG_START);
    }
}

void BasicExplorerClient::command_reset(const crosbot_msgs::ControlCommandPtr command) {
    std_msgs::String msg;
    msg.data = CONTROL_COMMAND_RESET_MAP;
    pubPostrackReset.publish(msg);
    pubGraphSlamReset.publish(msg);
}

} // namespace rsa17


