/*
 * common.hpp
 *
 *  Created on: 05/07/2016
 *      Author: rescue
 */

#ifndef TIMOTHYW_EMU_COMMON_HPP_
#define TIMOTHYW_EMU_COMMON_HPP_

#define STATUS_ERROR(STPUB, FMT, ...)                                                        \
        STPUB->sendStatus(crosbot_msgs::ControlStatus::LEVEL_ERROR, FMT, __VA_ARGS__);    \
        ROS_ERROR(FMT, __VA_ARGS__);

#define STATUS_INFO(STPUB, FMT, ...)                                                        \
        STPUB->sendStatus(crosbot_msgs::ControlStatus::LEVEL_INFO, FMT, __VA_ARGS__);       \
        ROS_INFO(FMT, __VA_ARGS__);

#define STATUS_WARN(STPUB, FMT, ...)                                                        \
        STPUB->sendStatus(crosbot_msgs::ControlStatus::LEVEL_WARNING, FMT, __VA_ARGS__);    \
        ROS_WARN(FMT, __VA_ARGS__);

#define TURTLEBOT_CONTROL_NAMESPACE             "turtlebot"

#define TURTLEBOT_COMMAND_GO_TO_POINT           "turtlebot_command_go_to_point"
#define TURTLEBOT_COMMAND_LEFTWALL_FOLLOW       "turtlebot_command_leftwall_follow"
#define TURTLEBOT_COMMAND_RIGHTWALL_FOLLOW      "turtlebot_command_rightwall_follow"

#endif /* TIMOTHYW_EMU_COMMON_HPP_ */
