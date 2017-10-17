/*
 * control_node.cpp
 *
 *  Created on: 22/07/2016
 *      Author: Timothy Wiley
 */

#include <ros/ros.h>

#include <timothyw_emu/controlWrapper.hpp>

#include <cstdio>
#include <cstdlib>

#define LOG_START    "TurtlebotNode ::"

namespace timothyw_emu {

int main_timothywControlWrapper(int argc, char** argv) {
    ros::init(argc, argv, "timothyw_controlwrapper");

    // Node Handle - Use '~' when loading config parameters
    // Use without '~' when publish/subscribe/service
    ros::NodeHandle nh("~");

    // Crosbot control wrapper
    TBControlWrapperPtr controlWrapper = new TBControlWrapper();
    controlWrapper->configure();
    controlWrapper->startup();

    while (ros::ok()) {
        ROS_INFO("%s Spinning", LOG_START);

        ros::spin();
    }

    // Disconnect
    controlWrapper->shutdown();

    return 0;
}

} // namespace timothyw_turtlebot

// Actual main method outside of namespace
int main(int argc, char** argv) {
    timothyw_emu::main_timothywControlWrapper(argc, argv);
}

