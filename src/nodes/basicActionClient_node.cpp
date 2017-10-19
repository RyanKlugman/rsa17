/*
 * control_node.cpp
 *
 *  Created on: 22/07/2016
 *      Author: Timothy Wiley
 */

#include <ros/ros.h>

#include <rsa17/basicClient/basicClient.hpp>

#include <cstdio>
#include <cstdlib>

#define LOG_START    "BasicExplorerClientNode ::"

namespace rsa17 {

int main_taskManager(int argc, char** argv) {
    ros::init(argc, argv, "basic_explorer_client");

    // Node Handle - Use '~' when loading config parameters
    // Use without '~' when publish/subscribe/service
    ros::NodeHandle nh("~");

    // Crosbot control wrapper
    BasicExplorerClientPtr controlWrapper = new BasicExplorerClient();
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

} // namespace rsa17

// Actual main method outside of namespace
int main(int argc, char** argv) {
    rsa17::main_taskManager(argc, argv);
}

