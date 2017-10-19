/*
 * astar.cpp
 *
 *  Created on: 22/02/2013
 *      Author: mmcgill
 */

#include <ros/ros.h>

#include <rsa17/explorerROS.hpp>


int main(int argc, char** argv) {
    ros::init(argc, argv, "rsa17r");

    ros::NodeHandle nh;

    crosbot::ExplorerROSNode node;
    node.configure();
    node.startup();

    while (ros::ok()) {
        ros::spin();
    }
    node.shutdown();

    return 0;
}



