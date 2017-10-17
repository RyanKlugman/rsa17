/*
 * controlWrapper.hpp
 *
 *  Created on: 04/07/2016
 *      Author: rescue
 */

#ifndef TIMOTHYW_EMU_CONTROLWRAPPER_HPP_
#define TIMOTHYW_EMU_CONTROLWRAPPER_HPP_

#include <crosbot/geometry/points.hpp>
#include <crosbot/handle.hpp>
#include <crosbot/controls/command.hpp>
#include <crosbot/controls/status.hpp>

#include <ros/publisher.h>
#include <ros/service.h>
#include <tf/transform_listener.h>

namespace timothyw_emu {

class TBControlWrapper : public crosbot::HandledObject {
public:
    TBControlWrapper();
    virtual ~TBControlWrapper();

    void configure();
    void startup();
    void shutdown();

    // Command callbacks
    void callback_receivedCommand(const crosbot_msgs::ControlCommandPtr command);

private:
    // Configuration
    std::string aStarSrvName;
    std::string exploreModeSrvName;
    std::string explorePathSrvName;
    std::string robot_frame;
    std::string world_frame;

    // Publishers/subscribers/services
    crosbot::CrosbotCommandPtr crosbotCommand;
    crosbot::CrosbotStatusPtr crosbotStatus;
    ros::ServiceClient aStarSrv;
    ros::ServiceClient exploreModeSrv;
    ros::ServiceClient explorePathSrv;
    ros::Publisher pubPostrackReset;
    ros::Publisher pubGraphSlamReset;
    tf::TransformListener tfListener;

    /**
     * Set mode of crosbot_explore
     */
    void setExploreMode(int mode);

    void command_reset(const crosbot_msgs::ControlCommandPtr command);
};
typedef crosbot::Handle<TBControlWrapper> TBControlWrapperPtr;

} // namespace timothyw_turtlebot

#endif /* TIMOTHYW_EMU_CONTROLWRAPPER_HPP_ */
