/*
 *
 *  Created on: 18/10/2016
 *      Author: rescue
 */

#ifndef CROSBOT_EXPLORE_ACTION_PARAMETERS_HPP_
#define CROSBOT_EXPLORE_ACTION_PARAMETERS_HPP_

#include <string>

namespace rsa17 {

/**
 * Exploration modes for crosbot explorer - placed in header files to allow extensible modes
 * which cannot be achieved through the explore.action class
 */
class ExploreMode {
public:
    enum {
        MODE_STOP       = 0,
        MODE_RESUME     = 1,
        MODE_LEFT_WALL  = 2,
        MODE_RIGHT_WALL = 3,
        MODE_WAYPOINT   = 4
    };
};

/**
 * Common exploration status feedback for crosbot explorer - placed in header files to allow extensible status messages
 * which cannot be achieved through the explore.action class
 */
class ExplorerStatus {
public:
    enum Status {
        STATUS_PAUSED   = 0,
        STATUS_RUNNING  = 1,
        STATUS_ARRIVED  = 2,
        STATUS_BLOCKED  = 3
    };

    static rsa17::ExplorerStatus::Status statusFromInt(int status);
    static std::string statusToString(rsa17::ExplorerStatus::Status status);
};

} // namespace rsa17


#endif /* CROSBOT_EXPLORE_ACTION_PARAMETERS_HPP_ */
