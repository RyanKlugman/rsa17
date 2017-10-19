/*
 * actionParameters.cpp
 *
 *  Created on: 18/10/2016
 *      Author: rescue
 */

#include <rsa17/basicClient/actionParameters.hpp>

namespace rsa17 {

rsa17::ExplorerStatus::Status ExplorerStatus::statusFromInt(int status) {
    return static_cast<rsa17::ExplorerStatus::Status>(status);
}

std::string ExplorerStatus::statusToString(rsa17::ExplorerStatus::Status status) {
    std::string statusStr = "";
    if (status == rsa17::ExplorerStatus::Status::STATUS_ARRIVED) {
        statusStr = "Arrived";
    } else if (status == rsa17::ExplorerStatus::Status::STATUS_BLOCKED) {
        statusStr = "Blocked";
    } else if (status == rsa17::ExplorerStatus::Status::STATUS_PAUSED) {
        statusStr = "Paused";
    } else if (status == rsa17::ExplorerStatus::Status::STATUS_RUNNING) {
        statusStr = "Running";
    }
    return statusStr;
}

} // namespace rsa17
