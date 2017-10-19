/*
 * driveParameters.hpp
 *
 *  Created on: 11/10/2016
 *      Author: rescue
 */

#ifndef CROSBOT_EXPLORER_DRIVEPARAMETERS_HPP_
#define CROSBOT_EXPLORER_DRIVEPARAMETERS_HPP_

#include <crosbot/thread.hpp>
#include <crosbot/geometry/poses.hpp>

namespace crosbot {

struct DriveParameters {
public:
    double maxVel;
    double maxTurn;
    double minVel;
    double minTurn;
    double turnOnly;

    ReadWriteLock driveLock;
    Pose driveTarget;

    DriveParameters() :
        maxVel(0.5), maxTurn(DEG2RAD(15)),
        minVel(0.1), minTurn(0.1),
        turnOnly(DEG2RAD(30)), driveTarget(INFINITY, INFINITY, INFINITY)
    {}

    inline Pose getDriveTarget() {
        Lock lock(driveLock);
        return driveTarget;
    }
    inline void setDriveTarget(Pose pose) {
        Lock lock(driveLock, true);
        driveTarget = pose;
    }
};

} // namespace crosbot



#endif /* DRIVEPARAMETERS_HPP_ */
