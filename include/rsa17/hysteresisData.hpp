/*
 * hystersisData.hpp
 *
 *  Created on: 05/03/2017
 *      Author: rescue
 */

#ifndef CROSBOT_EXPLORE_HYSTERSIS_DATA_HPP_
#define CROSBOT_EXPLORE_HYSTERSIS_DATA_HPP_

#include <cmath>
#include <deque>
#include <vector>

#include <crosbot/data/time.hpp>

namespace crosbot {

struct HysteresisData {
    static const int NO_CLUSTER = -1;
    static const int NO_INDEX = -1;

    struct TimestampedPose {
        TimestampedPose() :
            pose(INFINITY, INFINITY, INFINITY),
            timestamp()
        {};
        TimestampedPose(const Pose& pose, const Time& timestamp) :
            pose(pose),
            timestamp(timestamp)
        {};

        Pose pose;
        Time timestamp;
    };

    struct Cluster {
        Cluster() :
            centre(INFINITY, INFINITY, INFINITY),
            centreHeading(INFINITY),
            centreDistance(INFINITY)
        {};

        std::deque<TimestampedPose> targets;

        Point3D centre;
        double centreHeading;
        double centreDistance;

        bool isEmpty() {
            return targets.size() == 0;
        }

        void reset() {
            targets.clear();
            centre = Point3D(INFINITY, INFINITY, INFINITY);
            centreHeading = INFINITY;
            centreDistance = INFINITY;
        }
    };

    struct Hysteresis {
        Hysteresis() :
            index(NO_CLUSTER),
            timestamp()
        {};
        Hysteresis(const int index, const Time& timestamp) :
            index(index),
            timestamp(timestamp)
        {};

        int index;
        Time timestamp;
    };

    HysteresisData() :
        index(NO_INDEX)
    {};

    // Clusters to match new pose to
    std::vector<Cluster> clusters;

    // Hysteresis record of selected clusters
    std::deque<Hysteresis> hysteresis;
    int index;

    // Reset full hysteresis
    void reset() {
        clusters.clear();
        hysteresis.clear();
        index = NO_INDEX;
    }
};

} // namespace crosbot

#endif /* CROSBOT_EXPLORE_HYSTERSIS_DATA_HPP_ */
