/*
 * hystersisData.hpp
 *
 *  Created on: 05/03/2017
 *      Author: rescue
 */

#ifndef CROSBOT_EXPLORE_HYSTERSIS_PARAMETERS_HPP_
#define CROSBOT_EXPLORE_HYSTERSIS_PARAMETERS_HPP_

#include <cmath>
#include <vector>

#include <crosbot/data/time.hpp>
#include <crosbot/geometry/defines.hpp>

namespace crosbot {

struct HysteresisParameters {
    Duration clusterTimeout;        // TTL of poses in clusters
    bool     enabled;               // Is hysteresis enabled?
    int      hysteresisSize;        // maximum size of the hysteresis
    Duration hysteresisTimeout;     // TTL of items in the hysteresis (typically shorter than TTL of clusters)
    double   marginDistance;        // Distance difference of target relative to a cluster centre
    double   marginHeading;         // Heading difference of target relative to a cluster centre

    HysteresisParameters() :
        clusterTimeout(2.0),
        enabled(true),
        hysteresisSize(5),
        hysteresisTimeout(0.5),
        marginDistance(0.2),
        marginHeading(DEG2RAD(25))
    {};
};

} // namespace crosbot

#endif /* CROSBOT_EXPLORE_HYSTERSIS_PARAMETERS_HPP_ */
