/*
 * astarParameters.hpp
 *
 *  Created on: 06/03/2017
 *      Author: rescue
 */

#ifndef CROSBOT_EXPLORE_ASTAR_PARAMETERS_HPP_
#define CROSBOT_EXPLORE_ASTAR_PARAMETERS_HPP_

#include <cmath>

namespace crosbot {

struct AStarParameters {
public:

    double k;

    double weight_expanded;
    double weight_partial;
    double weight_restricted;
    double weight_skeleton;

    bool findTraversibleGoal;

    AStarParameters() :
        k(25),
        weight_expanded(1.0),
        weight_partial(20.0),
        weight_restricted(INFINITY),
        weight_skeleton(1.0),
        findTraversibleGoal(false)
    {};

    AStarParameters(const AStarParameters& other) :
        k(other.k),
        weight_expanded(other.weight_expanded),
        weight_partial(other.weight_partial),
        weight_restricted(other.weight_restricted),
        weight_skeleton(other.weight_skeleton),
        findTraversibleGoal(other.findTraversibleGoal)
    {};
};

} // namespace crosbot


#endif /* CROSBOT_EXPLORE_ASTAR_PARAMETERS_HPP_ */
