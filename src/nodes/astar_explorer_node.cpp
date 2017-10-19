/*
 * move.cpp
 *
 *  Created on: 27/08/2014
 *      Author: mmcgill
 */

#include <ros/ros.h>

#include <crosbot/config.hpp>
#include <rsa17/nodes/astarExplorerNode.hpp>

#define LOG_START                   "AStarExplorerROSNode ::"
#define LOG_START_PLANNING          "AStarExplorerROSNodePlanningThread ::"

namespace crosbot {

PlanningThreadMove::PlanningThreadMove(AStarExplorerROSNode *moveNode) :
    crosbot::Thread("AStarExplorerROSNodePlanningThread"),
    crosbot::HandledObject(),
    moveNode(moveNode)
{
    if (moveNode == NULL){
        ROS_ERROR("%s Null move node", LOG_START_PLANNING);
    }
}

PlanningThreadMove::~PlanningThreadMove() {
}

void PlanningThreadMove::run() {
    if (moveNode != NULL) {
        moveNode->planningLoop();
    }
}

AStarExplorerROSNode::AStarExplorerROSNode() :
    planningOperating(false),
    rate_astarReplan(1),
    frequency_voronoiUpdate(1.0),
    aStarReconfig_server(NULL),
    differentPlanningGrid(false),
    pointDistance(1.0),
    lastAstarVoronoiUpdate(0)
{
}

AStarExplorerROSNode::~AStarExplorerROSNode() {
}

void AStarExplorerROSNode::configure() {
    ExplorerROSNode::configure();

    ROS_INFO("%s configuring", LOG_START);
    ros::NodeHandle paramNH("~");

    // Allow different voronoi constraints for a* and other methods
    ConfigElementPtr cfg = new ROSConfigElement(paramNH);
    ConfigElementPtr astarCfg = cfg->getChild("astar");
    ConfigElementPtr voronoiCfg = astarCfg->getChild("voronoi");
    if (voronoiCfg != NULL) {
        astarVoronoiConstraints.restricted = voronoiCfg->getParamAsDouble("restrict", astarVoronoiConstraints.restricted);
        astarVoronoiConstraints.partial = voronoiCfg->getParamAsDouble("partial", astarVoronoiConstraints.partial);
        astarVoronoiConstraints.expand = voronoiCfg->getParamAsDouble("expand", astarVoronoiConstraints.expand);
        astarVoronoiConstraints.orphanThreshold = voronoiCfg->getParamAsInt("orphan", astarVoronoiConstraints.orphanThreshold);
        astarVoronoiConstraints.windowSize = voronoiCfg->getParamAsDouble("windowSize", astarVoronoiConstraints.windowSize);
        ROS_INFO("%s restrict: %.2lf - partial: %.2lf - expand: %.2lf - orphan: %d", LOG_START, astarVoronoiConstraints.restricted, astarVoronoiConstraints.partial, astarVoronoiConstraints.expand, astarVoronoiConstraints.orphanThreshold);
        voronoiConstraints = astarVoronoiConstraints;
    }

    // Configure A* Planner
    ConfigElementPtr astarPlannerCfg = astarCfg->getChild("planner");
    astarGridSub_name = astarCfg->getParam("map_sub", gridSub_name);
    astarImgPub_name = astarCfg->getParam("astar_image_pub", "/rsa17/astar_voronoi_image");
    frequency_voronoiUpdate = astarCfg->getParamAsDouble("frequency_voronoiUpdate", 5.0);
    pointDistance = astarCfg->getParamAsDouble("pointDistance", 1.0);
    rate_astarReplan = astarCfg->getParamAsDouble("rate_astarReplan", 1);

    differentPlanningGrid = astarGridSub_name != gridSub_name;
    AStarParameters astarParams = planner.getParameters();
    astarParams.k = astarCfg->getParamAsDouble("risk_k", astarParams.k);
    astarParams.weight_expanded = astarCfg->getParamAsDouble("weight_expanded", astarParams.weight_expanded);
    astarParams.weight_partial = astarCfg->getParamAsDouble("weight_partial", astarParams.weight_partial);
    planner.setParamaters(astarParams);

    // Dynamic reconfiguration
    ros::NodeHandle reconfigNh("~/astar");
    aStarReconfig_server = new AStarReconfigServer(aStarMutexReconfig, reconfigNh);
    publishReconfig(astarParams);
    aStarReconfig_callbackType = boost::bind(&AStarExplorerROSNode::callback_astarReconfigure, this, _1, _2);
    aStarReconfig_server->setCallback(aStarReconfig_callbackType);

    // Publishers/Subscribers
    ros::NodeHandle nh;
    astarImagePub = nh.advertise<sensor_msgs::Image>(astarImgPub_name, 1);
    if (differentPlanningGrid) {
        astarGridSub = nh.subscribe(astarGridSub_name, 1, &AStarExplorerROSNode::callback_astarOccGrid, this);
    }

    // Planning Thread
    planningThread = new PlanningThreadMove(this);

    ROS_INFO("%s Configure done", LOG_START);
}

void AStarExplorerROSNode::startup() {
    ROS_INFO("%s astar_explorer_node starting up", LOG_START);
    ExplorerROSNode::startup();
}

void AStarExplorerROSNode::shutdown() {
    ExplorerROSNode::shutdown();

    // Close publishers/subscribers
    astarGridSub.shutdown();

    // Clear data structures
    astarWaypoints.clear();
    lastAstarVoronoiUpdate = ros::Time(0);

    // Ensure planning thread has stopped
    planningOperating = false;
    for (uint32_t i = 0; i < DEFAULT_WAIT_FOR_THREAD_CLOSE && planningThread->isAlive(); i + 100) {
        usleep(100000);
    }
    if (planningThread->isAlive()) {
        ROS_ERROR("%s Planning thread failed to shut down.", LOG_START);
    }
}

void AStarExplorerROSNode::callback_astarOccGrid(const nav_msgs::OccupancyGridConstPtr& astarLatestMap) {
    Lock lock(astarRosLock, true);
    this->astarLatestMap = astarLatestMap;
}

void AStarExplorerROSNode::callback_astarReconfigure(rsa17::CrosbotAStarReconfigConfig &config, uint32_t level) {
    astarVoronoiConstraints.restricted = config.astar_voronoi_restricted;
    astarVoronoiConstraints.partial = config.astar_voronoi_partial;
    astarVoronoiConstraints.expand = config.astar_voronoi_expand;
    astarVoronoiConstraints.orphanThreshold = config.astar_voronoi_orphanThreshold;
    ROS_INFO("%s new dynamic config: \t restrict: %.2lf, partial %.2lf, expand: %.2lf", LOG_START, astarVoronoiConstraints.restricted, astarVoronoiConstraints.partial, astarVoronoiConstraints.expand);

    AStarParameters astarParams = planner.getParameters();
    astarParams.weight_expanded = config.weight_expanded;
    astarParams.weight_partial = config.weight_partial;
    planner.setParamaters(astarParams);
    ROS_INFO("%s Got A* reconfig. W(expand): %.2lf, W(partial):%.2lf", LOG_START, config.weight_expanded, config.weight_partial);
}

void AStarExplorerROSNode::publishReconfig(AStarParameters& params) {
    // Set config values
    rsa17::CrosbotAStarReconfigConfig config;
    config.astar_voronoi_expand = astarVoronoiConstraints.expand;
    config.astar_voronoi_orphanThreshold = astarVoronoiConstraints.orphanThreshold;
    config.astar_voronoi_partial = astarVoronoiConstraints.partial;
    config.astar_voronoi_restricted = astarVoronoiConstraints.restricted;
    config.weight_expanded = params.weight_expanded;
    config.weight_partial = params.weight_partial;

    // Call service
    aStarReconfig_server->updateConfig(config);
}

void AStarExplorerROSNode::loadLatestVoronoi() {
    // Load local voronoi
    ExplorerROSNode::loadLatestVoronoi();

    // Load A* voronoi
    nav_msgs::OccupancyGridConstPtr latestMap;
    {
        Lock lock(astarRosLock, false);
        latestMap = this->astarLatestMap;
    }

    if (latestMap == NULL) {
        // No voronoi
        astarLatestVoronoi = NULL;
    } else if (astarLatestVoronoi != NULL && astarLatestVoronoi->timestamp == Time(latestMap->header.stamp)) {
        // Don't update latest voronoi
    } else {
        // Update new voronoi
        if (debugMsgs) {
            ROS_INFO("%s Loading new A* voronoi grid: restrict: %.1lf, partial: %.1lf, expand: %.1lf", LOG_START, astarVoronoiConstraints.restricted, astarVoronoiConstraints.partial, astarVoronoiConstraints.expand);
        }
        Pose robot = getLatestPose();
        astarLatestVoronoi = new VoronoiGrid(*latestMap, astarVoronoiConstraints, robot);
    }
}

VoronoiGridPtr AStarExplorerROSNode::getLatestAstarVoronoi() {
    VoronoiGridPtr voronoi = NULL;

    if (differentPlanningGrid) {
        voronoi = astarLatestVoronoi;
    } else {
        voronoi = getLatestVoronoi();
    }

    return voronoi;
}

void AStarExplorerROSNode::handleExploreMode(rsa17::SetExplorerModeRequest &request,
        rsa17::SetExplorerModeResponse &response, bool& enableDrive)  {
    if (request.mode == rsa17::MoveMode::MODE_ASTAR) {
        ROS_INFO("%s A* mode activated", LOG_START);
        enableDrive = false;

        // A* goal is first element of path
        if (request.path.poses.size() > 0) {
            searchParams.resetWallFollowData();
            searchParams.resetWaypointData();
            searchParams.strategy = SearchParameters::Waypoint;
            searchParams.targetOrientation = request.targetOrientation;
            voronoiConstraints = astarVoronoiConstraints;

            // TODO: go through path of goals - so you can have sequence of A* goals
            Pose3DStamped astarGoal(request.path.poses.front());
            setCurrentAStarGoal(astarGoal);

            // Set first waypoints
            VoronoiGridPtr voronoi;
            bool setResult = updateWaypoints();
            response.success = setResult;

            // Start planning thread
            planningOperating = true;
            planningThread->start();
        } else {
            ROS_ERROR("%s No pose provided for A* goal", LOG_START);
            response.success = false;
        }

    } else {
        ROS_INFO("%s Non-A* mode activated", LOG_START);
        ExplorerROSNode::handleExploreMode(request, response, enableDrive);

        // If mode has changed, stop planning thread
        if (!(request.mode == rsa17::MoveMode::MODE_RESUME || request.mode == rsa17::MoveMode::MODE_STOP)) {
            planningOperating = false;
        }
    }
}

void AStarExplorerROSNode::planningLoop() {
    ros::Rate rate(rate_astarReplan);

    VoronoiGridPtr voronoi;

    while (planningOperating) {
        if (!paused) {
            if (searchParams.atDestination) {
                // Check if at goal, pause exploration
                pause();
            } else {
                // Otherwise, update waypoints and continue driving
                updateWaypoints();
            }
        }

        // Sleep even if update failed to allow pause to go through first
        rate.sleep();
    }
}

bool AStarExplorerROSNode::setWaypoints(Plan& plan, const std::string& planFrameId, const std::string& navPathFrameId, const crosbot::Time time) {
    bool retVal = false;

    // Establish converted plan
    bool converted = false;
    std::string convertedFrameId;
    Plan emptyPlan;
    Plan revisedPlan;
    Plan& convertedPlan = emptyPlan;
    if (differentPlanningGrid) {
        // Convert plan points into frame of reference of navigation voronoi grid
        try {
            // Ensure transform
            //ROS_INFO("%s transforming from frame: %s to frame: %s", LOG_START, planFrameId.c_str(), navPathFrameId.c_str());
            tfListener.waitForTransform(navPathFrameId, planFrameId, time.toROS(), ros::Duration(0.5));
            //ROS_INFO("%s \t got transform", LOG_START);

            // Generate new path
            for (Pose& pose : plan) {
                geometry_msgs::PoseStamped oldPose;
                oldPose.header.frame_id = planFrameId;
                oldPose.header.stamp = time.toROS();
                oldPose.pose = pose.toROS();
                geometry_msgs::PoseStamped newPose;
                newPose.header.frame_id = navPathFrameId;
                tfListener.transformPose(navPathFrameId, oldPose, newPose);
                revisedPlan.push_back(crosbot::Pose(newPose));
            }

            convertedPlan = revisedPlan;
            convertedFrameId = navPathFrameId;
            converted = true;
        } catch (tf::TransformException& e) {
            ROS_WARN("%s setWaypoints(): Exception caught(%s).\n", LOG_START, e.what());
            converted = false;
        }
    } else {
        convertedPlan = plan;
        convertedFrameId = planFrameId;
        converted = true;
    }

    // Generate navPath
    if (converted) {
        ROS_INFO("%s Sending path in frame: %s", LOG_START, convertedFrameId.c_str());
        nav_msgs::Path navPath;
        navPath.header.frame_id = convertedFrameId;
        navPath.header.stamp = time.toROS();
        navPath.poses.resize(convertedPlan.size());
        for (size_t i = 0; i < convertedPlan.size(); ++i) {
            navPath.poses[i].pose = convertedPlan[i].toROS();
        }
        retVal = ExplorerROSNode::setWaypoints(navPath);
    } else {
        ROS_WARN("%s failed to convert path to navigation frame", LOG_START);
    }

    return retVal;
}

void AStarExplorerROSNode::setCurrentAStarGoal(const Pose3DStamped& goal) {
    Lock lock(dataLock, true);
    astarWaypoints.clear();
    currentAStarGoal = goal;
    ROS_INFO("%s Setting current A* Goal at: %s in frame %s", LOG_START, currentAStarGoal.toString().c_str(), goal.frame_id.c_str());
}

Pose3DStamped AStarExplorerROSNode::getCurrentAStarGoal(std::string frame_id, Time timestamp) {
    // Does not take lock as calling code already has lock

    Pose3DStamped aStarGoal(INFINITY, INFINITY, INFINITY, frame_id, Time());
    if (currentAStarGoal.isFinite()) {
        tf::StampedTransform transform;
        try {
            // Change time of goal to now
            geometry_msgs::PoseStamped goal = currentAStarGoal.toROS();
            goal.header.stamp = timestamp.toROS();

            geometry_msgs::PoseStamped aStarOut;
            tfListener.waitForTransform(frame_id, goal.header.frame_id, timestamp.toROS(), ros::Duration(0.5));
            tfListener.transformPose(frame_id, goal, aStarOut);
            aStarGoal = aStarOut;
        } catch (tf::TransformException& e) {
            ROS_WARN("%s getCurrentAStarGoal(): %s -> %s Exception caught(%s).\n", LOG_START, frame_id.c_str(), currentAStarGoal.frame_id.c_str(), e.what());
        }
    }

    return aStarGoal;
}

bool AStarExplorerROSNode::updateWaypoints() {
    if (debugMsgs) ROS_INFO("%s Updating Waypoints", LOG_START);

    // Revise entire A* Path
    ros::Time rosTNow(ros::Time::now());
    if (astarWaypoints.size() == 0 ||
        rosTNow - lastAstarVoronoiUpdate > ros::Duration(frequency_voronoiUpdate)) {
        if (debugMsgs) ROS_INFO("%s Updating entire A* path", LOG_START);
        calculateAStarPath();
    }

    // Revise waypoints to next A* goal
    // Load additional data
    Pose robot(INFINITY, INFINITY, INFINITY);
    VoronoiGridPtr localVoronoi = NULL;
    Time tNow(ros::Time::now());
    bool loaded = true;
    {{
        Lock lock(dataLock);
        std::string error;

        // Load latest data
        loadLatestPose();
        robot = getLatestPose();

        localVoronoi = getLatestVoronoi();
        if (localVoronoi == NULL) {
            loaded = false;
            ROS_WARN("%s Cannot update waypoints: %s", LOG_START, "Voronoi grid(s) not available");
        } else if (!robot.isFinite()) {
            loaded = false;
            ROS_WARN("%s Unable to determine current pose", LOG_START);
        }
    }}

    // Get A* voronoi grid image to load debug images
    ImagePtr debugImage = NULL;
    if (astarImagePub.getNumSubscribers() > 0) {
        debugImage = localVoronoi->getImage();
    }

    // Only loaded if there are waypoints calculated - voronoi could be loaded in between high-level calculation
    if (astarWaypoints.size() == 0) {
        loaded = false;
    }

    bool waypointsSet = false;
    if (loaded) {
        // Iterate through A* waypoints - removing any that are too close - but keep goal
        while (astarWaypoints.size() > 1 &&
               astarWaypoints.front().position.distanceTo(robot.position) < pointDistance) {
            astarWaypoints.pop_front();
        }

        Pose goal = astarWaypoints.front();

        // Plan to set
        planner.astarParameters.findTraversibleGoal = false;
        Plan plan = planner.getPath(localVoronoi, robot, goal, debugImage);
        if (debugMsgs) {
            ROS_INFO("%s Loaded path with %d waypoints to %s", LOG_START, (int) plan.size(), goal.position.toString().c_str());
//            int i = 0;
//            for (auto p : plan) {
//                ROS_INFO("%s \t %d to %s", LOG_START, i, p.position.toString().c_str());
//                ++i;
//            }
        }

        // Append remaining waypoints to path (to stop reaching goal)
        for (int i = 1; i < astarWaypoints.size(); ++i) {
            plan.push_back(astarWaypoints[i]);
        }

        // Send path to explorer
        waypointsSet = setWaypoints(plan, localVoronoi->frame, localVoronoi->frame, tNow);
        if (plan.size() == 0) {
            ROS_WARN("%s low-level plan is empty to %s", LOG_START, goal.position.toString().c_str());
        }
    }

    // Publish A* voronoi grid
    if (astarImagePub.getNumSubscribers() > 0 && debugImage != NULL) {
        astarImagePub.publish(debugImage->toROS());
    }

    return waypointsSet;
}

void AStarExplorerROSNode::calculateAStarPath() {
    Pose goal(INFINITY, INFINITY, INFINITY);
    Pose robot(INFINITY, INFINITY, INFINITY);
    VoronoiGridPtr astarVoronoi = NULL;

    // Clear existing path
    astarWaypoints.clear();

    // Load time
    Time tNow(ros::Time::now());

    bool loaded = true;
    {{
        Lock lock(dataLock);
        std::string error;

        // Load latest data
        loadLatestPose();
        robot = getLatestPose();

        astarVoronoi = getLatestAstarVoronoi();

        // Error checking
        if (astarVoronoi == NULL) {
            loaded = false;
            ROS_WARN("%s Cannot update waypoints: %s", LOG_START, "Voronoi grid(s) not available");
        } else if (!robot.isFinite()) {
            loaded = false;
            ROS_WARN("%s Unable to determine current pose", LOG_START);
        }

        // Get current A* goal
        if (loaded) {
            goal = getCurrentAStarGoal(astarVoronoi->frame, tNow);
            if (!goal.isFinite()) {
                loaded = false;
                ROS_WARN("%s Goal is infinite", LOG_START);
            }
        }
    }}
    if (debugMsgs) ROS_INFO("%s High-level - Updating Waypoints: loaded data: %d", LOG_START, loaded);

    if (loaded) {
        // TODO: maybe publish grid with full plan as well
        // Get A* voronoi grid image to load debug images
        //ImagePtr debugImage;
        //if (astarImagePub.getNumSubscribers() > 0) {
        //    debugImage = astarVoronoi->getImage();
        //}
        ImagePtr debugImage = NULL;

        // Plan path to goal
        planner.astarParameters.findTraversibleGoal = true;
        Plan plan = planner.getPath(astarVoronoi, robot, goal, debugImage);
        ROS_INFO("%s Loaded high-level path with %d waypoints to %s", LOG_START, (int) plan.size(), goal.position.toString().c_str());

        if (plan.size() == 0) {
            ROS_WARN("%s High-level A* plan is empty to %s", LOG_START, goal.position.toString().c_str());
        } else {
            Pose end = robot;
            for (Pose& pose : plan) {
                if (pose.position.distanceTo(end.position) > pointDistance) {
                    astarWaypoints.push_back(pose);
                    end = pose;
                }
            }

            // Ensure goal is added
            if (astarWaypoints.size() == 0 || astarWaypoints.back() != plan.back()) {
                astarWaypoints.push_back(plan.back());
            }
        }

        if (debugMsgs) ROS_INFO("%s high-level plan loaded", LOG_START);

        // Publish A* voronoi grid
        //if (astarImagePub.getNumSubscribers() > 0 && debugImage != NULL) {
        //    astarImagePub.publish(debugImage->toROS());
        //}
    }
}

} // namespace crosbot


int main(int argc, char** argv) {
    ros::init(argc, argv, "move");

    crosbot::AStarExplorerROSNode node;
    node.configure();
    node.startup();

    while (ros::ok()) {
        ros::spin();
    }
    node.shutdown();

    return 0;
}
