/*
 * astar.cpp
 *
 *  Created on: 22/02/2013
 *      Author: mmcgill
 */

#include <ros/ros.h>

#include <rsa17/explorerROS.hpp>

#include <crosbot/config.hpp>
#include <rsa17/ExplorerFeedback.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>


#define DEFAULT_BASEFRAME			    "/base_link"
#define DEFAULT_DRIVE_OPERATING_RATE    100
#define DEFAULT_LOADPOSE_WAIT4TRANSFORM 0.5
#define LOG_START                       "CrosbotExploreROS ::"

namespace crosbot {

ExplorerROSNode::ExplorerROSNode() :
    baseFrame(DEFAULT_BASEFRAME),
    gotReconfig(false),
    reconfig_server(NULL),
    currentGoalId(-1),
    latestPose(Pose(INFINITY, INFINITY, INFINITY))
{
}

ExplorerROSNode::~ExplorerROSNode() {
};

void ExplorerROSNode::callbackOccGrid(const nav_msgs::OccupancyGridConstPtr& latestMap) {
    Lock lock(lock_ros, true);
    if (debugMsgs) ROS_INFO("%s callbackOccGrid()", LOG_START);
    this->latestMap = latestMap;
}

void ExplorerROSNode::callbackHistory(const nav_msgs::PathConstPtr& latestHistory) {
    Lock lock(lock_ros, true);
    if (debugMsgs) ROS_INFO("%s callbackHistory()", LOG_START);
    this->latestHistory = latestHistory;
}

void ExplorerROSNode::callback_reconfigure(rsa17::CrosbotExploreReconfigConfig &config, uint32_t level) {
    // Lock loading of parameters
    Lock planningLock(planningThreadMutex);
	if (debugMsgs) ROS_INFO("%s callback_reconfigure()", LOG_START);
    localVoronoiConstraints.restricted = config.voronoi_restricted;
    localVoronoiConstraints.partial = config.voronoi_partial;
    localVoronoiConstraints.expand = config.voronoi_expand;
    localVoronoiConstraints.orphanThreshold = config.voronoi_orphanThreshold;
    ROS_INFO("%s new dynamic config: \t restrict: %.2lf, partial %.2lf, expand: %.2lf", LOG_START, localVoronoiConstraints.restricted, localVoronoiConstraints.partial, localVoronoiConstraints.expand);

    searchParams.minTravelDistance = config.min_travel_distance;
    searchParams.obstructionAngle = DEG2RAD(config.obstruction_angle);
    searchParams.proximityAngle = DEG2RAD(config.proximity_angle);
    searchParams.proximityDistance = config.proximity_distance;
    searchParams.skeletonSearchDistance = config.skeleton_search_distance;
    searchParams.wallSearchDistance = config.wall_search_distance;
    searchParams.wallSearchAngle = config.wall_search_angle;

    // Change grid sub
    std::string newGridSub_name = config.map_sub;
    if (newGridSub_name != "" && newGridSub_name != gridSub_name) {
        gridSub_name = newGridSub_name;

        ros::NodeHandle nh;
        subscribeGrid(nh);
    }

    driveParams.maxTurn = config.drive_max_turn;
    driveParams.maxVel = config.drive_max_vel;

    currentGoalId = -1;

    gotReconfig = true;
}

bool ExplorerROSNode::callbackExplorerMode(rsa17::SetExplorerModeRequest &request, rsa17::SetExplorerModeResponse &response) {
    ROS_INFO("%s Got exploration service call. Goal: %d", LOG_START, request.mode);
    currentGoalId = request.id;

    // Result of the goal action
    response.id = currentGoalId;

    // Configure and start explorer based on mode
    bool enableDrive = false;

    // Configure base on the goal mode
    handleExploreMode(request, response, enableDrive);

    // Configure drive
    bool driveOperating = false;
    if (enableDrive) {
        ROS_INFO("%s enabling drive", LOG_START);
        driveOperating = true;
        resume();
    } else if (!enableDrive || request.mode == rsa17::ExploreMode::MODE_STOP) {
        pause();
    }

    // Give response
    response.id = currentGoalId;
    response.success = true;

    return true;
}

void ExplorerROSNode::handleExploreMode(rsa17::SetExplorerModeRequest &request,
        rsa17::SetExplorerModeResponse &response, bool& enableDrive) {
    // Locking concurrency controls is done based on the selected mode
		if (debugMsgs) ROS_INFO("%s handleExploreMode()", LOG_START);
    // Load local voronoi constraints
    if (!gotReconfig) {
        ROS_WARN("%s Cannot begin exploration until configuration parameters loaded", LOG_START);
        response.success = false;
    } else if (request.mode == rsa17::ExploreMode::MODE_LEFT_WALL) {
        Lock planningLock(planningThreadMutex);
        Lock drivingLock(drivingThreadMutex);

        searchParams.resetWallFollowData();
        searchParams.resetWaypointData();
        searchParams.side = SearchParameters::Left;
        searchParams.strategy = SearchParameters::WallFollow;

        response.success = true;
        voronoiConstraints = localVoronoiConstraints;
    } else if (request.mode == rsa17::ExploreMode::MODE_RIGHT_WALL) {
        Lock planningLock(planningThreadMutex);
        Lock drivingLock(drivingThreadMutex);

        searchParams.resetWallFollowData();
        searchParams.resetWaypointData();
        searchParams.side = SearchParameters::Right;
        searchParams.strategy = SearchParameters::WallFollow;

        response.success = true;
        voronoiConstraints = localVoronoiConstraints;
    } else if (request.mode == rsa17::ExploreMode::MODE_WAYPOINT) {
        Lock planningLock(planningThreadMutex);
        Lock drivingLock(drivingThreadMutex);


        searchParams.resetWallFollowData();
        searchParams.resetWaypointData();
        searchParams.strategy = SearchParameters::Waypoint;
        searchParams.targetOrientation = request.targetOrientation;

        nav_msgs::Path path = request.path;
        bool setResult = setWaypoints(path);

        response.success = setResult;
        voronoiConstraints = localVoronoiConstraints;
    } else if (request.mode == rsa17::ExploreMode::MODE_RESUME) {
        enableDrive = true;

        response.success = false;
    } else if (request.mode == rsa17::ExploreMode::MODE_STOP) {
        // On pause, just stop
        response.success = true;
    }
}

void ExplorerROSNode::configure() {
    //Lock lock(mutexReconfig);
	ROS_INFO("%s configure()", LOG_START);
    // Configure base class
    Explorer::configure();

    // Read configuration/parameters
    ros::NodeHandle paramNH("~");
    ConfigElementPtr cfg = new ROSConfigElement(paramNH);

    // Explorer class config
    debugMsgs = cfg->getParamAsBool("debug", false);
    searchWhilePaused = cfg->getParamAsBool("searchWhilePaused", true);

    // ROS Config
    baseFrame = cfg->getParam("base_frame", DEFAULT_BASEFRAME);
    explorer_srv_name = cfg->getParam("explorer_srv", "/rsa17/explorer_service");
    explorer_feedback_pub_name = cfg->getParam("explorer_feedback_pub", "/rsa17/explorer_feedback");
    rate_driveThread = cfg->getParamAsDouble("rate_driveThread", 10);
    rate_planThread = cfg->getParamAsDouble("rate_planThread", 2);
    rate_voronoiThread = cfg->getParamAsDouble("rate_voronoiThread", 1);
    gridSub_name = cfg->getParam("map_sub", "map");
    historySub_name = cfg->getParam("history_sub", "history");
    imagePub_name = cfg->getParam("image_pub", "/rsa17/image");
    velPub_name = cfg->getParam("vel_pub", "cmd_vel");
    ConfigElementPtr voronoiCfg = cfg->getChild("voronoi");
    if (voronoiCfg != NULL) {
        localVoronoiConstraints.restricted = voronoiCfg->getParamAsDouble("restrict", localVoronoiConstraints.restricted);
        localVoronoiConstraints.partial = voronoiCfg->getParamAsDouble("partial", localVoronoiConstraints.partial);
        localVoronoiConstraints.expand = voronoiCfg->getParamAsDouble("expand", localVoronoiConstraints.expand);
        localVoronoiConstraints.orphanThreshold = voronoiCfg->getParamAsInt("orphan", localVoronoiConstraints.orphanThreshold);
        localVoronoiConstraints.windowSize = voronoiCfg->getParamAsDouble("windowSize", localVoronoiConstraints.windowSize);
        ROS_INFO("%s restrict: %.2lf - partial: %.2lf - expand: %.2lf - orphan: %d", LOG_START, localVoronoiConstraints.restricted, localVoronoiConstraints.partial, localVoronoiConstraints.expand, localVoronoiConstraints.orphanThreshold);
        voronoiConstraints = localVoronoiConstraints;
    }
    ConfigElementPtr searchCfg = cfg->getChild("search");
    if (searchCfg != NULL) {
        searchParams.minTravelDistance = searchCfg->getParamAsDouble("min_travel_distance", searchParams.minTravelDistance);
        searchParams.obstructionAngle = DEG2RAD(searchCfg->getParamAsDouble("obstruction_angle", RAD2DEG(searchParams.obstructionAngle)));
        searchParams.proximityAngle = DEG2RAD(searchCfg->getParamAsDouble("proximity_angle", RAD2DEG(searchParams.proximityAngle)));
        searchParams.proximityDistance = searchCfg->getParamAsDouble("proximity_distance", searchParams.proximityDistance);
        searchParams.skeletonSearchDistance = searchCfg->getParamAsDouble("skeleton_search_distance", searchParams.skeletonSearchDistance);
        searchParams.wallSearchDistance = searchCfg->getParamAsDouble("wall_search_distance", searchParams.wallSearchDistance);
        searchParams.wallSearchAngle = searchCfg->getParamAsDouble("wall_search_angle", searchParams.wallSearchAngle);
        ROS_INFO("%s min_travel_distance: %.2lf, obstruction_angle: %.2lf, proximity_angle: %.2lf, proximity_distance: %.2lf, skeleton_search_distance: %.2lf, wall_search_distance: %.2lf, wall_search_angle: %.2lf", LOG_START, searchParams.minTravelDistance, searchParams.obstructionAngle, searchParams.proximityAngle, searchParams.proximityDistance, searchParams.skeletonSearchDistance, searchParams.wallSearchDistance, searchParams.wallSearchAngle);
    }
    if (cfg != NULL) {
        driveParams.maxVel = cfg->getParamAsDouble("maxVel", driveParams.maxVel);
        driveParams.maxTurn = cfg->getParamAsDouble("maxTurn", driveParams.maxTurn);
        driveParams.minVel = cfg->getParamAsDouble("minVel", driveParams.minVel);
        driveParams.minTurn = cfg->getParamAsDouble("minTurn", driveParams.minTurn);
        ROS_INFO("%s maxVel: %.2lf, maxTurn: %.2lf, minVel: %.2lf, minTurn: %.2lf", LOG_START, driveParams.maxVel, driveParams.maxTurn, driveParams.minVel, driveParams.minTurn);
    }
    ConfigElementPtr hysteresisCfg = cfg->getChild("hysteresis");
    if (hysteresisCfg != NULL) {
        hysteresisParams.clusterTimeout = crosbot::Duration(hysteresisCfg->getParamAsDouble("cluster_timeout", hysteresisParams.clusterTimeout.toSec()));
        hysteresisParams.enabled = hysteresisCfg->getParamAsBool("enabled", hysteresisParams.enabled);
        hysteresisParams.hysteresisSize = hysteresisCfg->getParamAsInt("hysteresis_size", hysteresisParams.hysteresisSize);
        hysteresisParams.hysteresisTimeout = crosbot::Duration(hysteresisCfg->getParamAsDouble("hysteresis_timeout", hysteresisParams.hysteresisTimeout.toSec()));
        hysteresisParams.marginDistance = hysteresisCfg->getParamAsDouble("margin_distance", hysteresisParams.marginDistance);
        hysteresisParams.marginHeading = hysteresisCfg->getParamAsDouble("margin_heading", hysteresisParams.marginHeading);
        ROS_INFO("%s enabled: %d, hysteresis_size: %d, margin_distance: %.2lf, margin_heading: %.2lf", LOG_START, hysteresisParams.enabled, hysteresisParams.hysteresisSize,  hysteresisParams.marginDistance, hysteresisParams.marginHeading);        
    }

    // Advertise ROS subscribers and publishers
    ros::NodeHandle nh;
    subscribeGrid(nh);
    explorer_srv = nh.advertiseService(explorer_srv_name, &ExplorerROSNode::callbackExplorerMode, this);
    explorer_feedback_pub = nh.advertise<rsa17::ExplorerFeedback>(explorer_feedback_pub_name, 1);
    historySub = nh.subscribe(historySub_name, 1, &ExplorerROSNode::callbackHistory, this);
    voronoiImagePub = nh.advertise<sensor_msgs::Image>(imagePub_name, 1);
    velPub = nh.advertise<geometry_msgs::Twist>(velPub_name, 1);

    // Dynamic reconfiguration
    gotReconfig = false;
    reconfig_server = new ReconfigServer(mutexReconfig);
    publishReconfig(localVoronoiConstraints);
    reconfig_callbackType = boost::bind(&ExplorerROSNode::callback_reconfigure, this, _1, _2);
    reconfig_server->setCallback(reconfig_callbackType);

    ROS_INFO("%s configure done", LOG_START);
    ROS_INFO("%s \t %d", LOG_START, searchWhilePaused);
}

void ExplorerROSNode::startup() {
	ROS_INFO("%s startup()", LOG_START); 
    Explorer::startup();
}

void ExplorerROSNode::shutdown() {
	ROS_INFO("%s shutdown()", LOG_START); 
    Explorer::shutdown();

    // Stop reconfig
    reconfig_server->clearCallback();
    gotReconfig = false;
    delete reconfig_server;

    // Shutdown subs/pubs
    explorer_srv.shutdown();
    explorer_feedback_pub.shutdown();
    gridSub.shutdown();
    historySub.shutdown();
    voronoiImagePub.shutdown();
    velPub.shutdown();
}

void ExplorerROSNode::subscribeGrid(ros::NodeHandle& nh) {
    gridSub.shutdown();

    ROS_INFO("%s Subscribing to map topic: %s", LOG_START, gridSub_name.c_str());
    gridSub = nh.subscribe(gridSub_name, 1, &ExplorerROSNode::callbackOccGrid, this);
}

void ExplorerROSNode::loadLatestPose() {
	if (debugMsgs) ROS_INFO("%s loadLatestPose()", LOG_START);

    nav_msgs::OccupancyGridConstPtr latestMap;
    {
        Lock lock(lock_ros, false);
        latestMap = this->latestMap;
    }

    Pose newPose = Pose(INFINITY, INFINITY, INFINITY);
    if (latestMap != NULL) {
        ros::Time tNow = ros::Time::now();// - ros::Duration(1.0);
        newPose = getLatestPose(latestMap->header.frame_id, tNow);
        //newPose = getLatestPose(latestMap->header.frame_id, latestMap->header.stamp);
    }

    // Exchange new pose with atomic stored pose
    {
        Lock lock(lock_pose);
        latestPose = newPose;
    }
}

Pose ExplorerROSNode::getLatestPose(std::string frame_id, Time timestamp) {
    if (debugMsgs) ROS_INFO("%s getLatestPose() - %s, %s at time: %.5lf", LOG_START, frame_id.c_str(), baseFrame.c_str(), timestamp.toSec());

    tf::StampedTransform transform;
    bool loaded = false;

    try {
        tfListener.waitForTransform(frame_id, baseFrame, timestamp.toROS(), ros::Duration(DEFAULT_LOADPOSE_WAIT4TRANSFORM));
        tfListener.lookupTransform(frame_id, baseFrame, timestamp.toROS(), transform);
        loaded = true;
    } catch (tf::TransformException& e) {
        ROS_WARN("%s getLatestPose(): Exception caught(%s). time: %.5lf, %s - %s\n", LOG_START, e.what(), timestamp.toSec(), frame_id.c_str(), baseFrame.c_str());
        return Pose(INFINITY, INFINITY, INFINITY);
    }
    return Pose(transform);
}

void ExplorerROSNode::loadLatestVoronoi() {
	if (debugMsgs) ROS_INFO("%s loadLatestVoronoi()", LOG_START);
    nav_msgs::OccupancyGridConstPtr latestMap;
    {
        Lock lock(lock_ros, false);
        latestMap = this->latestMap;
    }

    if (latestMap == NULL) {
        // No voronoi to make
        latestVoronoi = NULL;
    } else if (latestVoronoi != NULL && latestVoronoi->timestamp == Time(latestMap->header.stamp)) {
        // No need to update voronoi
    } else {
        if (debugMsgs) {
            ROS_INFO("%s Loading new voronoi grid: restrict: %.1lf, partial: %.1lf, expand: %.1lf", LOG_START, voronoiConstraints.restricted, voronoiConstraints.partial, voronoiConstraints.expand);
        }

        Pose robot = getLatestPose();
        if (robot.isFinite()) {
            latestVoronoi = new VoronoiGrid(*latestMap, voronoiConstraints, robot);
        } else {
            ROS_WARN("%s Cannot update voronoi grid - robot pose is infinite", LOG_START);
        }
    }
}

Pose ExplorerROSNode::getLatestPose() {
	if (debugMsgs) ROS_INFO("%s getLatestPose()", LOG_START);

    /*nav_msgs::OccupancyGridConstPtr latestMap;
    {
        Lock lock(rosLock, false);
        latestMap = this->latestMap;
    }

    if (latestMap == NULL) {
        return Pose(INFINITY, INFINITY, INFINITY);
    }
    return getLatestPose(latestMap->header.frame_id, latestMap->header.stamp);*/

    Pose latest;
    {
        Lock lock(lock_pose);
        latest = latestPose;
    }
    return latest;
}

VoronoiGridPtr ExplorerROSNode::getLatestVoronoi() {
	if (debugMsgs) ROS_INFO("%s getLatestVoronoi()", LOG_START); 
    return latestVoronoi;
}

Pose ExplorerROSNode::findDriveTarget(const VoronoiGrid& voronoi, const Pose& robot) {
    Pose rval = Explorer::findDriveTarget(voronoi, robot);

    // TODO: to view in camera relative frame need camera_info topic (see http://wiki.ros.org/rviz/DisplayTypes/Camera)
    if (voronoiImagePub.getNumSubscribers() > 0) {
        ImagePtr image = getPlanImage(voronoi, robot, rval);
        voronoiImagePub.publish(image->toROS());
    }

    return rval;
}

bool ExplorerROSNode::stopMotors() {
	if (debugMsgs) ROS_INFO("%s stopMotors()", LOG_START); 
	
    if (((void*)(velPub)) == NULL) {
        return false;
    }

    ROS_INFO("%s stopMotors()", LOG_START);
    geometry_msgs::Twist twist;

    twist.linear.x = twist.linear.y = twist.linear.z = 0;
    twist.angular.x = twist.angular.y = twist.angular.z = 0;

    velPub.publish(twist);

    return true;
}

void ExplorerROSNode::driveTo(const Pose& relativePosition) {
	if (debugMsgs) ROS_INFO("%s driveTo()", LOG_START); 
    if (((void*)(velPub)) == NULL) {
        return;
    }


    double d = 0;   // Distance to travel
    double a = 0;   // Angle to rotate

    if (searchParams.rotateToPose) {
        double yaw, pitch, roll;
        relativePosition.getYPR(yaw, pitch, roll);
        d = 0;
        a = yaw;
        ROS_INFO("%s rotate only relative yaw: %.2lf", LOG_START, RAD2DEG(yaw));
    } else {
        d = relativePosition.position.distanceTo(Point());
        a = atan2(relativePosition.position.y, relativePosition.position.x);
    }


    geometry_msgs::Twist twist;

    twist.linear.x = twist.linear.y = twist.linear.z = 0;
    twist.angular.x = twist.angular.y = twist.angular.z = 0;

    if (fabs(a) >= driveParams.turnOnly) {
        twist.angular.z = (a<0) ? -driveParams.maxTurn : driveParams.maxTurn;
    } else {
        twist.angular.z = a / driveParams.turnOnly * driveParams.maxTurn;
        twist.angular.z = std::min(twist.angular.z, driveParams.maxTurn);
        twist.angular.z = std::max(twist.angular.z, -driveParams.maxTurn);
        if (twist.angular.z < driveParams.minTurn && twist.angular.z > -driveParams.minTurn) {
            if (twist.angular.z > 0) {
                twist.angular.z = driveParams.minTurn;
            } else {
                twist.angular.z = -driveParams.minTurn;
            }
        }

        // The closer the point is to the robot, the slower the robot drives
        twist.linear.x = (d / searchParams.wallSearchDistance) * driveParams.maxVel * (1 - (fabs(a) / driveParams.turnOnly));
        twist.linear.x = std::min(twist.linear.x, driveParams.maxVel);
        twist.linear.x = std::max(twist.linear.x, -driveParams.maxVel);
        if (twist.linear.x < driveParams.minVel && twist.linear.x > -driveParams.minVel) {
            if (twist.linear.x > 0) {
                twist.linear.x = driveParams.minVel;
            } else {
                twist.linear.x = -driveParams.minVel;
            }
        }
        ROS_INFO("%s twist command: linear.x: %.4lf, angular.z: %.4lf", LOG_START, twist.linear.x, twist.angular.z);
    }

    if (isnan(twist.linear.x)) {
        twist.linear.x = 0;
    }
    if (isnan(twist.angular.z)) {
        twist.angular.z = 0;
    }

    velPub.publish(twist);
}

bool ExplorerROSNode::setWaypoints(nav_msgs::Path& navPath) {
	if (debugMsgs) ROS_INFO("%s setWaypoints()", LOG_START); 
    // Get voronoi map
    VoronoiGridPtr voronoi = latestVoronoi;
    if (voronoi == NULL) {
        ROS_ERROR("%s No voronoi map to plot robot path on", LOG_START);
        return false;
    }

    // Waypoints to set
    std::vector< Pose > waypoints;
    if (navPath.poses.size() > 0) {
        // Transform map and nav path into the same frame
        tf::StampedTransform transform;
        try {
            ros::Time now = ros::Time::now();
            ROS_INFO("%s Setting transform from: %s to %s", LOG_START, voronoi->frame.c_str(), navPath.header.frame_id.c_str());
            tfListener.waitForTransform(voronoi->frame, navPath.header.frame_id, now, ros::Duration(DEFAULT_MAXWAIT4TRANSFORM));
            tfListener.lookupTransform(voronoi->frame, navPath.header.frame_id, now, transform);

            ROS_INFO("%s Transform is: %s", LOG_START, crosbot::Pose3D(transform.getOrigin(), transform.getRotation()).toString().c_str());
        } catch (tf::TransformException& ex) {
            ROS_ERROR("%s: Error getting transform for voronoi frame to navPath. (%s)\n", LOG_START, ex.what());
            return false;
        }

        // Set waypoints for search strategy

        waypoints.resize(navPath.poses.size());
        for (size_t i = 0; i < waypoints.size(); ++i) {
            waypoints[i] = transform * Pose(navPath.poses[i]).toTF();
            ROS_INFO("%s Adding waypoint %s", LOG_START, waypoints[i].position.toString().c_str());
        }
    }

    // Set waypoints
    crosbot::Pose3D currentPose = getLatestPose();
    ROS_INFO("%s Current pose: %s", LOG_START, currentPose.toString().c_str());
    searchParams.setWaypoints(waypoints, currentPose);

    return true;
}

void ExplorerROSNode::publishReconfig(VoronoiGrid::Constraints& constraints) {
	if (debugMsgs) ROS_INFO("%s publishReconfig()", LOG_START); 
    // Set config values
    rsa17::CrosbotExploreReconfigConfig config;

    config.voronoi_restricted = localVoronoiConstraints.restricted;
    config.voronoi_partial = localVoronoiConstraints.partial;
    config.voronoi_expand = localVoronoiConstraints.expand;
    config.voronoi_orphanThreshold = localVoronoiConstraints.orphanThreshold;

    config.min_travel_distance = searchParams.minTravelDistance;
    config.map_sub = gridSub_name;
    config.obstruction_angle = RAD2DEG(searchParams.obstructionAngle);
    config.proximity_angle = RAD2DEG(searchParams.proximityAngle);
    config.proximity_distance = searchParams.proximityDistance;
    config.skeleton_search_distance = searchParams.skeletonSearchDistance;
    config.wall_search_distance = searchParams.wallSearchDistance;
    config.wall_search_angle = searchParams.wallSearchAngle;

    config.drive_max_turn = driveParams.maxTurn;
    config.drive_max_vel = driveParams.maxVel;

    // Call service
    reconfig_server->updateConfig(config);
}

void ExplorerROSNode::statusChanged(rsa17::ExplorerStatus::Status status) {
	if (debugMsgs) ROS_INFO("%s statusChanged()", LOG_START); 
    rsa17::ExplorerFeedback feedback;
    feedback.id = currentGoalId;
    feedback.status = status;
    feedback.operating = !paused;

    // TODO: Deal with "blocked" and "arrived" changed status

    if (searchParams.strategy == SearchParameters::WallFollow && searchParams.side == SearchParameters::Left) {
        feedback.searchStrategy = rsa17::ExploreMode::MODE_LEFT_WALL;
    } else if (searchParams.strategy == SearchParameters::WallFollow && searchParams.side == SearchParameters::Right) {
        feedback.searchStrategy = rsa17::ExploreMode::MODE_RIGHT_WALL;
    } else if (searchParams.strategy == SearchParameters::Waypoint) {
        feedback.searchStrategy = rsa17::ExploreMode::MODE_WAYPOINT;
    }

    // TODO: replace
    explorer_feedback_pub.publish(feedback);
}

} // namespace crosbot

