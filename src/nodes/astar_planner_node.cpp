/*
 * astar.cpp
 *
 *  Created on: 22/02/2013
 *      Author: mmcgill
 */

#include <ros/ros.h>
#include <rsa17/astar.hpp>
#include <crosbot/utils.hpp>
#include <crosbot/thread.hpp>
#include <crosbot_map/voronoi.hpp>
#include <crosbot/config.hpp>

#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <rsa17/GetPath.h>

using namespace crosbot;
using namespace rsa17;

#define DEFAULT_BASEFRAME			"base_link"
#define LOG_START                   "AStarPlannerROSNode ::"

class AstarNode {
	std::string baseFrame;
	VoronoiGrid::Constraints voronoiConstraints;

	ros::Subscriber gridSub, historySub;
	ros::Publisher imagePub;
	tf::TransformListener tfListener;
	ros::ServiceServer pathServer;

	ReadWriteLock rosLock;
	nav_msgs::OccupancyGridConstPtr latestMap;
	nav_msgs::PathConstPtr latestHistory;

	ReadWriteLock voronoiLock;
	VoronoiGridPtr latestVoronoi;

	AStarPlanner planner;
public:
	AstarNode() :
		baseFrame(DEFAULT_BASEFRAME)
	{ voronoiConstraints.make4connected = false; }

    void callbackOccGrid(const nav_msgs::OccupancyGridConstPtr& latestMap) {
    	Lock lock(rosLock, true);
    	this->latestMap = latestMap;
    }

    void callbackHistory(const nav_msgs::PathConstPtr& latestHistory) {
    	Lock lock(rosLock, true);
    	this->latestHistory = latestHistory;
    }

    bool callbackGetPath(GetPath::Request& req, GetPath::Response& res) {
    	LOG("AstarNode::GetPath(): start (%.2lf, %.2lf, %.2lf) end (%.2lf, %.2lf, %.2lf)\n",
    			req.start.position.x, req.start.position.y, req.start.position.z,
    			req.end.position.x, req.end.position.y, req.end.position.z);

    	VoronoiGridPtr voronoi;
    	{{
    		Lock lock(rosLock);
			if (latestMap != NULL) {
				voronoi = latestVoronoi;
				if (voronoi == NULL || voronoi->timestamp < Time(latestMap->header.stamp)) {
					Pose robot(INFINITY,INFINITY,INFINITY);
					std::string mapFrame = latestMap->header.frame_id;
					ros::Time timestamp = latestMap->header.stamp;

					if (baseFrame != "") {
						try {
							tf::StampedTransform transform;
							tfListener.waitForTransform(mapFrame, baseFrame,
									timestamp, ros::Duration(DEFAULT_MAXWAIT4TRANSFORM));
							tfListener.lookupTransform(mapFrame,
									baseFrame, timestamp, transform);
							robot = transform;
						} catch (tf::TransformException& ex) {
							ERROR("astar: Error getting current robot transform. (%s)\n", ex.what());
							robot = Pose(INFINITY,INFINITY,INFINITY);
						}
					}
					voronoi = new VoronoiGrid(*latestMap, voronoiConstraints, robot);
					lock.unlock();

					Lock lock2(voronoiLock, true);
					if (latestVoronoi == NULL || latestVoronoi->timestamp < voronoi->timestamp) {
						latestVoronoi = voronoi;
					}
				}
			}
    	}}

    	if (voronoi == NULL) {
    		ERROR("AstarNode::GetPath(): No voronoi map available.\n");
    		return false;
    	}

    	ImagePtr image;
    	if (imagePub.getNumSubscribers() > 0) {
    		image = new Image(Image::RGB8, voronoi->height, voronoi->width);
    	}
    	// Do the A* search
    	planner.maxExpandC = ((voronoiConstraints.restricted > voronoiConstraints.partial?
    			voronoiConstraints.restricted: voronoiConstraints.partial) + voronoiConstraints.expand) /
    			voronoi->resolution;
    	Plan path = planner.getPath(voronoi, Pose(req.start), Pose(req.end), image);

    	if (imagePub.getNumSubscribers() > 0 && image != NULL) {
    		imagePub.publish(image->toROS());
    	}

    	// return the plan
    	res.path.header.stamp = ros::Time::now();
    	std::string frame_id = voronoi->frame;
    	res.path.header.frame_id = frame_id;
    	res.path.poses.resize(path.size());
    	for (size_t i = 0; i < path.size(); ++i) {
    		geometry_msgs::PoseStamped& pose = res.path.poses[i];
    		pose.header.frame_id = frame_id;
    		pose.pose = path[i].toROS();
    	}
    	LOG("AstarNode::GetPath(): Returning a path with %Zd poses.\n", path.size());
    	return true;
    }

	void initalise(ros::NodeHandle& nh) {
	    ROS_INFO("%s astar_planner_node initialising", LOG_START);
		// read configuration/parameters
		ros::NodeHandle paramNH("~");	// Because ROS's search order is buggered
		paramNH.param<std::string>("base_frame", baseFrame, DEFAULT_BASEFRAME);
		ConfigElementPtr cfg = new ROSConfigElement(paramNH);
		
		// TODO: read voronoi constraints
		ConfigElementPtr voronoiCfg = cfg->getChild("voronoi");
		if (voronoiCfg != NULL) {
				voronoiConstraints.restricted = voronoiCfg->getParamAsDouble("restrict", voronoiConstraints.restricted);
				voronoiConstraints.partial = voronoiCfg->getParamAsDouble("partial", voronoiConstraints.partial);
				voronoiConstraints.expand = voronoiCfg->getParamAsDouble("expand", voronoiConstraints.expand);
				voronoiConstraints.orphanThreshold = voronoiCfg->getParamAsInt("orphan", voronoiConstraints.orphanThreshold);
		}

	    // Configure A* Planner
	    ConfigElementPtr astarPlannerCfg = cfg->getChild("planner");
	    //planner.configure(astarPlannerCfg);

		gridSub = nh.subscribe("map", 1, &AstarNode::callbackOccGrid, this);
		historySub = nh.subscribe("history", 1, &AstarNode::callbackHistory, this);
		imagePub = nh.advertise<sensor_msgs::Image>("image", 1);

		pathServer = nh.advertiseService("get_path", &AstarNode::callbackGetPath, this);
	}

	void shutdown() {
		gridSub.shutdown();
		historySub.shutdown();
		imagePub.shutdown();

		pathServer.shutdown();
	}
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "astar");

    ros::NodeHandle nh;

    AstarNode node;
    node.initalise(nh);

    while (ros::ok()) {
        ros::spin();
    }
    node.shutdown();

    return 0;
}



