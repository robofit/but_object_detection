/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: David Chrapek
 * Supervised by: Vitezslav Beran (beranv@fit.vutbr.cz), Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 01/04/2012
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#ifndef _TRACKER_KALMAN_NODE_
#define _TRACKER_KALMAN_NODE_

#include <map>
#include <ros/ros.h> // Main header of ROS
#include <sensor_msgs/Image.h>

#include "but_objdet_msgs/DetectionArray.h"
#include "but_objdet/tracker/tracker_kalman.h"


// Indicates if to visualize detections and predictions in a window
#define VISUAL_OUTPUT 1


namespace but_objdet
{

struct DetM
{
    but_objdet_msgs::Detection det; // Detection
    TrackerKalman *kf; // Kalman filter for tracking of this detection
    int ttl; // Time to live
    int msTime; // Time of detection in milliseconds
};

/**
 * @class SampleDetectorRosconn
 */
class TrackerKalmanNode
{
public:
	TrackerKalmanNode();
	~TrackerKalmanNode();

private:
	void rosInit();

	bool predictDetections(but_objdet::PredictDetections::Request &req,
						   but_objdet::PredictDetections::Response &res);

	int rosTimeToMs(ros::Time stamp);

	void newDataCallback(const but_objdet_msgs::DetectionArrayConstPtr &detArrayMsg);

	void newImageCallback(const sensor_msgs::ImageConstPtr &imageMsg);

	ros::NodeHandle nh; // NodeHandle is the main access point for communication with ROS system
	ros::ServiceServer predictionSRV;
	ros::Subscriber detSub;
	ros::Subscriber imgSub;

	typedef std::map<int, DetM> DetMem;

	DetMem detectionMem; // Memory of currently considered detections

	// If a detection of an object didn't occur in the specified number of
	// last detections (specified by a value of this variable),
	// it is not considered any more.
	int defaultTtl;

	// If a detection of an object doesn't occur again during this period,
	// it is not considered any more.
	int defaultTtlTime;

	std::string winName;
};

}

#endif // _TRACKER_KALMAN_NODE_

