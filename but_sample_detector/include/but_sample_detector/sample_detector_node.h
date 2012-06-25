/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Hodan (xhodan04@stud.fit.vutbr.cz)
 * Supervised by: Vitezslav Beran (beranv@fit.vutbr.cz), Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 01/04/2012
 * Description: Sample detector demonstrating how to wrap a detector using ObjDet API into ROS.
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
#ifndef _SAMPLE_DETECTOR_NODE_
#define _SAMPLE_DETECTOR_NODE_

#include <ros/ros.h> // Main header of ROS
#include <sensor_msgs/Image.h>

#include "but_objdet/but_objdet.h"
#include "but_objdet/matcher/matcher_overlap.h"
#include "but_sample_detector/sample_detector.h"


namespace but_sample_detector
{

/**
 * @class SampleDetectorNode
 */
class SampleDetectorNode
{
public:
	SampleDetectorNode();
	virtual ~SampleDetectorNode();

private:
	void rosInit();

	void newDataCallback(const sensor_msgs::ImageConstPtr &image);

	int getNewObjectID();

	but_objdet::Objects detections; // Current detections
	but_objdet::Objects predictions; // Current predictions

	but_sample_detector::SampleDetector *sampleDetector; // Detector
	but_objdet::MatcherOverlap *matcherOverlap; // Matcher

	ros::NodeHandle nh; // NodeHandle is the main access point for communication with ROS system

	ros::Subscriber dataSub;
	
	ros::Publisher detectionsPub; // Publisher of detections
	
	ros::ServiceClient predictClient; // Client for comunication with tracker
									  // (using PredictDetections service)

	int lastObjectID; // Last assigned object ID
};

}

#endif // _SAMPLE_DETECTOR_NODE_

