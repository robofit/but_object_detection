/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: David Chrapek, Tomas Hodan
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
#ifndef _FLIP_IMAGE_NODE_
#define _FLIP_IMAGE_NODE_

#include <map>
#include <ros/ros.h> // Main header of ROS
#include <sensor_msgs/Image.h>

#include "but_objdet_msgs/DetectionArray.h"
#include "but_objdet/tracker/tracker_kalman.h"


// Indicates if to visualize detections and predictions in a window
#define VISUAL_OUTPUT 1


namespace but_objdet
{

/**
 * A class implementing the tracker node, which creates and maintains a Kalman filter
 * tracker for each detected object (if there is no detection of an object for
 * some time / number of frames, the tracker for that object is canceled).
 * It also advertises a service for prediction of the next state of detections,
 * (either of all of the currently maintained or of some specified object class or
 * object id).
 *
 * @author Tomas Hodan, Vitezslav Beran (beranv@fit.vutbr.cz), Michal Spanel (spanel@fit.vutbr.cz)
 */
class FlipImageNode
{
public:
	FlipImageNode();
	~FlipImageNode();

private:
    /**
     * ROS related initialization called from the constructor.
     */
	void rosInit();


    /**
     * A callback function called when a new Image is received. The image is used just
     * for visualization of detections and predictions, thus it doesn't influence
     * functionality of this node in any way.
     * @param imageMsg  Image message.
     */
	void newImageCallback(const sensor_msgs::ImageConstPtr &imageMsg);

        void newDepthCallback(const sensor_msgs::ImageConstPtr &imageMsg);
  

    ros::NodeHandle nh; // NodeHandle is the main access point for communication with ROS system
	
	
	ros::Subscriber imgSub;
        ros::Publisher imgPub;
        ros::Subscriber depthSub;
        ros::Publisher depthPub;
	std::string winName;
};

}

#endif // _TRACKER_KALMAN_NODE_

