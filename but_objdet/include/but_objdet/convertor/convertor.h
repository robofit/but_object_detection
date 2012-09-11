/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Hodan
 * Supervised by: Vitezslav Beran (beranv@fit.vutbr.cz), Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 01/04/2012
 * Description: Converts from but_objdet messages to standard C++ structures
 * and vice versa.
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
#ifndef _CONVERTOR_
#define _CONVERTOR_

#include <ros/ros.h> // Main header of ROS
#include "but_objdet/but_objdet.h"
#include "but_objdet_msgs/Detection.h"

namespace but_objdet
{

typedef std::vector<but_objdet_msgs::Detection> Detections;

/**
 * A class performing these conversions:
 *  1) Detection to Object
 *  2) Object to Detection
 *  3) A vector of Objects to a vector of Detections
 *  4) A vector of Detections to a vector of Objects
 * Notes:
 *  - Detection = ROS message defined in but_objdet_msgs package)
 *  - Object = C++ struct (defined in but_objdet.h located in but_objdet package)
 *  - Detection and Object contain equivalent items. Detection message is used
 *    to transfer data through ROS topics/services, while Object is used for
 *    processing within C++ classes.
 *
 * @author Tomas Hodan, Vitezslav Beran (beranv@fit.vutbr.cz), Michal Spanel (spanel@fit.vutbr.cz)
 */
class Convertor
{
public:
    /**
     * Conversion from Detection to Object.
     * @param A Detection message to be converted to an Object.
     * @return Resulting Object.
     */
	static Object detectionToButObject(const but_objdet_msgs::Detection &detection);

    /**
     * Conversion from a vector of Detection messages to a vector of Objects.
     * @param A vector of Detection messages to be converted to a vector of Objects.
     * @return Resulting vector of Objects.
     */
	static Objects detectionsToButObjects(const Detections &detections);

    /**
     * Conversion from Object to Detection.
     * @param An object to be converted to a Detection message.
     * @return Resulting Detection message.
     */
	static but_objdet_msgs::Detection butObjectToDetection(const Object &object, std_msgs::Header header);

    /**
     * Conversion from a vector of Objects to a vector of Detection messages.
     * @param A vector of Objects to be converted to a vector of Detection messages.
     * @return Resulting vector of Detection messages.
     */
	static Detections butObjectsToDetections(const Objects &objects, std_msgs::Header header);
};

}

#endif // _CONVERTOR_
 
