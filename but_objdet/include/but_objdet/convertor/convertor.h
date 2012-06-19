/**
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Hodan (xhodan04@stud.fit.vutbr.cz)
 * Date: 01.04.2012 (version 1.0)
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 * Description: Converts from but_objdet messages to standard C++ structures
 * and vice versa.
 *------------------------------------------------------------------------------
 */

#ifndef _CONVERTOR_
#define _CONVERTOR_

#include <ros/ros.h> // Main header of ROS
#include "but_objdet/but_objdet.h"
#include "but_objdet_msgs/Detection.h"

using namespace std;
using namespace but_objdet_msgs;


class Convertor
{
    public:
        static butObject detectionToButObject(Detection &detection);
        static vector<butObject> detectionsToButObjects(vector<Detection> &detections);
		
        static Detection butObjectToDetection(butObject &object, std_msgs::Header header);
        static vector<Detection> butObjectsToDetections(vector<butObject> &objects, std_msgs::Header header);
};

#endif // _CONVERTOR_
 
