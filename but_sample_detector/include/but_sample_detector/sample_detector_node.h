/**
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Hodan (xhodan04@stud.fit.vutbr.cz)
 * Date: 01.04.2012 (version 1.0)
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 * Description:
 * Sample detector demonstrating how to wrap a detector using ObjDet API into ROS.
 *------------------------------------------------------------------------------
 */

#ifndef _SAMPLE_DETECTOR_NODE_
#define _SAMPLE_DETECTOR_NODE_

#include <ros/ros.h> // Main header of ROS
#include <sensor_msgs/Image.h>

#include "but_objdet/but_objdet.h"
#include "but_objdet/matcher/matcher_overlap.h"
#include "but_sample_detector/sample_detector.h"


/**
 * @class SampleDetectorNode
 */
class SampleDetectorNode {
	public:
        SampleDetectorNode();		
		~SampleDetectorNode();
		
	private:
	    void rosInit();
	    void newDataCallback(const sensor_msgs::ImageConstPtr &image);
	    int getNewObjectID();
	
	    vector<butObject> detections; // Current detections
	    vector<butObject> predictions; // Current predictions
	
	    SampleDetector *sampleDetector; // Detector
	    MatcherOverlap *matcherOverlap; // Matcher
	    
	    ros::NodeHandle nh; // NodeHandle is the main access point for communication with ROS system
	    
	    ros::Subscriber dataSub;
	    
	    ros::Publisher detectionsPub; // Publisher of detections
	    
        ros::ServiceClient predictClient; // Client for comunication with tracker
                                          // (using PredictDetections service)
                                          
        int lastObjectID; // Last assigned object ID
};

#endif // _SAMPLE_DETECTOR_NODE_

