/**
 * Developed by dcgm-robotics@FIT group
 * Author: David Chrapek
 * Date: 01.04.2012 (version 1.0)
 *
 * License: BUT OPEN SOURCE LICENSE
 *------------------------------------------------------------------------------
 */

#ifndef _TRACKER_KALMAN_NODE_
#define _TRACKER_KALMAN_NODE_

#include <map>
#include <ros/ros.h> // Main header of ROS
#include <sensor_msgs/Image.h>
#include "but_objdet_msgs/DetectionArray.h"
#include "but_objdet/tracker/tracker_kalman.h"


// Indicates if to visualize detections and predictions in a window
#define VISUAL_OUTPUT 1

typedef struct {
    but_objdet_msgs::Detection det; // Detection
    TrackerKalman *kf; // Kalman filter for tracking of this detection
    int ttl; // Time to live
    int msTime; // Time of detection in miliseconds
} TDetM;

/**
 * @class SampleDetectorRosconn
 */
class TrackerKalmanNode {
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
	    	    
	    std::map<int, TDetM> detectionMem; // Memory of currently considered detections
	    
	    // If a detection of an object didn't occur in the specified number of
	    // last detections (specified by a value of this variable),
	    // it is not considered any more.
	    int defaultTtl;
	    
	    // If a detection of an object doesn't occur again during this period,
	    // it is not considered any more.
	    int defaultTtlTime;
	    
	    string winName;
};

#endif // _TRACKER_KALMAN_NODE_

