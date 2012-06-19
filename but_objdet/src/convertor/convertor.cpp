/**
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Hodan (xhodan04@stud.fit.vutbr.cz)
 * Date: 01.04.2012 (version 1.0)
 *
 * License: BUT OPEN SOURCE LICENSE
 *------------------------------------------------------------------------------
 */

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
 
#include "but_objdet/but_objdet.h"
#include "but_objdet_msgs/Detection.h"
#include "but_objdet/convertor/convertor.h"

using namespace but_objdet_msgs; 
 
/* -----------------------------------------------------------------------------
 * Conversion from Detection msg to butObject
 */
butObject Convertor::detectionToButObject(Detection &detection)
{
    butObject object;
        
    object.m_id = detection.m_id;
    object.m_class = detection.m_class;
    object.m_score = detection.m_score;
    
    object.m_pos_2D.x = detection.m_pos_2D.x;
    object.m_pos_2D.y = detection.m_pos_2D.y;
    object.m_pos_2D.z = detection.m_pos_2D.z;
    
    object.m_bb.x = detection.m_bb.x,
    object.m_bb.y = detection.m_bb.y,
    object.m_bb.width = detection.m_bb.width,
    object.m_bb.height = detection.m_bb.height;
    
    object.m_angle = detection.m_angle;
    
    object.m_speed.x = detection.m_speed.x;
    object.m_speed.y = detection.m_speed.y;
    object.m_speed.z = detection.m_speed.z;
        
    // Convert Image msg to Mat
    try {
        object.m_mask = cv_bridge::toCvCopy(detection.m_mask)->image;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        //return;
    }
    
    return object;
}


/* -----------------------------------------------------------------------------
 * Conversion from vector of Detection msgs to vector of butObjects
 */
vector<butObject> Convertor::detectionsToButObjects(vector<Detection> &detections)
{
    vector<butObject> objects;
    
    for(unsigned int i = 0; i < detections.size(); i++) {
        objects.push_back(detectionToButObject(detections[i]));
    }
    
    return objects;
}


/* -----------------------------------------------------------------------------
 * Conversion from butObject to Detection msg
 */
Detection Convertor::butObjectToDetection(butObject &object, std_msgs::Header header)
{
    Detection detection;
    
    detection.header = header;

    detection.m_id = object.m_id;
    detection.m_class = object.m_class;
    detection.m_score = object.m_score;
    
    detection.m_pos_2D.x = object.m_pos_2D.x;
    detection.m_pos_2D.y = object.m_pos_2D.y;
    detection.m_pos_2D.z = object.m_pos_2D.z;
    
    detection.m_bb.x = object.m_bb.x,
    detection.m_bb.y = object.m_bb.y,
    detection.m_bb.width = object.m_bb.width,
    detection.m_bb.height = object.m_bb.height;
    
    detection.m_angle = object.m_angle;
    
    detection.m_speed.x = object.m_speed.x;
    detection.m_speed.y = object.m_speed.y;
    detection.m_speed.z = object.m_speed.z;

    // Convert Mat to Image msg
    cv_bridge::CvImage mask;
    mask.encoding = sensor_msgs::image_encodings::TYPE_8UC1; // It is supposed that mask is of type CV_8UC1
    mask.image    = object.m_mask; // cv::Mat
    
    //sensor_msgs::Image img = *(mask.toImageMsg());
    detection.m_mask = *(mask.toImageMsg());
    
    return detection;
}


/* -----------------------------------------------------------------------------
 * Conversion from vector of butObjects to vector of Detection msgs
 */
vector<Detection> Convertor::butObjectsToDetections(vector<butObject> &objects, std_msgs::Header header)
{
    vector<Detection> detections;
    
    for(unsigned int i = 0; i < objects.size(); i++) {
        detections.push_back(butObjectToDetection(objects[i], header));
    }
    
    return detections;
}

