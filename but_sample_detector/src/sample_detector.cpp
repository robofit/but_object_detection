/**
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Hodan (xhodan04@stud.fit.vutbr.cz)
 * Date: 01.04.2012 (version 1.0)
 *
 * License: LGPL
 *
 * Description:
 * Sample detector demonstrating how to wrap a detector using ObjDet API into ROS.
 *------------------------------------------------------------------------------
 */

#include <ros/ros.h> // Main header of ROS
#include <opencv2/highgui/highgui.hpp> // OpenCV available within a vision_opencv ROS stack

#include "but_objdet/but_objdet.h"
#include "but_objdet/matcher/matcher_overlap.h"
#include "but_sample_detector/sample_detector.h"

using namespace cv;
using namespace std;
using namespace but_objdet;


namespace but_sample_detector
{

/* -----------------------------------------------------------------------------
 * Constructor
 */
SampleDetector::SampleDetector()
{
    // initialize random seed
    srand(time(NULL));
}


/* -----------------------------------------------------------------------------
 * Destructor
 */
SampleDetector::~SampleDetector()
{

}


/* -----------------------------------------------------------------------------
 * Initialization
 */
void SampleDetector::init( string config_filename )
{
    // ...
}


/* -----------------------------------------------------------------------------
 * Set a paramater to the specified value
 */
void SampleDetector::setParam( int param_id, double value )
{
    // ...
}


/* -----------------------------------------------------------------------------
 * Get a value of the specified paramater
 */
double SampleDetector::getParam( int param_id )
{
    // ...
    return 0;
}


/* -----------------------------------------------------------------------------
 * Function providing predictions, which can be taken into account during detection
 */
void SampleDetector::prediction( Objects& objects, int FLAGS )
{
    // ...
}


/* -----------------------------------------------------------------------------
 * Detection (FAKE)
 */
void SampleDetector::detect( const Mat& rgb, const Mat& depth, Objects& objects, int FLAGS )
{
    objects.clear();

    // Set a bounding box of a FAKE detection
    Object detection;
    detection.m_bb.x = 100 + (rand() % 20); 
    detection.m_bb.y = 100 + (rand() % 20); 
    detection.m_bb.width = 100 + (rand() % 5);
    detection.m_bb.height = 100 + (rand() % 5);
    
    detection.m_pos_2D.x = detection.m_bb.x + (detection.m_bb.width / 2);
    detection.m_pos_2D.y = detection.m_bb.y + (detection.m_bb.height / 2);
    
    detection.m_class = unknown;
    detection.m_score = 0;
    
    objects.push_back(detection);
}

}

