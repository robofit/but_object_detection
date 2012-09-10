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
detection.m_angle = 0;

detection.m_mask = Mat(cvSize(0, 0), CV_8U);

detection.m_timestamp = 0;
detection.m_speed = cv::Point3f(0,0,0);



    
    objects.push_back(detection);
}

}

