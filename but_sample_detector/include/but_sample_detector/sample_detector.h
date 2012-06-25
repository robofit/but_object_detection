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
#ifndef _SAMPLE_DETECTOR_
#define _SAMPLE_DETECTOR_

#include <opencv2/opencv.hpp>
#include "but_objdet/but_objdet.h"


namespace but_sample_detector
{

/**
 * @class SampleDetector
 */
class SampleDetector : public but_objdet::ObjectDetector
{
public:
	SampleDetector();
	virtual ~SampleDetector();

	// Methods from ObjectDetector abstract class
	void init( std::string config_filename );

	void setParam( int param_id, double value );

	double getParam( int param_id );

	void detect( const cv::Mat& rgb, const cv::Mat& depth, but_objdet::Objects& objects, int FLAGS );
	
	void prediction( but_objdet::Objects& objects, int FLAGS );
};

}

#endif // _SAMPLE_DETECTOR_

