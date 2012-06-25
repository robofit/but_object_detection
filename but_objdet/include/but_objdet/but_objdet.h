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
#ifndef _BUT_OBJDET_
#define _BUT_OBJDET_

#include <opencv2/opencv.hpp>
#include <vector>

#define BUT_OBJDET_GET_MASKS  1        // extract and store object masks
#define BUT_OBJDET_CONTINUOUS 2        // assume the adjacent following frames (for tracking, etc.)
#define BUT_OBJDET_FLAG_2     4

namespace but_objdet
{

enum ObjClass {
  unknown,
  head,
  chair,
  person,
  plant,
  lgv,
  bowl
};


struct Object
{
    int m_id;               // object identifier
    int m_class;            // object class
    float m_score;          // detection score (0.0, 1.0)

    cv::Point3f m_pos_2D;       // position in image + depth value
    cv::Rect    m_bb;           // bounding box in image
    cv::Mat     m_mask;         // object mask (CV_8U type)
    float       m_angle;        // object orientation
    cv::Point3f m_speed;        // changes in image and depth
};

typedef std::vector<Object> Objects;

class ObjectDetector
{
public:
	virtual ~ObjectDetector() {}

    virtual void init( std::string config_filename ) = 0;

    virtual void setParam( int param_id, double value ) = 0;
    virtual double getParam( int param_id ) = 0;

    virtual void detect( const cv::Mat& rgb, const cv::Mat& depth, Objects& objects, int FLAGS ) = 0;
    virtual void prediction( Objects& objects, int FLAGS ) = 0;
};

}

#endif // _BUT_OBJDET_

