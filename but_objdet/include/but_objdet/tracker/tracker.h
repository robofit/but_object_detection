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
#ifndef _TRACKER_
#define _TRACKER_

#include <opencv2/opencv.hpp>
#include "but_objdet/but_objdet.h"

namespace but_objdet
{

class Tracker
{
public:
	virtual ~Tracker() {}

    virtual bool init(const cv::Mat& measurement, bool secDerivate) = 0;

    virtual const cv::Mat& predict(int64 miliseconds) = 0;

    virtual const cv::Mat& update(const cv::Mat& measurement, int64 miliseconds) = 0;
};

}

#endif // _TRACKER_

