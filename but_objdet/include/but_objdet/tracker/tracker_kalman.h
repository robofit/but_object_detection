/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: David Chrapek
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
#ifndef _TRACKER_KALMAN_
#define _TRACKER_KALMAN_

#include <opencv2/video/tracking.hpp>
#include "but_objdet/tracker/tracker.h"

namespace but_objdet
{

class TrackerKalman : public Tracker
{
public:
    TrackerKalman();
    virtual ~TrackerKalman();
    
	//Initialization. Have to be called before calling either predict or update
	//measurement parameter: expected matrix with one row containing parameters
	// to predict with values initialized from the first measurement
	//secDerivate parameter: specifies if the prediction is considerate with second
	// derivate (acceleration) of movement (default is with -> swecDerivate = true)
	// or just with the first derivate (velocity)
	bool init(const cv::Mat& measurement, bool secDerivate = true);

	//Given the miliseconds in the parameter it will return Mat with predicate
	//state after those miliseconds (just informative method: not neccesary to call it at all)
	const cv::Mat& predict(int64 miliseconds = 1000);

	//Given the next measurement (in measurement parameter) and miliseconds
	// (in miliseconds parameter) passed since last update
	// (or initialization if this is first update)
	//it will update its state and return the filtered estimate of true state
	//counted from the measurement and prediction
	const cv::Mat& update(const cv::Mat& measurement, int64 miliseconds = 1000);

private:
	void modifyTransMat(int64 miliseconds);

private:
	cv::KalmanFilter KF;
	cv::Mat temp;
	bool _secDerivate;
};

}

#endif // _TRACKER_KALMAN_
