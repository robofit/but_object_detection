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

/**
 * An abstract class to be inherited by every tracker created using ObjDet API.
 * The purpose of a tracker is to predict the next state (at a requested time)
 * of a some measurement (typically of a bounding box size and position).
 *
 * @author Tomas Hodan, Vitezslav Beran (beranv@fit.vutbr.cz), Michal Spanel (spanel@fit.vutbr.cz)
 */
class Tracker
{
public:
	virtual ~Tracker() {}

    /**
     * Initialization, which have to be called before calling either predict or update.
	 * @param measurement  A matrix with one row containing parameters (to be later predicted)
	 * with values initialized from the first measurement.
	 * @param secDerivate  Specifies if the prediction is considerate with second
	 * derivate (acceleration) of movement (secDerivate = true)
	 * or just with the first derivate (velocity)
	 * @return  True if the initialization was successful, Fasle otherwise.
     */
    virtual bool init(const cv::Mat& measurement, bool secDerivate) = 0;

    /**
     * Prediction of the measurement next state.
     * @param miliseconds  Time (= number of miliseconds passed since the last update)
     * for which the next state should be predicted.
     * @return  Prediction for the requested time.
     */
    virtual const cv::Mat& predict(int64 miliseconds) = 0;

    /**
     * Update the measurement.
     * @param measurement  New measurement to be added into account.
     * @param miliseconds  Miliseconds passed since the last update.
     * @return  Updates its state and returns the filtered estimate of true state
	 * counted from the measurement and prediction.
	*/
    virtual const cv::Mat& update(const cv::Mat& measurement, int64 miliseconds) = 0;
};

}

#endif // _TRACKER_

