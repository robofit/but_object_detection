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

/**
 * A class implementing tracking based on Kalman filter.
 *
 * @author Tomas Hodan, Vitezslav Beran (beranv@fit.vutbr.cz), Michal Spanel (spanel@fit.vutbr.cz)
 */
class TrackerKalman : public Tracker
{
public:
    TrackerKalman();
    virtual ~TrackerKalman();
    
	/**
     * Implementation of the virtual function from the Tracker abstract class.
     */
	bool init(const cv::Mat& measurement, bool secDerivate = true);

	/**
     * Implementation of the virtual function from the Tracker abstract class.
     */
	const cv::Mat& predict(int64 miliseconds = 1000);

    /**
     * Implementation of the virtual function from the Tracker abstract class.
     */
	const cv::Mat& update(const cv::Mat& measurement, int64 miliseconds = 1000);

private:
    /**
     * Modification of Kalman filter's transition matrix according to elapsed time.
     * It is used while predicting and updating the measurement.
     * @param miliseconds  Elapsed time.
     */
	void modifyTransMat(int64 miliseconds);

private:
	cv::KalmanFilter KF;
	cv::Mat temp;
	bool _secDerivate;
};

}

#endif // _TRACKER_KALMAN_

