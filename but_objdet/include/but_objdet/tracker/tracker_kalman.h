/**
 * Developed by dcgm-robotics@FIT group
 * Author: David Chrapek
 * Date: 01.04.2012 (version 1.0)
 *
 * License: BUT OPEN SOURCE LICENSE
 *------------------------------------------------------------------------------
 */

#ifndef _TRACKER_KALMAN_
#define _TRACKER_KALMAN_

#include <opencv2/video/tracking.hpp>
#include "but_objdet/tracker/tracker.h"

using namespace cv;


class TrackerKalman : public Tracker
{
public:
    TrackerKalman();
    ~TrackerKalman();
    
	//Initialization. Have to be called before calling either predict or update
	//measurement parameter: expected matrix with one row containing parameters
	// to predict with values initialized from the first measurement
	//secDerivate parameter: specifies if the prediction is considerate with second
	// derivate (acceleration) of movement (default is with -> swecDerivate = true)
	// or just with the first derivate (velocity)
	bool init(const Mat& measurement, bool secDerivate = true);

	//Given the miliseconds in the parameter it will return Mat with predicate
	//state after those miliseconds (just informative method: not neccesary to call it at all)
	const Mat& predict(int64 miliseconds = 1000);

	//Given the next measurement (in measurement parameter) and miliseconds
	// (in miliseconds parameter) passed since last update
	// (or initialization if this is first update)
	//it will update its state and return the filtered estimate of true state
	//counted from the measurement and prediction
	const Mat& update(const Mat& measurement, int64 miliseconds = 1000);

private:
	void modifyTransMat(int64 miliseconds);

private:
	KalmanFilter KF;
	Mat temp;
	bool _secDerivate;
};

#endif // _TRACKER_KALMAN_
