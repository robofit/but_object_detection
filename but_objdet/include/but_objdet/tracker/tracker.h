/**
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Hodan (xhodan04@stud.fit.vutbr.cz)
 * Date: 01.04.2012 (version 1.0)
 *
 * License: BUT OPEN SOURCE LICENSE
 *------------------------------------------------------------------------------
 */

#ifndef _TRACKER_
#define _TRACKER_

#include <opencv2/opencv.hpp>
#include "but_objdet/but_objdet.h"

using namespace cv;


class Tracker
{
    virtual bool init(const Mat& measurement, bool secDerivate) = 0;
    virtual const Mat& predict(int64 miliseconds) = 0;
    virtual const Mat& update(const Mat& measurement, int64 miliseconds) = 0;
};

#endif // _TRACKER_

