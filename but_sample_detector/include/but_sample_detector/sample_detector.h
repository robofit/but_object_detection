/**
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Hodan (xhodan04@stud.fit.vutbr.cz)
 * Date: 01.04.2012 (version 1.0)
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 * Description:
 * Sample detector demonstrating how to wrap a detector using ObjDet API into ROS.
 *------------------------------------------------------------------------------
 */
 
#ifndef _SAMPLE_DETECTOR_
#define _SAMPLE_DETECTOR_

#include <opencv2/opencv.hpp>
#include "but_objdet/but_objdet.h"

using namespace std;


/**
 * @class SampleDetector
 */
class SampleDetector : public butObjectDetector {
	public:
        SampleDetector();	
		~SampleDetector();
	
	    // Methods from butObjectDetector abstract class
		void init( string config_filename );
		void setParam( int param_id, double value );
		double getParam( int param_id );
		void detect( const Mat& rgb, const Mat& depth, vector<butObject>& objects, int FLAGS );
		void prediction( vector<butObject>& objects, int FLAGS );
};

#endif // _SAMPLE_DETECTOR_

