/**
 * Developed by dcgm-robotics@FIT group
 * Author: David Chrapek
 * Date: 01.04.2012 (version 1.0)
 *
 * License: BUT OPEN SOURCE LICENSE
 *------------------------------------------------------------------------------
 */
 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include "but_objdet/tracker/tracker_kalman.h"

using namespace cv;


TrackerKalman::TrackerKalman()
{
}

TrackerKalman::~TrackerKalman()
{
}

bool TrackerKalman::init(const Mat& measurement, bool secDerivate)
{
	//only the values to predict can be accepet. so the measurement matrix have
	//to have vector (either row or column) of length at least 1 and type of CV_32F
	if(&measurement == NULL)
		return false;

	if(measurement.dims != 2)
		return false;

	int nParams;
	if(measurement.cols > measurement.rows)
	{
		nParams = measurement.cols;
		if((measurement.rows != 1) || (measurement.cols < 1) || (measurement.type() != CV_32F))
			return false;
	}
	else
	{
		nParams = measurement.rows;
		if((measurement.rows < 1) || (measurement.cols != 1) || (measurement.type() != CV_32F))
			return false;
	}

	_secDerivate = secDerivate;
	
	if(secDerivate)
	{
		//3*: for position, first (velocity) and second (acceleration) derivate
		KF.init(3 * nParams, measurement.cols);

		KF.transitionMatrix.create(3 * nParams, 3 * nParams, CV_32F);

		//setting up the translation matrix having 0.5 for acceleration, and 1 for
		//velocity and position as the movement equation is: x' = x + v + a/2
		KF.transitionMatrix.setTo(Scalar(0));
		for(int i = 0; i < (3 * nParams); i++)
			KF.transitionMatrix.at<float>(i, i) = 1.0;
		for(int i = 0; i < (2 * nParams); i++)
			KF.transitionMatrix.at<float>(i, i + nParams) = 1.0;
		for(int i = 0; i < nParams; i++)
			KF.transitionMatrix.at<float>(i, i + 2 * nParams) = 0.5;
	}
	else
	{
		//2*: for position and first (velocity) derivate 
		KF.init(2 * nParams, nParams);

		KF.transitionMatrix.create(2 * nParams, 2 * nParams, CV_32F);

		//setting up the translation matrix having 1 for velocity and position
		//as the movement equation is: x' = x + v
		KF.transitionMatrix.setTo(Scalar(0));
		for(int i = 0; i < (2 * nParams); i++)
			KF.transitionMatrix.at<float>(i,i) = 1.0;
		for(int i = 0; i < nParams; i++)
			KF.transitionMatrix.at<float>(i,i + nParams) = 1.0;
	}

	//setting the initialized state from which the first prediction will be counted
	KF.statePost.setTo(Scalar(0));
	for(int i = 0; i < nParams; i++)
		KF.statePost.at<float>(i) = measurement.at<float>(i);

	setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, Scalar::all(.1));

	temp.create(1, nParams, CV_32F);

	return true;
}

//it will modify the trans. matrix according to time elapsed
void TrackerKalman::modifyTransMat(int64 miliseconds)
{
	//number of predicted parameters (only the position, neither the velocity
	// nor the acceleration)
	int nParams;
	//the factor for the translation (could be without / 1000.0, its just normalization
	// so that whole process is in seconds (but ultimately its irrelevant, if not used anywhere)
	float factor = miliseconds / 1000.0;
	//in case of second derivate
	if(_secDerivate)
	{
		//1/3 of size of trans. matrix is the predicted parameters, couse the other
		//thirds are the 1st and 2nd derivate
		nParams = KF.transitionMatrix.cols / 3;

		//need to put the factor in only at position of velocity and acceleration
		//for the rows expressing position
		for(int i = 0; i < nParams; i++)
		{
			KF.transitionMatrix.at<float>(i, i + nParams) = factor;
			KF.transitionMatrix.at<float>(i, i + 2 * nParams) = 0.5 * factor;
		}
		//need to put the factor in only at position of acceleration for the rows
		//expressing acceleration
		for(int i = 0; i < nParams; i++)
		{
			KF.transitionMatrix.at<float>(i + nParams, i + 2 * nParams) = factor;
		}
	}
	//in case of only first derivate
	else
	{
		//1/2 of size of trans. matrix is the predicted parameters, couse the other
		//half are the 1st derivate
		nParams = KF.transitionMatrix.cols / 2;
		//need to put the factor in only at position of velocity for the rows 
		//expressing position
		for(int i = 0; i < nParams; i++)
		{
			KF.transitionMatrix.at<float>(i, i + nParams) = factor;
		}
	}
}

const Mat& TrackerKalman::predict(int64 miliseconds)
{
	//has to modify the trans. matrix of kalman to acount for the time passed
	modifyTransMat(miliseconds);
	temp = KF.transitionMatrix * KF.statePost;
	return temp;
}

const Mat& TrackerKalman::update(const Mat& measurement, int64 miliseconds)
{
	//has to modify the trans. matrix of kalman to acount for the time passed
	modifyTransMat(miliseconds);
	KF.predict();

	return KF.correct(measurement.t());
}



/*
example transition matrix for ""x" and "y" as predicted parameters with 1st and 2nd derivate

dx = first derivate of x => velocity of x
ddx = second derivate of x => acceleration of x
x' = the next step of x

only 1st. derivate (equation is: x' = x + dx):
x	0	dx	0	=	1	0	1	0	expressing: x' = x + dx
0	y	0	dy	=	0	1	0	1	expressing: y' = y + dy
0	0	dx	0	=	0	0	1	0	expressing: dx' = dx
0	0	0	dy	=	0	0	0	1	expressing: dy' = dy

2nd. derivate (equations are: x' = x + dx + ddx/2 and dx' = dx + ddx):
x	0	dx	0	ddx/2	0		=	1	0	1	0	0.5	0	expressing: x' = x + dx	+ ddx/2
0	y	0	dy	0		ddy/2	=	0	1	0	1	0	0.5	expressing: y' = y + dy	+ ddy/2
0	0	dx	0	ddx		0		=	0	0	1	0	1	0	expressing: dx' = dx + ddx
0	0	0	dy	0		ddy		=	0	0	0	1	0	1	expressing: dy' = dy +ddy
0	0	0	0	ddx		0		=	0	0	0	0	1	0	expressing: dxx' = ddx
0	0	0	0	0		ddy		=	0	0	0	0	0	1	expressing: dyy' = ddy
*/
