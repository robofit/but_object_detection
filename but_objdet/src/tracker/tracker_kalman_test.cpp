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

#include <iostream>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include "but_objdet/tracker/tracker_kalman.h"

using namespace cv;
using namespace std;


#define PAUSE_LENGTH 100

struct rect_info_struct { float x, y, w, h, noisyX, noisyY, noisyW, noisyH, dx, dy, dw, dh, ddx, ddy, ddw, ddh; };
struct rect_info_struct rect_info;

vector<Rect> rectHistory,kalmanHistory;

//just to simulate the next measurement
void update(int rate)
{
	if(rand() % (3000 / PAUSE_LENGTH) == 1)	//treba jedne..je ejdno..potebuji jednou ya tri sekundy
	{
		rect_info.ddx += (rand() % 5) - 2;
		if(rect_info.ddx > 2)
			rect_info.ddx = 2;
		if(rect_info.ddx < -2)
			rect_info.ddx = -2;
	}

	//abz nevzsle y obrazu
	if(rect_info.x < 10)
	{
		rect_info.dx = 0;
		rect_info.ddx = 1;
	}
	if(rect_info.x > 490)
	{
		rect_info.dx = 0;
		rect_info.ddx = -1;
	}

	rect_info.dx += ((rate / 1000.0) * rect_info.ddx);
	rect_info.x += ((rate / 1000.0) * (rect_info.dx + rect_info.ddx / 2));

	if(rand() % (3000 / PAUSE_LENGTH) == 1)	//treba jedne..je ejdno..potebuji jednou ya tri sekundy
	{
		rect_info.ddy += (rand() % 5) - 2;
		if(rect_info.ddy > 2)
			rect_info.ddy = 2;
		if(rect_info.ddy < -2)
			rect_info.ddy = -2;
	}

	//abz nevzsle y obrazu
	if(rect_info.y < 10)
	{
		rect_info.dy = 0;
		rect_info.ddy = 1;
	}
	if(rect_info.y > 490)
	{
		rect_info.dy = 0;
		rect_info.ddy = -1;
	}

	rect_info.dy += ((rate / 1000.0) * rect_info.ddy);
	rect_info.y += ((rate / 1000.0) * (rect_info.dy + rect_info.ddy / 2));

	if(rand() % (3000 / PAUSE_LENGTH) == 1)	//treba jedne..je ejdno..potebuji jednou ya tri sekundy
	{
		rect_info.ddw += ((rand() % 3) - 1) / 5.0;
		if(rect_info.ddw > 2)
			rect_info.ddw = 2;
		if(rect_info.ddw < -2)
			rect_info.ddw = -2;
	}

	//abz nevzsle y obrazu
	if(rect_info.w < 20)
	{
		rect_info.w = 20;
		rect_info.dw = 0;
		rect_info.ddw = 1;
	}
	if(rect_info.w > 70)
	{
		rect_info.w = 70;
		rect_info.dw = 0;
		rect_info.ddw = -1;
	}

	rect_info.dw += ((rate / 1000.0) * rect_info.ddw);
	rect_info.w += ((rate / 1000.0) * (rect_info.dw + rect_info.ddw / 2));

	if(rand() % (3000 / PAUSE_LENGTH) == 1)	//treba jedne..je ejdno..potebuji jednou ya tri sekundy
	{
		rect_info.ddh += ((rand() % 3) - 1) / 5.0;
		if(rect_info.ddh > 2)
			rect_info.ddh = 2;
		if(rect_info.ddh < -2)
			rect_info.ddh = -2;
	}

	//abz nevzsle y obrazu
	if(rect_info.h < 20)
	{
		rect_info.h = 20;
		rect_info.dh = 0;
		rect_info.ddh = 1;
	}
	if(rect_info.h > 70)
	{
		rect_info.h = 70;
		rect_info.dh = 0;
		rect_info.ddh = -1;
	}

	rect_info.dh += ((rate / 1000.0) * rect_info.ddh);
	rect_info.h += ((rate / 1000.0) * (rect_info.dh + rect_info.ddh / 2));

	rect_info.noisyX = (rand() % 41) - 20;
	rect_info.noisyX += rect_info.x;
	rect_info.noisyY = (rand() % 41) - 20;
	rect_info.noisyY += rect_info.y;
	rect_info.noisyW = (rand() % 41) - 20;
	rect_info.noisyW += rect_info.w;
	rect_info.noisyH = (rand() % 41) - 20;
	rect_info.noisyH += rect_info.h;
}

int main (int argc, char * const argv[]) {
    Mat img(500, 500, CV_8UC3);
    Mat_<float> state(12, 1); //x, y, w, h, dx, dy, dw, dh, ddx, ddy, ddw, ddh ;w = width; h = height; d = derivation 
    Mat processNoise(12, 1, CV_32F);
    Mat_<float> measurement(4,1); measurement.setTo(Scalar(0));
    char code = (char)-1;
	
	namedWindow("kalman");
	
	srand ( time(NULL) );

    for(;;)
    {
		rect_info.x = img.cols / 2.0;
		rect_info.y = img.rows / 2.0;
		rect_info.w = 50;
		rect_info.h = 50;
		rect_info.dx = 0;
		rect_info.dy = 0;
		rect_info.dw = 0;
		rect_info.dh = 0;
		rect_info.ddx = 0;
		rect_info.ddy = 0;
		rect_info.ddw = 0;
		rect_info.ddh = 0;
		Mat initialPos(1, 4, CV_32F);

		//first measurement
		update(PAUSE_LENGTH);

		//initial position..the first measurement
		initialPos.at<float>(0) = rect_info.noisyX;
		initialPos.at<float>(1) = rect_info.noisyY;
		initialPos.at<float>(2) = rect_info.noisyW;
		initialPos.at<float>(3) = rect_info.noisyH;

		//creating the kalman and initialize it with the initial measurement and the desire to predict it with 2nd derivate
		TrackerKalman mKF;
		mKF.init(initialPos, true);

		//just vectors for painting the history
		rectHistory.clear();
		kalmanHistory.clear();
		
        for(;;)
        {
			int measurementTime = (rand() % 300) + 100;

			//update the simulation so in real situation: "get a new measurements"
			update(measurementTime);


			//not modifying the state of filter, just the precdiction what it will look like in given time
            Mat prediction = mKF.predict(measurementTime);

			//prediction rectangle just for visualization: later drawn on screen
            Rect predictRect(prediction.at<float>(0),prediction.at<float>(1), prediction.at<float>(2), prediction.at<float>(3));
			
			//here is simulation of the new measurement
            measurement(0) = rect_info.noisyX;
			measurement(1) = rect_info.noisyY;
			measurement(2) = rect_info.noisyW;
			measurement(3) = rect_info.noisyH;
			
			//measured rectangle just for later visualization
			Rect measRect(measurement(0), measurement(1), measurement(2), measurement(3));

			//just for drawing the history
			rectHistory.push_back(measRect);

			//calling update with the new measured values and the time from the previous call of update (or from init) the return is the final estimate
			Mat estimated = mKF.update(measurement, PAUSE_LENGTH);

			//storing the estimate to drawn it later
			Rect stateRect(estimated.at<float>(0), estimated.at<float>(1), estimated.at<float>(2), estimated.at<float>(3));

			//just for drawing the history line
			kalmanHistory.push_back(stateRect);
	
			//drawing the rectangle: measured (noisy) is red, the ideal is green, predicted is blue and the final estimate is white
            img = Scalar::all(0);
			rectangle(img, Rect(predictRect.x - predictRect.width / 2, predictRect.y - predictRect.height / 2, predictRect.width, predictRect.height), Scalar(255, 0, 0), 3);
			rectangle(img, Rect(stateRect.x - stateRect.width / 2, stateRect.y - stateRect.height / 2, stateRect.width, stateRect.height), Scalar(255, 255, 255), 3);
			rectangle(img, Rect(measRect.x - measRect.width / 2, measRect.y - measRect.height / 2, measRect.width, measRect.height), Scalar(0, 0, 255), 3);
			rectangle(img, Rect(rect_info.x - rect_info.w / 2, rect_info.y - rect_info.h / 2, rect_info.w, rect_info.h), Scalar(0, 255, 0), 3);

			//drawing the history lines
			for (unsigned i = 0; (int)i < (int)rectHistory.size()-1; i++)
			{
				line(img, Point(rectHistory[i].x, rectHistory[i].y), Point(rectHistory[i+1].x, rectHistory[i+1].y), Scalar(255,255,0), 1);
			}
			for (unsigned i = 0; (int)i < (int)kalmanHistory.size()-1; i++)
			{
				line(img, Point(kalmanHistory[i].x, kalmanHistory[i].y), Point(kalmanHistory[i+1].x, kalmanHistory[i+1].y), Scalar(0,255,0), 1);
			}
			
			//show the results
            imshow( "kalman", img );

			//wait a while for next run
            code = (char)waitKey(measurementTime);
			
            if( code > 0 )
                break;
        }
        if( code == 27 || code == 'q' || code == 'Q' )
            break;
    }
	
    return 0;
}



///////////////END TEST KALMAN
