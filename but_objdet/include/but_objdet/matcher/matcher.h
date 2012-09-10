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
#ifndef _MATCHER_
#define _MATCHER_

#include "but_objdet/but_objdet.h"

namespace but_objdet
{

/**
  * A structure to represent a detection-prediction match.
  */
struct Match
{
    int detId; // Detection index
    int predId; // Prediction index
};

/**
  * A vector of detection-prediction matches.
  */
typedef std::vector<Match> Matches;


/**
 * An abstract class to be inherited by every matcher created using ObjDet API.
 * The purpose of a matcher is to find corresponding pairs (detection, prediction)
 * using some matching criterion (e.g. overlap of their bounding boxes).
 * 
 * Predictions are obtained from the previous detections and because we would like
 * to know if there is any new detection corresponding to the same object
 * as some of the predictions, we want to find the detection-prediction matches.
 *
 * @author Tomas Hodan, Vitezslav Beran (beranv@fit.vutbr.cz), Michal Spanel (spanel@fit.vutbr.cz)
 */
class Matcher
{
public:
    /**
     * A destructor of Matcher.
     */
	virtual ~Matcher() {}

    /**
     * Matching function.
     * @param detections  A vector of detections.
     * @param predictions  A vector of predictions.
     * @param matches  (output) A vector of detection-prediction matches.
     */
    virtual void match(const Objects &detections, const Objects &predictions, Matches &matches) = 0;
};

}

#endif // _MATCHER_
 
