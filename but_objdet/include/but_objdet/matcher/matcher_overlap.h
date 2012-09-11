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
#ifndef _MATCHER_OVERLAP_
#define _MATCHER_OVERLAP_

#include "but_objdet/but_objdet.h"
#include "but_objdet/matcher/matcher.h"

namespace but_objdet
{

/**
 * A class implementing matching of detections and predictions based on their
 * bounding boxes overlap.
 */
class MatcherOverlap : public Matcher
{
public:
    /**
     * MatcherOverlap constructor.
     * @param min  The bounding boxes are considered to be matching each other if
     * their overlapping area represents at least min% of each of them.
     */
	MatcherOverlap(float min=50);

    /**
     * A function to set the minimal overlap.
     * @param min  The bounding boxes are considered to be matching each other if
     * their overlapping area represents at least min% of each of them.
     */
	void setMinOverlap(float min=50);

	/**
     * Implementation of the virtual matching function from the Matcher abstract class.
     */
	void match(const Objects &detections, const Objects &predictions, Matches &matches);

private:
	float minOverlap;
};

}

#endif // _MATCHER_OVERLAP_

