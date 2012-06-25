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
 * @class MatcherOverlap
 */
class MatcherOverlap : public Matcher
{
public:
	MatcherOverlap(float min=50);

	void setMinOverlap(float min=50);

	// Method from Matcher abstract class
	void match(const Objects &detections, const Objects &predictions, Matches &matches);

private:
	float minOverlap;
};

}

#endif // _MATCHER_OVERLAP_
