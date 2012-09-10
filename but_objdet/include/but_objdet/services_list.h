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
#ifndef BUT_OBJDET_SERVICES_LIST_H
#define BUT_OBJDET_SERVICES_LIST_H

namespace but_objdet
{
	/**
     * Name of a service to obtain predictions of detections (provided by tracker).
     */
	const std::string BUT_OBJDET_PredictDetections_SRV("/but_objdet/predict_detections");

	/**
     * Name of a service to obtain objects (provided by tracker).
     */
	const std::string BUT_OBJDET_GetObjects_SRV("/but_objdet/get_objects");
}

#endif // BUT_OBJDET_SERVICES_LIST_H
