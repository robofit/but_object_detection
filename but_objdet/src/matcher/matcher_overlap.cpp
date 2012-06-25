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
 
#include "but_objdet/matcher/matcher_overlap.h"
 
using namespace std;
 

namespace but_objdet
{
 
/* -----------------------------------------------------------------------------
 * Constructor
 */
 MatcherOverlap::MatcherOverlap(float min)
 {
    minOverlap = min;
 }
 
 
/* -----------------------------------------------------------------------------
 * Matching function
 *
 * A detection and a prediction are considered as similar, if they are of the
 * same class (m_class) and their overlapping area represents at least minOverlap%
 * of each of them.
 */
void MatcherOverlap::match(const Objects &detections, const Objects &predictions, Matches &matches)
{
    matches.resize(detections.size());
    
    // Take each detection and find the most overlapping prediction
    for(unsigned int i = 0; i < detections.size(); i++) {
    
        // Get left/right X and top/bottom Y coordinates of detection BB
        int detLeftX = detections[i].m_bb.x;
        int detRightX = detections[i].m_bb.x + detections[i].m_bb.width;
        int detTopY = detections[i].m_bb.y;
        int detBottomY = detections[i].m_bb.y + detections[i].m_bb.height;
        
        // Area of detection BB
        float detArea = detections[i].m_bb.width * detections[i].m_bb.height;
        
        float bestOverlapped = 0; // The best overlapping percentage so far
        int bestPredId = -1; // The most similar prediction so far
        
        // Go through all predictions and find the most similar one
        for(unsigned int j = 0; j < predictions.size(); j++) {
        
            // If the prediction is not from the same class, do not consider it
            if(detections[i].m_class != predictions[j].m_class) continue;
        
            float overlapped = 0; // Overlapping percentage (the smaller one of both BBs)
        
            // Get left/right X and top/bottom Y coordinates of prediction BB
            int predLeftX = predictions[j].m_bb.x;
            int predRightX = predictions[j].m_bb.x + predictions[j].m_bb.width;
            int predTopY = predictions[j].m_bb.y;
            int predBottomY = predictions[j].m_bb.y + predictions[j].m_bb.height;
            
            // Area of prediction BB
            float predArea = predictions[i].m_bb.width * predictions[j].m_bb.height;
            
            // Test if detection BB overlaps with prediction BB
            if(detRightX > predLeftX && detLeftX < predRightX && // Test if the BBs overlap in X direction
               detBottomY > predTopY && detTopY < predBottomY) { // Test if the BBs overlap in Y direction
                
                // Get left/right X and top/bottom Y coordinates of the overlapped region
                int overlapLeftX = max(detLeftX, predLeftX);
                int overlapRightX = min(detRightX, predRightX);
                int overlapTopY = max(detTopY, predTopY);
                int overlapBottomY = min(detBottomY, predBottomY);
                
                // Calculate area of the overlapped region
                float overlapArea = (overlapRightX - overlapLeftX) * (overlapBottomY - overlapTopY);
                
                // Calculate how many percent of detection BB is overlapped
                // (do the same also for predicition BB)
                float detOverlapped = (overlapArea * 100) / detArea;
                float predOverlapped = (overlapArea * 100) / predArea;
                
                // Overlapping area must represent more than minOverlap%
                // (for both, detection BB and prediction BB)
                if(detOverlapped >= minOverlap && predOverlapped >= minOverlap) {
                   overlapped = min(detOverlapped, predOverlapped);
                }
            }
            
            // Test if this prediction is the best so far
            if(overlapped > bestOverlapped) {
                bestOverlapped = overlapped;
                bestPredId = j;
            }
        }
        
        // Save the match with the most similar prediction
        matches[i].detId = i;
        matches[i].predId = bestPredId;
    }

}


/* -----------------------------------------------------------------------------
 * Sets minimum overlap (in percent) which must be between a detection
 * and a prediction to be considered as similar (the overlapping
 * area must represent at least minOverlap% of each of them).
 */
 void MatcherOverlap::setMinOverlap(float min)
 {
    minOverlap = min;
 }
 
 }

