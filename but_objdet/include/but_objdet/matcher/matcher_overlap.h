/**
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Hodan (xhodan04@stud.fit.vutbr.cz)
 * Date: 01.04.2012 (version 1.0)
 *
 * License: BUT OPEN SOURCE LICENSE
 *------------------------------------------------------------------------------
 */
 
#ifndef _MATCHER_OVERLAP_
#define _MATCHER_OVERLAP_

#include "but_objdet/but_objdet.h"
#include "but_objdet/matcher/matcher.h"

using namespace std;

/**
 * @class MatcherOverlap
 */
class MatcherOverlap : public Matcher {
	public:	
	    MatcherOverlap(float min=50);
	    void setMinOverlap(float min=50);
	    
	    // Method from Matcher abstract class
		void match(vector<butObject> &detections, vector<butObject> &predictions, vector<TMatch> &matches);
		
    private:
        float minOverlap;
};

#endif // _MATCHER_OVERLAP_
