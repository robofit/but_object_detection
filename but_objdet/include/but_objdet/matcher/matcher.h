/**
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Hodan (xhodan04@stud.fit.vutbr.cz)
 * Date: 01.04.2012 (version 1.0)
 *
 * License: BUT OPEN SOURCE LICENSE
 *------------------------------------------------------------------------------
 */

#ifndef _MATCHER_
#define _MATCHER_

#include "but_objdet/but_objdet.h"

using namespace std;


typedef struct {
    int detId; // Detection index
    int predId; // Prediction index
} TMatch;


class Matcher
{
    virtual void match(vector<butObject> &detections, vector<butObject> &predictions, vector<TMatch> &matches) = 0;
};

#endif // _MATCHER_
 
