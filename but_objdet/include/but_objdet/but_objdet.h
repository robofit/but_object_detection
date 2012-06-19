#ifndef _BUT_OBJDET_
#define _BUT_OBJDET_

#include <opencv2/opencv.hpp>

using namespace cv;

#define BUT_OBJDET_GET_MASKS  1        // extract and store object masks
#define BUT_OBJDET_CONTINUOUS 2        // assume the adjacent following frames (for tracking, etc.)
#define BUT_OBJDET_FLAG_2     4

enum butObjClass {
  unknown,
  head,
  chair,
  person,
  plant,
  lgv,
  bowl
};


struct butObject
{
    int m_id;               // object identifier
    int m_class;            // object class
    float m_score;          // detection score (0.0, 1.0)

    Point3f m_pos_2D;       // position in image + depth value
    Rect    m_bb;           // bounding box in image
    Mat     m_mask;         // object mask (CV_8U type)
    float   m_angle;        // object orientation
    Point3f m_speed;        // changes in image and depth
};


class butObjectDetector
{
    virtual void init( string config_filename ) = 0;

    virtual void setParam( int param_id, double value ) = 0;
    virtual double getParam( int param_id ) = 0;

    virtual void detect( const Mat& rgb, const Mat& depth, std::vector<butObject>& objects, int FLAGS ) = 0;
    virtual void prediction( std::vector<butObject>& objects, int FLAGS ) = 0;
};

#endif // _BUT_OBJDET_

