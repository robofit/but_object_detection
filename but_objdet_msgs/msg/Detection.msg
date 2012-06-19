
# This message is supposed to be used to transfer an object of butObject class,
# thus it contains the same items.
#-------------------------------------------------------------------------------
Header header

int32                 m_id     # object identifier
int32                 m_class  # object class
float32               m_score  # detection score (0.0, 1.0) 
geometry_msgs/Point32 m_pos_2D # position in image and depth value 
Rect                  m_bb     # bounding box in image
sensor_msgs/Image     m_mask   # object mask
float32               m_angle  # object orientation
geometry_msgs/Point32 m_speed  # changes in image and depth
