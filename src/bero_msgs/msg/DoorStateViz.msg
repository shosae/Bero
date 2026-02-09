std_msgs/Header header

sensor_msgs/Image image 

string status  # "opened", "closed", "moving", "unknown", "disabled"

int32 count_opened
int32 count_closed
int32 count_moving

int32 bbox_x1
int32 bbox_y1
int32 bbox_x2
int32 bbox_y2
string bbox_class
float32 bbox_conf
