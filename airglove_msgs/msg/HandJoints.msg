uint8 SIDE_LEFT=0
uint8 SIDE_RIGHT=1
uint8 JOINT_COUNT=26
uint8 JOINT_FIELDS=8

std_msgs/Header header
uint8 side
uint32 pair
uint32 seq
float32[208] joints
bool[26] valid
