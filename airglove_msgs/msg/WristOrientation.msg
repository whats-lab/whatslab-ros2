# Spine 3.0.0 wrist value (OSC message 18 / 19, /pair/N/{left,right}/wrist/get), as Spine sends it.
#
# The raw back-of-hand (dorsum) IMU quaternion: the sensor's [w, x, y, z] reordered by Spine to
# [y, x, z, -w]. Not normalised, not converted to any world or ROS frame. It is NOT a wrist pose;
# use it only together with your own sensor-to-world alignment.

uint8 SIDE_LEFT=0
uint8 SIDE_RIGHT=1

std_msgs/Header header
uint8 side
uint32 pair
uint32 seq
float32[4] raw         # [y, x, z, -w] of the dorsum IMU
