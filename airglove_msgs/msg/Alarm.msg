# Spine 3.0.0 device alarm (OSC message 13, /pair/N/device/alarm/get).

uint8 SIDE_LEFT=0
uint8 SIDE_RIGHT=1
uint8 SIDE_UNKNOWN=255           # Spine 이 손을 지정하지 않은 알람

std_msgs/Header header
uint32 pair
string code                  # e.g. "RESTART", "ATLAS_DEVICE_ERROR"
uint8 side
