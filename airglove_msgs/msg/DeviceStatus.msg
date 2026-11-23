# Spine 3.0.0 device status (OSC message 3, reply to the driver's status request every second)
# plus the Spine heartbeat (OSC message 14).

std_msgs/Header header
uint32 pair
bool left_connected
bool right_connected
int32 spine_protocol         # 3 for Spine 3.0.0; 0 when Spine has not answered the heartbeat
float32 spine_age            # seconds since the last heartbeat reply; -1 when none yet
