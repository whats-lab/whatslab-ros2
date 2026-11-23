# Result of the last haptics command (Spine OSC message 11 / 12, /pair/N/{left,right}/hapt/ret/get).
# success is false when the glove is not connected or still busy with the previous command.

uint8 SIDE_LEFT=0
uint8 SIDE_RIGHT=1

std_msgs/Header header
uint32 pair
uint8 side
bool success
