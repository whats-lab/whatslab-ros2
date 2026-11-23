# Finger haptics command (Spine OSC message 9 / 10, /pair/N/{left,right}/hapt/set).
#
# strength: thumb, index, middle, ring, little. 0 off, 1 weak, 2 medium, 3 strong (clamped to 0..3).
# Publish continuously while the glove should vibrate: airglove_driver sends zeros when no command
# has arrived for its haptics_timeout (default 1 s), when no node publishes on this topic any more,
# and when it exits. Spine also stops when it has not heard the driver's heartbeat for 3 s.

uint8 OFF=0
uint8 WEAK=1
uint8 MEDIUM=2
uint8 STRONG=3

uint8[5] strength
