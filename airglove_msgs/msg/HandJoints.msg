# Spine 3.0.0 hand joints (OSC message 1 / 2, /pair/N/{left,right}/quat/get), as Spine sends them.
#
# 26 OpenXR XrHandJointEXT joints (PALM, WRIST, THUMB_METACARPAL .. LITTLE_TIP), each 8 floats:
#   joints[j*8 + 0..3] = qx qy qz qw   orientation
#   joints[j*8 + 4..6] = px py pz      position, metres
#   joints[j*8 + 7]    = radius        metres
# Relative to the wrist (WRIST = identity), OpenXR hand-joint axes for both hands:
# -Z along the bone towards the fingertip, +Y on the back of the hand, X = Y x Z.
#
# valid[j] is false when Spine sent a non-finite / non-normalisable value for joint j; that
# joint is then reported as identity at the origin.
#
# header.stamp: time the driver received the frame (Spine sends no timestamp).
# header.frame_id: "airglove_<side>_wrist".
# Field layout matches robotPlatform platform_msgs/HandJoints (+ pair).

uint8 SIDE_LEFT=0
uint8 SIDE_RIGHT=1
uint8 JOINT_COUNT=26
uint8 JOINT_FIELDS=8

std_msgs/Header header
uint8 side
uint32 pair            # Spine pair (/pair/N)
uint32 seq             # frames received for this hand by the driver (Spine sends no sequence number)
float32[208] joints
bool[26] valid
