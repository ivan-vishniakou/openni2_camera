# the same enum as defined in NiteCEnums.h of the Nite body tracker
uint8 JOINT_HEAD = 0
uint8 JOINT_NECK = 1
uint8 JOINT_LEFT_SHOULDER = 2
uint8 JOINT_RIGHT_SHOULDER = 3
uint8 JOINT_LEFT_ELBOW = 4
uint8 JOINT_RIGHT_ELBOW = 5
uint8 JOINT_LEFT_HAND = 6
uint8 JOINT_RIGHT_HAND = 7
uint8 JOINT_TORSO = 8
uint8 JOINT_LEFT_HIP = 9
uint8 JOINT_RIGHT_HIP = 10
uint8 JOINT_LEFT_KNEE = 11
uint8 JOINT_RIGHT_KNEE = 12
uint8 JOINT_LEFT_FOOT = 13
uint8 JOINT_RIGHT_FOOT = 14
uint8 JOINT_NUMBER = 15

uint8 SKELETON_NONE = 0
uint8 SKELETON_CALIBRATION_ERROR_NOT_IN_POSE = 1
uint8 SKELETON_CALIBRATING = 2
uint8 SKELETON_TRACKED = 3

uint8 user_id
uint8 state
geometry_msgs/Point position
NiteJoint[] joints