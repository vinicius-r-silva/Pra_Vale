#defined states
_NOTHING           = 0
_ENABLE_VELODYME   = 1
_ARM_CHANGING_POSE = 2
_FOLLOW_TRACK      = 3
_FOUND_FIRE_FRONT  = 4
_FOUND_FIRE_LEFT   = 5
_FOUND_FIRE_RIGHT  = 6
_FOUND_FIRE_TOUCH  = 7
_SETTING_UP_HOKUYO = 8
_INITIAL_SETUP     = 9
_ROBOT_ROTATION    = 10
_ROBOT_ON_LEFT     = 11
_HOKUYO_READING    = 12
_HOKUYO_FOLLOWING  = 13
_FOUND_STAIR       = 14

#const for minimum angle difference betwwen the current angle and the desired angle
# used to calculates if a joint of the arm is in place
_MAX_JOINT_ANGLE_DIFF = 0.1


_TRACK_ON_RIGHT = 1
_TRACK_ON_FRONT = 2
_TRACK_ON_LEFT  = 3
_CLOCKWISE      = 4
_ANTI_CLOCKWISE = 5