#defined states
#the details to each state is detailed on the state_handler
NOTHING             = 0
ENABLE_VELODYME     = 1
ARM_CHANGING_POSE   = 2
FOUND_FIRE_FRONT    = 4
FOUND_FIRE_LEFT     = 5
FOUND_FIRE_RIGHT    = 6
FOUND_FIRE_TOUCH    = 7
SETTING_UP_HOKUYO   = 8
INITIAL_SETUP       = 9
ROBOT_ROTATION      = 10
ROBOT_CLOCKWISE     = 11
ROBOT_ANTICLOCKWISE = 12
HOKUYO_READING      = 13
FOUND_STAIR         = 14
LEAVING_FIRE        = 15
NARROW_PATH         = 16
END_STAIR           = 17
IN_STAIR            = 18
CLIMB_STAIR         = 19
BEAM_FIND           = 20
ROBOT_ON_THE_LEFT   = 21
FIRE_FOUND_BY_CAM   = 22
FORCE_VELODYME      = 23


#const for minimum angle difference betwwen the current angle and the desired angle
# used to calculates if a joint of the arm is in place
MAX_JOINT_ANGLE_DIFF = 0.05  


DEBUGGING = 1
PRINT_STATES = 1