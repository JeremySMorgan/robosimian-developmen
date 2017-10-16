import math
import datetime

class RobotUtils(object):

    COLORS = {								# Unix codes for special priting
        "HEADER"		: 	{"value":'\033[95m', "code":0},
        "OKBLUE" 		: 	{"value":'\033[94m', "code":1},
        "OKGREEN" 		: 	{"value":'\033[92m', "code":2},
        "WARNING" 		: 	{"value":'\033[93m', "code":3},
        "FAIL" 			: 	{"value":'\033[91m', "code":4},
        "ENDC" 			: 	{"value":'\033[0m', "code":5},
        "BOLD" 			: 	{"value":'\033[1m', "code":6},
        "UNDERLINE" 	: 	{"value":'\033[4m', "code":7},
        "STANDARD"		:	{"value":'', "code":8}
	}


    # Link indexes
    TORSO_LINK_INDEX = 5
    f_r_active_dofs = [7, 8, 9, 10, 11, 12, 13]
    f_l_active_dofs = [31, 32, 33, 34, 35, 36, 37]
    b_r_active_dofs = [15,16,17,18,19,20,21]
    b_l_active_dofs = [23,24,25,26,27,28,29]


    # robot states
    FORWARD = "FORWARD"
    BACKWARD = "BACKWARD"
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    BASE_STATE = "BASE_STATE"

    F_R_FOOT = "F_R_FOOT"
    F_L_FOOT = "F_L_FOOT"
    B_R_FOOT = "B_R_FOOT"
    B_L_FOOT = "B_L_FOOT"

    LEG_BASE_STATE = "BASE_STATE"
    LEG_F_EXTEND_STATE = "LEG_F_EXTEND"
    LEG_B_EXTEND_STATE = "LEG_B_EXTEND"

    robot_states = [FORWARD, BACKWARD, LEFT, RIGHT]
    robot_turning_States = [LEFT, RIGHT]

    left_feet = [F_L_FOOT, B_L_FOOT]
    right_feet = [F_R_FOOT, B_R_FOOT]
    end_affectors = [ F_R_FOOT, F_L_FOOT, B_R_FOOT, B_L_FOOT]
    leg_states = [LEG_B_EXTEND_STATE,LEG_BASE_STATE,LEG_F_EXTEND_STATE]


    # App wide multithreading constants
    KEEP_THREAD_ALIVE                   = "thread_is_alive"

    # Gamepad input
    GAMEPAD_LEAST_SIGNIFICANT_INPUT     = .01

    # Objective Planner
    OBJECTIVE_PLANNER_UPDATE_DELAY     = .05

    # Debugging
    OBJ_PLANNER_DEBUGGING_ENABLED               = True
    GAMEPAD_DEBUGGING_ENABLED                   = False
    HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED = True
    MOTION_THREAD_DEBUGGING_ENABLED             = True

    # Delay Constants
    CONTROLLER_DT               = .01
    GAMEPAD_UPDATE_DELAY        = .01
    SIMULATION_FRAME_DELAY      = .01

    # Global IK constants
    IK_MAX_DEVIATION            = 1

    # Robot Configuration Constants
    LIMB1_START_CONFIG_OFFSET   = math.pi / 4

    # Reset Constants
    RESET_LEG_STEP_TIME         = 2
    MINIMUM_X_DELTA             = .005
    MINIMUM_Y_DELTA             = .005
    MINIMUM_Z_DELTA             = .005

    # Turn Constants
    TURN_TIME                   = 2
    TORSO_YAW_ROTATE_TIME       = 3
    TORSO_YAW_ROTATE_ANGLE      = 15

    # Step Constants
    TORSO_SHIFT_TIME            = 2
    STEP_X_DELTA                = .2
    STEP_Z_MAX_HIEGHT           = .15
    TORSO_SHIFT_DELTA           = STEP_X_DELTA
    STEP_TIME                   = 2

    # Simulation constants
    SIMULATION_ENABLED          = True


    @staticmethod
    def ColorPrinter( caller, message, color):
        # [03/Apr/2017 18:37:10]
        time = datetime.datetime.now()
        prefix = "["+str(time.day)+"/"+str(time.month)+ "/" + str(time.year) + " " + str(time.hour) + ":" + str(time.minute) + ":" + str(time.second) +  " ] "
        prefix = prefix +  RobotUtils.COLORS["BOLD"]["value"]+ RobotUtils.COLORS["UNDERLINE"]["value"]+ caller+ RobotUtils.COLORS["ENDC"]["value"]+ ":"
        print  prefix, RobotUtils.COLORS[color]["value"] ,message , RobotUtils.COLORS["ENDC"]["value"]

