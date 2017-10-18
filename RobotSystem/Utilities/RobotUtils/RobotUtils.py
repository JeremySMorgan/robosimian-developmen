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

    # Debugging
    OBJ_PLANNER_DEBUGGING_ENABLED               = False
    GAMEPAD_DEBUGGING_ENABLED                   = False
    HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED = True
    MOTION_THREAD_DEBUGGING_ENABLED             = False

    # Delay Constants
    CONTROLLER_DT                       = .01
    GAMEPAD_UPDATE_DELAY                = .01
    OBJECTIVE_PLANNER_UPDATE_DELAY      = .05

    # Global IK constants
    IK_MAX_DEVIATION                    = .25

    # Robot Configuration Constants
    LIMB1_START_CONFIG_OFFSET           = math.pi / 3

    # Reset Constants
    MINIMUM_DIST_TO_CAUSE_RESET         = .05
    MINIMUM_X_DELTA                     = .01
    MINIMUM_Y_DELTA                     = .01
    MINIMUM_Z_DELTA                     = .01

    # Turn Constants
    TORSO_YAW_ROTATE_ANGLE              = 15

    # Step Constants
    STEP_X_DELTA                        = .15
    STEP_Z_MAX_HIEGHT                   = .05
    TORSO_SHIFT_DELTA                   = STEP_X_DELTA
    TORSO_LEFT_SHIFT                    = .075

    # Simulation constants
    SIMULATION_ENABLED                  = True
    PHYSICS_ENABLED                     = True
    INCLUDE_TERRAIN                     = True

    # Movement timing constants
    if PHYSICS_ENABLED:
        RESET_LEG_STEP_TIME                 = 3
        TURN_TIME                           = 5
        TORSO_SHIFT_TIME                    = 4
        STEP_TIME                           = 6
        TORSO_YAW_ROTATE_TIME               = 3

    else:
        RESET_LEG_STEP_TIME                 = 1
        TURN_TIME                           = 1
        TORSO_SHIFT_TIME                    = 1
        STEP_TIME                           = 1
        TORSO_YAW_ROTATE_TIME               = 1


    @staticmethod
    def ColorPrinter( caller, message, color):
        # [03/Apr/2017 18:37:10]
        time = datetime.datetime.now()

        if color not in RobotUtils.COLORS:
            color = RobotUtils.COLORS.get("STANDARD").get("value")

        prefix = "["+str(time.day)+"/"+str(time.month)+ "/" + str(time.year) + " " + str(time.hour) + ":" + str(time.minute) + ":" + str(time.second) +  " ] "
        prefix = prefix +  RobotUtils.COLORS["BOLD"]["value"]+ RobotUtils.COLORS["UNDERLINE"]["value"]+ caller+ RobotUtils.COLORS["ENDC"]["value"]+ ":"
        print  prefix, RobotUtils.COLORS[color]["value"] ,message , RobotUtils.COLORS["ENDC"]["value"]

    @staticmethod
    def get_euclidian_diff(xyz1, xyz2):

        d_x = (xyz1[0] - xyz2[0])**2
        d_y = (xyz1[1] - xyz2[1])**2
        d_z = (xyz1[2] - xyz2[2])**2

        return math.sqrt( d_x +d_y  +d_z    )


    @staticmethod
    def always_true_func():
        return True
