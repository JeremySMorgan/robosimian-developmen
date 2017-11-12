from __future__ import print_function
import math
import datetime
import numpy as np

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

    robot_states = [FORWARD, BACKWARD, LEFT, RIGHT]
    robot_turning_States = [LEFT, RIGHT]

    left_feet     = [F_L_FOOT, B_L_FOOT ]
    right_feet    = [F_R_FOOT, B_R_FOOT ]
    end_affectors = [F_R_FOOT, F_L_FOOT, B_R_FOOT, B_L_FOOT]

    # App wide multithreading constants
    KEEP_THREAD_ALIVE                   = "thread_is_alive"

    # Gamepad input
    GAMEPAD_LEAST_SIGNIFICANT_INPUT     = .01

    # Debugging
    OBJ_PLANNER_DEBUGGING_ENABLED                = False
    USER_INPUT_DEBUGGING_ENABLED                 = False
    HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED  = True

    # Delay Constants
    CONTROLLER_DT                       = .01
    GAMEPAD_UPDATE_DELAY                = .01
    OBJECTIVE_PLANNER_UPDATE_DELAY      = .05

    # Global IK constants
    IK_MAX_DEVIATION                    = .615

    # Base State Parameters
    STARTING_CONFIG                     = [ 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.05424, 0.66479, 0.07701, -1.79261, -0.13906, -0.44839, 0.12875, 0.00000, 0.08129, -0.72312, -0.03797, 2.07474, 0.12972, 0.22139, -0.22298, 0.00000, -0.07511, 0.72165, 0.03403, -2.07091, -0.11534, -0.22331, 0.21007, 0.00000, 0.08189, -0.70402, -0.03012, 2.07355, 0.11415, 0.20279, -0.21321]
    BASE_STATE_X_DELTA                  = .32
    BASE_STATE_Y_DELTA                  = BASE_STATE_X_DELTA
    BASE_STATE_Z_DELTA                  = -.5 # This value is the delta z from the torso to the ground (/end affectors center point)

    # Robot Measurements
    SHOULDER_X                          = .293260631869
    SHOULDER_Y                          = .206456211
    SHOULDER_Z                          = -0.2085299999988
    LEG_LENGTH                          = .775
    SHOULDER_TORSO_XY_EUCLIDEAN_DIF     = 0.3603472707
    DELTA_Z_SHOULDER_END_AFFECTOR       = 0.980182 * (BASE_STATE_Z_DELTA) + .20965
    END_AFFECTOR_RADIUS_TO_SHOULDER     = math.sqrt(LEG_LENGTH**2 - DELTA_Z_SHOULDER_END_AFFECTOR**2)
    SHOULDER_TORSO_PSI_RADS             = .620089205322
    FORWARD_BACK_LEGS_EUCLIDEAN_DIST    = 2 * (SHOULDER_X + BASE_STATE_X_DELTA)
    LEFT_RIGHT_LEGS_EUCLIDEAN_DIST      = 2 * (SHOULDER_Y + BASE_STATE_Y_DELTA)

    # Visualization - determines if _2DSupport Objects are shown
    RESET_VISUALIZATION_ENABLED         = True
    FORWARD_STEP_VISUALIZATION_ENABLED  = True

    # Reset Constants
    MINIMUM_DIST_TO_CAUSE_RESET         = .001
    MINIMUM_X_DELTA                     = .05
    MINIMUM_Y_DELTA                     = .05
    MINIMUM_Z_DELTA                     = .015      # all z deltas above this will trigger a reset
    MAX_ALLOWABLE_END_EFFECTR_ANGLE_ERR       = .5
    MAX_ALLOWABLE_LR_ERR                = .01
    MAX_ALLOWABLE_FB_ERR                = .025

    # Turn Constants
    TORSO_YAW_ROTATE_ANGLE              = 15
    TURNING_MIDSTEP_SLEEP_T             = .5

    # Step Constants
    STEP_Z_MAX_HIEGHT                   = .05

    TORSO_SHIFT_DELTA                   = .1                            # TODO: This should be optimized or calculated.
    TORSO_LEFT_SHIFT                    = BASE_STATE_X_DELTA / 2.5

    # Simulation constants
    SIMULATION_ENABLED                  = True
    PHYSICS_ENABLED                     = True
    INCLUDE_TERRAIN                     = True

    # Movement timing constants
    if PHYSICS_ENABLED:
        INITIALIZATION_STEP_TIME            = 2
        RESET_LEG_STEP_TIME                 = 5
        TURN_TIME                           = 5
        TORSO_SHIFT_TIME                    = 7
        STEP_TIME                           = 6
        TORSO_YAW_ROTATE_TIME               = 5

    else:
        INITIALIZATION_STEP_TIME            = .5
        RESET_LEG_STEP_TIME                 = 1
        TURN_TIME                           = 1
        TORSO_SHIFT_TIME                    = 1
        STEP_TIME                           = .5
        TORSO_YAW_ROTATE_TIME               = 1.5


    @staticmethod
    def ColorPrinter( caller, message, color):

        # [03/Apr/2017 18:37:10]
        time = datetime.datetime.now()

        if color not in RobotUtils.COLORS:
            color = RobotUtils.COLORS.get("STANDARD").get("value")

        prefix = "["+str(time.day)+"/"+str(time.month)+ "/" + str(time.year) + " " + str(time.hour) + ":" + str(time.minute) + ":" + str(time.second) +  " ] "
        prefix = prefix +  RobotUtils.COLORS["BOLD"]["value"]+ RobotUtils.COLORS["UNDERLINE"]["value"]+ caller+ RobotUtils.COLORS["ENDC"]["value"]+ ":"
        print_Str = prefix + " "+RobotUtils.COLORS[color]["value"] + " "+message + " "+RobotUtils.COLORS["ENDC"]["value"]

        print(print_Str)


    @staticmethod
    def angle_between_three_points(p1, p2, p3):

        # see https://stackoverflow.com/questions/35176451/python-code-to-calcualte-angle-between-three-point-using-thier-3d-coordinates

        a = np.array(p1)
        b = np.array(p2)
        c = np.array(p3)

        ba = a - b
        bc = c - b

        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))

        return np.degrees(np.arccos(cosine_angle))


    @staticmethod
    def get_euclidian_diff(xyz1, xyz2):

        d_x = (xyz1[0] - xyz2[0])**2
        d_y = (xyz1[1] - xyz2[1])**2
        d_z = (xyz1[2] - xyz2[2])**2

        return math.sqrt( d_x +d_y  +d_z    )

    @staticmethod
    def always_true_func():
        return True

    @staticmethod
    def pp_list(list):
        ret = "[ "
        for i in list:
            ret += "%.5f" % i
            ret += ", "
        ret = ret[:-2]
        ret += "] "
        return ret

    @staticmethod
    def pp_double(dbl):
        s = "%.4f" % dbl
        return s

