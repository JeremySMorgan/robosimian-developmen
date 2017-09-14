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


    # App Wide constants used to convey function intent
    FORWARD = "FORWARD"
    BACKWARD = "BACKWARD"
    LEFT = "LEFT"
    RIGHT = "RIGHT"

    F_R_FOOT = "F_R_FOOT"
    F_L_FOOT = "F_L_FOOT"
    B_R_FOOT = "B_R_FOOT"
    B_L_FOOT = "B_L_FOOT"

    LEG_BASE_STATE = "BASE_STATE"
    LEG_F_EXTEND_STATE = "LEG_F_EXTEND"
    LEG_B_EXTEND_STATE = "LEG_B_EXTEND"

    directions = [FORWARD,BACKWARD,LEFT,RIGHT]
    end_affectors = [ F_R_FOOT, F_L_FOOT, B_R_FOOT, B_L_FOOT]
    leg_states = [LEG_B_EXTEND_STATE,LEG_BASE_STATE,LEG_F_EXTEND_STATE]


    # Robot Configuration Constants
    LIMB1_START_CONFIG_OFFSET = math.pi / 4


    # Step Constants
    TORSO_SHIFT_DELTA = .15
    TORSO_SHIFT_TIME = 1.5
    STEP_X_DELTA = .2
    STEP_Z_MAX_HIEGHT = .15
    STEP_TIME = .5
    DESIRED_FOOT_ROTATION = [0, 0, -1.0, 0, 1, 0, 1, 0, 0]


    # Simulation constants
    SIMULATION_FRAME_DELAY = .005
    SIMULATION_ENABLED = True


    @staticmethod
    def ColorPrinter( caller, message, color):
        # [03/Apr/2017 18:37:10]
        time = datetime.datetime.now()
        prefix = "["+str(time.day)+"/"+str(time.month)+ "/" + str(time.year) + " " + str(time.hour) + ":" + str(time.minute) + ":" + str(time.second) +  " ] "
        prefix = prefix +  RobotUtils.COLORS["BOLD"]["value"]+ RobotUtils.COLORS["UNDERLINE"]["value"]+ caller+ RobotUtils.COLORS["ENDC"]["value"]+ ":"
        print  prefix, RobotUtils.COLORS[color]["value"] ,message , RobotUtils.COLORS["ENDC"]["value"]

