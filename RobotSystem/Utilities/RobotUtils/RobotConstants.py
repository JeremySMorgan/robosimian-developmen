from __future__ import print_function
import math
import datetime
import numpy as np

class RobotConstants(object):


    '''
        Stores an individuals gait parameters
           param_list = [BASE_STATE_X_DELTA, BASE_STATE_Y_DELTA, BASE_STATE_Z_DELTA, STEP_Z_MAX_HIEGHT, TORSO_SHIFT_DELTA, TORSO_LEFT_SHIFT, STEP_TIME, TORSO_SHIFT_TIME   ]
    '''

    def __init__(self, physics_enabled, param_list = None ):

        # Link indexes
        self.TORSO_LINK_INDEX = 5
        self.f_r_active_dofs = [7, 8, 9, 10, 11, 12, 13]
        self.f_l_active_dofs = [31, 32, 33, 34, 35, 36, 37]
        self.b_r_active_dofs = [15,16,17,18,19,20,21]
        self.b_l_active_dofs = [23,24,25,26,27,28,29]

        # robot states
        self.FORWARD = "FORWARD"
        self.BACKWARD = "BACKWARD"
        self.LEFT = "LEFT"
        self.RIGHT = "RIGHT"
        self.BASE_STATE = "BASE_STATE"

        self.F_R_FOOT = "F_R_FOOT"
        self.F_L_FOOT = "F_L_FOOT"
        self.B_R_FOOT = "B_R_FOOT"
        self.B_L_FOOT = "B_L_FOOT"

        self.allowed_states = [self.FORWARD, self.BASE_STATE, self.LEFT, self.RIGHT]

        self.left_feet     = [self.F_L_FOOT, self.B_L_FOOT ]
        self.right_feet    = [self.F_R_FOOT, self.B_R_FOOT ]
        self.end_affectors = [self.F_R_FOOT, self.F_L_FOOT, self.B_R_FOOT, self.B_L_FOOT]

        # App wide multithreading constants
        self.KEEP_THREAD_ALIVE                   = "thread_is_alive"

        # Gamepad input
        self.GAMEPAD_LEAST_SIGNIFICANT_INPUT     = .01

        # Debugging
        self.OBJ_PLANNER_DEBUGGING_ENABLED                = False
        self.USER_INPUT_DEBUGGING_ENABLED                 = False
        self.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED  = True

        # Delay Constants
        self.CONTROLLER_DT                       = .01
        self.GAMEPAD_UPDATE_DELAY                = .01
        self.OBJECTIVE_PLANNER_UPDATE_DELAY      = .05

        # Global IK constants
        self.IK_MAX_DEVIATION                    = .115

        # Base State Parameters

        # starting config first below has legs high - prone to torso collisions
        self.STARTING_CONFIG = [ 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, -0.05933, -1.44834, -0.44436, 2.14054, 0.06762, -2.25009, 0.55330, 0.00000, 0.07207, 1.43561, 0.44054, -2.12907, -0.07428, 2.25008, -0.54579, 0.00000, -0.06722, -1.43800, -0.43682, 2.13177, 0.07243, -2.25090, 0.54626, 0.00000, 0.06733, 1.43761, 0.43633, -2.13147, -0.07257, 2.25096, -0.54595]

        self.BASE_STATE_X_DELTA                  = param_list[0] if param_list else .35
        self.BASE_STATE_Y_DELTA                  = param_list[1] if param_list else .2
        self.BASE_STATE_Z_DELTA                  = param_list[2] if param_list else -.5 # This value is the delta z from the torso to the ground (/end affectors center point)

        # Robot Measurements
        self.SHOULDER_X                          = .293260631869
        self.SHOULDER_Y                          = .206456211
        self.SHOULDER_Z                          = -0.2085299999988
        self.LEG_LENGTH                          = .775
        self.SHOULDER_TORSO_XY_EUCLIDEAN_DIF     = 0.3603472707
        self.DELTA_Z_SHOULDER_END_AFFECTOR       = 0.980182 * (self.BASE_STATE_Z_DELTA) + .20965
        self.END_AFFECTOR_RADIUS_TO_SHOULDER     = math.sqrt(self.LEG_LENGTH**2 - self.DELTA_Z_SHOULDER_END_AFFECTOR**2)
        self.SHOULDER_TORSO_PSI_RADS             = .620089205322
        self.FORWARD_BACK_LEGS_EUCLIDEAN_DIST    = 2 * (self.SHOULDER_X + self.BASE_STATE_X_DELTA)
        self.LEFT_RIGHT_LEGS_EUCLIDEAN_DIST      = 2 * (self.SHOULDER_Y + self.BASE_STATE_Y_DELTA)

        # Visualization - determines if _2DSupport Objects are shown
        self.RESET_VISUALIZATION_ENABLED         = True
        self.FORWARD_STEP_VISUALIZATION_ENABLED  = True
        self.TURNING_VISUALIZATION_ENABLED       = True

        # Reset Constants
        self.MINIMUM_DIST_TO_CAUSE_RESET         = .005
        self.MINIMUM_X_DELTA                     = .05
        self.MINIMUM_Y_DELTA                     = .05
        self.MINIMUM_Z_DELTA                     = .015      # all z deltas above this will trigger a reset
        self.MAX_ALLOWABLE_END_EFFECTR_ANGLE_ERR = .5
        self.MAX_ALLOWABLE_LR_ERR                = .01
        self.MAX_ALLOWABLE_FB_ERR                = .025

        # Turn Constants
        self.TORSO_YAW_ROTATE_ANGLE              = 15
        self.TURNING_MIDSTEP_SLEEP_T             = .5
        self.RESET_MIDSTEP_SLEEP_T             = .5

        # Step Constants
        self.STEP_Z_MAX_HIEGHT                   = param_list[3] if param_list else .05

        self.TORSO_SHIFT_DELTA                   = param_list[4] if param_list else .1                               # TODO: This should be optimized or calculated.
        self.TORSO_LEFT_SHIFT                    = param_list[5] if param_list else (self.BASE_STATE_X_DELTA / 2.5)

        # Simulation constants
        self.SIMULATION_ENABLED                  = True
        self.PHYSICS_ENABLED                     = False
        self.INCLUDE_TERRAIN                     = True

        # Movement timing constants
        if physics_enabled:
            self.INITIALIZATION_STEP_TIME            = 2
            self.RESET_LEG_STEP_TIME                 = 4
            self.TURN_TIME                           = 4
            self.TORSO_SHIFT_TIME                    = param_list[7] if param_list else 4
            self.STEP_TIME                           = param_list[6] if param_list else 4
            self.TORSO_YAW_ROTATE_TIME               = 4

        else:
            self.INITIALIZATION_STEP_TIME            = .5
            self.RESET_LEG_STEP_TIME                 = 1
            self.TURN_TIME                           = 1
            self.TORSO_SHIFT_TIME                    = param_list[7] if param_list else 2
            self.STEP_TIME                           = param_list[6] if param_list else 2
            self.TORSO_YAW_ROTATE_TIME               = 1.5

