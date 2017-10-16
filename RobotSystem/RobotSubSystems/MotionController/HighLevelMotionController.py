#!/usr/bin/python
import math
import time
from klampt.model import ik


class HighLevelMotionController(object):
    def __init__(self, robot, RobotUtils, Controller):
        self.robosimian = robot
        self.RobotUtils = RobotUtils
        self.initialize_feet()
        self.MotionPlanner = None
        self.MotionController = Controller



    def initialize_motion_planner(self, motion_planner):
        self.MotionPlanner = motion_planner



    def initialize_feet(self):

        self.f_r_foot = self.robosimian.link(self.RobotUtils.f_r_active_dofs[len(self.RobotUtils.f_r_active_dofs) - 1])
        self.f_l_foot = self.robosimian.link(self.RobotUtils.f_l_active_dofs[len(self.RobotUtils.f_l_active_dofs) - 1])
        self.b_r_foot = self.robosimian.link(self.RobotUtils.b_r_active_dofs[len(self.RobotUtils.b_r_active_dofs) - 1])
        self.b_l_foot = self.robosimian.link(self.RobotUtils.b_l_active_dofs[len(self.RobotUtils.b_l_active_dofs) - 1])




    def set_inital_config(self):

        # Get configuration array
        q = self.robosimian.getConfig()

        # Set joints to 0
        for i in range(len(q)):
            q[i] = 0

        link2_offset_indexes = [8, 16, 24, 32]
        link4_offset_indexes = [10, 18, 26, 34]

        pos_sign = [1, -1, 1, -1]
        neg_sign = [-1, 1, -1, 1]

        for i in range(len(link2_offset_indexes)):
            q[link2_offset_indexes[i]] = pos_sign[i] * self.RobotUtils.LIMB1_START_CONFIG_OFFSET

        for i in range(len(link4_offset_indexes)):
            q[link4_offset_indexes[i]] = neg_sign[i] * (self.RobotUtils.LIMB1_START_CONFIG_OFFSET + (math.pi / 2.0))

        self.robosimian.setConfig(q)



    #                                              Callable Motion APIS
    # --------------------------------------------                      ------------------------------------------------
    #
    # Desc: These functions will perform their respective motion (right/left turn, forward/backward) until their thread
    #       is suspended. These functions are called by the ObjectiveManager
    #
    # Contains:
    #           - make_right_turn
    #           - make_left_turn
    #           - forward_walk
    #           - backward_walk

    def make_right_turn(self, thread):

        """
        @summary: Highest level callable method for making a right turn.
        @param MotionThread: MotionThread object holding the thread that this method is run by
        @return: None
        """

        if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
            self.RobotUtils.ColorPrinter(self.__class__.__name__, "Starting right turn", "STANDARD")

        for leg in self.RobotUtils.end_affectors:
            self.make_leg_turn(leg, self.RobotUtils.RIGHT)

        self.make_torso_rotation(-self.RobotUtils.TORSO_YAW_ROTATE_ANGLE)
        self.reset_to_base_state()


    def make_left_turn(self, MotionThread):

        """
        @summary: Highest level callable method for making a left turn.
        @param MotionThread: MotionThread object holding the thread that this method is run by
        @return: None
        """

        if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
            self.RobotUtils.ColorPrinter(self.__class__.__name__, "Starting left turn", "STANDARD")

        for leg in self.RobotUtils.end_affectors:
            self.make_leg_turn(leg, self.RobotUtils.LEFT)

        self.make_torso_rotation(self.RobotUtils.TORSO_YAW_ROTATE_ANGLE)
        self.reset_to_base_state()


    def forward_walk(self, MotionThread):

        """
        @summary: Highest level callable method for making a forward step. This method will run until the motion thread
                    is shutdown at which point it will reset the robot to a stable state
        @param MotionThread: MotionThread object holding the thread that this method is run by
        @return: None
        """


        if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
            self.RobotUtils.ColorPrinter(self.__class__.__name__, "Starting forward walk", "STANDARD")

        translation = [self.RobotUtils.TORSO_SHIFT_DELTA, 0, 0]

        f_extend_state = self.RobotUtils.LEG_F_EXTEND_STATE

        # Begin walk
        for leg in self.RobotUtils.end_affectors:
            self.make_leg_step(leg, f_extend_state, MotionThread)
            self.make_leg_step(leg, f_extend_state, MotionThread)
            self.make_leg_step(leg, f_extend_state, MotionThread)
            self.make_leg_step(leg, f_extend_state, MotionThread)

        self.make_torso_shift_from_local_xyz_translation(translation, MotionThread)


    def backward_walk(self, MotionThread):

        """
        @summary: Highest level callable method for making a backward step. This method will run until the motion thread
                    is shutdown at which point it will reset the robot to a stable state
        @param MotionThread: MotionThread object holding the thread that this method is run by
        @return: None
        """

        if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
            self.RobotUtils.ColorPrinter(self.__class__.__name__, "Starting backward walk", "STANDARD")

        translation = [-self.RobotUtils.TORSO_SHIFT_DELTA, 0, 0]

        f_back_extend_state = self.RobotUtils.LEG_B_EXTEND_STATE

        # Begin walk
        for leg in self.RobotUtils.end_affectors:
            self.make_leg_step(leg, f_back_extend_state, MotionThread)
            self.make_leg_step(leg, f_back_extend_state, MotionThread)
            self.make_leg_step(leg, f_back_extend_state, MotionThread)
            self.make_leg_step(leg, f_back_extend_state, MotionThread)

        self.make_torso_shift_from_local_xyz_translation(translation, MotionThread)








    #                                      Threaded Moiton in Speficied Time APIS
    # -----------------------------------                                         ------------------------------------
    #
    # Desc: These functions will executed a desired step pattern in a speficied amount of time (specified in RobotUtils)
    #       and add each new calculated robot state to the motion queue. They will return False if they detect that the
    #       thread they are being run in is suspended, otherwise true
    #
    # Contains:
    #           - linear_leg_step_to_local_xyz_in_t
    #           - make_torso_shift_from_local_xyz_translation


    # TODO: implement MotionThread detection
    def linear_leg_step_to_local_xyz_in_t(self, link_name, local_end_xyz, t):

        """
        @param link_name: link name to move
        @param local_end_xyz: local xyz destination
        @param t: amount of time to perform the step in
        @return: False if thread was suspended mid step. True otherwise
        """

        robot_states = []

        link = self.get_foot_from_foot_name(link_name)

        # Retrieve appropriate variables
        active_dofs = self.get_active_dofs_from_foot_name(link_name)
        desired_orientation = self.MotionPlanner.get_desired_foot_rotation(link_name)

        # Time calculations
        step_time = t
        delay = self.RobotUtils.SIMULATION_FRAME_DELAY
        i_max = int(float(step_time) / float(delay))

        global_start_xyz = link.getWorldPosition([0, 0, 0])
        global_end_xyz = self.MotionPlanner.get_world_xyz_from_local_xyz(local_end_xyz)

        # Check to see that the foot is actually moving. Will throw division by 0 error otherwise
        x_delta = abs(global_end_xyz[0] - global_start_xyz[0])
        y_delta = abs(global_end_xyz[1] - global_start_xyz[1])
        z_delta = abs(global_end_xyz[2] - global_start_xyz[2])

        if x_delta < self.RobotUtils.MINIMUM_X_DELTA and y_delta < self.RobotUtils.MINIMUM_X_DELTA and z_delta < self.RobotUtils.MINIMUM_Z_DELTA:
            return [self.robosimian.getConfig()]

        ik_max_deviation = self.RobotUtils.IK_MAX_DEVIATION

        for i in range(i_max):

            xyz_des = self.MotionPlanner.get_linear_mid_motion_xyz(global_start_xyz, global_end_xyz, i, i_max)
            goal = ik.objective(link, R=desired_orientation, t=xyz_des)
            res = ik.solve_nearby(goal, activeDofs=active_dofs, maxDeviation=ik_max_deviation,
                                  feasibilityCheck=HighLevelMotionController.feasibility_check)
            q = self.robosimian.getConfig()
            robot_states.append(q)

            if res:
                pass
            else:
                self.RobotUtils.ColorPrinter(self.__class__.__name__, "linear_leg_step_to_local_xyz_in_t ik failure", "FAIL")

            time.sleep(delay)

            if False:
                if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                    status = "Suspending motion thread for: " + link_name + " linear motion"
                    self.RobotUtils.ColorPrinter(self.__class__.__name__, status, "FAIL")
                break

        return robot_states


    # TODO: implement MotionThread detection
    def make_torso_shift_from_local_xyz_translation(self, xyz_translation, MotionThread):

        """

        @param xyz_translation: local xyz destination for torso shift
        @param MotionThread:    parent Motion Thread
        @return: False if thread was suspended mid step. True otherwise
        """

        robot_states = []

        # Time calculations
        step_time = self.RobotUtils.TORSO_SHIFT_TIME
        delay = self.RobotUtils.SIMULATION_FRAME_DELAY
        i_max = int(float(step_time) / float(delay))

        global_xyz_start = self.MotionPlanner.get_world_xyz_from_local_xyz([0, 0, 0])
        global_xyz_end = self.MotionPlanner.get_world_xyz_from_local_xyz(xyz_translation)

        start_x = global_xyz_start[0]
        start_y = global_xyz_start[1]
        start_z = global_xyz_start[2]

        end_x = global_xyz_end[0]
        end_y = global_xyz_end[1]
        end_z = global_xyz_end[2]

        x_delta = end_x - start_x
        y_delta = end_y - start_y
        z_delta = end_z - start_z

        for i in range(i_max):

            global_x = start_x + ((float(i) / float(i_max)) * x_delta)
            global_y = start_y + ((float(i) / float(i_max)) * y_delta)
            global_z = start_z + ((float(i) / float(i_max)) * z_delta)

            global_xyz = [global_x, global_y, global_z]

            self.shift_torso_to_global_xyz(global_xyz)

            q = self.robosimian.getConfig()
            robot_states.append(q)

            time.sleep(self.RobotUtils.SIMULATION_FRAME_DELAY)

            if False:
                if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                    self.RobotUtils.ColorPrinter(self.__class__.__name__, "Suspending motion thread", "FAIL")
                break

        return robot_states


    # TODO: implement MotionThread detection
    def make_torso_rotation(self, MotionThread):

        # Time calculations
        step_time = self.RobotUtils.TORSO_SHIFT_TIME
        delay = self.RobotUtils.SIMULATION_FRAME_DELAY
        i_max = int(float(step_time) / float(delay))

        degree = self.RobotUtils.TORSO_YAW_ROTATE_ANGLE

        yaw_offset = 10 * float(degree) / float(i_max)

        for i in range(i_max / 10):
            self.rotate_torso_from_yaw_offset(yaw_offset)

            time.sleep(self.RobotUtils.SIMULATION_FRAME_DELAY * 10)










    #                                      Non-Threaded Low Level IK Solver APIS
    # ----------------------------------                                         ------------------------------------
    #
    # Desc: These functions will calculate a new robot state given a speficied change (torso rotation, foot movement, ect)
    #       Will return new robot configuration to the caller. Functions do not deal with threading
    #
    # Contains:
    #           - linear_leg_step_to_local_xyz_in_t
    #           - make_torso_shift_from_local_xyz_translation



    def rotate_torso_from_yaw_offset(self, yaw_offset_deg):

        """
        @summary rotates the robot by a speficied yaw offset (which should be very small). Appends the new robot state
                    to the motion queue. Does not handle a MotionThread
        @param yaw_offset_deg: degree to rotate torso by
        @return: None
        """

        active_dofs = []
        for i in range(6, 38):
            active_dofs.append(i)

        torso = self.robosimian.link(self.RobotUtils.TORSO_LINK_INDEX)

        f_l = self.f_l_foot
        f_r = self.f_r_foot
        b_l = self.b_l_foot
        b_r = self.b_r_foot

        # World Leg Positions
        f_l_global = f_l.getWorldPosition([0, 0, 0])
        f_r_global = f_r.getWorldPosition([0, 0, 0])
        b_l_global = b_l.getWorldPosition([0, 0, 0])
        b_r_global = b_r.getWorldPosition([0, 0, 0])

        # Desired Leg orientation
        f_l_desired_orientation = self.MotionPlanner.get_desired_foot_rotation(self.RobotUtils.F_L_FOOT)
        f_r_desired_orientation = self.MotionPlanner.get_desired_foot_rotation(self.RobotUtils.F_R_FOOT)
        b_l_desired_orientation = self.MotionPlanner.get_desired_foot_rotation(self.RobotUtils.B_L_FOOT)
        b_r_desired_orientation = self.MotionPlanner.get_desired_foot_rotation(self.RobotUtils.B_R_FOOT)

        # ik obkectives
        f_l_r_const = ik.objective(f_l, R=f_l_desired_orientation, t=f_l_global)
        f_r_r_const = ik.objective(f_r, R=f_r_desired_orientation, t=f_r_global)
        b_l_r_const = ik.objective(b_l, R=b_l_desired_orientation, t=b_l_global)
        b_r_r_const = ik.objective(b_r, R=b_r_desired_orientation, t=b_r_global)

        des_torso_rotation = self.MotionPlanner.get_desired_torso_R_from_yaw_offset(yaw_offset_deg)

        global_torso_xyz = torso.getWorldPosition([0, 0, 0])

        torso_obj = ik.objective(torso, R=des_torso_rotation, t=global_torso_xyz)

        goal = [f_l_r_const, f_r_r_const, b_l_r_const, b_r_r_const, torso_obj]

        ik_max_deviation = self.RobotUtils.IK_MAX_DEVIATION

        if ik.solve_nearby(goal, maxDeviation=ik_max_deviation,
                           feasibilityCheck=HighLevelMotionController.feasibility_check):
            pass
        else:
            if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                self.RobotUtils.ColorPrinter(self.__class__.__name__, "torso ik failure", "FAIL")



    def shift_torso_to_global_xyz(self, global_xyz):

        """
        @summary shifts the torso to a specified global coordinate and appends new robot state to motion queue. Position
                 change must be small as this function does not access the parent Motion Thread
        @param  global_xyz: desired global xyz end destination
        @return: None
        """

        torso = self.robosimian.link(self.RobotUtils.TORSO_LINK_INDEX)

        f_l = self.f_l_foot
        f_r = self.f_r_foot
        b_l = self.b_l_foot
        b_r = self.b_r_foot

        # World Leg Positions
        f_l_global = f_l.getWorldPosition([0, 0, 0])
        f_r_global = f_r.getWorldPosition([0, 0, 0])
        b_l_global = b_l.getWorldPosition([0, 0, 0])
        b_r_global = b_r.getWorldPosition([0, 0, 0])

        # Desired Leg orientation
        left_leg_desired_orientation = self.MotionPlanner.get_desired_foot_rotation(self.RobotUtils.F_L_FOOT)
        right_leg_desired_orientation = self.MotionPlanner.get_desired_foot_rotation(self.RobotUtils.F_R_FOOT)

        # ik obkectives
        f_l_r_const = ik.objective(f_l, R=left_leg_desired_orientation, t=f_l_global)
        f_r_r_const = ik.objective(f_r, R=right_leg_desired_orientation, t=f_r_global)
        b_l_r_const = ik.objective(b_l, R=left_leg_desired_orientation, t=b_l_global)
        b_r_r_const = ik.objective(b_r, R=right_leg_desired_orientation, t=b_r_global)

        torso_obj = ik.objective(torso, R=torso.getTransform()[0], t=global_xyz)

        goal = [f_l_r_const, f_r_r_const, b_l_r_const, b_r_r_const, torso_obj]

        ik_max_deviation = 10 * self.RobotUtils.IK_MAX_DEVIATION

        res = ik.solve_nearby(goal, maxDeviation=ik_max_deviation,
                              feasibilityCheck=HighLevelMotionController.feasibility_check)

        if res:
            pass
        else:
            if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                self.RobotUtils.ColorPrinter(self.__class__.__name__, "torso ik failure", "FAIL")








    @staticmethod
    def feasibility_check():
        return True














    def reset_to_base_state(self):

        if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
            self.RobotUtils.ColorPrinter(self.__class__.__name__, "Resetting to base state", "STANDARD")

        local_leg_xyzs = []

        # reset legs to base state
        for end_affector in self.RobotUtils.end_affectors:
            local_end_xyz = self.MotionPlanner.get_local_foot_base_state_from_foot_name(end_affector)
            current_local_xyz = self.get_foot_from_foot_name(end_affector).getLocalPosition([0, 0, 0])

            t = self.RobotUtils.RESET_LEG_STEP_TIME

            self.linear_leg_step_to_local_xyz_in_t(end_affector, local_end_xyz, t)

            # Append to local_leg_xyzs

            local_xyz = self.get_foot_from_foot_name(end_affector).getLocalPosition([0, 0, 0])

        # reset torso to desired config
        local_x_center = 0
        local_y_center = 0

        for xyz in local_leg_xyzs:
            local_x_center += xyz[0]
            local_y_center += xyz[1]

        local_x_center /= 4
        local_y_center /= 4

        local_xyz_offset = [local_x_center, local_y_center, 0]

        global_xyz_end = self.MotionPlanner.get_world_xyz_from_local_xyz([local_x_center, local_y_center, 0])

        # translate torso to center_x, center_y
        self.shift_torso_to_global_xyz(global_xyz_end)

        # Get desired yaw
        f_l_local_xyz = self.get_foot_from_foot_name(self.RobotUtils.F_L_FOOT).getLocalPosition([0, 0, 0])
        b_l_local_xyz = self.get_foot_from_foot_name(self.RobotUtils.B_L_FOOT).getLocalPosition([0, 0, 0])



























    def make_leg_turn(self, end_affector_name, direction):

        local_end_xyz = self.MotionPlanner.get_local_turn_desitination(end_affector_name, direction)

        step_time = self.RobotUtils.TURN_TIME

        self.linear_leg_step_to_local_xyz_in_t(end_affector_name, local_end_xyz, step_time)






    def make_leg_step(self, end_affector_name, end_leg_state, make_leg_step=None):

        if not end_leg_state in self.RobotUtils.leg_states:
            self.RobotUtils.ColorPrinter(self.__class__.__name__, "Error: end leg state unrecognized", "FAIL")
            return None

        # Determine appropriate end local coordinates
        if end_leg_state == self.RobotUtils.LEG_F_EXTEND_STATE:
            local_end_xyz = self.MotionPlanner.get_extended_foot_local_xyz(end_affector_name, self.RobotUtils.FORWARD)

        elif end_leg_state == self.RobotUtils.LEG_B_EXTEND_STATE:
            local_end_xyz = self.MotionPlanner.get_extended_foot_local_xyz(end_affector_name,
                                                                           self.RobotUtils.BACKWARD)

        elif end_leg_state == self.RobotUtils.LEG_BASE_STATE:
            local_end_xyz = self.MotionPlanner.get_local_foot_base_state_from_foot_name(end_affector_name)

        else:
            self.RobotUtils.ColorPrinter(self.__class__.__name__, "Error: end leg state is recognized by RobotUtils,"
                                                                  "but not by make_leg_step()", "FAIL")
            return None

        step_time = self.RobotUtils.STEP_TIME

        mid_step_robot_states = self.linear_leg_step_to_local_xyz_in_t(end_affector_name, local_end_xyz, step_time)

        dt = self.RobotUtils.CONTROLLER_DT

        # self.MotionController.setLinear( mid_step_robot_states[0], dt)

        t = 0
        for i in range(len(mid_step_robot_states)):
            self.MotionController.addLinear(mid_step_robot_states[i], dt)
            t += dt
            # time.sleep(dt)






    # Helper function to return the leg joints of a given foot.
    def get_active_dofs_from_foot_name(self, foot_name):

        if not foot_name in self.RobotUtils.end_affectors:
            self.RobotUtils.ColorPrinter(self.__class__.__name__,
                                         "get_active_dofs_from_foot_name: Error: foot name unrecognized", "FAIL")
            return None

        if foot_name == self.RobotUtils.B_L_FOOT:
            return self.RobotUtils.b_l_active_dofs

        elif (foot_name == self.RobotUtils.B_R_FOOT):
            return self.RobotUtils.b_r_active_dofs

        elif foot_name == self.RobotUtils.F_L_FOOT:
            return self.RobotUtils.f_l_active_dofs

        else:
            return self.RobotUtils.f_r_active_dofs

    def get_foot_from_foot_name(self, foot_name):

        return self.MotionPlanner.get_foot_from_foot_name(foot_name)
