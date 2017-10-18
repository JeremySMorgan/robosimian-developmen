#!/usr/bin/python
import math
import time
import Queue
from klampt.model import ik
from klampt import vis

class HighLevelMotionController(object):

    def __init__(self, robot, RobotUtils, Controller):

        self.robosimian = robot

        self.f_r_foot = self.robosimian.link(RobotUtils.f_r_active_dofs[len(RobotUtils.f_r_active_dofs) - 1])
        self.f_l_foot = self.robosimian.link(RobotUtils.f_l_active_dofs[len(RobotUtils.f_l_active_dofs) - 1])
        self.b_r_foot = self.robosimian.link(RobotUtils.b_r_active_dofs[len(RobotUtils.b_r_active_dofs) - 1])
        self.b_l_foot = self.robosimian.link(RobotUtils.b_l_active_dofs[len(RobotUtils.b_l_active_dofs) - 1])

        self.RobotUtils = RobotUtils
        self.MotionPlanner = None
        self.MotionController = Controller
        self.motion_queue = Queue.Queue()
        self.motion_thread_currently_running = False

    #                                              Initiate Class
    # ------------------------------------------                      ------------------------------------------------
    #
    # Desc: Initiates self
    #
    # Contains:
    #           - initialize_motion_planner
    #           - set_inital_config


    def initialize_motion_planner(self, motion_planner):
        self.MotionPlanner = motion_planner

        f_r_base = self.MotionPlanner.get_local_foot_base_state_from_foot_name(self.RobotUtils.F_R_FOOT)

        vis.add("Front Right foot base state", f_r_base)


    def set_inital_config(self):

        # Get configuration array
        q = self.robosimian.getConfig()

        # Set joints to
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

        self.motion_queue.put_nowait(q)



    #                                                Control Loop
    # ------------------------------------------                      ------------------------------------------------
    #
    # Desc: Acts as main control loop of robot
    #
    # Contains:
    #           - control_loop
    #           - clear_motion_queue



    def control_loop(self):


        print "motion queue size: ",self.motion_queue.qsize()
        if not self.motion_queue.empty():

            calculated_next_config = self.motion_queue.get_nowait()
            self.MotionController.setLinear(calculated_next_config, self.RobotUtils.CONTROLLER_DT)


    def clear_motion_queue(self):
        self.motion_queue.empty()





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
    #           - make_leg_step_by_new_state
    #           - reset_to_base_state

    def make_right_turn(self, MotionThread):

        """
        @summary: Highest level callable method for making a right turn.
        @param MotionThread: MotionThread object holding the thread that this method is run by
        @return: None
        """

        if self.motion_thread_currently_running:
            if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                status = "Different motion thread is currently running, terminating"
                self.RobotUtils.ColorPrinter((self.__class__.__name__ + ".make_right_turn()"),status, "FAIL")
            return

        self.motion_thread_currently_running = True

        if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
            self.RobotUtils.ColorPrinter((self.__class__.__name__+".make_right_turn()"), "Starting right turn", "STANDARD")

        cancelled = False
        torso_rotation_angle = -self.RobotUtils.TORSO_YAW_ROTATE_ANGLE
        step_time = self.RobotUtils.TURN_TIME

        while 1:

            for leg in self.RobotUtils.end_affectors:

                local_end_xyz = self.MotionPlanner.get_local_turn_desitination(leg, torso_rotation_angle)

                vis.add((leg+" - local"),local_end_xyz)
                vis.add((leg+" - global"),self.MotionPlanner.get_world_xyz_from_local_xyz(local_end_xyz))

                if not self.linear_leg_step_to_local_xyz_in_t(leg, local_end_xyz, step_time, MotionThread):
                    cancelled = True
                    break


            if cancelled:
                break

            print "starting right make torso rotation"
            if not self.make_torso_rotation( torso_rotation_angle, MotionThread): break

        self.clear_motion_queue()
        self.reset_to_base_state()
        self.motion_thread_currently_running = False




    def make_left_turn(self, MotionThread):


        if self.motion_thread_currently_running:
            if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                status = "Different motion thread is currently running, terminating"
                self.RobotUtils.ColorPrinter((self.__class__.__name__ + ".make_left_turn()"),status, "FAIL")
            return

        self.motion_thread_currently_running = True


        if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
            self.RobotUtils.ColorPrinter((self.__class__.__name__+".make_left_turn()"), "Starting left turn", "STANDARD")

        cancelled = False
        torso_rotation_angle = self.RobotUtils.TORSO_YAW_ROTATE_ANGLE
        step_time = self.RobotUtils.TURN_TIME

        while 1:

            for leg in self.RobotUtils.end_affectors:

                local_end_xyz = self.MotionPlanner.get_local_turn_desitination(leg, torso_rotation_angle)

                vis.add((leg+" - local"),local_end_xyz)
                vis.add((leg+" - global"),self.MotionPlanner.get_world_xyz_from_local_xyz(local_end_xyz))

                if not self.linear_leg_step_to_local_xyz_in_t(leg, local_end_xyz, step_time, MotionThread):
                    cancelled = True
                    break

            if cancelled:
                break

            if not self.make_torso_rotation( torso_rotation_angle, MotionThread): break

        self.clear_motion_queue()
        self.reset_to_base_state()


        self.motion_thread_currently_running = False





    def forward_walk(self, MotionThread):

        """
        @summary: Highest level callable method for making a forward step. This method will run until the motion thread
                    is shutdown at which point it will reset the robot to a stable state
        @param MotionThread: MotionThread object holding the thread that this method is run by
        @return: None
        """

        if self.motion_thread_currently_running:
            if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                status = "Different motion thread is currently running, terminating"
                self.RobotUtils.ColorPrinter((self.__class__.__name__ + ".forward()"), status, "FAIL")
            return

        self.motion_thread_currently_running = True

        if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
            self.RobotUtils.ColorPrinter((self.__class__.__name__+".forward_walk()"), "Starting forward walk", "STANDARD")

        delta_x = self.RobotUtils.TORSO_SHIFT_DELTA

        translation = [delta_x, 0, 0]
        quarter_translation = [translation[0]/4, 0, 0]
        half_translation = [translation[0]/2, 0, 0]

        f_extend_state = self.RobotUtils.LEG_F_EXTEND_STATE
        base_state = self.RobotUtils.BASE_STATE

        cancelled = False

        f_r = self.RobotUtils.F_R_FOOT
        b_r = self.RobotUtils.B_R_FOOT
        f_l = self.RobotUtils.F_L_FOOT
        b_l = self.RobotUtils.B_L_FOOT

        self.reset_to_base_state()

        sleep_t = 0

        while 1:
        #if True:

            if not self.make_torso_shift_from_local_xyz_translation(half_translation, MotionThread):
                break

            time.sleep(sleep_t)
            if not self.make_leg_step_by_new_state(b_r, f_extend_state, MotionThread):
            #if not self.make_leg_step_by_delta_x(b_r, delta_x, MotionThread):
                    break

            time.sleep(sleep_t)
            if not self.make_leg_step_by_new_state(f_r, f_extend_state, MotionThread):

                #if not self.make_leg_step_by_delta_x(f_r, delta_x, MotionThread):
                break

            time.sleep(sleep_t)
            if not self.make_torso_shift_from_local_xyz_translation(half_translation, MotionThread):
                break

            time.sleep(sleep_t)
            if not self.make_leg_step_by_new_state(b_l, f_extend_state, MotionThread):
                #if not self.make_leg_step_by_delta_x(b_l, delta_x, MotionThread):
                break

            time.sleep(sleep_t)
            if not self.make_leg_step_by_new_state(f_l, f_extend_state, MotionThread):
                #if not self.make_leg_step_by_delta_x(f_l, delta_x, MotionThread):
                break


        self.clear_motion_queue()
        self.reset_to_base_state()

        self.motion_thread_currently_running = False





    def backward_walk(self, MotionThread):

        """
        @summary: Highest level callable method for making a backward step. This method will run until the motion thread
                    is shutdown at which point it will reset the robot to a stable state
        @param MotionThread: MotionThread object holding the thread that this method is run by
        @return: None
        """

        if self.motion_thread_currently_running:
            if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                status = "Different motion thread is currently running, terminating"
                self.RobotUtils.ColorPrinter((self.__class__.__name__ + ".backward()"), status, "FAIL")
            return

        self.motion_thread_currently_running = True

        if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
            self.RobotUtils.ColorPrinter((self.__class__.__name__+"backward_walk()"), "Starting backward walk", "STANDARD")

        translation = [-self.RobotUtils.TORSO_SHIFT_DELTA, 0, 0]

        f_back_extend_state = self.RobotUtils.LEG_B_EXTEND_STATE

        cancelled = False

        while 1:

            for leg in self.RobotUtils.end_affectors:
                if not self.make_leg_step_by_new_state(leg, f_back_extend_state, MotionThread): cancelled = True; break
                if not self.make_leg_step_by_new_state(leg, f_back_extend_state, MotionThread): cancelled = True; break
                if not self.make_leg_step_by_new_state(leg, f_back_extend_state, MotionThread): cancelled = True; break
                if not self.make_leg_step_by_new_state(leg, f_back_extend_state, MotionThread): cancelled = True; break

            if cancelled:
                break

            if not self.make_torso_shift_from_local_xyz_translation(translation, MotionThread): break

        self.clear_motion_queue()
        self.reset_to_base_state()
        self.motion_thread_currently_running = False





    def make_leg_step_by_delta_x(self, end_affector_name, delta_x, MotionThread):

        link =  self.MotionPlanner.get_foot_from_foot_name(end_affector_name)

        local_xyz = link.getLocalPosition([0,0,0])

        new_x = local_xyz[0] + delta_x

        local_end_xyz = [new_x, local_xyz[1], local_xyz[2]]

        step_time = self.RobotUtils.STEP_TIME

        return self.linear_leg_step_to_local_xyz_in_t(end_affector_name, local_end_xyz, step_time, MotionThread)






    def make_leg_step_by_new_state(self, end_affector_name, end_leg_state, MotionThread):

        if not end_leg_state in self.RobotUtils.leg_states:
            self.RobotUtils.ColorPrinter((self.__class__.__name__+"make_leg_step_by_new_state()"), "Error: end leg state unrecognized", "FAIL")
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
            self.RobotUtils.ColorPrinter((self.__class__.__name__+"make_leg_step_by_new_state()"), "Error: end leg state is recognized by RobotUtils,"
                                                                  "but not by make_leg_step_by_new_state()", "FAIL")
            return None

        step_time = self.RobotUtils.STEP_TIME

        result = self.linear_leg_step_to_local_xyz_in_t(end_affector_name, local_end_xyz, step_time, MotionThread)

        return result


    def reset_to_base_state(self):

        if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
            self.RobotUtils.ColorPrinter((self.__class__.__name__+"reset_to_base_state()"), "Resetting to base state", "STANDARD")


        # There are a number of scenarios to consider here
        #
        #   1) robot is currently in the base state                  -> exit function
        #   2) Legs are in base state but torso yaw is rotated       -> rotate torso
        #   3) Legs are NOT in base state
        #       3a) if one leg is moved it will be in base state     -> Move that leg
        #       3b) else


        t = self.RobotUtils.RESET_LEG_STEP_TIME


        # reset legs to base state if not positioned correctly
        for end_affector in self.RobotUtils.end_affectors:

            link = self.MotionPlanner.get_foot_from_foot_name(end_affector)
            link_local_xyz = link.getLocalPosition([0,0,0])
            local_end_xyz = self.MotionPlanner.get_local_foot_base_state_from_foot_name(end_affector)

            euclidian_delta = self.RobotUtils.get_euclidian_diff(link_local_xyz, local_end_xyz)

            if euclidian_delta > self.RobotUtils.MINIMUM_DIST_TO_CAUSE_RESET:
                self.linear_leg_step_to_local_xyz_in_t(end_affector, local_end_xyz, t)





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


    def linear_leg_step_to_local_xyz_in_t(self, link_name, local_end_xyz, step_time, MotionThread=None):

        """
        @param link_name: link name to move
        @param local_end_xyz: local xyz destination
        @param step_time: amount of time to perform the step in
        @param MotionThread
        @return: False if thread was suspended mid step. True otherwise
        """

        link = self.get_foot_from_foot_name(link_name)

        # Retrieve appropriate variables
        active_dofs = self.get_active_dofs_from_foot_name(link_name)
        desired_orientation = self.MotionPlanner.get_desired_foot_rotation(link_name)

        # Time calculations
        delay = self.RobotUtils.CONTROLLER_DT
        i_max = int( step_time / float(delay))

        global_start_xyz = link.getWorldPosition([0, 0, 0])
        global_end_xyz = self.MotionPlanner.get_world_xyz_from_local_xyz(local_end_xyz)

        # Check to see that the foot is actually moving. Will throw division by 0 error otherwise
        x_delta = abs(global_end_xyz[0] - global_start_xyz[0])
        y_delta = abs(global_end_xyz[1] - global_start_xyz[1])
        z_delta = abs(global_end_xyz[2] - global_start_xyz[2])

        ik_max_deviation = self.RobotUtils.IK_MAX_DEVIATION


        if self.RobotUtils.get_euclidian_diff(global_start_xyz, global_end_xyz) < self.RobotUtils.MINIMUM_DIST_TO_CAUSE_RESET:
            #self.RobotUtils.ColorPrinter((self.__class__.__name__+".linear_leg_step_to_local_xyz_in_t():"),"Negiligble change- Exiting function", "OKGREEN")
            return True


        for i in range(i_max):


            if MotionThread is None or ( (not MotionThread is None ) and MotionThread.is_alive()):

                xyz_des = self.MotionPlanner.get_linear_mid_motion_xyz(global_start_xyz, global_end_xyz, i, i_max)
                goal = ik.objective(link, R=desired_orientation, t=xyz_des)


                res = ik.solve_nearby(goal, activeDofs=active_dofs, maxDeviation=ik_max_deviation,
                                      feasibilityCheck=self.RobotUtils.always_true_func)

                # Failed
                if not res:
                    self.RobotUtils.ColorPrinter((self.__class__.__name__+".linear_leg_step_to_local_xyz_in_t()"), "linear_leg_step_to_local_xyz_in_t ik failure", "FAIL")

                else:
                    self.motion_queue.put(self.robosimian.getConfig())

                time.sleep(delay)

            else:
                if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                    status = "Suspending motion thread for: " + link_name + " linear motion"
                    self.RobotUtils.ColorPrinter(self.__class__.__name__, status, "FAIL")
                return False

        return True


    def make_torso_shift_from_local_xyz_translation(self, xyz_translation, MotionThread):

        """

        @param xyz_translation: local xyz destination for torso shift
        @param MotionThread:    parent Motion Thread
        @return: False if thread was suspended mid step. True otherwise
        """

        robot_states = []

        # Time calculations
        delay = self.RobotUtils.CONTROLLER_DT
        i_max = int(self.RobotUtils.TORSO_SHIFT_TIME / delay)

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


            if MotionThread is None or ( (not MotionThread is None ) and MotionThread.is_alive()):


                global_x = start_x + ((float(i) / float(i_max)) * x_delta)
                global_y = start_y + ((float(i) / float(i_max)) * y_delta)
                global_z = start_z + ((float(i) / float(i_max)) * z_delta)

                global_xyz = [global_x, global_y, global_z]

                self.shift_torso_to_global_xyz(global_xyz)

                q = self.robosimian.getConfig()
                robot_states.append(q)

                time.sleep(delay)

            else:
                if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                    self.RobotUtils.ColorPrinter((self.__class__.__name__+"make_torso_shift_from_local_xyz_translation()"), "Suspending motion thread", "FAIL")
                return False

        return True



    def make_torso_rotation(self, degree, MotionThread):

        offset = 1
        delay = self.RobotUtils.CONTROLLER_DT
        i_max = int(math.fabs(degree))

        if degree < 0:
            offset *= -1

        for i in range(i_max):

            if MotionThread.is_alive():
                self.rotate_torso_from_yaw_offset(offset)
                time.sleep(delay)
            else:
                return False
        return True


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

        res = ik.solve_nearby(goal, maxDeviation=ik_max_deviation, feasibilityCheck=self.RobotUtils.always_true_func)

        if not res:
            self.RobotUtils.ColorPrinter(self.__class__.__name__, "torso ik failure", "FAIL")

        else:
            self.motion_queue.put(self.robosimian.getConfig())


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
                              feasibilityCheck=self.RobotUtils.always_true_func)

        if res:
            self.motion_queue.put(self.robosimian.getConfig())
        else:
            if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                self.RobotUtils.ColorPrinter(self.__class__.__name__, "torso ik failure", "FAIL")








    #                                             Helper Functions
    # -------------------------------------                             ---=-----------------------------------
    #
    # Desc: Class specific helper functions
    #
    # Contains:
    #           - linear_leg_step_to_local_xyz_in_t
    #           - make_torso_shift_from_local_xyz_translation

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











'''
def forward_walk(self, MotionThread):

    """
    @summary: Highest level callable method for making a forward step. This method will run until the motion thread
                is shutdown at which point it will reset the robot to a stable state
    @param MotionThread: MotionThread object holding the thread that this method is run by
    @return: None
    """

    if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
        self.RobotUtils.ColorPrinter((self.__class__.__name__+".forward_walk()"), "Starting forward walk", "STANDARD")

    translation = [self.RobotUtils.TORSO_SHIFT_DELTA, 0, 0]
    quarter_translation = [translation[0]/4, 0, 0]

    f_extend_state = self.RobotUtils.LEG_F_EXTEND_STATE

    cancelled = False

    while 1:

        for leg in self.RobotUtils.end_affectors:
            if not self.make_leg_step_by_new_state(leg, f_extend_state, MotionThread): cancelled = True; break
            if not self.make_leg_step_by_new_state(leg, f_extend_state, MotionThread): cancelled = True; break
            if not self.make_leg_step_by_new_state(leg, f_extend_state, MotionThread): cancelled = True; break
            if not self.make_leg_step_by_new_state(leg, f_extend_state, MotionThread): cancelled = True; break

        if cancelled:
            break

        if not self.make_torso_shift_from_local_xyz_translation(translation, MotionThread): break


    if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
        self.RobotUtils.ColorPrinter((self.__class__.__name__+".forward_walk()"), "Ending forward walk", "STANDARD")

    self.clear_motion_queue()
    self.reset_to_base_state()












'''