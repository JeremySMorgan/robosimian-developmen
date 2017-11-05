#!/usr/bin/python
import math
import time
import Queue
from klampt.model import ik
from klampt import vis
import time

class HighLevelMotionController(object):

    def __init__(self, robot, RobotUtils, Controller):

        self.robosimian = robot

        self.f_r_end_affector = self.robosimian.link(RobotUtils.f_r_active_dofs[len(RobotUtils.f_r_active_dofs) - 1])
        self.f_l_end_affector = self.robosimian.link(RobotUtils.f_l_active_dofs[len(RobotUtils.f_l_active_dofs) - 1])
        self.b_r_end_affector = self.robosimian.link(RobotUtils.b_r_active_dofs[len(RobotUtils.b_r_active_dofs) - 1])
        self.b_l_end_affector = self.robosimian.link(RobotUtils.b_l_active_dofs[len(RobotUtils.b_l_active_dofs) - 1])

        self.RobotUtils = RobotUtils
        self.MotionPlanner = None
        self.MotionController = Controller
        self.motion_queue = Queue.Queue()
        self.motion_thread_currently_running = False

        self.measured_controller_dt = RobotUtils.CONTROLLER_DT
        self.block_start_time = 0
        self.control_loop_calls = 0
        self.first_control_loop = True
        self.readings_in_block = 5.0

        self.initialization_complete = False

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

    def set_inital_config(self):


        if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
            status = "starting initilization "
            self.RobotUtils.ColorPrinter((self.__class__.__name__+"set_inital_config()"),status,"STANDARD")


        pos_x_shoulder_base = self.RobotUtils.SHOULDER_X
        pos_y_shoulder_base = self.RobotUtils.SHOULDER_Y

        pos_x_end_aff_delta = self.RobotUtils.BASE_STATE_X_DELTA
        pos_y_end_aff_delta = self.RobotUtils.BASE_STATE_Y_DELTA

        end_eff_z_pos       = self.RobotUtils.BASE_STATE_Z_DELTA

        f_r_local_base_state = [ pos_x_shoulder_base + pos_x_end_aff_delta, -pos_y_shoulder_base - pos_y_end_aff_delta, end_eff_z_pos ]
        f_l_local_base_state = [ pos_x_shoulder_base + pos_x_end_aff_delta, pos_y_shoulder_base + pos_y_end_aff_delta, end_eff_z_pos ]
        b_l_local_base_state = [ -pos_x_shoulder_base - pos_x_end_aff_delta, pos_y_shoulder_base + pos_y_end_aff_delta, end_eff_z_pos ]
        b_r_local_base_state = [ -pos_x_shoulder_base  - pos_x_end_aff_delta, -pos_y_shoulder_base - pos_y_end_aff_delta, end_eff_z_pos ]

        end_effactors = [ self.RobotUtils.F_R_FOOT,  self.RobotUtils.F_L_FOOT,   self.RobotUtils.B_L_FOOT,   self.RobotUtils.B_R_FOOT ]
        base_states =   [ f_r_local_base_state,      f_l_local_base_state,       b_l_local_base_state,       b_r_local_base_state     ]

        step_time = self.RobotUtils.INITIALIZATION_STEP_TIME

        for i in range(len(base_states)):
            local_xyz_des = base_states[i]
            end_effactor = end_effactors[i]
            self.linear_leg_step_to_local_xyz_in_t(end_effactor, local_xyz_des, step_time)

        if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
            status = "Initilization Finished"
            self.RobotUtils.ColorPrinter((self.__class__.__name__+"set_inital_config()"),status,"OKGREEN" )

        self.initialization_complete = True




    #                                                Control Loop
    # ------------------------------------------                      ------------------------------------------------
    #
    # Desc: Acts as main control loop of robot
    #
    # Contains:
    #           - control_loop
    #           - clear_motion_queue


    def control_loop(self):

        self.control_loop_calls += 1

        if self.first_control_loop:
            self.first_control_loop = False
            self.block_start_time = time.time()

        if self.control_loop_calls == self.readings_in_block:
            sum_block_t = ( time.time() - self.block_start_time)
            self.measured_controller_dt =  sum_block_t/  self.readings_in_block
            self.control_loop_calls = 0
            self.first_control_loop = True

        print_str = "motion queue size: "+str(self.motion_queue.qsize())+ "measured controller dt:"+str(self.measured_controller_dt)
        #print print_str
        if not self.motion_queue.empty():

            calculated_next_config = self.motion_queue.get_nowait()
            self.MotionController.setLinear(calculated_next_config, self.RobotUtils.CONTROLLER_DT)


    def clear_motion_queue(self):
        self.motion_queue.empty()


    def add_q_to_motion_queue(self,q):
        self.motion_queue.put_nowait(q)


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

        if not self.initialization_complete:
            if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                status = "Initilaization not complete, terminating"
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

                if not self.linear_leg_step_to_local_xyz_in_t(leg, local_end_xyz, step_time, MotionThread):
                    cancelled = True
                    break


            if cancelled:
                break

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

        if not self.initialization_complete:
            if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                status = "Initilaization not complete, terminating"
                self.RobotUtils.ColorPrinter((self.__class__.__name__ + ".make_right_turn()"),status, "FAIL")
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

        if not self.initialization_complete:
            if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                status = "Initilaization not complete, terminating"
                self.RobotUtils.ColorPrinter((self.__class__.__name__ + ".make_right_turn()"),status, "FAIL")
            return

        self.motion_thread_currently_running = True

        if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
            self.RobotUtils.ColorPrinter((self.__class__.__name__+".forward_walk()"), "Starting forward walk", "STANDARD")

        f_extend_state = self.RobotUtils.LEG_F_EXTEND_STATE
        base_state = self.RobotUtils.BASE_STATE

        f_r = self.RobotUtils.F_R_FOOT
        b_r = self.RobotUtils.B_R_FOOT
        f_l = self.RobotUtils.F_L_FOOT
        b_l = self.RobotUtils.B_L_FOOT

        if self.RobotUtils.PHYSICS_ENABLED:
            sleep_t = 2
        else:
            sleep_t = .5

        delta_x = self.RobotUtils.TORSO_SHIFT_DELTA
        left_shift = self.RobotUtils.TORSO_LEFT_SHIFT

        l_torso_shift = [ delta_x, left_shift, 0 ]
        r_torso_shift = [ delta_x, -1*left_shift, 0 ]

        while 1:

            if not self.make_torso_shift_from_local_xyz_translation(l_torso_shift, MotionThread):
                break

            time.sleep(sleep_t)
            if not self.make_leg_step_by_new_state(b_r, base_state, MotionThread):
                break

            time.sleep(sleep_t)
            f_r_local_xyz_des = self.MotionPlanner.get_local_end_affector_base_state_from_torso_translation(f_r, r_torso_shift, 0)
            if not self.make_leg_step_to_local_xyz(f_r, f_r_local_xyz_des, MotionThread):
                break

            time.sleep(sleep_t)
            if not self.make_torso_shift_from_local_xyz_translation(r_torso_shift, MotionThread):
                break

            time.sleep(sleep_t)
            if not self.make_leg_step_by_new_state(b_l, base_state, MotionThread):
                break

            time.sleep(sleep_t)
            if not self.make_leg_step_by_new_state(f_l, base_state, MotionThread):
                break

            time.sleep(sleep_t)


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

        if not self.initialization_complete:
            if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                status = "Initilaization not complete, terminating"
                self.RobotUtils.ColorPrinter((self.__class__.__name__ + ".make_right_turn()"),status, "FAIL")
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

        link =  self.MotionPlanner.get_end_affector_from_end_affector_name(end_affector_name)

        local_xyz = link.getLocalPosition([0,0,0])

        new_x = local_xyz[0] + delta_x

        local_end_xyz = [new_x, local_xyz[1], local_xyz[2]]

        step_time = self.RobotUtils.STEP_TIME

        return self.linear_leg_step_to_local_xyz_in_t(end_affector_name, local_end_xyz, step_time, MotionThread)



    def make_leg_step_to_local_xyz(self, end_affector_name, local_end_xyz, MotionThread):


        step_time = self.RobotUtils.STEP_TIME

        result = self.linear_leg_step_to_local_xyz_in_t(end_affector_name, local_end_xyz, step_time, MotionThread)

        return result


    def make_leg_step_by_new_state(self, end_affector_name, end_leg_state, MotionThread):

        if not end_leg_state in self.RobotUtils.leg_states:
            self.RobotUtils.ColorPrinter((self.__class__.__name__+"make_leg_step_by_new_state()"), "Error: end leg state unrecognized", "FAIL")
            return None

        # Determine appropriate end local coordinates
        if end_leg_state == self.RobotUtils.LEG_F_EXTEND_STATE:
            local_end_xyz = self.MotionPlanner.get_extended_end_affector_local_xyz(end_affector_name, self.RobotUtils.FORWARD)

        elif end_leg_state == self.RobotUtils.LEG_B_EXTEND_STATE:
            local_end_xyz = self.MotionPlanner.get_extended_end_affector_local_xyz(end_affector_name,
                                                                           self.RobotUtils.BACKWARD)

        elif end_leg_state == self.RobotUtils.LEG_BASE_STATE:
            local_end_xyz = self.MotionPlanner.get_local_end_affector_base_state_from_end_affector_name(end_affector_name)

        else:
            self.RobotUtils.ColorPrinter((self.__class__.__name__+"make_leg_step_by_new_state()"), "Error: end leg state is recognized by RobotUtils,"
                                                                  "but not by make_leg_step_by_new_state()", "FAIL")
            return None

        return self.make_leg_step_to_local_xyz(end_affector_name, local_end_xyz, MotionThread)



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


            link = self.MotionPlanner.get_end_affector_from_end_affector_name(end_affector)
            link_local_xyz = link.getLocalPosition([0,0,0])
            local_end_xyz = self.MotionPlanner.get_local_end_affector_base_state_from_end_affector_name(end_affector)

            euclidian_delta = self.RobotUtils.get_euclidian_diff(link_local_xyz, local_end_xyz)

            global_end_xyz = self.MotionPlanner.get_world_xyz_from_local_xyz(local_end_xyz)
            global_start_xyz = self.MotionPlanner.get_world_xyz_from_local_xyz(link_local_xyz)

            vis.add((end_affector+" END XYZ"), global_end_xyz)
            vis.add((end_affector+" START XYZ"), global_start_xyz)

            if end_affector == "B_L_FOOT":
                print "testing: Back Left Foot, euclidean delta:",euclidian_delta, ", min to move:",self.RobotUtils.MINIMUM_DIST_TO_CAUSE_RESET
                print "current local xyz:",link_local_xyz
                print "          end xyz:",local_end_xyz

            if euclidian_delta > self.RobotUtils.MINIMUM_DIST_TO_CAUSE_RESET:
                print "RESETTING"
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


    def linear_leg_step_to_local_xyz_in_t(self, link_name, local_end_xyz, step_time, MotionThread=None, append_to_m_queue=True):

        """
        @param link_name: link name to move
        @param local_end_xyz: local xyz destination
        @param step_time: amount of time to perform the step in
        @param MotionThread
        @return: False if thread was suspended mid step. True otherwise
        """

        link = self.get_end_affector_from_end_affector_name(link_name)

        # Retrieve appropriate variables
        active_dofs = self.get_active_dofs_from_end_affector_name(link_name)
        desired_orientation = self.MotionPlanner.get_desired_end_affector_rotation(link_name)

        # Time calculations
        delay = self.measured_controller_dt
        i_max = int( step_time / float(delay))

        global_start_xyz = link.getWorldPosition([0, 0, 0])
        global_end_xyz = self.MotionPlanner.get_world_xyz_from_local_xyz(local_end_xyz)

        ik_max_deviation = self.RobotUtils.IK_MAX_DEVIATION

        if self.RobotUtils.get_euclidian_diff(global_start_xyz, global_end_xyz) < self.RobotUtils.MINIMUM_DIST_TO_CAUSE_RESET:
            self.RobotUtils.ColorPrinter((self.__class__.__name__+".linear_leg_step_to_local_xyz_in_t():"),("Negiligble change for leg:"+link_name+"Exiting function"), "STANDARD")
            return True

        t_start = time.time()

        for i in range(i_max):


            if MotionThread is None or ( (not MotionThread is None ) and MotionThread.is_alive()):

                xyz_des = self.MotionPlanner.get_linear_mid_motion_xyz(global_start_xyz, global_end_xyz, i, i_max)
                goal = ik.objective(link, R=desired_orientation, t=xyz_des)


                res = ik.solve_nearby(goal, activeDofs=active_dofs, maxDeviation=ik_max_deviation,
                                      feasibilityCheck=self.RobotUtils.always_true_func)

                # Failed
                if not res:
                    pass#self.RobotUtils.ColorPrinter((self.__class__.__name__+".linear_leg_step_to_local_xyz_in_t()"), "linear_leg_step_to_local_xyz_in_t ik failure", "FAIL")

                else:
                    if append_to_m_queue:
                        self.add_q_to_motion_queue(self.robosimian.getConfig())

                time.sleep(delay)
                delay = self.measured_controller_dt

            else:
                if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                    status = "Suspending motion thread for: " + link_name + " linear motion"
                    self.RobotUtils.ColorPrinter(self.__class__.__name__, status, "FAIL")
                return False

        t_end =  time.time()
        t_total = t_end - t_start
        print( ("Step took: "+str(t_total)+"seconds") )
        return True


    def make_torso_shift_from_local_xyz_translation(self, xyz_translation, MotionThread):

        """

        @param xyz_translation: local xyz destination for torso shift
        @param MotionThread:    parent Motion Thread
        @return: False if thread was suspended mid step. True otherwise
        """

        # Time calculations
        delay = self.measured_controller_dt
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

                self.add_q_to_motion_queue(self.robosimian.getConfig())

                time.sleep(delay)
                delay = self.measured_controller_dt

            else:
                if self.RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                    self.RobotUtils.ColorPrinter((self.__class__.__name__+"make_torso_shift_from_local_xyz_translation()"), "Suspending motion thread", "FAIL")
                return False

        return True



    def make_torso_rotation(self, degree, MotionThread):

        offset = 1
        delay = self.measured_controller_dt
        i_max = int(math.fabs(degree))

        if degree < 0:
            offset *= -1

        for i in range(i_max):

            if MotionThread.is_alive():
                self.rotate_torso_from_yaw_offset(offset)
                time.sleep(delay)
                delay = self.measured_controller_dt
            else:
                return False
        return True


    #                                      Non-Threaded Low Level IK Solver APIS
    # ----------------------------------                                         ------------------------------------
    #
    # Desc: These functions will calculate a new robot state given a speficied change (torso rotation, end_affector movement, ect)
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

        f_l = self.f_l_end_affector
        f_r = self.f_r_end_affector
        b_l = self.b_l_end_affector
        b_r = self.b_r_end_affector

        # World Leg Positions
        f_l_global = f_l.getWorldPosition([0, 0, 0])
        f_r_global = f_r.getWorldPosition([0, 0, 0])
        b_l_global = b_l.getWorldPosition([0, 0, 0])
        b_r_global = b_r.getWorldPosition([0, 0, 0])

        # Desired Leg orientation
        f_l_desired_orientation = self.MotionPlanner.get_desired_end_affector_rotation(self.RobotUtils.F_L_FOOT)
        f_r_desired_orientation = self.MotionPlanner.get_desired_end_affector_rotation(self.RobotUtils.F_R_FOOT)
        b_l_desired_orientation = self.MotionPlanner.get_desired_end_affector_rotation(self.RobotUtils.B_L_FOOT)
        b_r_desired_orientation = self.MotionPlanner.get_desired_end_affector_rotation(self.RobotUtils.B_R_FOOT)

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
            self.add_q_to_motion_queue(self.robosimian.getConfig())


    def shift_torso_to_global_xyz(self, global_xyz):

        """
        @summary shifts the torso to a specified global coordinate and appends new robot state to motion queue. Position
                 change must be small as this function does not access the parent Motion Thread
        @param  global_xyz: desired global xyz end destination
        @return: None
        """

        torso = self.robosimian.link(self.RobotUtils.TORSO_LINK_INDEX)

        f_l = self.f_l_end_affector
        f_r = self.f_r_end_affector
        b_l = self.b_l_end_affector
        b_r = self.b_r_end_affector

        # World Leg Positions
        f_l_global = f_l.getWorldPosition([0, 0, 0])
        f_r_global = f_r.getWorldPosition([0, 0, 0])
        b_l_global = b_l.getWorldPosition([0, 0, 0])
        b_r_global = b_r.getWorldPosition([0, 0, 0])

        # Desired Leg orientation
        left_leg_desired_orientation = self.MotionPlanner.get_desired_end_affector_rotation(self.RobotUtils.F_L_FOOT)
        right_leg_desired_orientation = self.MotionPlanner.get_desired_end_affector_rotation(self.RobotUtils.F_R_FOOT)

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
            self.add_q_to_motion_queue(self.robosimian.getConfig())
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

    # Helper function to return the leg joints of a given end_affector.
    def get_active_dofs_from_end_affector_name(self, end_affector_name):

        if not end_affector_name in self.RobotUtils.end_affectors:
            self.RobotUtils.ColorPrinter(self.__class__.__name__,
                                         "get_active_dofs_from_end_affector_name: Error: end_affector name unrecognized", "FAIL")
            return None

        if end_affector_name == self.RobotUtils.B_L_FOOT:
            return self.RobotUtils.b_l_active_dofs

        elif (end_affector_name == self.RobotUtils.B_R_FOOT):
            return self.RobotUtils.b_r_active_dofs

        elif end_affector_name == self.RobotUtils.F_L_FOOT:
            return self.RobotUtils.f_l_active_dofs

        else:
            return self.RobotUtils.f_r_active_dofs


    def get_end_affector_from_end_affector_name(self, end_affector_name):

        return self.MotionPlanner.get_end_affector_from_end_affector_name(end_affector_name)



