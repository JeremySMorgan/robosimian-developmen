#!/usr/bin/python
import math
import numpy as np
import time
import Queue
from klampt.model import ik
from klampt import vis
import time
from ...Utilities.RobotUtils.RobotUtils import RobotUtils

class HighLevelMotionController(object):

    def __init__(self, robot, Controller):

        self.robosimian = robot

        self.f_r_end_affector = self.robosimian.link(RobotUtils.f_r_active_dofs[len(RobotUtils.f_r_active_dofs) - 1])
        self.f_l_end_affector = self.robosimian.link(RobotUtils.f_l_active_dofs[len(RobotUtils.f_l_active_dofs) - 1])
        self.b_r_end_affector = self.robosimian.link(RobotUtils.b_r_active_dofs[len(RobotUtils.b_r_active_dofs) - 1])
        self.b_l_end_affector = self.robosimian.link(RobotUtils.b_l_active_dofs[len(RobotUtils.b_l_active_dofs) - 1])

        # stores whether a leg is currently making a step
        #  [F_R, F_L, B_R, B_L]
        self.link_currently_midstep = [False, False, False, False]

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
    #           - initialize_motion_plannerR
    #           - set_inital_config


    def initialize_motion_planner(self, motion_planner):

        self.MotionPlanner = motion_planner

    def set_inital_config(self):

        pos_x_shoulder_base = RobotUtils.SHOULDER_X
        pos_y_shoulder_base = RobotUtils.SHOULDER_Y

        pos_x_end_aff_delta = RobotUtils.BASE_STATE_X_DELTA
        pos_y_end_aff_delta = RobotUtils.BASE_STATE_Y_DELTA

        end_eff_z_pos       = RobotUtils.BASE_STATE_Z_DELTA

        f_r_local_base_state = [ pos_x_shoulder_base + pos_x_end_aff_delta, -pos_y_shoulder_base - pos_y_end_aff_delta, end_eff_z_pos ]
        f_l_local_base_state = [ pos_x_shoulder_base + pos_x_end_aff_delta, pos_y_shoulder_base + pos_y_end_aff_delta, end_eff_z_pos ]
        b_l_local_base_state = [ -pos_x_shoulder_base - pos_x_end_aff_delta, pos_y_shoulder_base + pos_y_end_aff_delta, end_eff_z_pos ]
        b_r_local_base_state = [ -pos_x_shoulder_base  - pos_x_end_aff_delta, -pos_y_shoulder_base - pos_y_end_aff_delta, end_eff_z_pos ]

        end_effactors = [ RobotUtils.F_R_FOOT,  RobotUtils.F_L_FOOT,   RobotUtils.B_L_FOOT,   RobotUtils.B_R_FOOT ]
        base_states =   [ f_r_local_base_state,      f_l_local_base_state,       b_l_local_base_state,       b_r_local_base_state     ]

        step_time = RobotUtils.INITIALIZATION_STEP_TIME

        for i in range(len(base_states)):
            local_xyz_des = base_states[i]
            end_effactor = end_effactors[i]
            vis.add((end_effactor+" local base state"),local_xyz_des)
            self.linear_leg_step_to_local_xyz_in_t(end_effactor, local_xyz_des, step_time, override_max_euclid_dist = True)

        self.MotionPlanner.save_base_states()

        if RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
            status = "Initilization Finished"
            RobotUtils.ColorPrinter((self.__class__.__name__+"set_inital_config()"),status,"OKGREEN" )

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

        print_str = "motion queue size: "+str(self.motion_queue.qsize())+ ",\t measured dt:"+RobotUtils.pp_double(self.measured_controller_dt)
        #print print_str
        if not self.motion_queue.empty():

            calculated_next_config = self.motion_queue.get_nowait()
            self.MotionController.setLinear(calculated_next_config, RobotUtils.CONTROLLER_DT)


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
            if RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                status = "Different motion thread is currently running, terminating"
                RobotUtils.ColorPrinter((self.__class__.__name__ + ".make_right_turn()"),status, "FAIL")
            return

        if not self.initialization_complete:
            if RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                status = "Initilaization not complete, terminating"
                RobotUtils.ColorPrinter((self.__class__.__name__ + ".make_right_turn()"),status, "FAIL")
            return

        self.motion_thread_currently_running = True

        if RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
            RobotUtils.ColorPrinter((self.__class__.__name__+".make_right_turn()"), "Starting right turn", "STANDARD")

        cancelled = False
        torso_rotation_angle = -RobotUtils.TORSO_YAW_ROTATE_ANGLE
        step_time = RobotUtils.TURN_TIME
        midstep_sleep_t = RobotUtils.TURNING_MIDSTEP_SLEEP_T

        while 1:
            for leg in RobotUtils.end_affectors:

                local_end_xyz = self.MotionPlanner.get_local_turn_desitination(leg, torso_rotation_angle)

                if not self.linear_leg_step_to_local_xyz_in_t(leg, local_end_xyz, step_time, MotionThread):
                    cancelled = True
                    break
                time.sleep(midstep_sleep_t)

            if cancelled:
                break

            time.sleep(midstep_sleep_t)
            if not self.make_torso_rotation_from_degree_offset(torso_rotation_angle, MotionThread): break

        self.clear_motion_queue()
        self.reset_to_base_state()
        self.motion_thread_currently_running = False




    def make_left_turn(self, MotionThread):


        if self.motion_thread_currently_running:
            if RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                status = "Different motion thread is currently running, terminating"
                RobotUtils.ColorPrinter((self.__class__.__name__ + ".make_left_turn()"), status, "FAIL")
            return

        if not self.initialization_complete:
            if RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                status = "Initilaization not complete, terminating"
                RobotUtils.ColorPrinter((self.__class__.__name__ + ".make_right_turn()"), status, "FAIL")
            return

        self.motion_thread_currently_running = True

        if RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
            RobotUtils.ColorPrinter((self.__class__.__name__+".make_left_turn()"), "Starting left turn", "STANDARD")

        cancelled = False
        torso_rotation_angle = RobotUtils.TORSO_YAW_ROTATE_ANGLE
        step_time = RobotUtils.TURN_TIME
        midstep_sleep_t = RobotUtils.TURNING_MIDSTEP_SLEEP_T

        while 1:
            for leg in RobotUtils.end_affectors:

                local_end_xyz = self.MotionPlanner.get_local_turn_desitination(leg, torso_rotation_angle)

                if not self.linear_leg_step_to_local_xyz_in_t(leg, local_end_xyz, step_time, MotionThread):
                    cancelled = True
                    break
                time.sleep(midstep_sleep_t)

            if cancelled:
                break

            time.sleep(midstep_sleep_t)
            if not self.make_torso_rotation_from_degree_offset(torso_rotation_angle, MotionThread):
                break

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
            if RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                status = "Different motion thread is currently running, terminating"
                RobotUtils.ColorPrinter((self.__class__.__name__ + ".forward()"), status, "FAIL")
            return

        if not self.initialization_complete:
            if RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                status = "Initilaization not complete, terminating"
                RobotUtils.ColorPrinter((self.__class__.__name__ + ".make_right_turn()"),status, "FAIL")
            return

        self.motion_thread_currently_running = True

        if RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
            RobotUtils.ColorPrinter((self.__class__.__name__+".forward_walk()"), "Starting forward walk", "STANDARD")

        f_r = RobotUtils.F_R_FOOT
        b_r = RobotUtils.B_R_FOOT
        f_l = RobotUtils.F_L_FOOT
        b_l = RobotUtils.B_L_FOOT

        if RobotUtils.PHYSICS_ENABLED:
            sleep_t = 2
        else:
            sleep_t = .5

        delta_x = RobotUtils.TORSO_SHIFT_DELTA
        left_shift = RobotUtils.TORSO_LEFT_SHIFT

        l_torso_shift = [delta_x, left_shift, 0]
        r_torso_shift = [delta_x, -1*left_shift, 0]

        while 1:

            if not self.make_torso_shift_from_local_xyz_translation(l_torso_shift, MotionThread):
                break

            time.sleep(sleep_t)
            if not self.make_leg_step_to_base_state_defined_by_current_torso_pos(b_r, MotionThread):

                break

            time.sleep(sleep_t)
            f_r_local_xyz_des = self.MotionPlanner.get_local_end_affector_base_state_from_torso_translation(f_r, r_torso_shift, 0)
            if not self.make_leg_step_to_local_xyz(f_r, f_r_local_xyz_des, MotionThread):
                break

            time.sleep(sleep_t)
            if not self.make_torso_shift_from_local_xyz_translation(r_torso_shift, MotionThread):
                break

            time.sleep(sleep_t)
            if not self.make_leg_step_to_base_state_defined_by_current_torso_pos(b_l, MotionThread):
                break

            time.sleep(sleep_t)
            if not self.make_leg_step_to_base_state_defined_by_current_torso_pos(f_l, MotionThread):
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
            if RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                status = "Different motion thread is currently running, terminating"
                RobotUtils.ColorPrinter((self.__class__.__name__ + ".backward()"), status, "FAIL")
            return

        if not self.initialization_complete:
            if RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                status = "Initilaization not complete, terminating"
                RobotUtils.ColorPrinter((self.__class__.__name__ + ".make_right_turn()"),status, "FAIL")
            return

        self.motion_thread_currently_running = True

        if RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
            RobotUtils.ColorPrinter((self.__class__.__name__+"backward_walk()"), "Starting backward walk", "STANDARD")

        # Implement Here

        self.motion_thread_currently_running = False



    def make_leg_step_to_local_xyz(self, end_affector_name, local_end_xyz, MotionThread):


        step_time = RobotUtils.STEP_TIME

        result = self.linear_leg_step_to_local_xyz_in_t(end_affector_name, local_end_xyz, step_time, MotionThread)

        return result



    def make_leg_step_to_base_state_defined_by_current_torso_pos(self,end_affector_name, MotionThread = None):



        local_end_xyz = self.MotionPlanner.get_local_end_affector_base_state_from_end_affector_name(end_affector_name)
        return self.make_leg_step_to_local_xyz(end_affector_name, local_end_xyz, MotionThread)


    def reset_to_base_state(self):

        '''
        @summary resets the robot to base state. Algorithm ensure stability. see https://github.com/JeremySMorgan/planning_world_robosimian-gait-development for explanation
        @return: None
        '''

        if RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
            RobotUtils.ColorPrinter((self.__class__.__name__+"reset_to_base_state()"), "Resetting to base state", "STANDARD")

        default_torso_z = self.robosimian.getConfig()[2]
        default_end_affectr_z = RobotUtils.BASE_STATE_Z_DELTA
        step_time = RobotUtils.RESET_LEG_STEP_TIME
        sleep_t = 1.5

        for end_affector in RobotUtils.end_affectors:

            if self.link_currently_midstep[RobotUtils.end_affectors.index(end_affector)]:
                end_affector_current_world_xyz = self.MotionPlanner.get_end_affector_from_end_affector_name(end_affector).getWorldPosition([0,0,0])
                end_xyz = [end_affector_current_world_xyz[0], end_affector_current_world_xyz[1], default_end_affectr_z]
                self.linear_leg_step_to_global_xyz_in_t(end_affector,end_xyz,step_time)

        if self.MotionPlanner.legs_make_base_state():

            if RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                RobotUtils.ColorPrinter((self.__class__.__name__ + "reset_to_base_state()"), "Robot's leg currently make base state - shifting and rotating torso to center", "STANDARD")
            yaw_offset_deg, torso_world_xyz = self.MotionPlanner.get_delta_yaw_deg_and_world_torso_xyz_commanded_from_legs()
            self.make_torso_rotation_from_degree_offset(yaw_offset_deg)
            self.make_torso_shift_to_world_xyz(torso_world_xyz)

        elif self.MotionPlanner.three_legs_make_base_state():

            if RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                #RobotUtils.ColorPrinter((self.__class__.__name__ + "reset_to_base_state()"), "3 Legs make a base state. Resetting", "STANDARD")
                pass
            end_affector_to_move = self.MotionPlanner.three_legs_make_base_state()
            torso_final_yaw_degrees, Bt = self.MotionPlanner.get_abs_yaw_deg_and_world_torso_xyz_commanded_from_legs(excluded_leg=end_affector_to_move)
            end_affector_to_move_base_xyz = self.MotionPlanner.get_end_affectr_base_world_xyz_from_torso_world_xyz_and_yaw_deg( end_affector_to_move, Bt, torso_final_yaw_degrees)
            q = self.robosimian.getConfig()
            curr_torso_world_x = q[0]
            curr_torso_world_y = q[1]
            curr_torso_world_z = q[1]
            curr_torso_world_xyz = [curr_torso_world_x, curr_torso_world_y, curr_torso_world_z]

            # Initialize support triangles and range circles
            end_affectors = [RobotUtils.F_R_FOOT, RobotUtils.F_L_FOOT, RobotUtils.B_R_FOOT, RobotUtils.B_L_FOOT]
            support_triangle = self.MotionPlanner.get_world_support_triangle_from_excluded_end_affector(end_affector_to_move)
            _2DGeometry_objs = [support_triangle]
            for end_affectr in end_affectors:
                range_circle = self.MotionPlanner.get_end_affector_2D_support_circle_from_name(end_affectr, circle_name=end_affectr)
                _2DGeometry_objs.append(range_circle)

            end_affect_base_state_range_circle = self.MotionPlanner.get_end_affector_2D_support_circle_from_name(end_affector_to_move, circle_name="end_affectr to move base state")
            _2DGeometry_objs.append(end_affect_base_state_range_circle)
            if RobotUtils.RESET_VISUALIZATION_ENABLED:
                for poly in _2DGeometry_objs: poly.visualize()

            time.sleep(sleep_t)

            # Shift torso to center of support triangle if it is not in it already
            if not self.MotionPlanner.point_is_in_multiple_support_polygon_intersections(curr_torso_world_xyz, _2DGeometry_objs):

                global_support_tri_center_xy = self.MotionPlanner.get_centroid_from_multiple_poly_intersections(_2DGeometry_objs)
                global_support_tri_center_xyz = [global_support_tri_center_xy[0],global_support_tri_center_xy[1], default_torso_z]
                self.make_torso_shift_to_world_xyz(global_support_tri_center_xyz)


            time.sleep(sleep_t)
            self.linear_leg_step_to_global_xyz_in_t(end_affector_to_move, end_affector_to_move_base_xyz, step_time)

            time.sleep(sleep_t)
            self.make_torso_shift_to_world_xyz(Bt)

            time.sleep(sleep_t)
            self.make_torso_rotation_to_abs_angle(torso_final_yaw_degrees)

            if RobotUtils.RESET_VISUALIZATION_ENABLED:
                for poly in _2DGeometry_objs: poly.remove_visualization()

        else:

            curr_torso_x = self.robosimian.getConfig()[0]
            curr_torso_y = self.robosimian.getConfig()[1]
            curr_torso_z = self.robosimian.getConfig()[2]

            P_0bl = self.MotionPlanner.b_l_end_affector.getWorldPosition([0,0,0])
            P_0br = self.MotionPlanner.b_r_end_affector.getWorldPosition([0,0,0])
            P_0fl = self.MotionPlanner.f_l_end_affector.getWorldPosition([0,0,0])
            P_0fr = self.MotionPlanner.f_r_end_affector.getWorldPosition([0,0,0])

            B_bl  = self.MotionPlanner.get_world_xyz_from_local_xyz(self.MotionPlanner.get_local_end_affector_base_state_from_end_affector_name(RobotUtils.B_L_FOOT))
            B_br  = self.MotionPlanner.get_world_xyz_from_local_xyz(self.MotionPlanner.get_local_end_affector_base_state_from_end_affector_name(RobotUtils.B_R_FOOT))
            B_fl  = self.MotionPlanner.get_world_xyz_from_local_xyz(self.MotionPlanner.get_local_end_affector_base_state_from_end_affector_name(RobotUtils.F_L_FOOT))
            B_fr  = self.MotionPlanner.get_world_xyz_from_local_xyz(self.MotionPlanner.get_local_end_affector_base_state_from_end_affector_name(RobotUtils.F_R_FOOT))
            B_t   = [curr_torso_x, curr_torso_y, curr_torso_z]

            # --- left legs
            front_left_support_triangle         = self.MotionPlanner.get_support_polygon_from_points([P_0bl, P_0fr, P_0br], name="front left ST")
            back_left_support_triangle          = self.MotionPlanner.get_support_polygon_from_points([B_fl, P_0fr, P_0br], name="back left ST")
            f_l_end_affector_range_circle       = self.MotionPlanner.get_end_affector_2D_support_circle_from_name(RobotUtils.F_L_FOOT, circle_name="front left range")
            f_l_end_affector_range_circle_at_bs = self.MotionPlanner.get_end_affector_2D_support_circle_from_name(RobotUtils.F_L_FOOT, circle_name="front left at base",at_point=B_fl)
            b_l_end_affector_range_circle       = self.MotionPlanner.get_end_affector_2D_support_circle_from_name(RobotUtils.B_L_FOOT, circle_name="back left range")
            b_l_end_affector_range_circle_at_bs = self.MotionPlanner.get_end_affector_2D_support_circle_from_name(RobotUtils.B_L_FOOT, circle_name="back left range at base",at_point=B_bl)
            f_r_end_affector_range_circle       = self.MotionPlanner.get_end_affector_2D_support_circle_from_name(RobotUtils.F_R_FOOT, circle_name="front right range")
            b_r_end_affector_range_circle       = self.MotionPlanner.get_end_affector_2D_support_circle_from_name(RobotUtils.B_R_FOOT, circle_name="back right range")

            polys = [front_left_support_triangle, back_left_support_triangle, f_l_end_affector_range_circle, b_l_end_affector_range_circle, f_r_end_affector_range_circle, b_r_end_affector_range_circle,f_l_end_affector_range_circle_at_bs, b_l_end_affector_range_circle_at_bs]
            if RobotUtils.RESET_VISUALIZATION_ENABLED:
                for poly in polys: poly.visualize()

            # Shift torso to Pt
            Pt = self.MotionPlanner.get_centroid_from_multiple_poly_intersections(polys)
            Pt = [Pt[0],Pt[1], curr_torso_z]

            time.sleep(sleep_t)
            self.make_torso_shift_to_world_xyz(Pt)

            time.sleep(sleep_t)
            self.linear_leg_step_to_global_xyz_in_t( RobotUtils.F_L_FOOT, B_fl, step_time)

            time.sleep(sleep_t)
            self.linear_leg_step_to_global_xyz_in_t( RobotUtils.B_L_FOOT, B_bl, step_time)

            if RobotUtils.RESET_VISUALIZATION_ENABLED:
                for poly in polys: poly.remove_visualization()

            # --- right legs

            # Support triangles and end affector ranges
            front_right_support_triangle        = self.MotionPlanner.get_support_polygon_from_points([B_fl, B_bl, B_fr])
            back_right_support_triangle         = self.MotionPlanner.get_support_polygon_from_points([B_fl, B_bl, P_0fr])
            f_l_end_affector_range_circle = self.MotionPlanner.get_end_affector_2D_support_circle_from_name( RobotUtils.F_L_FOOT, circle_name="front left range")
            b_l_end_affector_range_circle = self.MotionPlanner.get_end_affector_2D_support_circle_from_name( RobotUtils.B_L_FOOT, circle_name="back left range")

            f_r_end_affector_range_circle = self.MotionPlanner.get_end_affector_2D_support_circle_from_name( RobotUtils.F_R_FOOT, circle_name="front right range")
            f_r_end_affector_range_circle_at_bs  = self.MotionPlanner.get_end_affector_2D_support_circle_from_name(RobotUtils.F_R_FOOT, circle_name="front right range at base",at_point=B_fr)

            b_r_end_affector_range_circle       = self.MotionPlanner.get_end_affector_2D_support_circle_from_name(RobotUtils.B_R_FOOT, circle_name="back right range")
            b_r_end_affector_range_circle_at_bs  = self.MotionPlanner.get_end_affector_2D_support_circle_from_name(RobotUtils.B_R_FOOT, circle_name="back right range at base", at_point=B_br)

            polys = [ front_right_support_triangle,back_right_support_triangle,  f_l_end_affector_range_circle, b_l_end_affector_range_circle, f_r_end_affector_range_circle, b_r_end_affector_range_circle, f_r_end_affector_range_circle_at_bs, b_r_end_affector_range_circle_at_bs]
            if RobotUtils.RESET_VISUALIZATION_ENABLED:
                for poly in polys: poly.visualize()

            # Shift torso to Pt
            Pt2 = self.MotionPlanner.get_centroid_from_multiple_poly_intersections(polys)
            Pt2 = [Pt2[0],Pt2[1], curr_torso_z]

            time.sleep(sleep_t)
            self.make_torso_shift_to_world_xyz(Pt2)

            time.sleep(sleep_t)
            self.linear_leg_step_to_global_xyz_in_t(RobotUtils.F_R_FOOT, B_fr, step_time)

            time.sleep(sleep_t)
            self.linear_leg_step_to_global_xyz_in_t(RobotUtils.B_R_FOOT, B_br, step_time)

            time.sleep(sleep_t)
            self.make_torso_shift_to_world_xyz(B_t)

            if RobotUtils.RESET_VISUALIZATION_ENABLED:
                for poly in polys: poly.remove_visualization()

    #                                      Threaded Moiton in Speficied Time APIS
    # -----------------------------------                                         ------------------------------------
    #
    # Desc: These functions will executed a desired step pattern in a speficied amount of time (specified in RobotUtils)
    #       and add each new calculated robot state to the motion queue. They will return False if they detect that the
    #       thread they are being run in is suspended, otherwise true
    #
    # Contains:
    #           - linear_leg_step_to_global_xyz_in_t
    #           - linear_leg_step_to_local_xyz_in_t
    #           - make_torso_shift_from_local_xyz_translation


    def linear_leg_step_to_global_xyz_in_t(self, link_name, global_end_xyz, step_time, MotionThread=None, append_to_m_queue=True, override_max_euclid_dist=False):

        """
        @param link_name: link name to move
        @param global_end_xyz: local xyz destination
        @param step_time: amount of time to perform the step in
        @param MotionThread
        @return: False if thread was suspended mid step. True otherwise
        """

        link = self.get_end_affector_from_end_affector_name(link_name)
        self.link_currently_midstep[RobotUtils.end_affectors.index(link_name)] = True

        # Retrieve appropriate variables
        active_dofs = self.get_active_dofs_from_end_affector_name(link_name)
        desired_orientation = self.MotionPlanner.get_desired_end_affector_rotation(link_name)

        # Time calculations
        delay = self.measured_controller_dt
        i_max = int( step_time / float(delay))

        global_start_xyz = link.getWorldPosition([0, 0, 0])

        ik_max_deviation = RobotUtils.IK_MAX_DEVIATION

        if not override_max_euclid_dist:
            euclidian_dif = RobotUtils.get_euclidian_diff(global_start_xyz, global_end_xyz)
            print "euclidian dif for",link_name,":",RobotUtils.pp_double(euclidian_dif), " minumum to cause reset: ",RobotUtils.MINIMUM_DIST_TO_CAUSE_RESET
            if euclidian_dif < RobotUtils.MINIMUM_DIST_TO_CAUSE_RESET:
                print "RESET Triggered"
                #RobotUtils.ColorPrinter((self.__class__.__name__+".linear_leg_step_to_global_xyz_in_t():"),("Negiligble change for leg: "+link_name+", exiting function"), "STANDARD")
                return True

        t_start = time.time()

        for i in range(i_max):

            if MotionThread is None or ( (not MotionThread is None ) and MotionThread.is_alive()):

                xyz_des = self.MotionPlanner.get_linear_mid_motion_xyz(global_start_xyz, global_end_xyz, i, i_max)
                goal = ik.objective(link, R=desired_orientation, t=xyz_des)


                res = ik.solve_nearby(goal, activeDofs=active_dofs, maxDeviation=ik_max_deviation,
                                      feasibilityCheck=RobotUtils.always_true_func)

                # Failed
                if not res:
                    RobotUtils.ColorPrinter((self.__class__.__name__+".linear_leg_step_to_global_xyz_in_t()"), " ik failure", "FAIL")

                else:
                    if append_to_m_queue:
                        self.add_q_to_motion_queue(self.robosimian.getConfig())

                time.sleep(delay)
                delay = self.measured_controller_dt

            else:
                if RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                    status = "Suspending motion thread for: " + link_name + " linear motion"
                    RobotUtils.ColorPrinter(self.__class__.__name__, status, "FAIL")
                return False



        t_total = time.time() - t_start
        #print( ("Step took: "+str(t_total)+"seconds") )
        self.link_currently_midstep[RobotUtils.end_affectors.index(link_name)] = False
        return True





    def linear_leg_step_to_local_xyz_in_t(self, link_name, local_end_xyz, step_time, MotionThread=None, append_to_m_queue=True, override_max_euclid_dist=False):

        """
        @param link_name: link name to move
        @param local_end_xyz: local xyz destination
        @param step_time: amount of time to perform the step in
        @param MotionThread
        @return: False if thread was suspended mid step. True otherwise
        """

        global_end_xyz = self.MotionPlanner.get_world_xyz_from_local_xyz(local_end_xyz)

        return self.linear_leg_step_to_global_xyz_in_t(link_name, global_end_xyz, step_time, MotionThread, append_to_m_queue, override_max_euclid_dist)



    def make_torso_shift_to_world_xyz(self, world_xyz, MotionThread=None):

        """

        @param xyz_translation: local xyz destination for torso shift
        @param MotionThread:    parent Motion Thread
        @return: False if thread was suspended mid step. True otherwise
        """

        # Time calculations
        delay = self.measured_controller_dt
        i_max = int(RobotUtils.TORSO_SHIFT_TIME / delay)

        global_xyz_start = self.MotionPlanner.get_world_xyz_from_local_xyz([0, 0, 0])

        start_x = global_xyz_start[0]
        start_y = global_xyz_start[1]
        start_z = global_xyz_start[2]

        end_x = world_xyz[0]
        end_y = world_xyz[1]
        end_z = world_xyz[2]

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
                if RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                    RobotUtils.ColorPrinter((self.__class__.__name__+"make_torso_shift_from_local_xyz_translation()"), "Suspending motion thread", "FAIL")
                return False

        return True

    def make_torso_shift_from_local_xyz_translation(self, xyz_translation, MotionThread=None):

        """

        @param xyz_translation: local xyz destination for torso shift
        @param MotionThread:    parent Motion Thread
        @return: False if thread was suspended mid step. True otherwise
        """

        # Time calculations
        delay = self.measured_controller_dt
        i_max = int(RobotUtils.TORSO_SHIFT_TIME / delay)

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
                if RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                    RobotUtils.ColorPrinter((self.__class__.__name__+"make_torso_shift_from_local_xyz_translation()"), "Suspending motion thread", "FAIL")
                return False

        return True


    def make_torso_rotation_to_abs_angle(self, degree, MotionThread=None):

        degree_offset = degree - np.rad2deg(self.MotionPlanner.get_current_torso_yaw_rads())

        return self.make_torso_rotation_from_degree_offset(degree_offset, MotionThread=MotionThread)


    def make_torso_rotation_from_degree_offset(self, degree, MotionThread=None):

        '''

        @param degree: degree offset to rotate torso by
        @param MotionThread: MotionThread containing this torso rotation
        @return: True if success, False if motion thread stopped midway through
        '''

        # negligible change
        if np.fabs( degree ) < 1:
            return True

        offset = 1
        i_max = int(math.fabs(degree))
        sum_time = RobotUtils.TORSO_YAW_ROTATE_TIME
        robot_utils_specified_delay = sum_time / i_max

        if degree < 0:
            offset *= -1

        i_max = int(i_max)
        for i in range(i_max):
            if MotionThread is None or ((not MotionThread is None) and MotionThread.is_alive()):

                self.rotate_torso_from_yaw_offset(offset)

                if self.measured_controller_dt > robot_utils_specified_delay:
                    time.sleep(self.measured_controller_dt)
                else:
                    time.sleep(robot_utils_specified_delay)

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

        torso = self.robosimian.link(RobotUtils.TORSO_LINK_INDEX)

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
        f_l_desired_orientation = self.MotionPlanner.get_desired_end_affector_rotation(RobotUtils.F_L_FOOT)
        f_r_desired_orientation = self.MotionPlanner.get_desired_end_affector_rotation(RobotUtils.F_R_FOOT)
        b_l_desired_orientation = self.MotionPlanner.get_desired_end_affector_rotation(RobotUtils.B_L_FOOT)
        b_r_desired_orientation = self.MotionPlanner.get_desired_end_affector_rotation(RobotUtils.B_R_FOOT)

        # ik obkectives
        f_l_r_const = ik.objective(f_l, R=f_l_desired_orientation, t=f_l_global)
        f_r_r_const = ik.objective(f_r, R=f_r_desired_orientation, t=f_r_global)
        b_l_r_const = ik.objective(b_l, R=b_l_desired_orientation, t=b_l_global)
        b_r_r_const = ik.objective(b_r, R=b_r_desired_orientation, t=b_r_global)

        des_torso_rotation = self.MotionPlanner.get_desired_torso_R_from_yaw_offset(yaw_offset_deg)

        global_torso_xyz = torso.getWorldPosition([0, 0, 0])

        torso_obj = ik.objective(torso, R=des_torso_rotation, t=global_torso_xyz)

        goal = [f_l_r_const, f_r_r_const, b_l_r_const, b_r_r_const, torso_obj]

        ik_max_deviation = RobotUtils.IK_MAX_DEVIATION

        res = ik.solve_nearby(goal, maxDeviation=ik_max_deviation, feasibilityCheck=RobotUtils.always_true_func)

        if not res:
            RobotUtils.ColorPrinter(self.__class__.__name__, "torso ik failure", "FAIL")

        else:
            self.add_q_to_motion_queue(self.robosimian.getConfig())


    def shift_torso_to_global_xyz(self, global_xyz):

        """
        @summary shifts the torso to a specified global coordinate and appends new robot state to motion queue. Position
                 change must be small as this function does not access the parent Motion Thread
        @param  global_xyz: desired global xyz end destination
        @return: None
        """

        torso = self.robosimian.link(RobotUtils.TORSO_LINK_INDEX)

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
        left_leg_desired_orientation = self.MotionPlanner.get_desired_end_affector_rotation(RobotUtils.F_L_FOOT)
        right_leg_desired_orientation = self.MotionPlanner.get_desired_end_affector_rotation(RobotUtils.F_R_FOOT)

        # ik obkectives
        f_l_r_const = ik.objective(f_l, R=left_leg_desired_orientation, t=f_l_global)
        f_r_r_const = ik.objective(f_r, R=right_leg_desired_orientation, t=f_r_global)
        b_l_r_const = ik.objective(b_l, R=left_leg_desired_orientation, t=b_l_global)
        b_r_r_const = ik.objective(b_r, R=right_leg_desired_orientation, t=b_r_global)

        torso_obj = ik.objective(torso, R=torso.getTransform()[0], t=global_xyz)

        goal = [f_l_r_const, f_r_r_const, b_l_r_const, b_r_r_const, torso_obj]

        ik_max_deviation = 10 * RobotUtils.IK_MAX_DEVIATION

        res = ik.solve_nearby(goal, maxDeviation=ik_max_deviation,
                              feasibilityCheck=RobotUtils.always_true_func)

        if res:
            self.add_q_to_motion_queue(self.robosimian.getConfig())
        else:
            if RobotUtils.HIGH_LEVEL_MOTION_PLANNER_DEBUGGING_ENABLED:
                RobotUtils.ColorPrinter(self.__class__.__name__, "torso ik failure", "FAIL")








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

        if not end_affector_name in RobotUtils.end_affectors:
            RobotUtils.ColorPrinter(self.__class__.__name__,
                                         "get_active_dofs_from_end_affector_name: Error: end_affector name unrecognized", "FAIL")
            return None

        if end_affector_name == RobotUtils.B_L_FOOT:
            return RobotUtils.b_l_active_dofs

        elif (end_affector_name == RobotUtils.B_R_FOOT):
            return RobotUtils.b_r_active_dofs

        elif end_affector_name == RobotUtils.F_L_FOOT:
            return RobotUtils.f_l_active_dofs

        else:
            return RobotUtils.f_r_active_dofs


    def get_end_affector_from_end_affector_name(self, end_affector_name):

        return self.MotionPlanner.get_end_affector_from_end_affector_name(end_affector_name)



