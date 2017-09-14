#!/usr/bin/python
import math
import time
from klampt.model import ik,coordinates


class MotionController(object):

    def __init__(self,robot,RobotUtils):
        self.robosimian = robot
        self.RobotUtils = RobotUtils
        self.initialize_feet()
        self.MotionPlanner = None

        if RobotUtils.SIMULATION_ENABLED:
            from klampt import vis


    def initialize_motion_planner(self,motion_planner):
        self.MotionPlanner = motion_planner



    def initialize_feet(self):

        self.f_r_foot = self.robosimian.link(self.RobotUtils.f_r_active_dofs[len(self.RobotUtils.f_r_active_dofs) - 1])
        self.f_l_foot = self.robosimian.link(self.RobotUtils.f_l_active_dofs[len(self.RobotUtils.f_l_active_dofs) - 1])
        self.b_r_foot = self.robosimian.link(self.RobotUtils.b_r_active_dofs[len(self.RobotUtils.b_r_active_dofs) - 1])
        self.b_l_foot = self.robosimian.link(self.RobotUtils.b_l_active_dofs[len(self.RobotUtils.b_l_active_dofs) - 1])

    def setInitalConfig(self):

        # Get configuration array
        q = self.robosimian.getConfig()

        # Set joints to 0
        for i in range(len(q)):
            q[i] = 0

        link2_offset_indexes = [8,16,24,32]
        link4_offset_indexes = [10,18,26,34]

        pos_sign = [1,-1,1,-1]
        neg_sign = [-1,1,-1,1]

        for i in range(len(link2_offset_indexes)):
            q[link2_offset_indexes[i]] = pos_sign[i] * self.RobotUtils.LIMB1_START_CONFIG_OFFSET

        for i in range(len(link4_offset_indexes)):
            q[link4_offset_indexes[i]] = neg_sign[i] * (self.RobotUtils.LIMB1_START_CONFIG_OFFSET + (math.pi/2.0) )

        self.robosimian.setConfig(q)


    def shift_torso_from_global_xyz_translation(self,translation):

        active_dofs = []
        for i in range(6,38):
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
        desired_orientation = self.RobotUtils.DESIRED_FOOT_ROTATION

        # ik obkectives
        f_l_r_const = ik.objective(f_l, R=desired_orientation, t=f_l_global)
        f_r_r_const = ik.objective(f_r, R=desired_orientation, t=f_r_global)
        b_l_r_const = ik.objective(b_l, R=desired_orientation, t=b_l_global)
        b_r_r_const = ik.objective(b_r, R=desired_orientation, t=b_r_global)

        torso_obj = ik.objective(torso, R=torso.getTransform()[0], t=translation)

        goal = [f_l_r_const, f_r_r_const, b_l_r_const, b_r_r_const, torso_obj]

        if ik.solve(goal):
            pass
        else:
            self.RobotUtils.ColorPrinter(self.__class__.__name__, "torso ik failure", "FAIL")


    def make_torso_shift_from_local_xyz_translation(self, xyz_translation):

        # Time calculations
        step_time = self.RobotUtils.TORSO_SHIFT_TIME
        delay = self.RobotUtils.SIMULATION_FRAME_DELAY
        i_max = int(float(step_time) / float(delay))

        global_xyz_start = self.MotionPlanner.get_world_xyz_from_local_xyz([0,0,0])
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

            global_x = start_x + ((float(i)/float(i_max)) * x_delta)
            global_y = start_y  + ((float(i)/float(i_max)) * y_delta)
            global_z = start_z  + ((float(i)/float(i_max)) * z_delta)

            global_xyz = [global_x, global_y, global_z]

            self.shift_torso_from_global_xyz_translation(global_xyz)

            time.sleep(self.RobotUtils.SIMULATION_FRAME_DELAY)


    def make_leg_step(self, end_affector_name, start_leg_state, end_leg_state):

        # Check to see parameters are valid
        if not start_leg_state in self.RobotUtils.leg_states:
            self.RobotUtils.ColorPrinter(self.__class__.__name__, "Error: start leg state unrecognized", "FAIL")
            return None

        if not end_leg_state in self.RobotUtils.leg_states:
            self.RobotUtils.ColorPrinter(self.__class__.__name__, "Error: end leg state unrecognized", "FAIL")
            return None

        if self.MotionPlanner is None:
            self.RobotUtils.ColorPrinter(self.__class__.__name__, "Error: MotionPlanner has not been provided", "FAIL")
            return None

        # Determine appropriate start local coordinates
        if start_leg_state == self.RobotUtils.LEG_F_EXTEND_STATE:
            local_start_xyz = self.MotionPlanner.get_extended_foot_local_xyz(end_affector_name, self.RobotUtils.FORWARD)

        elif start_leg_state == self.RobotUtils.LEG_B_EXTEND_STATE:
            local_start_xyz = self.MotionPlanner.get_extended_foot_local_xyz(end_affector_name, self.RobotUtils.BACKWARD)

        elif start_leg_state == self.RobotUtils.LEG_BASE_STATE:
            local_start_xyz = self.MotionPlanner.get_local_foot_base_state_from_foot_name(end_affector_name)

        else:
            self.RobotUtils.ColorPrinter(self.__class__.__name__, "Error: start leg state is recognized by RobotUtils,"
                                                                  "but not by make_leg_step()", "FAIL")
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

        # Retrieve appropriate variables
        link = self.get_foot_from_foot_name(end_affector_name)
        active_dofs = self.get_active_dofs_from_foot_name(end_affector_name)
        desired_orientation = self.RobotUtils.DESIRED_FOOT_ROTATION

        # Time calculations
        step_time = self.RobotUtils.STEP_TIME
        delay = self.RobotUtils.SIMULATION_FRAME_DELAY
        i_max = int(float(step_time) / float(delay))

        global_start_xyz = self.MotionPlanner.get_world_xyz_from_local_xyz(local_start_xyz)
        global_end_xyz = self.MotionPlanner.get_world_xyz_from_local_xyz(local_end_xyz)

        for i in range(i_max):

            xyz_des = self.MotionPlanner.get_mid_step_local_xyz(global_start_xyz, global_end_xyz, i, i_max)

            goal = ik.objective(link, R=desired_orientation, t=xyz_des)
            ik.solve(goal, activeDofs=active_dofs)

            time.sleep(delay)



    def get_active_dofs_from_foot_name(self,foot_name):

        if not foot_name in self.RobotUtils.end_affectors:
            self.RobotUtils.ColorPrinter(self.__class__.__name__, "get_active_dofs_from_foot_name: Error: foot name unrecognized", "FAIL")
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

        if not foot_name in self.RobotUtils.end_affectors:
            self.RobotUtils.ColorPrinter(self.__class__.__name__, "Error: foot name unrecognized", "FAIL")
            return None

        if foot_name == self.RobotUtils.B_L_FOOT:
            foot = self.b_l_foot

        elif (foot_name == self.RobotUtils.B_R_FOOT):
            foot = self.b_r_foot

        elif foot_name == self.RobotUtils.F_L_FOOT:
            foot = self.f_l_foot

        else:
            foot = self.f_r_foot

        return foot