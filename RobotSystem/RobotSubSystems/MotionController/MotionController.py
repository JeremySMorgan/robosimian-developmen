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


    def setRobotConfig(self,q):
        self.robosimian.setConfig(q)


    def setInitalConfig(self):

        if self.robosimian:

            q = self.robosimian.getConfig()

            link2_offset_indexes = [8,16,24,32]
            link4_offset_indexes = [10,18,26,34]

            pos_sign = [1,-1,1,-1]
            neg_sign = [-1,1,-1,1]

            for i in range(len(link2_offset_indexes)):
                q[link2_offset_indexes[i]] = pos_sign[i] * self.RobotUtils.LIMB1_START_CONFIG_OFFSET

            for i in range(len(link4_offset_indexes)):
                q[link4_offset_indexes[i]] = neg_sign[i] * (self.RobotUtils.LIMB1_START_CONFIG_OFFSET + (math.pi/2.0) )

            self.robosimian.setConfig(q)

        else:
            status_message =  "robosimian is of type:",type(self.robosimian)
            self.RobotUtils.ColorPrinter(self.__class__.__name__, status_message, "FAIL")



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
        print "active dofs:",active_dofs
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