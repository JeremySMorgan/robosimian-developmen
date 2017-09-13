#!/usr/bin/python
import math

class MotionController(object):

    def __init__(self,robot,RobotUtils):
        self.robosimian = robot
        self.RobotUtils = RobotUtils
        self.initialize_feet()


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
