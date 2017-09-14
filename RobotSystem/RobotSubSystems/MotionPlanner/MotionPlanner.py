#!/usr/bin/python
from klampt.math import so3


class MotionPlanner():

    def __init__(self,robot,RobotUtils):
        self.robosimian = robot
        self.RobotUtils = RobotUtils

        self.initialize_feet()
        self.initialize_shoulders()


    def initialize_shoulders(self):

        self.f_r_shoulder = self.robosimian.link(self.RobotUtils.f_r_active_dofs[0])
        self.f_l_shoulder = self.robosimian.link(self.RobotUtils.f_l_active_dofs[0])
        self.b_r_shoulder = self.robosimian.link(self.RobotUtils.b_r_active_dofs[0])
        self.b_l_shoulder = self.robosimian.link(self.RobotUtils.b_l_active_dofs[0])

    def initialize_feet(self):

        self.f_r_foot = self.robosimian.link(self.RobotUtils.f_r_active_dofs[len(self.RobotUtils.f_r_active_dofs) - 1])
        self.f_l_foot = self.robosimian.link(self.RobotUtils.f_l_active_dofs[len(self.RobotUtils.f_l_active_dofs) - 1])
        self.b_r_foot = self.robosimian.link(self.RobotUtils.b_r_active_dofs[len(self.RobotUtils.b_r_active_dofs) - 1])
        self.b_l_foot = self.robosimian.link(self.RobotUtils.b_l_active_dofs[len(self.RobotUtils.b_l_active_dofs) - 1])

    def get_mid_step_global_xyz(self, startXYZ, endXYZ, i, i_max):

        local_xyz = self.get_mid_step_local_xyz(startXYZ,endXYZ,i,i_max)

        global_xyz = self.get_world_xyz_from_local_xyz(local_xyz)

        return global_xyz


    def get_mid_step_local_xyz(self, startXYZ, endXYZ, i, i_max):

        # prevents division by 0
        if i == 0: i = 1

        x_start = startXYZ[0]
        y_start = startXYZ[1]
        z_start = startXYZ[2]

        x_end = endXYZ[0]
        y_end = endXYZ[1]
        z_end = endXYZ[2]

        y_delta = y_end - y_start
        y = float(i)/float(i_max) * y_delta

        x_delta = x_end - x_start
        x = float(i)/float(i_max) * x_delta

        # Arc height
        h = self.RobotUtils.STEP_Z_MAX_HIEGHT
        b = x_delta

        # see https://www.desmos.com/calculator/v8wb6o83jh
        z_offset =  ((-4*h)/(b**2)) * x * (x-b)

        #print "x:",x,"\ty:",z_offset,"\t\tb:",b,"\th:",h

        res = [ x_start + x, y_start + y , z_start + z_offset ]

        return res



    def get_world_xyz_from_local_xyz(self,local_xyz):


        torso = self.robosimian.link(self.RobotUtils.TORSO_LINK_INDEX)

        R = torso.getTransform()[0]
        T = torso.getTransform()[1]

        rotated_point = so3.apply(R, local_xyz)

        translated_and_rotated_point = [rotated_point[0] + T[0], rotated_point[1] + T[1], rotated_point[2] + T[2]]

        return translated_and_rotated_point


    # Saves the foot positions when the robot is located at the origin
    def save_base_foot_states(self):

        self.local_f_r_foot_base_state = self.f_r_foot.getWorldPosition([0, 0, 0])
        self.local_f_l_foot_base_state = self.f_l_foot.getWorldPosition([0, 0, 0])
        self.local_b_r_foot_base_state = self.b_r_foot.getWorldPosition([0, 0, 0])
        self.local_b_l_foot_base_state = self.b_l_foot.getWorldPosition([0, 0, 0])


    def get_extended_foot_local_xyz(self, foot_name, direction):

        base_foot_state = self.get_local_foot_base_state_from_foot_name(foot_name)

        # Make a copy of the foot state
        xyz = [base_foot_state[0], base_foot_state[1], base_foot_state[2]]

        # Add x offset (dependent on direction)
        if direction == self.RobotUtils.FORWARD:
            xyz[0] += self.RobotUtils.STEP_X_DELTA
        else:
            xyz[0] -= self.RobotUtils.STEP_X_DELTA

        return xyz



    def get_local_foot_base_state_from_foot_name(self, foot_name):
        
        if not foot_name in self.RobotUtils.end_affectors:
            self.RobotUtils.ColorPrinter(self.__class__.__name__,"Error: foot name unrecognized","FAIL")
            return None

        if foot_name == self.RobotUtils.B_L_FOOT:
            base_foot_state = self.local_b_l_foot_base_state

        elif (foot_name == self.RobotUtils.B_R_FOOT):
            base_foot_state = self.local_b_r_foot_base_state

        elif foot_name == self.RobotUtils.F_L_FOOT:
            base_foot_state = self.local_f_l_foot_base_state

        else:
            base_foot_state = self.local_f_r_foot_base_state

        return base_foot_state




    def get_foot_from_foot_name(self,foot_name):
        
        if not foot_name in self.RobotUtils.end_affectors:
            self.RobotUtils.ColorPrinter(self.__class__.__name__,"Error: foot name unrecognized","FAIL")
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