#!/usr/bin/python
import math
from klampt.math import so3
from klampt import vis


class MotionPlanner():

    def __init__(self,robot,RobotUtils):

        self.robosimian = robot
        self.RobotUtils = RobotUtils

        self.f_r_foot = self.robosimian.link(self.RobotUtils.f_r_active_dofs[len(self.RobotUtils.f_r_active_dofs) - 1])
        self.f_l_foot = self.robosimian.link(self.RobotUtils.f_l_active_dofs[len(self.RobotUtils.f_l_active_dofs) - 1])
        self.b_r_foot = self.robosimian.link(self.RobotUtils.b_r_active_dofs[len(self.RobotUtils.b_r_active_dofs) - 1])
        self.b_l_foot = self.robosimian.link(self.RobotUtils.b_l_active_dofs[len(self.RobotUtils.b_l_active_dofs) - 1])

        self.local_f_r_foot_base_state = self.f_r_foot.getWorldPosition([0, 0, 0])
        self.local_f_l_foot_base_state = self.f_l_foot.getWorldPosition([0, 0, 0])
        self.local_b_r_foot_base_state = self.b_r_foot.getWorldPosition([0, 0, 0])
        self.local_b_l_foot_base_state = self.b_l_foot.getWorldPosition([0, 0, 0])

        self.f_r_shoulder = self.robosimian.link(self.RobotUtils.f_r_active_dofs[0])
        self.f_l_shoulder = self.robosimian.link(self.RobotUtils.f_l_active_dofs[0])
        self.b_r_shoulder = self.robosimian.link(self.RobotUtils.b_r_active_dofs[0])
        self.b_l_shoulder = self.robosimian.link(self.RobotUtils.b_l_active_dofs[0])



    def get_linear_mid_motion_xyz(self, startXYZ, endXYZ, i, i_max):

        # prevents division by 0
        if i == 0: i = 1

        x_start = float(startXYZ[0])
        y_start = float(startXYZ[1])
        z_start = float(startXYZ[2])

        x_end = float(endXYZ[0])
        y_end = float(endXYZ[1])
        z_end = float(endXYZ[2])

        x_delta = x_end - x_start
        x = float(i) / float(i_max) * x_delta

        y_delta = y_end - y_start
        y = float(i)/float(i_max) * y_delta

        z_delta = z_end - z_start
        z = float(i)/float(i_max) * z_delta

        # Arc height
        h = float(self.RobotUtils.STEP_Z_MAX_HIEGHT)
        b = x_delta

        # see https://www.desmos.com/calculator/v8wb6o83jh
        #       ((-4*h)/(b**2)) * x * (x-b) -> Parabolic Arc
        #       + z                         -> linear Z offset
        z_offset =  ((-4*h)/(b**2)) * x * (x-b) + z

        # print "x:",x,"\ty:",z_offset,"\t\tb:",b,"\th:",h

        res = [ x_start + x, y_start + y , z_start + z_offset ]

        return res


    def get_desired_foot_rotation(self, foot_name):

        """
        @summary Returns a rotation matrix for a given
        @param foot_name:
        @return:
        """

        robot_yaw_rad = self.get_current_torso_yaw_rads()

        if foot_name in self.RobotUtils.left_feet:
            r = [0, 0, -1, 0, -1, 0, -1, 0, 0]

        else:
            r = [0, 0, -1, 0, 1, 0, 1, 0, 0]

        yaw_rotation_aa = ([0, 0, 1], robot_yaw_rad)
        yaw_rotation_R = so3.from_axis_angle(yaw_rotation_aa)

        return so3.mul(yaw_rotation_R, r)




    def get_desired_torso_R_from_yaw_offset(self, yaw_offset_deg):

        """
        @summary
        @param yaw_offset_deg:
        @return:
        """

        def pp(x):
            if math.fabs(x) < .001:
                return 0
            if math.fabs(x-1) < .0001:
                return 1
            if math.fabs(x+1) < .00001:
                return -1
            return x

        current_torso_yaw_rad = self.get_current_torso_yaw_rads()
        desired_torso_yaw_rad = current_torso_yaw_rad + math.radians(yaw_offset_deg)

        axis_angle = ( [0,0,1], desired_torso_yaw_rad)

        desired_r = so3.from_axis_angle( axis_angle )

        """
        print "\n\n\nOriginal Torso R:"
        for i in range(3):
            print pp(torso_R[i]),pp(torso_R[i+1]),pp(torso_R[i+2])
        print "\nNew Torso R:"
        for i in range(3):
            print pp(desired_r[i]),pp(desired_r[i+1]),pp(desired_r[i+2])

        
        print "current degree:",math.degrees(current_torso_yaw_rad),"\toffset:",yaw_offset_deg,"\tdesired degree:",math.degrees(desired_torso_yaw_rad),"\tdesired_torso_yaw_rad:",desired_torso_yaw_rad
        """

        return desired_r


    def get_current_torso_yaw_rads(self):

        q = self.robosimian.getConfig()

        return q[3]


    def get_world_xyz_from_local_xyz(self,local_xyz):

        torso = self.robosimian.link(self.RobotUtils.TORSO_LINK_INDEX)

        R = torso.getTransform()[0]
        T = torso.getTransform()[1]

        rotated_point = so3.apply(R, local_xyz)

        translated_and_rotated_point = [rotated_point[0] + T[0], rotated_point[1] + T[1], rotated_point[2] + T[2]]

        return translated_and_rotated_point




    def get_local_foot_base_state_from_torso_translation(self, leg, translation, yaw_rotation_offset_degrees):

        current_torso_yaw_rads = self.get_current_torso_yaw_rads()

        final_torso_yaw_rads = current_torso_yaw_rads + math.radians(yaw_rotation_offset_degrees)

        local_base_state_xyz = self.get_local_foot_base_state_from_foot_name(leg)

        aa = ([0,0,1], final_torso_yaw_rads)
        R = so3.from_axis_angle(aa)

        new_local_base_unshifted = so3.apply(R, local_base_state_xyz)

        local_end_des = [ new_local_base_unshifted[0] + translation[0], new_local_base_unshifted[1]+translation[1], new_local_base_unshifted[2]+translation[2]]

        return local_end_des




    def get_local_turn_desitination(self, foot_name, offset_deg):

        degree_correction = 0
        if foot_name == self.RobotUtils.F_R_FOOT:
            degree_correction = -90
        elif foot_name == self.RobotUtils.B_R_FOOT:
            degree_correction = 90

        P0 = self.get_local_foot_base_state_from_foot_name(foot_name)

        P0_x = P0[0]
        P0_y = P0[1]
        P0_z = P0[2]

        P0_xy_len = math.sqrt(P0_x ** 2 + P0_y ** 2)

        phi = math.degrees(math.acos(P0_x / P0_xy_len)) + degree_correction

        phi_des = phi + offset_deg

        phi_des_rads = math.radians(phi_des)

        des_x = P0_xy_len * math.cos(phi_des_rads)
        des_y = P0_xy_len * math.sin(phi_des_rads)

        return [des_x, des_y, P0_z]

        '''
        print foot_name,"offset:",offset_deg

        return self.get_local_foot_base_state_from_torso_translation( foot_name, [0,0,0], offset_deg)
        '''



    def get_extended_foot_local_xyz(self, foot_name, direction):

        back_leg = foot_name in [self.RobotUtils.B_L_FOOT,self.RobotUtils.B_R_FOOT ]

        a = 2

        base_foot_state = self.get_local_foot_base_state_from_foot_name(foot_name)

        # Make a copy of the foot state
        xyz = [base_foot_state[0], base_foot_state[1], base_foot_state[2]]

        # Add x offset (dependent on direction)
        if direction == self.RobotUtils.FORWARD:
            if back_leg:
                xyz[0] += a*self.RobotUtils.STEP_X_DELTA
            else:
                xyz[0] += self.RobotUtils.STEP_X_DELTA
        else:
            if back_leg:
                xyz[0] -= a*self.RobotUtils.STEP_X_DELTA
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
            print_str = "Error: "+foot_name+" unrecognized"
            self.RobotUtils.ColorPrinter(self.__class__.__name__,print_str,"FAIL")
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




"""
        if foot_name  == self.RobotUtils.B_L_FOOT:
            local_shoulder_xyz = self.b_l_shoulder.getLocalPosition([0, 0, 0])
            print "local shoulder xyz for BACK LEFT Shoulders:", local_shoulder_xyz

        elif foot_name == self.RobotUtils.B_R_FOOT:
            local_shoulder_xyz = self.b_r_shoulder.getLocalPosition([0, 0, 0])
            print "local shoulder xyz for BACK RIGHT Shoulders:", local_shoulder_xyz

        elif foot_name == self.RobotUtils.F_R_FOOT:
            local_shoulder_xyz = self.f_r_shoulder.getLocalPosition([0, 0, 0])
            local_shoulder_xyz[0] *= -1
            print "local shoulder xyz for RIGHT FORWARD Shoulders:", local_shoulder_xyz

        else:
            local_shoulder_xyz = self.f_l_shoulder.getLocalPosition([0,0,0])
            local_shoulder_xyz[0] *= -1
            print "local shoulder xyz for LEFT FORWARD Shoulders:", local_shoulder_xyz


        base_foot_state = self.get_local_foot_base_state_from_foot_name( foot_name )
        delta_x  = 0#self.RobotUtils.STEP_X_DELTA

        if direction == self.RobotUtils.FORWARD:
            local_shoulder_xyz[0] += delta_x
        else:
            local_shoulder_xyz[0] -= delta_x

        return [local_shoulder_xyz[0], base_foot_state[1],base_foot_state[2] ]


"""