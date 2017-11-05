#!/usr/bin/python
import math
from klampt.math import so3
from klampt import vis


class MotionPlanner():

    def __init__(self,robot,RobotUtils):

        self.robosimian = robot
        self.RobotUtils = RobotUtils

        self.f_r_end_affector = self.robosimian.link(self.RobotUtils.f_r_active_dofs[len(self.RobotUtils.f_r_active_dofs) - 1])
        self.f_l_end_affector = self.robosimian.link(self.RobotUtils.f_l_active_dofs[len(self.RobotUtils.f_l_active_dofs) - 1])
        self.b_r_end_affector = self.robosimian.link(self.RobotUtils.b_r_active_dofs[len(self.RobotUtils.b_r_active_dofs) - 1])
        self.b_l_end_affector = self.robosimian.link(self.RobotUtils.b_l_active_dofs[len(self.RobotUtils.b_l_active_dofs) - 1])

    # TODO: Figure out why this needs to be worldPosition, not localPosition as would be expected
    def save_base_states(self):

        self.local_f_r_end_affector_base_state = self.f_r_end_affector.getWorldPosition([0, 0, 0])
        self.local_f_l_end_affector_base_state = self.f_l_end_affector.getWorldPosition([0, 0, 0])
        self.local_b_r_end_affector_base_state = self.b_r_end_affector.getWorldPosition([0, 0, 0])
        self.local_b_l_end_affector_base_state = self.b_l_end_affector.getWorldPosition([0, 0, 0])


    def legs_make_base_state(self):

        '''
        @summary: This function returns true if the legs CAN compose a base state, that is that the robot could enter a
                    base state by only moving the torso
        @return: boolean
        '''

        print "legs_make_base_state unimplememnted"

        return None


    def get_legs_xyz_yaw(self):

        '''
        @summary: This function returns the yaw difference between the torso's commanded base state and the
                    legs' commandedbase state
        @return: int: angle offset
        '''

        print "get_legs_xyz_yaw unimplememnted"

        return None



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


    def get_desired_end_affector_rotation(self, end_affector_name):

        """
        @summary Returns a rotation matrix for a given
        @param end_affector_name:
        @return:
        """

        robot_yaw_rad = self.get_current_torso_yaw_rads()

        if end_affector_name in self.RobotUtils.left_feet:
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

        current_torso_yaw_rad = self.get_current_torso_yaw_rads()
        desired_torso_yaw_rad = current_torso_yaw_rad + math.radians(yaw_offset_deg)

        axis_angle = ( [0,0,1], desired_torso_yaw_rad)

        desired_r = so3.from_axis_angle( axis_angle )

        return desired_r


    def get_current_torso_yaw_rads(self):

        q = self.robosimian.getConfig()

        return q[3]


    def get_world_xyz_from_local_xyz(self, local_xyz):

        torso = self.robosimian.link(self.RobotUtils.TORSO_LINK_INDEX)

        R = torso.getTransform()[0]
        T = torso.getTransform()[1]

        rotated_point = so3.apply(R, local_xyz)

        translated_and_rotated_point = [rotated_point[0] + T[0], rotated_point[1] + T[1], rotated_point[2] + T[2]]

        return translated_and_rotated_point


    def get_local_end_affector_base_state_from_torso_translation(self, leg, translation, yaw_rotation_offset_degrees):

        yaw_rot_offset = math.radians(yaw_rotation_offset_degrees)

        local_base_state_xyz = self.get_local_end_affector_base_state_from_end_affector_name(leg)

        aa = ([0,0,1], yaw_rot_offset)
        R = so3.from_axis_angle(aa)

        new_local_base_rotated_unshifted = so3.apply(R, local_base_state_xyz)

        return [ new_local_base_rotated_unshifted[0] + translation[0], new_local_base_rotated_unshifted[1] + translation[1], new_local_base_rotated_unshifted[2] + translation[2]]


    def get_local_turn_desitination(self, end_affector_name, offset_deg):

        return self.get_local_end_affector_base_state_from_torso_translation( end_affector_name, [0,0,0], offset_deg)



    def get_extended_end_affector_local_xyz(self, end_affector_name, direction):

        back_leg = end_affector_name in [self.RobotUtils.B_L_FOOT,self.RobotUtils.B_R_FOOT ]

        a = 2

        base_end_affector_state = self.get_local_end_affector_base_state_from_end_affector_name(end_affector_name)

        # Make a copy of the end_affector state
        xyz = [base_end_affector_state[0], base_end_affector_state[1], base_end_affector_state[2]]

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


    def get_local_end_affector_base_state_from_end_affector_name(self, end_affector_name):
        
        if not end_affector_name in self.RobotUtils.end_affectors:
            self.RobotUtils.ColorPrinter(self.__class__.__name__,"Error: end_affector name unrecognized","FAIL")
            return None

        if end_affector_name == self.RobotUtils.B_L_FOOT:
            base_end_affector_state = self.local_b_l_end_affector_base_state

        elif (end_affector_name == self.RobotUtils.B_R_FOOT):
            base_end_affector_state = self.local_b_r_end_affector_base_state

        elif end_affector_name == self.RobotUtils.F_L_FOOT:
            base_end_affector_state = self.local_f_l_end_affector_base_state

        else:
            base_end_affector_state = self.local_f_r_end_affector_base_state

        return base_end_affector_state



    def get_end_affector_from_end_affector_name(self,end_affector_name):

        if not end_affector_name in self.RobotUtils.end_affectors:
            print_str = "Error: "+end_affector_name+" unrecognized"
            self.RobotUtils.ColorPrinter(self.__class__.__name__,print_str,"FAIL")
            return None

        if end_affector_name == self.RobotUtils.B_L_FOOT:
            end_affector = self.b_l_end_affector

        elif (end_affector_name == self.RobotUtils.B_R_FOOT):
            end_affector = self.b_r_end_affector

        elif end_affector_name == self.RobotUtils.F_L_FOOT:
            end_affector = self.f_l_end_affector

        else:
            end_affector = self.f_r_end_affector

        return end_affector


