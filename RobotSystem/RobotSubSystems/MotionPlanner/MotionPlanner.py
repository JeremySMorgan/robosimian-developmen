#!/usr/bin/python
import math
from klampt.math import so3
from klampt import vis


class MotionPlanner():

    def __init__(self,robot,RobotUtils, Vector, SupportPolygon, _2DLegRadius):

        self.robosimian = robot
        self.RobotUtils = RobotUtils

        self.Vector = Vector
        self._2DSupportPolygon = SupportPolygon
        self._2DLegRadius = _2DLegRadius

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

        f_l_curr = self.f_l_end_affector.getWorldPosition([0, 0, 0])
        f_r_curr = self.f_r_end_affector.getWorldPosition([0, 0, 0])
        b_l_curr = self.b_l_end_affector.getWorldPosition([0, 0, 0])
        b_r_curr = self.b_r_end_affector.getWorldPosition([0, 0, 0])

        f_l_base = self.local_f_l_end_affector_base_state
        f_r_base = self.local_f_r_end_affector_base_state
        b_l_base = self.local_b_l_end_affector_base_state
        b_r_base = self.local_b_r_end_affector_base_state

        f_l_vector = self.Vector( [ (f_l_base[0] - f_l_curr[0]), (f_l_base[1] - f_l_curr[1]), (f_l_base[2] - f_l_curr[2])  ])
        f_r_vector = self.Vector( [ (f_r_base[0] - f_r_curr[0]), (f_r_base[1] - f_r_curr[1]), (f_r_base[2] - f_r_curr[2])  ])
        b_l_vector = self.Vector( [ (b_l_base[0] - b_l_curr[0]), (b_l_base[1] - b_l_curr[1]), (b_l_base[2] - b_l_curr[2])  ])
        b_r_vector = self.Vector( [ (b_r_base[0] - b_r_curr[0]), (b_r_base[1] - b_r_curr[1]), (b_r_base[2] - b_r_curr[2])  ])

        if f_l_vector.multiple_vector_directions_are_equal([f_r_vector, b_l_vector, b_r_vector]):
            if f_r_vector.multiple_vector_directions_are_equal([b_l_vector, b_r_vector]):
                if b_l_vector.multiple_vector_directions_are_equal([b_r_vector]):
                    return True
        return False



    def three_legs_make_base_state(self):

        '''
        @summary: This function returns true if the legs CAN compose a base state, that is that the robot could enter a
                    base state by only moving the torso and one leg.
        @return: end affector name of fourth leg that can be moved to make the legs into a base state (definied in RobotUtils)
        '''

        f_l_curr = self.f_l_end_affector.getWorldPosition([0, 0, 0])
        f_r_curr = self.f_r_end_affector.getWorldPosition([0, 0, 0])
        b_l_curr = self.b_l_end_affector.getWorldPosition([0, 0, 0])
        b_r_curr = self.b_r_end_affector.getWorldPosition([0, 0, 0])

        f_l_base = self.local_f_l_end_affector_base_state
        f_r_base = self.local_f_r_end_affector_base_state
        b_l_base = self.local_b_l_end_affector_base_state
        b_r_base = self.local_b_r_end_affector_base_state


        f_l_vector = self.Vector( [ (f_l_base[0] - f_l_curr[0]), (f_l_base[1] - f_l_curr[1]), (f_l_base[2] - f_l_curr[2])  ])
        f_r_vector = self.Vector( [ (f_r_base[0] - f_r_curr[0]), (f_r_base[1] - f_r_curr[1]), (f_r_base[2] - f_r_curr[2])  ])
        b_l_vector = self.Vector( [ (b_l_base[0] - b_l_curr[0]), (b_l_base[1] - b_l_curr[1]), (b_l_base[2] - b_l_curr[2])  ])
        b_r_vector = self.Vector( [ (b_r_base[0] - b_r_curr[0]), (b_r_base[1] - b_r_curr[1]), (b_r_base[2] - b_r_curr[2])  ])

        # Front right
        if f_l_vector.multiple_vector_directions_are_equal([b_l_vector, b_r_vector]) and not f_l_vector.vector_directions_are_equal(f_r_vector):
            return self.RobotUtils.F_R_FOOT

        # Front left
        if f_r_vector.multiple_vector_directions_are_equal([b_l_vector, b_r_vector]) and not f_r_vector.vector_directions_are_equal(f_l_vector):
            return self.RobotUtils.F_L_FOOT

        # Back left
        if b_r_vector.multiple_vector_directions_are_equal([f_r_vector, f_l_vector]) and not b_r_vector.vector_directions_are_equal(b_l_vector):
            return self.RobotUtils.B_L_FOOT

        # Back right
        if b_l_vector.multiple_vector_directions_are_equal([f_r_vector, f_l_vector]) and not b_l_vector.vector_directions_are_equal(b_r_vector):
            return self.RobotUtils.B_R_FOOT

        return False


    def get_world_support_triangle_from_excluded_end_affector(self, excluded_end_effector):

        '''
        @summary returns a support triangle made by all end affectors except for the one specified
        @param excluded_end_effector: name of end affector to exclude from support triangle
        @return: SupportTriangle
        '''

        end_affectors = []
        end_affectors_temp = self.RobotUtils.end_affectors
        for end_affector in end_affectors_temp:
            end_affectors.append(end_affector)

        end_affectors.remove(excluded_end_effector)

        P = []
        for end_affector in end_affectors:
            link = self.get_end_affector_from_end_affector_name(end_affector)
            link_world_xyz = link.getWorldPosition([0,0,0])
            P.append(link_world_xyz)

        support_tri = self._2DSupportPolygon(P)

        return support_tri


    def get_support_polygon_from_points(self, P, name=None):

        '''
        @summary returns a _2DSupportPolygon created by the parameterized array of Points
        @param P: list of xyz coordinates
        @return: _2DSupportPolygon object
        '''

        return self._2DSupportPolygon(P, name=name)

    def point_is_in_support_polygon_intersection(self, support_poly1, support_poly2, P):

        '''
        @summary returns whether a point is inside the intersection of two _2DSupportPolygon
        @param support_poly1: _2DSupportPolygon
        @param support_poly2: _2DSupportPolygon
        @param P: [x,y,z] array
        @return: boolean
        '''

        return support_poly1.support_poly_is_in_intersection_with_other_2d_support_poly(support_poly2, P)


    def get_end_affector_circle_from_xyz_r(self, xyz, r, name=None):

        '''
        @summary returns a _2DLegRadius object with the specified x,y and r
        @param xyz: [x,y,z] list containing centerpoint of the _2DLegRadius object
        @param r: radius of circle
        @return: _2DLegRadius object
        '''

        return self._2DLegRadius(xyz, r, name=name)


    def get_centroid_from_multiple_poly_intersections(self, _2Dgeometryobjects):

        if len(_2Dgeometryobjects) < 2:
            self.RobotUtils.ColorPrinter((self.__class__.__name__ + ".get_centroid_from_multiple_poly_intersections()"),
                                         "Error: _2Dgeometryobjects has less than two objects", "FAIL")
            return False

        first_obj = _2Dgeometryobjects[0]
        rest = _2Dgeometryobjects[1:]

        return first_obj.xy_centroid_from_list_of_2DGeometry_objects(rest)



    def get_centroid_of_support_polygon_intersection(self, support_poly1, support_poly2):

        '''
        @summary this function returns the xy centroid of the intersection of two _2DSupportPolygon objects
        @param support_poly1: _2DSupportPolygon
        @param support_poly2: _2DSupportPolygon
        @return: [x, y]
        '''

        return support_poly1.xy_centroid_of_intersection(support_poly2)


    def get_torso_and_legs_delta_yaw_deg_and_xyz_offset(self, excluded_leg = None):

        '''
        @summary: This function returns the yaw difference between the torso's commanded base state and the
                    legs' commandedbase state. If excluded leg as passed, this functino will not use that leg in
                    its calculations
        @return: int: angle offset
        '''

        f_l_world = self.f_l_end_affector.getWorldPosition([0, 0, 0])
        f_r_world = self.f_r_end_affector.getWorldPosition([0, 0, 0])
        b_l_world = self.b_l_end_affector.getWorldPosition([0, 0, 0])
        b_r_world = self.b_r_end_affector.getWorldPosition([0, 0, 0])

        q = self.robosimian.getConfig()
        robot_world_x = q[0]
        robot_world_y = q[1]

        if excluded_leg:

            print "End affectors:",self.RobotUtils.end_affectors
            print "excluded_leg:",excluded_leg

            if not excluded_leg in self.RobotUtils.end_affectors:
                self.RobotUtils.ColorPrinter((self.__class__.__name__ + ".get_legs_xyz_yaw()"),
                                             "Error: excluded leg is a valid end affector name", "FAIL")
                return False


            if excluded_leg == self.RobotUtils.B_R_FOOT or excluded_leg == self.RobotUtils.F_L_FOOT:
                diaganol_pos_1 = b_l_world
                diaganol_pos_2 = f_r_world

            else:
                diaganol_pos_1 = b_r_world
                diaganol_pos_2 = f_l_world

            if excluded_leg in self.RobotUtils.left_feet:
                legs_x_delta = f_r_world[0] - b_r_world[0]
                legs_y_delta = f_r_world[1] - b_r_world[1]

            else:
                legs_x_delta = f_l_world[0] - b_l_world[0]
                legs_y_delta = f_l_world[1] - b_l_world[1]

            legs_commanded_x  = (diaganol_pos_1[0] + diaganol_pos_2[0]) / 2.0
            legs_commanded_y  = (diaganol_pos_1[1] + diaganol_pos_2[1]) / 2.0

            xyz_offset = [legs_commanded_x - robot_world_x, legs_commanded_y - robot_world_y, 0]

        else:

            if not self.legs_make_base_state():
                self.RobotUtils.ColorPrinter((self.__class__.__name__+".get_legs_xyz_yaw()"),"Error: Legs do not make a base state","FAIL")

            legs_x_delta = f_l_world[0] - b_l_world[0]
            legs_y_delta = f_l_world[1] - b_l_world[1]

            legs_commanded_x  = (f_l_world[0] + f_r_world[0] + b_l_world[0] + b_r_world[0]) / 4
            legs_commanded_y  = (f_l_world[1] + f_r_world[1] + b_l_world[1] + b_r_world[1]) / 4

            xyz_offset = [  legs_commanded_x- robot_world_x,  legs_commanded_y - robot_world_y, 0]


        legs_yaw_degree = math.degrees(math.atan(legs_y_delta / legs_x_delta))

        return legs_yaw_degree, xyz_offset





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

        '''
        @param leg: string from RobotUtils specifying end affector
        @param translation: torso translation
        @param yaw_rotation_offset_degrees: yaw rotation
        @return: return the local end position commanded by the torso and yaw offset
        '''

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


