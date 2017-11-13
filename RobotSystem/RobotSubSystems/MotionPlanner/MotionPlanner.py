#!/usr/bin/python
import math
from klampt.math import so3
from klampt import vis
from klampt.model import trajectory
import numpy as np
from ...Utilities.RobotUtils.RobotUtils import RobotUtils
from ...Utilities._2D_GeometryObjects._2DLegRadius import _2DLegRadius
from ...Utilities._2D_GeometryObjects._2DSupportPolygon import _2DSupportPolygon
from ...Utilities.Vector.Vector import Vector

class MotionPlanner():

    def __init__(self, robot):

        self.robosimian = robot

        self.f_r_end_affector = self.robosimian.link(RobotUtils.f_r_active_dofs[len(RobotUtils.f_r_active_dofs) - 1])
        self.f_l_end_affector = self.robosimian.link(RobotUtils.f_l_active_dofs[len(RobotUtils.f_l_active_dofs) - 1])
        self.b_r_end_affector = self.robosimian.link(RobotUtils.b_r_active_dofs[len(RobotUtils.b_r_active_dofs) - 1])
        self.b_l_end_affector = self.robosimian.link(RobotUtils.b_l_active_dofs[len(RobotUtils.b_l_active_dofs) - 1])

        # initialized in save_base_states(). Used by legs_make_base_state()
        # [F_R_FOOT, F_L_FOOT, B_R_FOOT, B_L_FOOT]
        self.base_state_angles = None


    def save_base_states(self):

        '''
        @summary: This function saves the world positions of the end affectors. Note that the robot needs to be at the origin
                    or the end affectors will be saved to an incorrect location.
        @return: None
        '''

        print "SAVE BASE STATES CALLED"

        bl = self.b_l_end_affector.getWorldPosition([0, 0, 0])
        br = self.b_r_end_affector.getWorldPosition([0, 0, 0])
        fl = self.f_l_end_affector.getWorldPosition([0, 0, 0])
        fr = self.f_r_end_affector.getWorldPosition([0, 0, 0])


        self.local_f_r_end_affector_base_state = fr
        self.local_f_l_end_affector_base_state = fl
        self.local_b_r_end_affector_base_state = br
        self.local_b_l_end_affector_base_state = bl

        self.br_base_angle_deg = (RobotUtils.angle_between_three_points(bl, br, fr) % 360)
        self.fl_base_angle_deg = (RobotUtils.angle_between_three_points(fr, fl, bl) % 360)
        self.bl_base_angle_deg = (RobotUtils.angle_between_three_points(fl, bl, br) % 360)
        self.fr_base_angle_deg = (RobotUtils.angle_between_three_points(br, fr, fl) % 360)


        #print "br base state angle:",self.br_base_angle_deg
        #print "fl base state angle:",self.fl_base_angle_deg
        #print "bl base state angle:",self.bl_base_angle_deg
        #print "fr base state angle:",self.fr_base_angle_deg


        self.base_state_angles = [self.fr_base_angle_deg, self.fl_base_angle_deg, self.br_base_angle_deg, self.bl_base_angle_deg]


    def legs_make_base_state(self):

        # TODO: Fix bug. base states should not be rotated by torso rad, as this may not equal the yaw commanded by legs

        f_l_curr = self.f_l_end_affector.getWorldPosition([0, 0, 0])
        f_r_curr = self.f_r_end_affector.getWorldPosition([0, 0, 0])
        b_l_curr = self.b_l_end_affector.getWorldPosition([0, 0, 0])
        b_r_curr = self.b_r_end_affector.getWorldPosition([0, 0, 0])

        legs_x_delta = f_l_curr[0] - b_l_curr[0]
        legs_y_delta = f_l_curr[1] - b_l_curr[1]

        legs_commanded_yaw_rad = np.arctan2( legs_y_delta , legs_x_delta )  % 360

        #print "legs commanded yaw rad: ",legs_commanded_yaw_rad

        yaw_rotation_aa = ([0, 0, 1], legs_commanded_yaw_rad)
        yaw_rotation_R = so3.from_axis_angle(yaw_rotation_aa)

        f_l_base = self.local_f_l_end_affector_base_state
        f_r_base = self.local_f_r_end_affector_base_state
        b_l_base = self.local_b_l_end_affector_base_state
        b_r_base = self.local_b_r_end_affector_base_state

        f_l_base_rotated = so3.apply(yaw_rotation_R, f_l_base)
        f_r_base_rotated = so3.apply(yaw_rotation_R, f_r_base)
        b_l_base_rotated = so3.apply(yaw_rotation_R, b_l_base)
        b_r_base_rotated = so3.apply(yaw_rotation_R, b_r_base)

        # print "fl curr:",f_l_curr
        # print "fr curr:",f_r_curr
        # print "bl curr:",b_l_curr
        # print "br curr:",b_r_curr

        # print "fl base:",f_l_base
        # print "fr base:",f_r_base
        # print "bl base:",b_l_base
        # print "br base:",b_r_base

        f_l_vector = Vector( [ (f_l_base_rotated[0] - f_l_curr[0]), (f_l_base_rotated[1] - f_l_curr[1]), (f_l_base_rotated[2] - f_l_curr[2])  ])
        f_r_vector = Vector( [ (f_r_base_rotated[0] - f_r_curr[0]), (f_r_base_rotated[1] - f_r_curr[1]), (f_r_base_rotated[2] - f_r_curr[2])  ])
        b_l_vector = Vector( [ (b_l_base_rotated[0] - b_l_curr[0]), (b_l_base_rotated[1] - b_l_curr[1]), (b_l_base_rotated[2] - b_l_curr[2])  ])
        b_r_vector = Vector( [ (b_r_base_rotated[0] - b_r_curr[0]), (b_r_base_rotated[1] - b_r_curr[1]), (b_r_base_rotated[2] - b_r_curr[2])  ])

        #self.add_line_to_vis("fl",f_l_base_rotated, f_l_curr )
        #self.add_line_to_vis("fr",f_r_base_rotated, f_r_curr )
        #self.add_line_to_vis("bl",b_l_base_rotated, b_l_curr )
        #self.add_line_to_vis("br",b_r_base_rotated, b_r_curr )

        #print "\nin legs_make_base_state()"
        #print "front left vector:\t",f_l_vector
        #print "front right vector:\t",f_r_vector
        #print "back left vector:\t",b_l_vector
        #print "back right vector:\t",b_r_vector

        # Front right
        if f_l_vector.multiple_vector_directions_are_equal([b_l_vector, b_r_vector, f_r_vector]):

            if f_r_vector.multiple_vector_directions_are_equal([b_l_vector, b_r_vector]):

                if b_l_vector.multiple_vector_directions_are_equal([b_r_vector]):
                    #print "all vectors equal, returning true"
                    return True
                else:
                    pass
                    # bl and br are not equal
                    #print "bl and br vectors are NOT equal, returning false"

            else:
                pass
                #print "fr and [bl and br] vectors are NOT equal, returning false"
                # fr and [bl,br] not equal
        else:
            pass
            #print "fl and [bl  br or fr] vectors are NOT equal, returning false"
            # fl and [bl br fr ] not equal

        return False


    # TODO: This function likely has bugs. Need to test when the robot is rotated
    def three_legs_make_base_state(self):

        '''
        @summary: This function returns true if the legs CAN compose a base state, that is that the robot could enter a
                    base state by only moving the torso and one leg.
        @return: end affector name of fourth leg that can be moved to make the legs into a base state (definied in RobotUtils)
        '''

        torso_yaw_rad =  self.get_current_torso_yaw_rads()
        yaw_rotation_aa = ([0, 0, 1], torso_yaw_rad)
        yaw_rotation_R = so3.from_axis_angle(yaw_rotation_aa)

        f_l_curr = self.f_l_end_affector.getWorldPosition([0, 0, 0])
        f_r_curr = self.f_r_end_affector.getWorldPosition([0, 0, 0])
        b_l_curr = self.b_l_end_affector.getWorldPosition([0, 0, 0])
        b_r_curr = self.b_r_end_affector.getWorldPosition([0, 0, 0])

        f_l_base = self.local_f_l_end_affector_base_state
        f_r_base = self.local_f_r_end_affector_base_state
        b_l_base = self.local_b_l_end_affector_base_state
        b_r_base = self.local_b_r_end_affector_base_state

        f_l_base_rotated = so3.apply(yaw_rotation_R, f_l_base)
        f_r_base_rotated = so3.apply(yaw_rotation_R, f_r_base)
        b_l_base_rotated = so3.apply(yaw_rotation_R, b_l_base)
        b_r_base_rotated = so3.apply(yaw_rotation_R, b_r_base)

        #print "fl curr:",f_l_curr
        #print "fr curr:",f_r_curr
        #print "bl curr:",b_l_curr
        #print "br curr:",b_r_curr

        #print "fl base:",f_l_base
        #print "fr base:",f_r_base
        #print "bl base:",b_l_base
        #print "br base:",b_r_base


        f_l_vector = Vector( [ (f_l_base_rotated[0] - f_l_curr[0]), (f_l_base_rotated[1] - f_l_curr[1]), (f_l_base_rotated[2] - f_l_curr[2])  ])
        f_r_vector = Vector( [ (f_r_base_rotated[0] - f_r_curr[0]), (f_r_base_rotated[1] - f_r_curr[1]), (f_r_base_rotated[2] - f_r_curr[2])  ])
        b_l_vector = Vector( [ (b_l_base_rotated[0] - b_l_curr[0]), (b_l_base_rotated[1] - b_l_curr[1]), (b_l_base_rotated[2] - b_l_curr[2])  ])
        b_r_vector = Vector( [ (b_r_base_rotated[0] - b_r_curr[0]), (b_r_base_rotated[1] - b_r_curr[1]), (b_r_base_rotated[2] - b_r_curr[2])  ])

        #self.add_line_to_vis("fl",f_l_base_rotated, f_l_curr )
        #self.add_line_to_vis("fr",f_r_base_rotated, f_r_curr )
        #self.add_line_to_vis("bl",b_l_base_rotated, b_l_curr )
        #self.add_line_to_vis("br",b_r_base_rotated, b_r_curr )

        #print "\nin three_legs_make_base_state()"
        #print "front left vector:\t",f_l_vector
        #print "front right vector:\t",f_r_vector
        #print "back left vector:\t",b_l_vector
        #print "back right vector:\t",b_r_vector

        # Front right
        if f_l_vector.multiple_vector_directions_are_equal([b_l_vector, b_r_vector]) and not f_l_vector.vector_directions_are_equal(f_r_vector):
            return RobotUtils.F_R_FOOT

        # Front left
        if f_r_vector.multiple_vector_directions_are_equal([b_l_vector, b_r_vector]) and not f_r_vector.vector_directions_are_equal(f_l_vector):
            return RobotUtils.F_L_FOOT

        # Back left
        if b_r_vector.multiple_vector_directions_are_equal([f_r_vector, f_l_vector]) and not b_r_vector.vector_directions_are_equal(b_l_vector):
            return RobotUtils.B_L_FOOT

        # Back right
        if b_l_vector.multiple_vector_directions_are_equal([f_r_vector, f_l_vector]) and not b_l_vector.vector_directions_are_equal(b_r_vector):
            return RobotUtils.B_R_FOOT

        return False


    def get_world_support_triangle_from_excluded_end_affector(self, excluded_end_effector):

        '''
        @summary returns a support triangle made by all end affectors except for the one specified
        @param excluded_end_effector: name of end affector to exclude from support triangle
        @return: SupportTriangle
        '''

        end_affectors = []
        end_affectors_temp = RobotUtils.end_affectors
        for end_affector in end_affectors_temp:
            end_affectors.append(end_affector)

        end_affectors.remove(excluded_end_effector)

        P = []
        for end_affector in end_affectors:
            link = self.get_end_affector_from_end_affector_name(end_affector)
            link_world_xyz = link.getWorldPosition([0,0,0])
            P.append(link_world_xyz)

        return _2DSupportPolygon(P)


    def get_support_polygon_from_points(self, P, name=None):

        '''
        @summary returns a _2DSupportPolygon created by the parameterized array of Points
        @param P: list of xyz coordinates
        @return: _2DSupportPolygon object
        '''

        return _2DSupportPolygon(P, name=name)



    def point_is_in_multiple_support_polygon_intersections(self, curr_torso_world_xyz, _2DGeometry_objs):

        first_2DGeom_obj = _2DGeometry_objs[0]
        other_2DGeom_objs = _2DGeometry_objs[1:]
        return first_2DGeom_obj.point_is_inside_intersection_of_multiple_2DGeometry_objects(curr_torso_world_xyz, other_2DGeom_objs )


    def point_is_in_support_polygon_intersection(self, support_poly1, support_poly2, P):

        '''
        @summary returns whether a point is inside the intersection of two _2DSupportPolygon
        @param support_poly1: _2DSupportPolygon
        @param support_poly2: _2DSupportPolygon
        @param P: [x,y,z] array
        @return: boolean
        '''

        return support_poly1.support_poly_is_in_intersection_with_other_2d_support_poly(support_poly2, P)


    def get_end_affector_2D_support_circle_from_name(self,link_name, circle_name = None, at_point=None):

        '''
        @summary returns a _2DLegRadius object given the end affectors name
        @param link_name: string specifiing the link name
        @param circle_name: name of circle
        @return: _2DLegRadius object
        '''

        r = self.get_torso_range_from_end_affector(link_name, at_point=at_point)
        global_xyz = self.get_end_affector_from_end_affector_name(link_name).getWorldPosition([0,0,0])
        if at_point:
            global_xyz = at_point
        return _2DLegRadius(global_xyz, r, name=circle_name)


    def add_line_to_vis(self,name,p1,p2):

        traj = trajectory.Trajectory(milestones=[p1,p2])
        vis.add(name,traj)

    def print_config(self):
        print RobotUtils.pp_list(self.robosimian.getConfig())


    def get_torso_range_from_end_affector(self, end_affector_name, at_point=None ):

        R = RobotUtils.END_AFFECTOR_RADIUS_TO_SHOULDER
        S = RobotUtils.SHOULDER_TORSO_XY_EUCLIDEAN_DIF
        yaw = self.get_current_torso_yaw_rads()
        psi = RobotUtils.SHOULDER_TORSO_PSI_RADS

        shoulder_link = self.get_shoulder_from_end_affector(end_affector_name)
        shoulder_world_xyz = shoulder_link.getWorldPosition([0, 0, 0])

        #print "yaw (deg): ",np.rad2deg(yaw)

        #print "\nin get_torso_range_from_end_affector, end affector:",end_affector_name," shoulder:",shoulder_link.getName()

        if end_affector_name == RobotUtils.F_L_FOOT:

            link_global_xyz = self.f_l_end_affector.getWorldPosition([0, 0, 0])
            if at_point:
                link_global_xyz = at_point

            leg_dx = shoulder_world_xyz[0] - link_global_xyz[0]
            leg_dy = - (link_global_xyz[1] - shoulder_world_xyz[1])
            theta = np.arctan2(leg_dy, leg_dx )

            rx = R * np.cos(theta)
            ry = R * np.sin(theta)

            tx = - S * np.sin( (np.pi/2) - (yaw + psi))
            ty = - S * np.cos( (np.pi/2) - (yaw + psi))

            delta_y_max =  ry + ty
            delta_x_max = rx + tx

            #print "\n Front Left"
            #print "leg-torso dx, dy:", leg_dx, "\t", leg_dy
            #print "theta (deg):", np.rad2deg(theta)
            #print "rx:", rx, "ry:", ry
            #print "tx:", tx, "ty:", ty

        elif end_affector_name == RobotUtils.B_L_FOOT:

            link_global_xyz = self.b_l_end_affector.getWorldPosition([0, 0, 0])
            if at_point:
                link_global_xyz = at_point
            leg_dx = shoulder_world_xyz[0] - link_global_xyz[0]
            leg_dy = shoulder_world_xyz[1] - link_global_xyz[1]

            theta = np.arctan2(leg_dy, leg_dx )

            ry = R * np.sin(theta)
            rx = R * np.cos(theta)

            tx = S * np.cos( psi - yaw )
            ty = - S * np.sin( psi - yaw )

            #print "\n Back Left"
            #print "leg-torso dx, dy:\t", leg_dx, "\t", leg_dy
            #print "theta (deg):\t", np.rad2deg(theta)
            #print "rx:\t", rx, "\try:", ry
            #print "tx:\t", tx, "\tty:", ty

            delta_y_max = ry + ty
            delta_x_max = rx + tx

        elif end_affector_name == RobotUtils.F_R_FOOT:

            link_global_xyz = self.f_r_end_affector.getWorldPosition([0, 0, 0])
            if at_point:
                link_global_xyz = at_point
            leg_dx = shoulder_world_xyz[0] - link_global_xyz[0]
            leg_dy = shoulder_world_xyz[1] - link_global_xyz[1]
            theta = np.arctan2(leg_dy, leg_dx )

            rx = R * np.cos(theta)
            ry = R * np.sin(theta)

            tx = S * np.cos( yaw - psi - 180 )
            ty = S * np.sin( yaw - psi - 180 )
            
            #print "\n Front Right"
            #print "leg-torso dx, dy:", leg_dx, "\t", leg_dy
            #print "theta (deg):", np.rad2deg(theta)
            #print "rx:", rx, "ry:", ry
            #print "tx:", tx, "ty:", ty

            delta_y_max = ry + ty
            delta_x_max = rx + tx

        # back right
        else:
            link_global_xyz = self.b_r_end_affector.getWorldPosition([0, 0, 0])
            if at_point:
                link_global_xyz = at_point
            leg_dx = (shoulder_world_xyz[0] - link_global_xyz[0])
            leg_dy = (shoulder_world_xyz[1] - link_global_xyz[1])
            theta = np.arctan2(leg_dy, leg_dx )

            ry = R * np.sin(theta)
            rx = R * np.cos(theta)
            tx = S * np.cos(yaw + psi)
            ty = S * np.sin(yaw + psi)

            #print "\n Back Right"
            #print "leg-torso dx, dy:", leg_dx, "\t", leg_dy
            #print "theta (deg):", np.rad2deg(theta)
            #print "rx:", rx, "ry:", ry
            #print "tx:", tx, "ty:", ty

            delta_y_max = ry + ty
            delta_x_max = rx + tx

        R = np.sqrt(delta_x_max ** 2 + delta_y_max ** 2)

        #print "returning:",R
        return R



    def get_centroid_from_multiple_poly_intersections(self, _2Dgeometryobjects):

        if len(_2Dgeometryobjects) < 2:
            RobotUtils.ColorPrinter((self.__class__.__name__ + ".get_centroid_from_multiple_poly_intersections()"),
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

    def get_end_affectr_base_world_xyz_from_torso_world_xyz_and_yaw_deg(self, end_affector, torso_world_xyz, torso_final_yaw_degrees):

        end_affector_local_base_xyz  = self.get_local_end_affector_base_state_from_end_affector_name(end_affector)
        torso_final_yaw_rad = math.radians(torso_final_yaw_degrees)
        yaw_rotation_aa = ([0, 0, 1], torso_final_yaw_rad)
        yaw_rotation_R = so3.from_axis_angle(yaw_rotation_aa)
        rotated_local_base_xyz = so3.apply(yaw_rotation_R, end_affector_local_base_xyz)

        world_x = rotated_local_base_xyz[0] + torso_world_xyz[0]
        world_y = rotated_local_base_xyz[1] + torso_world_xyz[1]
        world_z = rotated_local_base_xyz[2]

        base_p =  [world_x, world_y, world_z]

        return base_p


    def get_torso_world_xyz(self):

        q = self.robosimian.getConfig()
        return [q[0],q[1],q[2]]


    def get_abs_yaw_deg_and_world_torso_xyz_commanded_from_legs(self, excluded_leg = None):

        torso_world_xyz = self.get_torso_world_xyz()
        delta_yaw_degrees, torso_commanded_xyz = self.get_delta_yaw_deg_and_world_torso_xyz_commanded_from_legs(excluded_leg = excluded_leg)

        abs_yaw = np.degrees(self.get_current_torso_yaw_rads()) + delta_yaw_degrees

        p1 = [torso_commanded_xyz[0],torso_commanded_xyz[1],torso_commanded_xyz[2]+.15]
        p2 = [torso_world_xyz[0],torso_world_xyz[1],torso_world_xyz[2]+.15]
        self.add_line_to_vis("torso shift to base", p1, p2)

        return abs_yaw, torso_commanded_xyz


    def get_delta_yaw_deg_and_world_torso_xyz_commanded_from_legs(self, excluded_leg = None):

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

        #print "in get_torso_and_legs_delta_yaw_deg_and_xyz_offset, exluded leg:",excluded_leg

        if excluded_leg:

            if not excluded_leg in RobotUtils.end_affectors:
                RobotUtils.ColorPrinter((self.__class__.__name__ + ".get_legs_xyz_yaw()"),
                                             "Error: excluded leg is a valid end affector name", "FAIL")
                return False

            if excluded_leg == RobotUtils.B_R_FOOT or excluded_leg == RobotUtils.F_L_FOOT:
                diaganol_pos_1 = b_l_world
                diaganol_pos_2 = f_r_world

            else:
                diaganol_pos_1 = b_r_world
                diaganol_pos_2 = f_l_world

            if excluded_leg in RobotUtils.left_feet:
                legs_x_delta = f_r_world[0] - b_r_world[0]
                legs_y_delta = f_r_world[1] - b_r_world[1]
                #print "considering left feet.\nlegs x,y delta:\t",legs_x_delta,"\t",legs_y_delta

            else:
                legs_x_delta = f_l_world[0] - b_l_world[0]
                legs_y_delta = f_l_world[1] - b_l_world[1]
                #print "considering right feet.\nlegs x,y delta:\t",legs_x_delta,"\t",legs_y_delta

            legs_commanded_x  = (diaganol_pos_1[0] + diaganol_pos_2[0]) / 2.0
            legs_commanded_y  = (diaganol_pos_1[1] + diaganol_pos_2[1]) / 2.0

        else:

            if not self.legs_make_base_state():
                RobotUtils.ColorPrinter((self.__class__.__name__+".get_legs_xyz_yaw()"),"Error: Legs do not make a base state","FAIL")
                return False

            legs_x_delta = f_l_world[0] - b_l_world[0]
            legs_y_delta = f_l_world[1] - b_l_world[1]

            #print "no excluded legs, considering left feet.\nlegs x,y delta:\t", legs_x_delta, "\t", legs_y_delta

            legs_commanded_x  = (f_l_world[0] + f_r_world[0] + b_l_world[0] + b_r_world[0]) / 4
            legs_commanded_y  = (f_l_world[1] + f_r_world[1] + b_l_world[1] + b_r_world[1]) / 4


        current_torso_yaw_deg = (math.degrees(self.get_current_torso_yaw_rads()) % 360)
        legs_yaw_degree = (math.degrees(np.arctan2( legs_y_delta , legs_x_delta ))  % 360)

        delta_yaw_degrees = -(current_torso_yaw_deg - legs_yaw_degree) % 360

        #print "\nlegs yaw degree:",legs_yaw_degree
        #print "current_torso_yaw_deg:",current_torso_yaw_deg
        #print "delta_yaw_degrees:",delta_yaw_degrees

        if delta_yaw_degrees > 180 and delta_yaw_degrees < 360:
            #print " delta yaw degrees is > 180 and < 360, sutracting 360"
            delta_yaw_degrees -= 360

        #print "delta_yaw_degrees normalizing [-180,180]:",delta_yaw_degrees,"\n"


        #print "legs_yaw_degree:", legs_yaw_degree
        #print "torso degree   :", current_torso_yaw_deg
        #print "delta_yaw_degrees:", delta_yaw_degrees, "\n"
        #print "legs commanded x, y:\t", legs_commanded_x, "\t", legs_commanded_y

        torso_world_xyz_commanded = [legs_commanded_x, legs_commanded_y, self.robosimian.getConfig()[2]]

        #self.add_line_to_vis("torso shift",robot_start,robot_end )

        return delta_yaw_degrees, torso_world_xyz_commanded





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
        x = 0
        if np.abs(x_delta) > .01:
            x = float(i) / float(i_max) * x_delta

        y_delta = y_end - y_start
        y = 0
        if np.abs(y_delta) > .01:
            y = float(i)/float(i_max) * y_delta

        z_delta = z_end - z_start
        z = float(i)/float(i_max) * z_delta

        # Arc height
        h = float(RobotUtils.STEP_Z_MAX_HIEGHT)
        b = x_delta

        # see https://www.desmos.com/calculator/v8wb6o83jh
        #       ((-4*h)/(b**2)) * x * (x-b) -> Parabolic Arc
        #       + z                         -> linear Z offset
        if b != 0:
            z_offset =  ( (-4*h) / (b**2)) * x * (x-b) + z
        else:
            z_offset = z

        #print "x:",x,"\ty:",z_offset,"\t\tb:",b,"\th:",h

        res = [ x_start + x, y_start + y , z_start + z_offset ]

        return res


    def get_desired_end_affector_rotation(self, end_affector_name):

        """
        @summary Returns a rotation matrix for a given
        @param end_affector_name:
        @return:
        """

        robot_yaw_rad = self.get_current_torso_yaw_rads()

        if end_affector_name in RobotUtils.left_feet:
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

        torso = self.robosimian.link(RobotUtils.TORSO_LINK_INDEX)

        R = torso.getTransform()[0]
        T = torso.getTransform()[1]

        rotated_point = so3.apply(R, local_xyz)

        translated_and_rotated_point = [rotated_point[0] + T[0], rotated_point[1] + T[1], rotated_point[2] + T[2]]

        return translated_and_rotated_point



    def get_local_end_affector_base_state_from_torso_translation(self, leg, translation, yaw_rotation_offset_degrees):

        '''
        @rtype: object
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


    def get_local_end_affector_base_state_from_end_affector_name(self, end_affector_name):
        
        if not end_affector_name in RobotUtils.end_affectors:
            RobotUtils.ColorPrinter(self.__class__.__name__,"Error: end_affector name unrecognized","FAIL")
            return None

        if end_affector_name == RobotUtils.B_L_FOOT:
            base_end_affector_state = self.local_b_l_end_affector_base_state

        elif (end_affector_name == RobotUtils.B_R_FOOT):
            base_end_affector_state = self.local_b_r_end_affector_base_state

        elif end_affector_name == RobotUtils.F_L_FOOT:
            base_end_affector_state = self.local_f_l_end_affector_base_state

        else:
            base_end_affector_state = self.local_f_r_end_affector_base_state

        return base_end_affector_state


    def get_shoulder_from_end_affector(self, end_affector_name):

        if not end_affector_name in RobotUtils.end_affectors:
            print_str = "Error: "+end_affector_name+" unrecognized"
            RobotUtils.ColorPrinter(self.__class__.__name__,print_str,"FAIL")
            return None

        if end_affector_name == RobotUtils.B_L_FOOT:
            end_affector = self.robosimian.link(RobotUtils.b_l_active_dofs[0])

        elif (end_affector_name == RobotUtils.B_R_FOOT):
            end_affector = self.robosimian.link(RobotUtils.b_r_active_dofs[0])

        elif end_affector_name == RobotUtils.F_L_FOOT:
            end_affector = self.robosimian.link(RobotUtils.f_l_active_dofs[0])

        else:
            end_affector = self.robosimian.link(RobotUtils.f_r_active_dofs[0])

        return end_affector


    def get_end_affector_from_end_affector_name(self,end_affector_name):

        if not end_affector_name in RobotUtils.end_affectors:
            print_str = "Error: "+end_affector_name+" unrecognized"
            RobotUtils.ColorPrinter(self.__class__.__name__,print_str,"FAIL")
            return None

        if end_affector_name == RobotUtils.B_L_FOOT:
            end_affector = self.b_l_end_affector

        elif (end_affector_name == RobotUtils.B_R_FOOT):
            end_affector = self.b_r_end_affector

        elif end_affector_name == RobotUtils.F_L_FOOT:
            end_affector = self.f_l_end_affector

        else:
            end_affector = self.f_r_end_affector

        return end_affector


