from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import numpy as np
from klampt.model import trajectory
from klampt import vis
import random

class _2DSupportPolygon(object):

    def __init__(self, P, name=None):

        self.name = name
        self.unchanged_points = P
        self.points = []
        for p in P:
            self.points.append(self.make_2d(p))

        self.shapely_poly = Polygon(self.points)

    def make_2d(self,_3dP):
        if len(_3dP) > 2:
            return (_3dP[0],_3dP[1])

    def point_is_inside(self, P):

        '''
        @summary returns whether a specified xyz coordinate is inside this support polygon/triangle
        @param P: xyz list
        @return: boolean
        '''

        point = Point(P[0],P[1])
        return self.shapely_poly.contains(point)


    def _2d_center_point(self):

        '''
        @summary returns the xy center point of a support polygon
        @return: tuple with x, y coordinates
        '''

        return self.shapely_poly.centroid.coords[0]

    def get_shapely_poly(self):
        return self.shapely_poly


    def point_is_inside_list_of_2DGeometry_objects(self, point_xyz, o_2D_objects):

        intersection = self.get_shapely_intersection_from_2DGeometry_objects(o_2D_objects)
        point = Point(point_xyz[0],point_xyz[1])

        return intersection.contains(point)



    def get_shapely_intersection_from_2DGeometry_objects(self, o_2DGeometry_objs):

        obj = self.shapely_poly
        o_objs = []
        for i in o_2DGeometry_objs:
            o_objs.append(i.shapely_poly)
        intersection = _2DSupportPolygon.get_intersection(obj, o_objs)

        return intersection


    def xy_centroid_from_list_of_2DGeometry_objects(self, o_2DGeometry_objs):

        '''
        @summary returns the centroid of the intersection area of numrerous _2D_xxx support ibjects
        @param o_2D_objects:
        @return:
        '''

        intersection = self.get_shapely_intersection_from_2DGeometry_objects(o_2DGeometry_objs)
        intersection_centroid = intersection.centroid.coords
        x = intersection_centroid.xy[0][0]
        y = intersection_centroid.xy[1][0]

        return [x,y]



    def point_is_inside_intersection_of_multiple_2DGeometry_objects(self, point_xyz, o_2D_objects):

        intersection = self.get_shapely_intersection_from_2DGeometry_objects(o_2D_objects)
        point = Point(point_xyz[0], point_xyz[1])

        return intersection.contains(point)


    @staticmethod
    def get_intersection(obj, o_objects):

        ret = obj.intersection(o_objects[0])
        if len(o_objects) >= 2:
            for o_obj in o_objects:
                ret = ret.intersection(o_obj)
        return ret


    def support_poly_is_in_intersection_with_other_2d_support_poly(self, o__2DSupportPolygon, P):

        if not type(o__2DSupportPolygon) == type(self):
            print "Error in support_poly_is_in_intersection_with_other_2d_support_poly(): o__2DSupportPolygon is not of type __2DSupportPolygon"
            return False

        o_poly = o__2DSupportPolygon.get_shapely_poly()
        intersection_poly = self.shapely_poly.intersection(o_poly)
        point = Point(P[0],P[1])

        return intersection_poly.contains(point)


    def xy_centroid_of_intersection(self, o__2DSupportPolygon):

        if not type(o__2DSupportPolygon) == type(self):
            print "Error in centroid_of_intersection(): o__2DSupportPolygon is not of type __2DSupportPolygon"
            return False

        o_poly = o__2DSupportPolygon.get_shapely_poly()

        intersection_poly = self.shapely_poly.intersection(o_poly)

        return intersection_poly.centroid.coords[0]


    def remove_visualization(self):
        vis.remove(self.name)


    def visualize(self):

        milestones = []

        for p in self.unchanged_points:
            milestones.append(p)

        milestones.append(self.unchanged_points[0])

        circle = trajectory.Trajectory(milestones=milestones)
        if not self.name:
            self.name = "supoprt polygon " + str(random.randint(1,1000))

        vis.add(self.name, circle)


    def plot_poly(self, poly, z, name=None):

        x, y = poly.exterior.coords.xy

        xyz = []

        for i in range(len(y)):
            p = [ x[i], y[i], z ]
            xyz.append(p)

        traj = trajectory.Trajectory(milestones=xyz)
        if name is None:
            name = "supoprt polygon " + str(random.randint(1,1000))
        name = str(name)
        vis.add(name, traj)