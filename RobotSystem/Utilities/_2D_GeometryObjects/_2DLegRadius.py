from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import numpy as np
from klampt.model import trajectory
from klampt import vis
import random

class _2DLegRadius(object):

    '''
        This class stores a shapely circle and is used to represent the range of end affectors. It only supports 2d circles
        which is not precise but is sufficient for current purposes.
    '''

    def __init__(self, P, r, name=None):

        self.name = name
        self.x = P[0]
        self.y = P[1]
        self.z = P[2]
        self.center = Point(self.make_2d(P))
        self.R      = r
        self.shapely_poly = self.center.buffer(self.R)


    def make_2d(self, _3dP):
        if len(_3dP) > 2:
            return (_3dP[0],_3dP[1])


    def point_is_inside_intersection_of_multiple_2DGeometry_objects(self, point_xyz, o_2D_objects):

        intersection = self.get_shapely_intersection_from_2DGeometry_objects(o_2D_objects)
        point = Point(point_xyz[0], point_xyz[1])

        return intersection.contains(point)

    def get_shapely_intersection_from_2DGeometry_objects(self, o_2DGeometry_objs):

        obj = self.shapely_poly
        o_objs = []
        for i in o_2DGeometry_objs:
            o_objs.append(i.shapely_poly)
        return _2DLegRadius.get_intersection(obj, o_objs)


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


    def _2d_center_point(self):

        '''
        @summary returns the xy center point of a support polygon
        @return: tuple with x, y coordinates
        '''

        return self.shapely_poly.centroid.coords[0]

    def point_is_inside(self, P):

        '''
        @summary returns whether a specified xyz coordinate is inside this circle
        @param P: xyz list
        @return: boolean
        '''

        point = Point(P[0],P[1])
        return self.shapely_poly.contains(point)

    def remove_visualization(self):
        vis.hide(self.name)

    def visualize(self):

        milestones = []
        for i in range(361):
            x = self.x + self.R*np.cos(np.radians(i))
            y = self.y + self.R*np.sin(np.radians(i))
            milestones.append([x,y,self.z])
        circle = trajectory.Trajectory(milestones=milestones)
        if not self.name:
            self.name = "circle "+str(random.randint(1,1000))
        vis.add(self.name, circle)