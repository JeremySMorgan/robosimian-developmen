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




    def xy_centroid_from_list_of_2DGeometry_objects(self, o_2D_objects):

        '''
        @summary returns the centroid of the intersection area of numrerous _2D_xxx support ibjects
        @param o_2D_objects:
        @return:
        '''

        intersect = self.shapely_poly.intersection(o_2D_objects[0].shapely_poly)

        if len(o_2D_objects) >= 2:
            for o_obj in o_2D_objects:
                intersect = intersect.intersection(o_obj.shapely_poly)

        intersection_centroid = intersect.centroid.coords
        x = intersection_centroid.xy[0][0]
        y = intersection_centroid.xy[1][0]

        return [x, y]





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




    def visualize(self):

        milestones = []

        for i in range(361):

            x = self.x + self.R*np.cos(np.radians(i))
            y = self.y + self.R*np.sin(np.radians(i))

            milestones.append([x,y,self.z])

        circle = trajectory.Trajectory(milestones=milestones)

        if self.name:
            name = self.name
        else:
            name = "circle "+str(random.randint(1,1000))

        vis.add(name, circle)