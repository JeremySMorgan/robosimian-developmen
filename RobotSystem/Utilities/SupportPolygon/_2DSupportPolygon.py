from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

class _2DSupportPolygon(object):

    def __init__(self, P):

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


