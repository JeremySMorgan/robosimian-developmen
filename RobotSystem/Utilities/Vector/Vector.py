import math

class Vector(object):

    vector_dir_error_threshold = .05

    def __init__(self, V, P0 = None):

        self.V = V

        self.dx = V[0]
        self.dy = V[1]
        self.dz = V[2]

        if P0:
            self.x0 = P0[0]
            self.y0 = P0[1]
            self.z0 = P0[2]

    def multiple_vector_directions_are_equal(self, oVectors):

        for oVector in oVectors:
            if not self.vector_directions_are_equal(oVector):
                return False
        return True


    def vector_directions_are_equal(self, oVector):

        if type(oVector) == type(self):
            if self.get_euclidian_diff(self.V, oVector.V) > Vector.vector_dir_error_threshold:
                return False
            return True
        else:
            print "Error in Vector.vector_directions_are_equal: oVector is not of type Vector"

    def __str__(self):
        return "<"+self.pp(self.dx)+","+self.pp(self.dy)+","+self.pp(self.dz)+">"


    def pp(self,dbl):
        s = "%.3f" % dbl
        return s


    def get_euclidian_diff(self, xyz1, xyz2):

        d_x = (xyz1[0] - xyz2[0])**2
        d_y = (xyz1[1] - xyz2[1])**2
        d_z = (xyz1[2] - xyz2[2])**2

        return math.sqrt( d_x +d_y  +d_z    )
