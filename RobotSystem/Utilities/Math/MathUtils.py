import math
import numpy as np

class MathUtils(object):

    @staticmethod
    def angle_between_three_points(p1, p2, p3):

        # see https://stackoverflow.com/questions/35176451/python-code-to-calcualte-angle-between-three-point-using-thier-3d-coordinates

        a = np.array(p1)
        b = np.array(p2)
        c = np.array(p3)

        ba = a - b
        bc = c - b

        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))

        return np.degrees(np.arccos(cosine_angle))


    @staticmethod
    def get_euclidian_diff(xyz1, xyz2):

        d_x = (xyz1[0] - xyz2[0])**2
        d_y = (xyz1[1] - xyz2[1])**2
        d_z = (xyz1[2] - xyz2[2])**2

        return math.sqrt( d_x +d_y  +d_z    )

    @staticmethod
    def always_true_func():
        return True
