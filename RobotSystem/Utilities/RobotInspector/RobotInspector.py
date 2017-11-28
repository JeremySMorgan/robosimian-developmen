from klampt.model import trajectory
import numpy as np
from klampt import vis
import time
from ...Utilities.Logging.Logger import Logger

class RobotInspector(object):
    
    def __init__(self, robosimian, MotionPlanner, HighLevelMotionController, RobotConstants):
        self.robosimian = robosimian
        self.MotionPlanner = MotionPlanner
        self.HighLevelMotionController = HighLevelMotionController
        self.RobotConstants = RobotConstants


    def shift_xy_rotate_yaw_when_initialization_done(self):

        while not self.HighLevelMotionController.initialization_complete:
            time.sleep(.05)

        self.shift_x(amount=3)
        self.shift_y(amount=2)
        self.rotate_yaw(degree=-5)

    def print_config(self):
        print Logger.pp_list(self.robosimian.getConfig())

    def update_torso_COM(self):

        x = self.robosimian.getConfig()[0]
        y = self.robosimian.getConfig()[1]
        z = self.robosimian.getConfig()[2]
        center = [x, y, z]
        ground = [x, y, z + self.RobotConstants.BASE_STATE_Z_DELTA]
        self.add_line_to_vis("torso xy", center, ground)

    def get_lr_distance(self):
        fl = self.MotionPlanner.get_end_affector_from_end_affector_name(self.RobotConstants.F_L_FOOT)
        fr = self.MotionPlanner.get_end_affector_from_end_affector_name(self.RobotConstants.F_R_FOOT)

        fl_xyz_w = fl.getWorldPosition([0,0,0])
        fr_xyz_w = fr.getWorldPosition([0, 0, 0])

        fl_xyz_l = fl.getLocalPosition([0,0,0])
        fr_xyz_l = fr.getLocalPosition([0, 0, 0])

        delta_xyz_w = [fl_xyz_w[0] - fr_xyz_w[0], fl_xyz_w[1] - fr_xyz_w[1], fl_xyz_w[2] - fr_xyz_w[2]]
        delta_xyz_l = [fl_xyz_l[0] - fr_xyz_l[0], fl_xyz_l[1] - fr_xyz_l[1], fl_xyz_l[2] - fr_xyz_l[2]]

        bs_y_delta = self.RobotConstants.BASE_STATE_Y_DELTA
        shoulder_y = self.RobotConstants.SHOULDER_Y
        sum_y       = bs_y_delta + shoulder_y

        print "\nfl, fr  world:\t",Logger.pp_list(fl_xyz_w), "\t", Logger.pp_list(fr_xyz_w)
        print "fl, fr  local:\t",Logger.pp_list(fl_xyz_l), "\t", Logger.pp_list(fr_xyz_l)
        print "delta xyz world:",Logger.pp_list(delta_xyz_w)
        print "delta xyz local:",Logger.pp_list(delta_xyz_l)
        print "base state y delta, shoulder y, sum, 2x sum:",Logger.pp_double(bs_y_delta),Logger.pp_double(shoulder_y),Logger.pp_double(sum_y),Logger.pp_double(2*sum_y),"\n"


    def get_fb_distance(self):

        fl = self.MotionPlanner.get_end_affector_from_end_affector_name(self.RobotConstants.F_L_FOOT)
        bl = self.MotionPlanner.get_end_affector_from_end_affector_name(self.RobotConstants.B_L_FOOT)

        fl_xyz_w = fl.getWorldPosition([0, 0, 0])
        bl_xyz_w = bl.getWorldPosition([0, 0, 0])

        fl_xyz_l = fl.getLocalPosition([0, 0, 0])
        bl_xyz_l = bl.getLocalPosition([0, 0, 0])

        delta_xyz_w = [fl_xyz_w[0] - bl_xyz_w[0], fl_xyz_w[1] - bl_xyz_w[1], fl_xyz_w[2] - bl_xyz_w[2]]
        delta_xyz_l = [fl_xyz_l[0] - bl_xyz_l[0], fl_xyz_l[1] - bl_xyz_l[1], fl_xyz_l[2] - bl_xyz_l[2]]

        bs_x_delta = self.RobotConstants.BASE_STATE_X_DELTA
        shoulder_x = self.RobotConstants.SHOULDER_X
        sum_x = bs_x_delta + shoulder_x

        print "\nfl, bl  world:\t", Logger.pp_list(fl_xyz_w), "\t", Logger.pp_list(bl_xyz_w)
        print "fl, bl  local:\t", Logger.pp_list(fl_xyz_l), "\t",Logger.pp_list(bl_xyz_w)
        print "delta xyz world:", Logger.pp_list(delta_xyz_w)
        print "delta xyz local:", Logger.pp_list(delta_xyz_l)
        print "base state x delta, shoulder x, sum, 2x sum:", Logger.pp_double(bs_x_delta), Logger.pp_double(shoulder_x), Logger.pp_double(sum_x), Logger.pp_double(2 * sum_x),"\n"



    def shift_y(self, amount=None):

        q = self.robosimian.getConfig()
        q[1] = .25
        if amount:
            q[1] = amount
        self.robosimian.setConfig(q)

    def shift_x(self,amount=None):
        q = self.robosimian.getConfig()
        q[0] = 1
        if amount:
            q[0] = amount
        self.robosimian.setConfig(q)

    def rotate_yaw(self, degree=None):

        q = self.robosimian.getConfig()

        q[3] = (1 * np.pi / 6)
        if degree==0 or degree!=None:
            q[3] = np.deg2rad(degree)

        self.robosimian.setConfig(q)


    def get_shoulder_torso_shoulder_angle_offset(self):

        q = self.robosimian.getConfig()
        q[0] = 0
        q[1] = 0
        self.robosimian.setConfig(q)

        shoulder_xyz = self.robosimian.link(31).getWorldPosition([0, 0, 0])

        self.add_line_to_vis("test",[0,0,shoulder_xyz[2]],shoulder_xyz)

        psi = np.arctan(shoulder_xyz[1]/shoulder_xyz[0])

        print "psi:",psi




    def get_shoulder_torso_xy_euclidean_dif(self):

        q = self.robosimian.getConfig()
        q[0] = 0
        q[1] = 0
        self.robosimian.setConfig(q)

        shoulder_xyz = self.robosimian.link(7).getWorldPosition([0, 0, 0])

        self.add_line_to_vis("test",[0,0,shoulder_xyz[2]],shoulder_xyz)

        l = np.sqrt(shoulder_xyz[0]**2 + shoulder_xyz[1]**2)

        print "shoulder_torso_xy_euclidean_dif:",l




    def add_line_to_vis(self,name,p1,p2):

        traj = trajectory.Trajectory(milestones=[p1,p2])
        vis.add(name,traj)


    def get_trajectory_circle(self, xyz, r ):

        x0 = xyz[0]
        y0 = xyz[1]
        z0 = xyz[2]

        milestones = []

        for i in range(361):

            x = x0 + r*np.cos(np.radians(i))
            y = y0 + r*np.sin(np.radians(i))

            milestones.append([x,y,z0])

        return trajectory.Trajectory(milestones=milestones)


    def get_leg_shoulder_delta_z(self):

        shoulder_xyz = self.robosimian.link(7).getWorldPosition([0, 0, 0])
        end_aff_xyz = self.robosimian.link(13).getWorldPosition([0, 0, 0])
        d_z = end_aff_xyz[2] - shoulder_xyz[1]

        print "Shoulder xyz:", shoulder_xyz
        print "end_aff_xyz:", end_aff_xyz
        print "dz: ", d_z

        # delta z offset    |    d_z
        #   .55             |    .3298
        #   .35             |    .133285
        #   .65             |    .4272240

        # Measured dz = 0.980182 * (delta_z_offset) - .20965


    def get_leg_length(self):

        q = self.robosimian.getConfig()
        q[7] = (3.141592 / 2.0)
        q[9] = (3.141592)
        q[11] = (3.141592)
        self.robosimian.setConfig(q)

        shoulder_xyz = self.robosimian.link(7).getWorldPosition([0, 0, 0])
        end_aff_xyz = self.robosimian.link(13).getWorldPosition([0, 0, 0])

        vis.add("shoulder", shoulder_xyz)
        vis.add("end aff", end_aff_xyz)


        d_x = end_aff_xyz[0] - shoulder_xyz[0]
        d_y = end_aff_xyz[1] - shoulder_xyz[1]
        d_z = end_aff_xyz[2] - shoulder_xyz[1]

        L = np.sqrt(d_x**2 + d_y**2)

        vis.add("leg radius",self.get_trajectory_circle(end_aff_xyz, L))
        self.add_line_to_vis("leg - shoulder",end_aff_xyz, shoulder_xyz)

        print "Shoulder xyz:", shoulder_xyz
        print "end_aff_xyz:", end_aff_xyz
        print "dx: ", d_x
        print "dy: ", d_y
        print "l:",L
        print "dz: ", d_z
