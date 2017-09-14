#!/usr/bin/python

import time
import sys
import math
from klampt.robotsim import setRandomSeed
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import RobotPoser
from klampt.math import so3
from klampt import *
from klampt.model import ik,coordinates

from RobotSubSystems.MotionController.MotionController import MotionController
from RobotSubSystems.MotionPlanner.MotionPlanner import MotionPlanner
from RobotSubSystems.RobotUtils.RobotUtils import RobotUtils

if RobotUtils.SIMULATION_ENABLED:
    from klampt import vis

class Hypervisor():

    def __init__(self,robot_file):

        self.robot_file = robot_file
        self.robosimian = None
        self.world = None
        self.MotionController = None
        self.MotionPlanner = None


    def start(self):

        world = WorldModel()
        res = world.readFile(self.robot_file)
        if not res:
            raise RuntimeError("Unable to load model")

        coordinates.setWorldModel(world)

        # Initialize Visualization
        self.initialize_visualization(world)

        # Save a copy of the robot locally
        self.robosimian = world.robot(0)

        # Create MotionController
        self.MotionController = MotionController(self.robosimian, RobotUtils)
        self.MotionController.setInitalConfig()

        # Create MotionPlanner
        self.MotionPlanner = MotionPlanner(self.robosimian,RobotUtils)
        self.MotionPlanner.save_base_foot_states()

        # Pass the Motion Controller the MotionPlanner
        self.MotionController.initialize_motion_planner(self.MotionPlanner)

        # Add visualizations items for debugging
        self.addVisualizationItems()

        RobotUtils.ColorPrinter(self.__class__.__name__,"Hypervisor initialization finished","OKBLUE")


    def addVisualizationItems(self):

        for leg_name in RobotUtils.end_affectors:
            base_pos = self.MotionPlanner.get_local_foot_base_state_from_foot_name(leg_name)
            vis.add(leg_name,base_pos)



    def initialize_visualization(self,world):
        vis.add("world", world)
        #vis.add("coordinates", coordinates.manager())
        vp = vis.getViewport()
        vp.w, vp.h = 1600, 1600
        vis.setViewport(vp)
        #vis.listItems(indent=4)
        vis.autoFitCamera()
        vis.show()


    def shift_robot(self):

        q = self.robosimian.getConfig()
        q[0] += .25
        q[3] += 3.141592 / 4.0
        self.robosimian.setConfig(q)

    def run_visualization(self):

        time.sleep(7)

        translation = [RobotUtils.STEP_X_DELTA, 0, 0]

        f_r = RobotUtils.F_R_FOOT
        f_l = RobotUtils.F_L_FOOT
        b_r = RobotUtils.B_R_FOOT
        b_l = RobotUtils.B_L_FOOT

        base_state = RobotUtils.LEG_BASE_STATE
        f_extend_state = RobotUtils.LEG_F_EXTEND_STATE

        # Begin walk
        self.MotionController.make_leg_step(f_r, base_state, f_extend_state)
        self.MotionController.make_leg_step(b_l, base_state, f_extend_state)
        self.MotionController.make_leg_step(b_r, base_state, f_extend_state)
        self.MotionController.make_leg_step(f_l, base_state, f_extend_state)

        self.MotionController.make_torso_shift_from_local_xyz_translation(translation)

        self.MotionController.make_leg_step(f_r, base_state, f_extend_state)
        self.MotionController.make_leg_step(b_l, base_state, f_extend_state)
        self.MotionController.make_leg_step(b_r, base_state, f_extend_state)
        self.MotionController.make_leg_step(f_l, base_state, f_extend_state)

        self.MotionController.make_torso_shift_from_local_xyz_translation(translation)

        self.MotionController.make_leg_step(f_r, base_state, f_extend_state)
        self.MotionController.make_leg_step(b_l, base_state, f_extend_state)
        self.MotionController.make_leg_step(b_r, base_state, f_extend_state)
        self.MotionController.make_leg_step(f_l, base_state, f_extend_state)

        self.MotionController.make_torso_shift_from_local_xyz_translation(translation)


        while RobotUtils.SIMULATION_ENABLED:

            # Update model
            vis.lock()
            vis.unlock()

            time.sleep(RobotUtils.SIMULATION_FRAME_DELAY)
            if not vis.shown():
                sys.exit()



















'''
f_r = RobotUtils.F_R_FOOT
f_l = RobotUtils.F_L_FOOT
b_r = RobotUtils.B_R_FOOT
b_l = RobotUtils.B_L_FOOT

base_state = RobotUtils.LEG_BASE_STATE
f_extend_state = RobotUtils.LEG_F_EXTEND_STATE

self.MotionController.make_leg_step( f_r, base_state, f_extend_state )
self.MotionController.make_leg_step( b_l, base_state, f_extend_state )
self.MotionController.make_leg_step(b_r, base_state, f_extend_state)
self.MotionController.make_leg_step(f_l, base_state, f_extend_state)
'''



