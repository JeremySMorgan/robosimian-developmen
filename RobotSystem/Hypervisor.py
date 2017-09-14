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

        # Add visualizations items for debugging
        self.addVisualizationItems()

        # Create MotionController
        self.MotionController = MotionController(self.robosimian, RobotUtils)
        self.MotionController.setInitalConfig()

        # Create MotionPlanner
        self.MotionPlanner = MotionPlanner(self.robosimian,RobotUtils)
        self.MotionPlanner.save_base_foot_states()

        # Pass the Motion Controller the MotionPlanner
        self.MotionController.initialize_motion_planner(self.MotionPlanner)

        RobotUtils.ColorPrinter(self.__class__.__name__,"Hypervisor initialization finished","OKBLUE")


    def addVisualizationItems(self):
        pass


    def initialize_visualization(self,world):
        vis.add("world", world)
        #vis.add("coordinates", coordinates.manager())
        vp = vis.getViewport()
        vp.w, vp.h = 1600, 1600
        vis.setViewport(vp)
        #vis.listItems(indent=4)
        vis.autoFitCamera()
        vis.show()

    def run_visualization(self):

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

        while RobotUtils.SIMULATION_ENABLED:

            # Update model
            vis.lock()
            vis.unlock()

            time.sleep(RobotUtils.SIMULATION_FRAME_DELAY)
            if not vis.shown():
                sys.exit()
