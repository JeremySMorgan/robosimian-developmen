#!/usr/bin/python

import sys
import time

from klampt import *
from klampt.model import coordinates

from RobotSubSystems.MotionController.HighLevelMotionController import HighLevelMotionController
from RobotSubSystems.MotionPlanner.MotionPlanner import MotionPlanner
from RobotSubSystems.ObjectiveManager.ObjectiveManager import ObjectiveManager
from RobotSubSystems.UserInput.UserInput import UserInput
from Utilities.RobotUtils.RobotUtils import RobotUtils

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

        # add world to visualization

        # save the simulation
        self.sim =  Simulator(world)
        self.sim.simulate(RobotUtils.CONTROLLER_DT)

        # Create robot controller
        self.controller = self.sim.controller(self.robosimian)
        self.controller.setRate(RobotUtils.CONTROLLER_DT)

        # Create HighLevelMotionController
        self.HighLevelMotionController = HighLevelMotionController(self.robosimian, RobotUtils, self.controller)
        self.HighLevelMotionController.set_inital_config()

        # Create HighLevelMotionController
        self.MotionPlanner = MotionPlanner(self.robosimian,RobotUtils)
        self.MotionPlanner.save_base_foot_states()

        # Pass the Motion Controller the MotionPlanner
        self.HighLevelMotionController.initialize_motion_planner(self.MotionPlanner)

        # User Input
        self.UserInput = UserInput(RobotUtils)
        self.UserInput.start()

        # Objective Manager
        self.ObjectiveManager = ObjectiveManager(RobotUtils, self.MotionPlanner, self.HighLevelMotionController, self.UserInput )

        RobotUtils.ColorPrinter(self.__class__.__name__,"Hypervisor initialization finished","OKBLUE")

        try:

            self.ObjectiveManager.start_objective_management_loop()

            while 1:
                time.sleep(1)

        except KeyboardInterrupt:
            self.shutdown()

    def rotate_torso_yaw(self):
        q = self.robosimian.getConfig()
        q[3] += 3.141592/4
        self.robosimian.setConfig(q)

    def shutdown(self):

        if self.ObjectiveManager:
            self.ObjectiveManager.shutdown()
        if self.UserInput:
            self.UserInput.shutdown()


    def initialize_visualization(self,world):

        if RobotUtils.SIMULATION_ENABLED:
            vis.add("world", world)
            vis.add("coordinates", coordinates.manager())
            vp = vis.getViewport()
            vp.w, vp.h = 800, 800
            vis.setViewport(vp)
            #vis.listItems(indent=4)
            vis.autoFitCamera()
            vis.show()


    def run_visualization(self):

        while RobotUtils.SIMULATION_ENABLED:

            # Update model
            vis.lock()


            vis.unlock()

            time.sleep(RobotUtils.SIMULATION_FRAME_DELAY)
            if not vis.shown():
                sys.exit()
