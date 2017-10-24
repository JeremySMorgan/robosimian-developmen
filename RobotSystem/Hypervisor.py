#!/usr/bin/python

import sys
import time

import math

from klampt import *
from klampt.model import coordinates

from RobotSubSystems.MotionController.HighLevelMotionController import HighLevelMotionController
from RobotSubSystems.MotionPlanner.MotionPlanner import MotionPlanner
from RobotSubSystems.ObjectiveManager.ObjectiveManager import ObjectiveManager
from RobotSubSystems.UserInput.UserInput import UserInput
from RobotSubSystems.StabilityManager.StabilityManager import StabilityManager
from Utilities.RobotUtils.RobotUtils import RobotUtils


if RobotUtils.SIMULATION_ENABLED:
    from klampt import vis

class Hypervisor():

    def __init__(self,robot_file, world_file):

        self.robot_file = robot_file
        self.world_file = world_file
        self.robosimian = None
        self.world = None
        self.MotionController = None
        self.MotionPlanner = None


    def start(self):

        world = WorldModel()

        res = world.readFile(self.robot_file)
        if not res:
            raise RuntimeError("Unable to load model")

        if RobotUtils.INCLUDE_TERRAIN:
            terrain = world.readFile(self.world_file)
            if not terrain:
                raise RuntimeError("Unable to load plane")


        self.initialize_visualization(world)
        self.controller_world = world.copy()
        self.robosimian = self.controller_world.robot(0)
        self.sim =  Simulator(world)

        # Create robot controller
        self.controller = self.sim.controller(0)
        self.controller.setRate(RobotUtils.CONTROLLER_DT)

        self.StabilityManager = StabilityManager(self.robosimian, self.sim, RobotUtils)

        # Create HighLevelMotionController
        self.HighLevelMotionController = HighLevelMotionController(self.robosimian, RobotUtils, self.controller)

        # Create HighLevelMotionController
        self.MotionPlanner = MotionPlanner(self.robosimian,RobotUtils)

        # Pass the Motion Controller the MotionPlanner and set the initial conifguration
        self.HighLevelMotionController.initialize_motion_planner(self.MotionPlanner)
        self.HighLevelMotionController.set_inital_config()
        self.MotionPlanner.save_base_states()

        # User Input
        self.UserInput = UserInput(RobotUtils)

        # Objective Manager
        self.ObjectiveManager = ObjectiveManager(RobotUtils, self.MotionPlanner, self.HighLevelMotionController, self.UserInput )

        RobotUtils.ColorPrinter(self.__class__.__name__,"Hypervisor initialization finished","OKBLUE")

        try:

            self.UserInput.start()
            self.ObjectiveManager.start_objective_management_loop()
            self.run_visualization()

        except KeyboardInterrupt:
            self.shutdown()


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
            vis.autoFitCamera()
            vis.show()


    def run_visualization(self):

        while RobotUtils.SIMULATION_ENABLED:


            # Update model
            vis.lock()

            if RobotUtils.PHYSICS_ENABLED:
                self.HighLevelMotionController.control_loop()

                self.sim.updateWorld()
                self.sim.simulate(RobotUtils.CONTROLLER_DT)
                self.StabilityManager.check_status()

            vis.unlock()

            time.sleep(RobotUtils.CONTROLLER_DT)
            if not vis.shown():
                sys.exit()


