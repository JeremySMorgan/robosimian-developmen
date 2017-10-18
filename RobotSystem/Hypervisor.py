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

        self.robosimian = world.robot(0)

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

        # Pass the Motion Controller the MotionPlanner
        self.HighLevelMotionController.initialize_motion_planner(self.MotionPlanner)

        # User Input
        self.UserInput = UserInput(RobotUtils)

        # Objective Manager
        self.ObjectiveManager = ObjectiveManager(RobotUtils, self.MotionPlanner, self.HighLevelMotionController, self.UserInput )

        RobotUtils.ColorPrinter(self.__class__.__name__,"Hypervisor initialization finished","OKBLUE")

        valid_run = True

        try:

            if valid_run:

                self.UserInput.start()
                self.ObjectiveManager.start_objective_management_loop()
                self.run_visualization()

            while 1:
                time.sleep(1)

        except KeyboardInterrupt:
            self.shutdown()





    def rotate_torso_yaw(self):
        q = self.robosimian.getConfig()
        q[3] = 3.141592/6
        self.robosimian.setConfig(q)

    def shift_x(self):
        q = self.robosimian.getConfig()
        q[0] -= .75
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
            vis.autoFitCamera()
            vis.show()


    def run_visualization(self):

        while RobotUtils.SIMULATION_ENABLED:

            # Update model
            vis.lock()

            if RobotUtils.PHYSICS_ENABLED:
                self.HighLevelMotionController.control_loop()

            vis.unlock()

            if RobotUtils.PHYSICS_ENABLED:
                self.sim.updateWorld()
                self.sim.simulate(RobotUtils.CONTROLLER_DT)

            time.sleep(RobotUtils.CONTROLLER_DT)
            if not vis.shown():
                sys.exit()










"""
            self.shift_x()
            self.rotate_torso_yaw()

            for i in range(30):
                self.HighLevelMotionController.rotate_torso_from_yaw_offset(1)
                time.sleep(.1)

            for i in range(50):
                self.HighLevelMotionController.rotate_torso_from_yaw_offset(-1)
                time.sleep(.1)

"""



"""
            self.rotate_torso_yaw()

            link = RobotUtils.F_L_FOOT

            translation = [0, 0, 0]

            for yaw_offset in range(1, 120, 20):

                calculated_new_des = self.MotionPlanner.get_local_foot_base_state_from_torso_translation(link,
                                                                                                         translation,
                                                                                                         yaw_offset)
                st = str(yaw_offset)
                vis.add(st, calculated_new_des)

"""
