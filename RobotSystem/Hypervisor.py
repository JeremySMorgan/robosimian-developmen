#!/usr/bin/python

import sys
import time

from klampt import *
from klampt.model import coordinates

from RobotSubSystems.MotionController.HighLevelMotionController import HighLevelMotionController
from RobotSubSystems.MotionPlanner.MotionPlanner import MotionPlanner
from RobotSubSystems.ObjectiveManager.ObjectiveManager import ObjectiveManager
from RobotSubSystems.StabilityManager.StabilityManager import StabilityManager
from RobotSubSystems.UserInput.UserInput import UserInput
from Utilities.RobotUtils.RobotUtils import RobotUtils
from Utilities.RobotInspector.RobotInspector import RobotInspector

if RobotUtils.SIMULATION_ENABLED:
    from klampt import vis


class Hypervisor():

    def __init__(self, robot_file, world_file):

        self.robot_file = robot_file
        self.world_file = world_file
        self.planning_world_robosimian = None
        self.world = None
        self.MotionController = None
        self.MotionPlanner = None
        self.gravity_paused = True

    def start(self):

        sim_world = WorldModel()

        res = sim_world.readFile(self.robot_file)
        if not res:
            raise RuntimeError("Unable to load model")

        if RobotUtils.INCLUDE_TERRAIN:
            terrain = sim_world.readFile(self.world_file)
            if not terrain:
                raise RuntimeError("Unable to load plane")

        self.planning_world = sim_world.copy()
        self.planning_world_robosimian = self.planning_world.robot(0)
        self.sim_world_roboosimian = sim_world.robot(0)

        # Set initial configurations. Note that this is may not be the base state, but it will be close to it.
        starting_config  = RobotUtils.STARTING_CONFIG
        self.sim_world_roboosimian.setConfig(starting_config)
        self.planning_world_robosimian.setConfig(starting_config)

        # Initialize visualization
        if RobotUtils.PHYSICS_ENABLED:
            self.initialize_visualization(sim_world)
        else:
            self.initialize_visualization(self.planning_world)

        # initialize simulation
        self.sim =  Simulator(sim_world)
        self.sim.setGravity([0,0,0])

        # Create robot controller
        self.controller = self.sim.controller(0)
        self.controller.setRate(RobotUtils.CONTROLLER_DT)

        # Initilialize RobotSubsystems
        self.SimWorldStabilityManager = StabilityManager(sim_world, self.sim)
        self.PlannerWorldStabilityManager = StabilityManager(self.planning_world, self.sim)
        self.HighLevelMotionController = HighLevelMotionController(self.planning_world_robosimian, self.controller)
        self.MotionPlanner = MotionPlanner(self.planning_world_robosimian)
        self.HighLevelMotionController.initialize_motion_planner(self.MotionPlanner)

        # Initialize Utility Classes
        self.UserInput = UserInput()
        self.ObjectiveManager = ObjectiveManager(self.MotionPlanner, self.HighLevelMotionController, self.UserInput )
        self.PlanningWorldRobotInspector = RobotInspector(self.planning_world_robosimian, self.MotionPlanner, self.HighLevelMotionController)
        self.SimWorldRobotInspector = RobotInspector(self.planning_world_robosimian, self.MotionPlanner, self.HighLevelMotionController)

        # Start RobotSubsystems
        self.UserInput.start()
        self.ObjectiveManager.start()

        RobotUtils.ColorPrinter(self.__class__.__name__,"Hypervisor initialization finished","OKBLUE")

        # run visualizations
        self.PlanningWorldRobotInspector.add_line_to_vis(" --- X --- ", [1, 0, 0], [0, 0, 0])
        self.PlanningWorldRobotInspector.add_line_to_vis(" --- Y --- ", [0, 1, 0], [0, 0, 0])
        self.run_visualization()


    def shutdown(self):

        if self.ObjectiveManager:
            self.ObjectiveManager.shutdown()
        if self.UserInput:
            self.UserInput.shutdown()


    def initialize_visualization(self, world):

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

            vis.lock()

            self.PlanningWorldRobotInspector.update_torso_COM()

            if RobotUtils.PHYSICS_ENABLED:
                if self.gravity_paused:
                    if self.HighLevelMotionController.initialization_complete:
                        self.sim.setGravity([0, 0, -9.8])
                        self.gravity_paused = False
                        self.MotionPlanner.save_base_states()

                self.HighLevelMotionController.control_loop()
                self.sim.updateWorld()
                self.sim.simulate(RobotUtils.CONTROLLER_DT)
                self.SimWorldStabilityManager.check_status()

            else:
                self.PlannerWorldStabilityManager.check_status()

            vis.unlock()
            time.sleep(RobotUtils.CONTROLLER_DT)
            if not vis.shown():
                sys.exit()
