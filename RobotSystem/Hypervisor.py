#!/usr/bin/python

import sys
import time

import numpy as np

from Utilities._2D_GeometryObjects._2DLegRadius import _2DLegRadius
from Utilities._2D_GeometryObjects._2DSupportPolygon import _2DSupportPolygon

from klampt import *
from klampt.model import coordinates
from klampt.model import trajectory

from RobotSubSystems.MotionController.HighLevelMotionController import HighLevelMotionController
from RobotSubSystems.MotionPlanner.MotionPlanner import MotionPlanner
from RobotSubSystems.ObjectiveManager.ObjectiveManager import ObjectiveManager
from RobotSubSystems.StabilityManager.StabilityManager import StabilityManager
from RobotSubSystems.UserInput.UserInput import UserInput
from Utilities.RobotUtils.RobotUtils import RobotUtils
from Utilities.Vector.Vector import Vector

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
        self.gravity_paused = True


    def start(self):

        world = WorldModel()

        res = world.readFile(self.robot_file)
        if not res:
            raise RuntimeError("Unable to load model")

        if RobotUtils.INCLUDE_TERRAIN:
            terrain = world.readFile(self.world_file)
            if not terrain:
                raise RuntimeError("Unable to load plane")


        self.planning_world = world.copy()

        if RobotUtils.PHYSICS_ENABLED:
            self.initialize_visualization(world)
        else:
            self.initialize_visualization(self.planning_world)

        self.robosimian = self.planning_world.robot(0)
        self.sim =  Simulator(world)
        self.sim.setGravity([0,0,0])

        # Create robot controller
        self.controller = self.sim.controller(0)
        self.controller.setRate(RobotUtils.CONTROLLER_DT)

        self.StabilityManager = StabilityManager(world, self.sim, RobotUtils)

        # Create HighLevelMotionController
        self.HighLevelMotionController = HighLevelMotionController(self.robosimian, RobotUtils, self.controller)

        # Create HighLevelMotionController
        self.MotionPlanner = MotionPlanner(self.robosimian, RobotUtils, Vector, _2DSupportPolygon, _2DLegRadius)

        # Pass the Motion Controller the MotionPlanner
        self.HighLevelMotionController.initialize_motion_planner(self.MotionPlanner)

        # User Input
        self.UserInput = UserInput(RobotUtils)

        # Objective Manager
        self.ObjectiveManager = ObjectiveManager(RobotUtils, self.MotionPlanner, self.HighLevelMotionController, self.UserInput )

        RobotUtils.ColorPrinter(self.__class__.__name__,"Hypervisor initialization finished","OKBLUE")

        running = True

        try:

            if running:
                if not RobotUtils.PHYSICS_ENABLED:

                    # TODO: set_initial_config() called twice.

                    self.HighLevelMotionController.set_inital_config()
                    self.MotionPlanner.save_base_states()

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

            # Update model
            vis.lock()

            if RobotUtils.PHYSICS_ENABLED:

                if self.gravity_paused:
                    if self.HighLevelMotionController.initialization_complete:
                        self.sim.setGravity([0, 0, -9.8])
                        self.gravity_paused = False
                        self.MotionPlanner.save_base_states()

                self.HighLevelMotionController.control_loop()
                self.sim.updateWorld()
                self.sim.simulate(RobotUtils.CONTROLLER_DT)
                self.StabilityManager.check_status()

            vis.unlock()

            time.sleep(RobotUtils.CONTROLLER_DT)
            if not vis.shown():
                sys.exit()



    def vis_test(self):

        name = "test"
        obj = [1,1,1]

        traj_name = "test circle"
        circle = self.get_trajectory_circle([.5,.5,.5],.5)

        vis.add(traj_name, circle)

        vis.add(name, obj)


    def get_trajectory_circle(self, xyz, r ):

        x0 = xyz[0]
        y0 = xyz[1]
        z0 = xyz[2]

        milestones = []

        for i in range(361):

            x = x0 + np.cos(np.radians(i))
            y = y0 + np.sin(np.radians(i))

            milestones.append([x,y,z0])

        return trajectory.Trajectory(milestones=milestones)




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

        d_y = end_aff_xyz[1] - shoulder_xyz[1]

        print "Shoulder xyz:", shoulder_xyz
        print "end_aff_xyz:", end_aff_xyz
        print "dy: ", d_y