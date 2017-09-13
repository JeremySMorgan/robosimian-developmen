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

        # Add visualizations items for debugging
        self.addVisualizationItems()

        # Save a copy of the robot locally
        self.robosimian = world.robot(0)

        # Create MotionController
        self.MotionController = MotionController(self.robosimian, RobotUtils)
        self.MotionController.setInitalConfig()

        # Create MotionPlanner
        self.MotionPlanner = MotionPlanner(self.robosimian,RobotUtils)
        self.MotionPlanner.save_base_foot_states()

        for leg in RobotUtils.end_affectors:
            for direction in [RobotUtils.FORWARD, RobotUtils.BACKWARD]:
                name = leg + ":"+direction
                res = self.MotionPlanner.get_extended_foot_local_XYZ(leg, direction)
                vis.add(name, res)

        RobotUtils.ColorPrinter(self.__class__.__name__,"Hypervisor initialization finished","OKBLUE")


    def addVisualizationItems(self):
        points = [[1,0,0],[0,1,0]]
        for point in points:
            label = "testpoint at",point
            vis.add(str(label),point)

    def initialize_visualization(self,world):
        vis.add("world", world)
        vis.add("coordinates", coordinates.manager())
        vp = vis.getViewport()
        vp.w, vp.h = 1600, 1600
        vis.setViewport(vp)
        #vis.listItems(indent=4)
        vis.autoFitCamera()
        vis.show()



    def run_visualization(self):

        i_max = 250

        foot_name = RobotUtils.F_R_FOOT
        direction = RobotUtils.FORWARD
        start_xyz = self.MotionPlanner.f_r_foot_base_state
        end_xyz = self.MotionPlanner.get_extended_foot_local_XYZ( foot_name, direction)
        link = self.MotionPlanner.get_foot_from_foot_name(foot_name)
        active_dofs = RobotUtils.f_r_active_dofs
        done = False

        while RobotUtils.SIMULATION_ENABLED:

            if not done:
                for i in range(i_max):

                    # Update model
                    vis.lock()

                    xyz_des = self.MotionPlanner.get_mid_step_local_XYZ(foot_name, direction, start_xyz, end_xyz, i, i_max)
                    ik_local = [0, 0, 0]
                    goal = ik.objective(link, local=ik_local, world=xyz_des)
                    ik.solve(goal, activeDofs=active_dofs)
                    vis.unlock()


                    time.sleep(RobotUtils.SIMULATION_FRAME_DELAY)
                    if not vis.shown():
                        sys.exit()
                done = True


        vis.lock()
        vis.unlock()

        time.sleep(RobotUtils.SIMULATION_FRAME_DELAY)
        if not vis.shown():
            sys.exit()


'''
xyz_des = getXYZfromPath(i, i_max, f_r_foot_world_pos, target_destination)
ik_local = [0, 0, 0]
goal = ik.objective(f_r_foot, local=ik_local, world=xyz_des)
ik.solve(goal, activeDofs=f_r_active_dofs)
i += 1
'''