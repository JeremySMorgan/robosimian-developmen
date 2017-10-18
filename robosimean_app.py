#!/usr/bin/env python

from RobotSystem.Hypervisor import Hypervisor

def main():
    robot_file = "../rockclimber/sim/robot/robosimian_caesar_new.urdf"
    terrain_file = "./Resources/terrains/plane.env"

    robot_hypervisor = Hypervisor(robot_file,robot_file)
    robot_hypervisor.start()
    robot_hypervisor.run_visualization()

if __name__ == "__main__":
    main()


