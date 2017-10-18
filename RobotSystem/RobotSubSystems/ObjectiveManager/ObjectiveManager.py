import threading
import time

from ...Utilities.MotionThread.MotionThread import MotionThread

class ObjectiveManager(object):


    # RobotUtils, self.MotionPlanner, self.HighLevelMotionController, self.UserInput )
    def __init__(self, RobotUtils, MotionPlanner, HighLevelMotionController, UserInput):

        # Robot Systems
        self.RobotUtils                     = RobotUtils
        self.MotionPlanner                  = MotionPlanner
        self.HighLevelMotionController      = HighLevelMotionController
        self.UserInput                      = UserInput

        # robot state
        self.current_robot_state            = RobotUtils.BASE_STATE

        # Threading
        self.robot_motion_thread            = None
        self.obj_manager_update_loop_thread = None

        # Debugging
        self.printed_current_state = False


    # Start input loop
    def start_objective_management_loop(self):

        self.obj_manager_update_loop_thread = MotionThread( self.obj_manager_update_loop, "obj_manager_update_loop")


    # 1: Read desired robot activity (turning/ going forward/back setting to base state)
    # 2: if CURRENTLY DOING the desired activity, keep doing it
    #    if CURRENTLY DOING a DIFFERENT activity, stop it and begin desired activity
    def update_agenda(self):

        desired_state = self.UserInput.get_desired_robot_state()

        # Currently doing desired state
        if self.current_robot_state == desired_state:
            pass

        else:

            self.printed_current_state = False

            # Cancel current movement
            if not self.robot_motion_thread is None:

                if self.RobotUtils.OBJ_PLANNER_DEBUGGING_ENABLED:
                    status = "Suspending " + self.robot_motion_thread.get_name() + " thread"
                    self.RobotUtils.ColorPrinter( (self.__class__.__name__+".update_agenda()" ), status, "FAIL")

                self.robot_motion_thread.shutdown()


            # In base state - done
            if desired_state == self.RobotUtils.BASE_STATE:
                self.current_robot_state = self.RobotUtils.BASE_STATE


            # Else start new motion
            elif desired_state == self.RobotUtils.LEFT:

                if self.RobotUtils.OBJ_PLANNER_DEBUGGING_ENABLED:
                    status = "starting 'make_left_turn' thread"
                    self.RobotUtils.ColorPrinter((self.__class__.__name__+".update_agenda()" ), status, "OKGREEN")

                self.robot_motion_thread = MotionThread(self.HighLevelMotionController.make_left_turn, "left turn", pass_motion_thread=True)
                self.current_robot_state = self.RobotUtils.LEFT


            elif desired_state == self.RobotUtils.RIGHT:

                if self.RobotUtils.OBJ_PLANNER_DEBUGGING_ENABLED:
                    status = "Starting right turn thread"
                    self.RobotUtils.ColorPrinter((self.__class__.__name__+".update_agenda()" ), status, "OKGREEN")

                self.current_robot_state = self.RobotUtils.RIGHT
                self.robot_motion_thread = MotionThread(self.HighLevelMotionController.make_right_turn, "right turn",pass_motion_thread=True)


            elif desired_state == self.RobotUtils.FORWARD:

                if self.RobotUtils.OBJ_PLANNER_DEBUGGING_ENABLED:
                    status = "Starting forward walk thread"
                    self.RobotUtils.ColorPrinter((self.__class__.__name__+".update_agenda()" ), status, "OKGREEN")

                self.current_robot_state = self.RobotUtils.FORWARD

                self.robot_motion_thread = MotionThread(self.HighLevelMotionController.forward_walk,"forward walk",pass_motion_thread=True)


            elif desired_state == self.RobotUtils.BACKWARD:

                if self.RobotUtils.OBJ_PLANNER_DEBUGGING_ENABLED:
                    status = "Starting backward walk thread"
                    self.RobotUtils.ColorPrinter((self.__class__.__name__+".update_agenda()" ), status, "OKGREEN")

                self.current_robot_state = self.RobotUtils.BACKWARD

                self.robot_motion_thread = MotionThread(self.HighLevelMotionController.backward_walk,"backward walk",pass_motion_thread=True)

            else:
                self.RobotUtils.ColorPrinter((self.__class__.__name__+".update_agenda()" ), "Update agenda error: desired robot state unrecognized","FAIL")



    def obj_manager_update_loop(self):

        while 1:
            if not self.obj_manager_update_loop_thread is None:
                if self.obj_manager_update_loop_thread.is_alive():
                    time.sleep(self.RobotUtils.OBJECTIVE_PLANNER_UPDATE_DELAY)
                    self.update_agenda()
                else:
                    break


    def shutdown(self):

        self.obj_manager_update_loop_thread.shutdown()

        status = "Suspended the Objective Management"

        if not self.robot_motion_thread is None:
            self.robot_motion_thread.shutdown()
            status += "and Motion"

        status += " thread"
        self.RobotUtils.ColorPrinter(self.__class__.__name__, status, "FAIL")


