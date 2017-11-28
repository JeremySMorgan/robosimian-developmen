import threading
import time
from ...Utilities.MotionThread.MotionThread import MotionThread
from ...Utilities.Logging.Logger import Logger

class ObjectiveManager(object):

    # RobotConstants, self.MotionPlanner, self.HighLevelMotionController, self.UserInput )
    def __init__(self, MotionPlanner, HighLevelMotionController, UserInput, RobotConstants):

        # Robot Systems
        self.MotionPlanner                  = MotionPlanner
        self.HighLevelMotionController      = HighLevelMotionController
        self.UserInput                      = UserInput
        self.RobotConstants                 = RobotConstants

        # robot state
        self.current_robot_state            = self.RobotConstants.BASE_STATE

        # Threading
        self.robot_motion_thread            = None
        self.obj_manager_update_loop_thread = None
        self.initializatoin_thread          = None

        # Debugging
        self.printed_current_state = False

    # Start input loop
    def start(self):

        self.initializatoin_thread = MotionThread( self.HighLevelMotionController.set_inital_config, "obj_manager_update_loop")
        self.obj_manager_update_loop_thread = MotionThread( self.obj_manager_update_loop, "obj_manager_update_loop")


    # 1: Read desired robot activity (turning/ going forward/back setting to base state)
    # 2: if CURRENTLY DOING the desired activity, keep doing it
    #    if CURRENTLY DOING a DIFFERENT activity, stop it and begin desired activity
    def update_agenda(self):

        desired_state = self.UserInput.get_desired_robot_state()
        if self.UserInput.get_print_config():
            self.MotionPlanner.print_config()

        # Currently doing desired state
        if self.current_robot_state == desired_state:
            pass

        else:

            self.printed_current_state = False

            # Cancel current movement
            if not self.robot_motion_thread is None:

                if self.RobotConstants.OBJ_PLANNER_DEBUGGING_ENABLED:
                    status = "Suspending " + self.robot_motion_thread.get_name() + " thread"
                    Logger.log( (self.__class__.__name__+".update_agenda()" ), status, "FAIL")

                self.robot_motion_thread.shutdown()


            # In base state - done
            if desired_state == self.RobotConstants.BASE_STATE:
                self.current_robot_state = self.RobotConstants.BASE_STATE


            # Else start new motion
            elif desired_state == self.RobotConstants.LEFT:

                if self.RobotConstants.OBJ_PLANNER_DEBUGGING_ENABLED:
                    status = "starting 'make_left_turn' thread"
                    Logger.log((self.__class__.__name__+".update_agenda()" ), status, "OKGREEN")

                self.robot_motion_thread = MotionThread(self.HighLevelMotionController.make_turn, "left turn", pass_motion_thread=True, arg=self.RobotConstants.LEFT)
                self.current_robot_state = self.RobotConstants.LEFT


            elif desired_state == self.RobotConstants.RIGHT:

                if self.RobotConstants.OBJ_PLANNER_DEBUGGING_ENABLED:
                    status = "Starting right turn thread"
                    Logger.log((self.__class__.__name__+".update_agenda()" ), status, "OKGREEN")

                self.current_robot_state = self.RobotConstants.RIGHT
                self.robot_motion_thread = MotionThread(self.HighLevelMotionController.make_turn, "right turn",pass_motion_thread=True, arg=self.RobotConstants.RIGHT)


            elif desired_state == self.RobotConstants.FORWARD:

                if self.RobotConstants.OBJ_PLANNER_DEBUGGING_ENABLED:
                    status = "Starting forward walk thread"
                    Logger.log((self.__class__.__name__+".update_agenda()" ), status, "OKGREEN")

                self.current_robot_state = self.RobotConstants.FORWARD

                self.robot_motion_thread = MotionThread(self.HighLevelMotionController.forward_walk,"forward walk",pass_motion_thread=True)


            elif desired_state == self.RobotConstants.BACKWARD:

                if self.RobotConstants.OBJ_PLANNER_DEBUGGING_ENABLED:
                    status = "Starting backward walk thread"
                    Logger.log((self.__class__.__name__+".update_agenda()" ), status, "OKGREEN")

                self.current_robot_state = self.RobotConstants.BACKWARD

                self.robot_motion_thread = MotionThread(self.HighLevelMotionController.backward_walk,"backward walk",pass_motion_thread=True)

            else:
                Logger.log((self.__class__.__name__+".update_agenda()" ), "Update agenda error: desired robot state unrecognized","FAIL")



    def obj_manager_update_loop(self):

        while 1:
            if not self.obj_manager_update_loop_thread is None:
                if self.obj_manager_update_loop_thread.is_alive():
                    time.sleep(self.RobotConstants.OBJECTIVE_PLANNER_UPDATE_DELAY)
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
        Logger.log(self.__class__.__name__, status, "FAIL")


