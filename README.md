# robosimian-development

## About
Walking gait for JPL's Robosimian quadruped robot. Uses Klamp't (http://motion.pratt.duke.edu/klampt/) for simulation, ik, and modeling.

## Media

#### video
www.youtube.com/watch?v=yeAvKYfvZMc

#### Image
Image of the simulated robot in its start configuration
<img src="https://github.com/JeremySMorgan/robosimian-gait-development/blob/master/robosimian.png" width="600" height="600" />








### Unfinished objective manager (not working)
```python

import threading
import time

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
        self.robot_motion_thread        = None

        # Debugging
        self.printed_current_state = False


    def start_objective_management_loop(self):

        # Start input loop
        self.obj_manager_update_loop_thread = threading.Thread(target=self.obj_manager_update_loop)
        self.obj_manager_update_loop_thread.__setattr__( self.RobotUtils.KEEP_THREAD_ALIVE, True)
        self.obj_manager_update_loop_thread.start()



    # 1: Read desired robot activity (turning/ going forward/back setting to base state)
    # 2: if CURRENTLY DOING the desired activity, keep doing it
    #    if CURRENTLY DOING a DIFFERENT activity, stop it and begin desired activity
    def update_agenda(self):

        desired_state = self.UserInput.get_desired_robot_state()

        # Currently doing desired state
        if self.current_robot_state == desired_state:

            if not self.printed_current_state:
                if self.RobotUtils.OBJ_PLANNER_DEBUGGING_ENABLED:
                    status = "Current robot state: "+desired_state+", equals desired robot state"
                    self.RobotUtils.ColorPrinter(self.__class__.__name__, status, "STANDARD")
                    self.printed_current_state = True
        else:

            self.printed_current_state = False

            status = "updating desired state and starting new motion thread"
            self.RobotUtils.ColorPrinter(self.__class__.__name__, status, "WARNING")

            # Cancel current movement
            if not self.robot_motion_thread is None:
                status = "Suspending previous motion thread"
                self.RobotUtils.ColorPrinter(self.__class__.__name__, status, "WARNING")
                self.robot_motion_thread.__setattr__( self.RobotUtils.KEEP_THREAD_ALIVE, False)


            # Cancel current state by returning to base state
            self.HighLevelMotionController.reset_to_base_state()

            # In base state - done
            if desired_state == self.RobotUtils.BASE_STATE:
                self.current_robot_state = self.RobotUtils.BASE_STATE

            # Else start new motion
            elif desired_state == self.RobotUtils.LEFT:

                status = "starting 'make_left_turn' thread"
                self.RobotUtils.ColorPrinter(self.__class__.__name__, status, "WARNING")

                self.robot_motion_thread = None
                self.robot_motion_thread = threading.Thread( target = self.HighLevelMotionController.multi_threading_tester, name="multi_threading_tester")
                self.robot_motion_thread.__setattr__(self.RobotUtils.KEEP_THREAD_ALIVE, True)

                self.robot_motion_thread.start()

                self.current_robot_state = self.RobotUtils.LEFT


            else:
                self.RobotUtils.ColorPrinter(self.__class__.__name__, "Update agenda error: desired robot state unrecognized","FAIL")


            print "TESTSETSETETEST"
            print "TESTSETSETETEST"
            print "TESTSETSETETEST"
            print "TESTSETSETETEST"
            print "TESTSETSETETEST"



    def obj_manager_update_loop(self):

        t = threading.currentThread()

        while 1:
            if getattr(t, self.RobotUtils.KEEP_THREAD_ALIVE, True):
                time.sleep(self.RobotUtils.OBJECTIVE_PLANNER_UPDATE_DELAY)
                self.update_agenda()
            else:
                self.RobotUtils.ColorPrinter(self.__class__.__name__, "Suspending ObjectiveManager update loop thread",
                                             "FAIL")


    def shutdown(self):
        self.obj_manager_update_loop.__setattr__( self.RobotUtils.KEEP_THREAD_ALIVE, False)




backward_step



```