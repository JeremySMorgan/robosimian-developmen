import time
import pygame
from pygame.locals import *
from ...Utilities.MotionThread.MotionThread import MotionThread
from ...Utilities.RobotUtils.RobotConstants import RobotConstants
from ...Utilities.Logging.Logger import Logger

class UserInput(object):

    def __init__(self, RobotConstants):

        self.RobotConstants = RobotConstants
        self.update_delay = self.RobotConstants.GAMEPAD_UPDATE_DELAY
        self.gamepad = None
        self.desired_robot_state = self.RobotConstants.BASE_STATE
        self.saved_robot_state_desired = self.desired_robot_state
        self.gampad_disconnected_status_printed  = False
        self.print_config = False

        self.r_x = 0
        self.r_y = 0
        self.l_x = 0
        self.l_y = 0

    def start(self):

        pygame.init()
        if pygame.joystick.get_count() == 1:
            self.gamepad = pygame.joystick.Joystick(0)
            self.gamepad.init()

        self.update_loop_thread = MotionThread(self.update_loop,"update_loop",pass_motion_thread=True)

    def shutdown(self):
        self.update_loop_thread.__setattr__(self.RobotConstants.KEEP_THREAD_ALIVE, False)
        Logger.log(self.__class__.__name__, "Suspending UserInput thread", "FAIL")

    def get_desired_robot_state(self):
        return self.desired_robot_state

    def get_print_config(self):
        if self.print_config:
            self.print_config = False
            return True
        return False

    def update_desired_direction(self):

        # Print Robot Config when joystick button is pressed
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                self.print_config = True


        if not self.gamepad is None:

            if pygame.joystick.get_count() == 1:

                pygame.event.pump()
                r_x = self.gamepad.get_axis(2)
                r_y = self.gamepad.get_axis(3)
                self.gampad_disconnected_status_printed = False

                # Left/ right desired
                if abs(r_x) > self.RobotConstants.GAMEPAD_LEAST_SIGNIFICANT_INPUT:

                    # left/ right AND forward/back desired
                    if abs(r_y) > self.RobotConstants.GAMEPAD_LEAST_SIGNIFICANT_INPUT:

                        # go forward/backward
                        if abs(r_y) > abs(r_x):
                            # Forward
                            if r_y < 0:
                                self.desired_robot_state = self.RobotConstants.FORWARD

                            # backward
                            else:
                                self.desired_robot_state = self.RobotConstants.BACKWARD

                        # Go left / right
                        else:

                            if r_x > 0:
                                self.desired_robot_state = self.RobotConstants.RIGHT

                            else:
                                self.desired_robot_state = self.RobotConstants.LEFT

                    # Either left of right
                    else:
                        if r_x > 0:
                            self.desired_robot_state = self.RobotConstants.RIGHT

                        else:
                            self.desired_robot_state = self.RobotConstants.LEFT

                else:

                    # Forward or backward desired
                    if abs(r_y) > self.RobotConstants.GAMEPAD_LEAST_SIGNIFICANT_INPUT:

                        # Forward
                        if r_y < 0:
                            self.desired_robot_state = self.RobotConstants.FORWARD

                        # backward
                        else:
                            self.desired_robot_state = self.RobotConstants.BACKWARD

                    # Base state desired
                    else:
                        self.desired_robot_state = self.RobotConstants.BASE_STATE

            # Prevent unallowed states
            if self.desired_robot_state not in self.RobotConstants.allowed_states:
                self.desired_robot_state = self.RobotConstants.BASE_STATE

        else:

            if pygame.joystick.get_count() == 1:
                self.gamepad = pygame.joystick.Joystick(0)
                self.gamepad.init()
                if self.RobotConstants.USER_INPUT_DEBUGGING_ENABLED:
                    Logger.log(self.__class__.__name__, "Gamepad connected", "OKGREEN")

            else:
                self.set_desired_state_from_keyboard()
                if self.RobotConstants.USER_INPUT_DEBUGGING_ENABLED:
                    if not self.gampad_disconnected_status_printed:
                        Logger.log(self.__class__.__name__, "Gamepad disconnected", "FAIL")
                        self.gampad_disconnected_status_printed = True



    def set_desired_state_from_keyboard(self):

        events = pygame.event.get()

        # Accounts for the possibility that user is pressing multiple jeys down at once.
        desired_states = []

        for event in events:
            if event.type == pygame.KEYDOWN:

                if event.key == pygame.K_LEFT:
                    desired_states.append(self.RobotConstants.LEFT)

                if event.key == pygame.K_RIGHT:
                    desired_states.append(self.RobotConstants.RIGHT)

                if event.key == pygame.K_UP:
                    desired_states.append(self.RobotConstants.FORWARD)

                if event.key == pygame.K_DOWN:
                    desired_states.append(self.RobotConstants.BACKWARD)

        # if the current state is in the list of pressed key states, give preference to this state, i.e. continue
        if self.desired_robot_state in desired_states:
            pass

        else:

            # If there are keys pressed
            if len(desired_states) > 0:
                self.desired_robot_state = desired_states[0]

            else:
                self.desired_robot_state = self.RobotConstants.BASE_STATE

        if self.RobotConstants.USER_INPUT_DEBUGGING_ENABLED:
            if not self.gampad_disconnected_status_printed:
                Logger.log(self.__class__.__name__, "Gamepad disconnected", "FAIL")
                self.gampad_disconnected_status_printed = True



    def print_debug(self):

        if self.RobotConstants.USER_INPUT_DEBUGGING_ENABLED:

            if not self.desired_robot_state == self.saved_robot_state_desired:

                status = "New desired robot state: " + self.desired_robot_state
                Logger.log(self.__class__.__name__, status, "STANDARD")

                self.saved_robot_state_desired = self.desired_robot_state




    def update_loop(self, t):

        while 1:
            if t.is_alive():
                time.sleep(self.update_delay)
                self.update_desired_direction()
                self.print_debug()

            else:
                self.shutdown()
                break
