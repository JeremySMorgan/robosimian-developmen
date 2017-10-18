import time
import pygame
import threading

class UserInput(object):

    def __init__(self, RobotUtils):

        self.RobotUtils = RobotUtils

        self.update_delay = RobotUtils.GAMEPAD_UPDATE_DELAY

        self.gamepad = None

        self.desired_robot_state = RobotUtils.BASE_STATE

        self.saved_robot_state_desired = self.desired_robot_state
        self.gampad_disconnected_status_printed  = False

        self.r_x = 0
        self.r_y = 0
        self.l_x = 0
        self.l_y = 0

    def start(self):

        # Get Controller
        pygame.init()

        if pygame.joystick.get_count() == 1:
            self.gamepad = pygame.joystick.Joystick(0)
            self.gamepad.init()

        # Start input loop
        self.update_loop_thread = threading.Thread(target=self.update_loop)
        self.update_loop_thread.__setattr__( self.RobotUtils.KEEP_THREAD_ALIVE, True)
        self.update_loop_thread.start()


    def shutdown(self):
        self.update_loop_thread.__setattr__( self.RobotUtils.KEEP_THREAD_ALIVE, False)

        self.RobotUtils.ColorPrinter(self.__class__.__name__, "Suspending UserInput thread", "FAIL")


    def get_desired_robot_state(self):
        return self.desired_robot_state


    def update_desired_direction(self):

        if not self.gamepad is None:


            if pygame.joystick.get_count() == 1:

                pygame.event.pump()

                r_x = self.gamepad.get_axis(2)
                r_y = self.gamepad.get_axis(3)

                self.gampad_disconnected_status_printed = False

                # Left/ right desired
                if abs(r_x) > self.RobotUtils.GAMEPAD_LEAST_SIGNIFICANT_INPUT:

                    # left/ right AND forward/back desired
                    if abs(r_y) > self.RobotUtils.GAMEPAD_LEAST_SIGNIFICANT_INPUT:

                        # go forward/backward
                        if abs(r_y) > abs(r_x):
                            # Forward
                            if r_y < 0:
                                self.desired_robot_state = self.RobotUtils.FORWARD

                            # backward
                            else:
                                self.desired_robot_state = self.RobotUtils.BACKWARD

                        # Go left / right
                        else:

                            if r_x > 0:
                                self.desired_robot_state = self.RobotUtils.RIGHT

                            else:
                                self.desired_robot_state = self.RobotUtils.LEFT

                    # Either left of right
                    else:
                        if r_x > 0:
                            self.desired_robot_state = self.RobotUtils.RIGHT

                        else:
                            self.desired_robot_state = self.RobotUtils.LEFT

                else:

                    # Forward or backward desired
                    if abs(r_y) > self.RobotUtils.GAMEPAD_LEAST_SIGNIFICANT_INPUT:

                        # Forward
                        if r_y < 0:
                            self.desired_robot_state = self.RobotUtils.FORWARD

                        # backward
                        else:
                            self.desired_robot_state = self.RobotUtils.BACKWARD

                    # Base state desired
                    else:
                        self.desired_robot_state = self.RobotUtils.BASE_STATE


        else:

            if pygame.joystick.get_count() == 1:
                self.gamepad = pygame.joystick.Joystick(0)
                self.gamepad.init()

                if self.RobotUtils.GAMEPAD_DEBUGGING_ENABLED:
                    self.RobotUtils.ColorPrinter(self.__class__.__name__, "Gamepad connected", "OKGREEN")

            else:

                self.set_desired_state_from_keyboard()

                if self.RobotUtils.GAMEPAD_DEBUGGING_ENABLED:
                    if not self.gampad_disconnected_status_printed:
                        self.RobotUtils.ColorPrinter(self.__class__.__name__, "Gamepad disconnected", "FAIL")
                        self.gampad_disconnected_status_printed = True



    def set_desired_state_from_keyboard(self):

        #print "in setset_desired_state_from_keyboard"

        events = pygame.event.get()

        # Accounts for the possibility that user is pressing multiple jeys down at once.
        desired_states = []

        for event in events:
            if event.type == pygame.KEYDOWN:

                if event.key == pygame.K_LEFT:
                    print "Left pressed"
                    desired_states.append(self.RobotUtils.LEFT)

                if event.key == pygame.K_RIGHT:
                    desired_states.append(self.RobotUtils.RIGHT)

                if event.key == pygame.K_UP:
                    desired_states.append(self.RobotUtils.FORWARD)

                if event.key == pygame.K_DOWN:
                    desired_states.append(self.RobotUtils.BACKWARD)

        print len(desired_states)

        # if the current state is in the list of pressed key states, give preference to this state, i.e. continue
        if self.desired_robot_state in desired_states:
            pass

        else:

            # If there are keys pressed
            if len(desired_states) > 0:
                self.desired_robot_state = desired_states[0]

            else:
                self.desired_robot_state = self.RobotUtils.BASE_STATE

        if self.RobotUtils.GAMEPAD_DEBUGGING_ENABLED:
            if not self.gampad_disconnected_status_printed:
                self.RobotUtils.ColorPrinter(self.__class__.__name__, "Gamepad disconnected", "FAIL")
                self.gampad_disconnected_status_printed = True



    def print_debug(self):

        if self.RobotUtils.GAMEPAD_DEBUGGING_ENABLED:

            if not self.desired_robot_state == self.saved_robot_state_desired:

                status = "New desired robot state: " + self.desired_robot_state
                self.RobotUtils.ColorPrinter(self.__class__.__name__, status, "STANDARD")

                self.saved_robot_state_desired = self.desired_robot_state




    def update_loop(self):

        t = threading.currentThread()

        while 1:

            if getattr(t, self.RobotUtils.KEEP_THREAD_ALIVE, True):
                time.sleep(self.update_delay)
                self.update_desired_direction()
                self.print_debug()
            else:
                self.shutdown()
                break
