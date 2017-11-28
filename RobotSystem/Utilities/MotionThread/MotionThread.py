import threading

class MotionThread(object):

    keep_alive_word = "keep_alive"

    def __init__(self, function_name, thread_name, pass_motion_thread=False, arg=None):


        self.thread = None
        self.arg = arg
        self.function_name = function_name
        self.thread_name = thread_name
        self.pass_motion_thread = pass_motion_thread
        self.start()


    def start(self):

        # pass arguments
        if self.arg:

            # Pass thread target function argument and self. Note that Motion Thread is passed as the last parameter
            if self.pass_motion_thread:
                self.thread = threading.Thread(target=self.function_name, name=self.thread_name, args=(self.arg, self,), )

            # dont pass any arguments
            else:
                self.thread = threading.Thread(target=self.function_name, name=self.thread_name)


        # No arguments
        else:

            # Pass thread target function self
            if self.pass_motion_thread:
                self.thread = threading.Thread(target=self.function_name, name=self.thread_name, args=(self,), )

            # dont pass any arguments
            else:
                self.thread = threading.Thread(target=self.function_name, name=self.thread_name)

        self.thread.__setattr__(MotionThread.keep_alive_word, True)
        self.thread.start()


    def is_alive(self):

        return getattr(self.thread, MotionThread.keep_alive_word, True)


    def shutdown(self):

        self.thread.__setattr__(MotionThread.keep_alive_word, False)


    def get_name(self):

        return self.thread.name