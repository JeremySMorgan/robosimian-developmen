
class StabilityManager(object):

    def __init__(self, robot, sim, RobotUtils):
        self.robosimian = robot
        self.sim = sim
        self.RobotUtils = RobotUtils
        self.vel_limits = self.robosimian.getVelocityLimits()
        self.torque_limits = self.robosimian.getTorqueLimits()




    def check_status(self):

        torques = self.sim.getActualTorques(0)
        vel = self.sim.getActualVelocity(0)

        # vel_limits - [inf, inf, inf, inf, inf, inf, 0.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        # torque_limits - [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]


        print_v = False
        print_t = False

        v_print_str = "Error: current velocity for: "
        t_print_str = "Error: current torque for: "

        for i in range(len(torques)):

            vel_m = 1000000 if type(self.vel_limits[i]) == str else self.vel_limits[i]
            torq_m = self.torque_limits[i]

            vel_c = vel[i] if vel[i] > 0 else (-1) * vel[i]
            torq_c = torques[i] if torques[i] > 0 else (-1) * torques[i]

            link_name = self.robosimian.link(i).getName()

            if vel_c > vel_m:
                v_print_str += link_name + ": " + str(vel_c) + "> max: " + str( vel_m) + "\t\t | "
                print_v = True

            if torq_c > torq_m:
                if i > 5:
                    t_print_str += link_name + ": " + str(torq_c) + "> max: " + str( torq_m) + "\t\t"
                    print_t = True


        if print_t:
            print(t_print_str)
        if print_v:
            print(v_print_str)

        if print_v or print_t:
            print()