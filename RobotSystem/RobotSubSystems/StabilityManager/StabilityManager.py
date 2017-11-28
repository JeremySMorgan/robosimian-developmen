from klampt.model.collide import WorldCollider
from ...Utilities.Logging.Logger import Logger

class StabilityManager(object):

    def __init__(self, world, sim):

        self.world = world
        self.robosimian = world.robot(0)
        self.sim = sim
        self.vel_limits = self.robosimian.getVelocityLimits()
        self.torque_limits = self.robosimian.getTorqueLimits()
        self.ColliisonCalculator = WorldCollider(world)


    def check_status(self):
        self.check_torque()
        self.check_v()
        self.check_collisions()


    def check_torque(self):

        # torque_limits - [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]

        torques = self.sim.getActualTorques(0)
        print_t = False
        t_print_str = "Error: current torque for: "
        for i in range(len(torques)):
            torq_m = self.torque_limits[i]

            torq_c = torques[i] if torques[i] > 0 else (-1) * torques[i]

            link_name = self.robosimian.link(i).getName()

            if torq_c > torq_m:
                if i > 5:
                    t_print_str += link_name + ": " + Logger.pp_double(torq_c) + "> max: " + str(torq_m) + "\t"
                    print_t = True

        if print_t:
            print(t_print_str)

    def check_collisions(self):

        collisions = self.ColliisonCalculator.robotSelfCollisions()

        for collision in collisions:
            link_1_name = collision[0].getName()
            link_2_name = collision[1].getName()
            error_str = "Collision detected between"+link_1_name+" and"+link_2_name
            Logger.log((self.__class__.__name__+".check_collisions()"),error_str,"FAIL" )



    def check_v(self):

        # vel_limits - [inf, inf, inf, inf, inf, inf, 0.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]

        vel = self.sim.getActualVelocity(0)
        print_v = False
        v_print_str = "Error: current V for "
        for i in range(len(vel)):

            vel_m = 1000000 if type(self.vel_limits[i]) == str else self.vel_limits[i]
            vel_c = vel[i] if vel[i] > 0 else (-1) * vel[i]
            link_name = self.robosimian.link(i).getName()
            if vel_c > vel_m:
                v_print_str += link_name + ": " + Logger.pp_double(vel_c) + ", max: " + str( vel_m) + "\t| "
                print_v = True

        if print_v:
            print(v_print_str)