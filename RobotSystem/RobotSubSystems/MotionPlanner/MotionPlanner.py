#!/usr/bin/python

class MotionPlanner():

    def __init__(self,robot,RobotUtils):
        self.robosimian = robot
        self.RobotUtils = RobotUtils

        self.initialize_feet()
        self.initialize_shoulders()



    def initialize_shoulders(self):

        self.f_r_shoulder = self.robosimian.link(self.RobotUtils.f_r_active_dofs[0])
        self.f_l_shoulder = self.robosimian.link(self.RobotUtils.f_l_active_dofs[0])
        self.b_r_shoulder = self.robosimian.link(self.RobotUtils.b_r_active_dofs[0])
        self.b_l_shoulder = self.robosimian.link(self.RobotUtils.b_l_active_dofs[0])




    def initialize_feet(self):

        self.f_r_foot = self.robosimian.link(self.RobotUtils.f_r_active_dofs[len(self.RobotUtils.f_r_active_dofs) - 1])
        self.f_l_foot = self.robosimian.link(self.RobotUtils.f_l_active_dofs[len(self.RobotUtils.f_l_active_dofs) - 1])
        self.b_r_foot = self.robosimian.link(self.RobotUtils.b_r_active_dofs[len(self.RobotUtils.b_r_active_dofs) - 1])
        self.b_l_foot = self.robosimian.link(self.RobotUtils.b_l_active_dofs[len(self.RobotUtils.b_l_active_dofs) - 1])


    def get_mid_step_local_XYZ(self, foot_name, direction, startXYZ, endXYZ, i, i_max):

        # prevents division by 0
        if i == 0: i = 1

        foot = self.get_foot_from_foot_name(foot_name)

        # Add x offset (dependent on direction)
        if direction == self.RobotUtils.FORWARD:
            direction_coeff = 1
        else:
            direction_coeff = -1

        x_start = startXYZ[0]
        z_start = startXYZ[2]
        x_end = endXYZ[0]

        step_length = x_end - x_start

        x = direction_coeff * float(i)/float(i_max) * step_length
        h = self.RobotUtils.STEP_Z_MAX_HIEGHT
        b = self.RobotUtils.STEP_X_DELTA

        # see https://www.desmos.com/calculator/v8wb6o83jh
        z_offset =  ((-4*h)/(b**2)) * x * (x-b)

        res = [ x_start + x, startXYZ[1], z_start + z_offset ]

        return res


    # Saves the foot positions when the robot is located at the origin
    def save_base_foot_states(self):

        self.f_r_foot_base_state = self.f_r_foot.getWorldPosition([0,0,0])
        self.f_l_foot_base_state = self.f_l_foot.getWorldPosition([0,0,0])
        self.b_r_foot_base_state = self.b_r_foot.getWorldPosition([0,0,0])
        self.b_l_foot_base_state = self.b_l_foot.getWorldPosition([0,0,0])


    def get_extended_foot_local_XYZ(self,foot_name,direction):

        base_foot_state = self.get_foot_base_state_from_foot_name(foot_name)

        # Make a copy of the foot state
        xyz = [base_foot_state[0], base_foot_state[1], base_foot_state[2]]

        # Add x offset (dependent on direction)
        if direction == self.RobotUtils.FORWARD:
            xyz[0] += self.RobotUtils.STEP_X_DELTA
        else:
            xyz[0] -= self.RobotUtils.STEP_X_DELTA

        return xyz




    def get_foot_world_pos_xyz(self,foot_name):

        foot = self.get_foot_from_foot_name(foot_name)

        return foot.getWorldPosition([0,0,0])



    def get_foot_base_state_from_foot_name(self,foot_name):
        
        if not foot_name in self.RobotUtils.end_affectors:
            self.RobotUtils.ColorPrinter(self.__class__.__name__,"Error: foot name unrecognized","FAIL")
            return None

        if foot_name == self.RobotUtils.B_L_FOOT:
            base_foot_state = self.b_l_foot_base_state

        elif (foot_name == self.RobotUtils.B_R_FOOT):
            base_foot_state = self.b_r_foot_base_state

        elif foot_name == self.RobotUtils.F_L_FOOT:
            base_foot_state = self.f_l_foot_base_state

        else:
            base_foot_state = self.f_r_foot_base_state

        return base_foot_state


    def get_foot_from_foot_name(self,foot_name):
        
        if not foot_name in self.RobotUtils.end_affectors:
            self.RobotUtils.ColorPrinter(self.__class__.__name__,"Error: foot name unrecognized","FAIL")
            return None

        if foot_name == self.RobotUtils.B_L_FOOT:
            foot = self.b_l_foot

        elif (foot_name == self.RobotUtils.B_R_FOOT):
            foot = self.b_r_foot

        elif foot_name == self.RobotUtils.F_L_FOOT:
            foot = self.f_l_foot

        else:
            foot = self.f_r_foot
            
        return foot