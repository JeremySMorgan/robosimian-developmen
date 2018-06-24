# robosimian-development

### About
Manuel control for JPL's Robosimian quadruped robot (https://www.jpl.nasa.gov/news/news.php?feature=4603). Uses Klamp't (http://motion.pratt.duke.edu/klampt/) for simulation, ik, and modeling. The program allows for forward, and left/right motion. Backward motion is not enabled.  

### Running the Program
  1. Edit the `path_to_duke_api` variable found on line 7 of `Hypervisor.py` with the correct path to /duke_api.
  2. Run the program `python robosimian_app.py`
  
  
e - emergency stop
p - print current configuration
left - turn left
up - move forward
right - turn right

### Constraint Satisfaction
kinematic feasiblity, non self-collision, and static stability are the constraints of this problem.  
The stability constraint, by design of the gait, is gaurenteed. Neither the self-collision or kinematic feasiblity constraints are ensured to be observed. 
This is because the program does not use any form of configurations space sampling to design motion. The program simply along preset trajectories, which
are parameterized by the `RobotConstants`. Granted, after several hours of randomized user input generated motion, neither a single self collisions nor kinematicly infeasible gait was generated with the hyperparameters listed below.

        self.INITIALIZATION_STEP_TIME = .25
        self.RESET_LEG_STEP_TIME = 4
        self.TURN_GAIT_STEP_TIME = 4
        self.RESET_TORSO_SHIFT_TIME = 4
        self.TORSO_SHIFT_TIME = 4
        self.STEP_TIME = 4
        self.TORSO_YAW_ROTATE_TIME = 4
        self.TURNING_MIDSTEP_SLEEP_T = .75
        self.RESET_MIDSTEP_SLEEP_T = .75
        self.FORWARD_GAIT_SLEEP_T = .75
        self.STEP_Z_MAX_HIEGHT =  .25
        self.TORSO_SHIFT_DELTA=  .25
        self.BASE_STATE_X_DELTA = .3
        self.BASE_STATE_Y_DELTA = .3
        self.BASE_STATE_Z_DELTA = -.5
        self.IK_MAX_DEVIATION = .115
        self.TORSO_SHIFT_IK_MAX_DEVIATION = .1
        self.END_RANGE_MULTIPLIER = .95   
        self.SUPPORT_TRIANGLE_SAFETY_MARGIN = .06
        self.STARTING_CONFIG = [ 0.0, 0.0, 0.0, 0.00, 0.0, 0.000,
                                 0.0000, 0.1736, -1.4061, -0.3820, 2.0791, 0.0775, -2.2306, 0.2612,
                                 0.0000, -0.1802, 1.4324, 0.4221, -2.1182, -0.0727, 2.2432, -0.2737,
                                 0.0000, 0.1484, -1.4239, -0.3409, 2.1105, 0.0628, -2.2482, 0.2285,
                                 0.0000, -0.1724, 1.3857, 0.3545, -2.0535, -0.0808, 2.2256, -0.2259
                                 ]
### Program Design
The program is multithreaded. The `UserInput` thread takes in user input, which is read by the `ObjectiveManager`, which determines what the robot should do (stop/reset/begin a motion/etc.) 
The `ObjectiveManager` starts or stops differnent motion loops in the `MotionController` class. These loops (forward/left/right) calculates configurations and adds
 them to a queue which the control loop dequeues and sends to the robot.
 
### RobotConstants

`RobotConstants.py` stores the gaits hyperparameters, and user settings. Notable variables are described below

    self.STARTING_CONFIG
The starting configuration sets the robot's base state and is fed as a bias to the ik solvers

 
    self.SIMULATION_ENABLED      
    self.INCLUDE_TERRAIN         
    self.ODE_PHYSICS_ENABLED     
    self.DUKE_MOTION_API_ENABLED 

Dictate whether the progam will enable visualization, terrain, the duke_api or the physics engine. note that the duke_api and the physics engine cannot be run at the same time

    self.END_RANGE_MULTIPLIER    
    self.SUPPORT_TRIANGLE_SAFETY_MARGIN

Scale the end effector range's range, that is, the radius of the circle in which the torso can safely move while ensuring kinematic feasability. Note that this is only a heauristic. .96 has been experimentally determined as an accurate scaling constant. The `SUPPORT_TRIANGLE_SAFETY_MARGIN` dictates the padding added to support triangles. 0 -> no padding, increasing values result in larger padding.

    self.BASE_STATE_X_DELTA
    self.BASE_STATE_Y_DELTA
    self.BASE_STATE_Z_DELTA

Dictates the end effector's base state coordinates. Note that the torso is centered at z=0, which is why the delta z from the torso to the end effectors is negative.


    self.RESET_VISUALIZATION_ENABLED        
    self.FORWARD_STEP_VISUALIZATION_ENABLED 
    self.TURNING_VISUALIZATION_ENABLED      
    self.RANGE_CIRCLE_VISUALIZATION_ENABLED 
    self.SUPPORT_TRIANGLE_VISUALIZATION_ENABLED
    self.COM_VISUALIZATION_ENABLED             
   
Enable/Disable graphics

    self.STEP_Z_MAX_HIEGHT           
    self.TORSO_SHIFT_DELTA    

Dictate step height and torso shift amount in each step.

    self.INITIALIZATION_STEP_TIME
    self.RESET_LEG_STEP_TIME                  
    self.TURN_GAIT_STEP_TIME                  
    self.RESET_TORSO_SHIFT_TIME               
    self.TORSO_SHIFT_TIME                     
    self.STEP_TIME                            
    self.TORSO_YAW_ROTATE_TIME                
    self.TURNING_MIDSTEP_SLEEP_T              
    self.RESET_MIDSTEP_SLEEP_T                
    self.FORWARD_GAIT_SLEEP_T
    
Set the time per step/ torso shift/ rotation, etc. aswell as the sleep times in between movements.

    self.AUTOMATED_USER_INPUT  
    self.AUTOMATED_GAITS                              = [self.FORWARD, self.LEFT, self.RIGHT]
    self.AUTOMATED_GAIT_PROBIBILITY_DIST              = [2,2,2]                                 # higher the number relative to others, higher the prob. of this with the associated action being taken
    self.AVERAGE_COMPLETE_CYCLES                      = 1.75
    
Enable automated testing of the program. Note that `AUTOMATED_GAITS` should take the form of `[self.FORWARD, self.LEFT, self.RIGHT]` where each entry is a movement type. `self.AUTOMATED_GAIT_PROBIBILITY_DIST` determines the distribution of motions from `AUTOMATED_GAITS`. The higher the number relative to others, the higher the probability of the associated action being taken. `AVERAGE_COMPLETE_CYCLE` determines on average how many cycles of each action will be completed.

    self.SAVE_CONFIGS 
    self.SAVE_CONFIG_FILE 
    self.SAVE_CONFIG_IK_FAILURE_KEY 
    self.SAVE_CONFIG_RESET_KEY 
    self.SAVE_CONFIG_SELF_COLLISION_ERR_KEY
    
Enable and configure configuration and status logging. Note that this is very useful when combined with the automated user input - to test a hyperparameters feasibility enable automated user input and logging, and run the program for several hours. Then search the log file for self collissions and ik errors. 
