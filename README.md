# robosimian-development

## About
Walking gait for JPL's Robosimian quadruped robot. Uses Klamp't (http://motion.pratt.duke.edu/klampt/) for simulation, ik, and modeling.

## Media

#### video
www.youtube.com/watch?v=yeAvKYfvZMc

#### Image
Image of the simulated robot in its start configuration
<img src="https://github.com/JeremySMorgan/robosimian-gait-development/blob/master/robosimian.png" width="600" height="600" />


### Variables

- `measured_controller_dt` Aproximates the true control loop delay time. This variable is used in all of the Threaded Moiton in Speficied Time APIS. Note that because of this the TURN/STEP/ect_TIME constants in *RobotUtils* are used as estimates for the total step time. 


### Notes
      
- Do not change the robots configuration before calling `initialize_motion_planner`, `initialize_motion_planner`, and `save_base_states` (in order). ex:
       
``` python 
self.HighLevelMotionController.initialize_motion_planner(self.MotionPlanner)
self.HighLevelMotionController.set_inital_config()
self.MotionPlanner.save_base_states()
```
   