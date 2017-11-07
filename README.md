# robosimian-development

### About
Walking gait for JPL's Robosimian quadruped robot (https://www.jpl.nasa.gov/news/news.php?feature=4603). Uses Klamp't (http://motion.pratt.duke.edu/klampt/) for simulation, ik, and modeling. Forward gait and reset algorithm ensure stability in all known input states. 

### Walking Algorithm



### Videos
See http://jeremymorgan.net/projects for videos of simulated gait.

### Reset Algorithm

```python

let:
	B_t = torso base state position
	B_(fr/fl/br/bl) = (front right/front left/ back right/ back left) base state position
	P_(fr/fl/br/bl) = (front right/front left/ back right/ back left) starting foot position
	tri(end_affector_1, end_affector_2, end_affector_3) = triangle defined by the specified triangles 
	ST_(fr/fl/br/bl) = Support Triangle defined by the three legs not specified in the variable name. For example ST_fr is the support triangle created by the fl, br, bl end affectors


if 4 legs make a base state B, regardless of torso position:
	Shift and rotate torso to torso position P commanded by B
else if 3 legs make a base state B, regardless of torso position:
    let ST_t = the support triangle defined by the three legs currently in the base state
    if the current torso position is not in ST_t:
	    shift torso to the center of ST_t
	step 4th leg that is not in B to commanded base state defined by B
	shift torso to B_t
else:
	# Move left legs (first stage)
	let ST_fl = tri( P_0bl, P_0fr, P_0br)
	let ST_bl = tri( B_fl, P_0fr, P_0br)
	let Pt = middle point in area defined by ( ST_fr intersection ST_bl ) 
	shift torso to Pt
	back left leg step to B_bl
	front left leg step to B_fl
	
	# Move right legs (second stage)
	let ST_fr = tri(B_br, B_bl, B_fl)
	let ST_bf = tri( P_0fr, B_bl, B_fl )

    shift torso to middle point in area defined by (ST_fr intersection ST_br)
    front right to B_fr
    back right to B_br
    shift torso to B_t
```

![Reset Stage 1](https://github.com/JeremySMorgan/robosimian-gait-development/blob/master/torso_stage_1.png)

![Reset Stage 1](https://github.com/JeremySMorgan/robosimian-gait-development/blob/master/reset_stage_2.png)



### Notable Variables

- `measured_controller_dt` Aproximates the true control loop delay time. This variable is used in all of the Threaded Moiton in Speficied Time APIS. Note that because of this the TURN/STEP/ect_TIME constants in *RobotUtils* are used as estimates for the total step time. 

