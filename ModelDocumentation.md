# Path Planner Model Documentation

For easier reading, feel free to open this file in a Markdown reader.

## Overview

There are several steps involved in generating the SDC's path.

The highest-level description can be found in main.cpp, starting at line 118. The steps are as follows:
1.  To produce the next path, the SDC first copies over the points from the previous path that have not yet been consumed by the simulator. (lines 118-128)
2.  Then it computes the car's state at the end of these unused path points. This will be the state used to represent the car for the remainder of the code. (liens 1 30-159)
3.  The car's state at the sensor fusion data about other cars on the road are then passed to the behavior planning function, which will return a coarse plan in the form of a target lane and speed. (line 162)
4.  Finally, the plan is passed to the trajetory generator. If there is still lots of path left to execute, the trajectory generator will do nothing and return. Otherwise, it will compute x and y coordinates to accomplish the plan. (lines 165-171)

## Behavior Planner

Much more can be said about the behavior planner and trajectory generator.

The main behavior planner function is PlanBehavior in behavior_planner.h. It works as follows:
1.  First, the car always tries to change into the leftmost lane if it is traveling fast enough and the lane change is safe. A lane change is safe if there is enough space between the SDC and the closest cars in the target lane, both in front and behind. (line 232-241)
2.  Failing that, if the SDC is getting close to the car in front of it (line 243), then:
    *  First see it is safe to change one lane to the left (a faster lane). (lines 245-250)
    *  If not, see if it is possible to change one lane to the right. (lines 251-s56)
    *  Otherwise, keep the current lane but slow down. (line 259)
3.  If there was no car in front of the SDC, then try to speed up, up to a maximum speed. It is okay to speed up if there is either no car in front of the SDC or the closest car is very far ahead. (lines 262-272)
4.  Finally, the default behavior of the car is to maintain its speed. (line 275)

## Trajectory Generator

The trajectory generator is the most complex part of the path planner. The sequence of the code is a bit out of order with the most logical way to explain it, so I'll start with an intuitive description, then repeat it in finer-grained detail.

The shape of the path is mostly determined by fitting a spline to a few of the last path points at the end of the previous path, and a few waypoints. The two closest waypoints (before and after the SDC's position) should be excluded to produce smooth curves!

The position along the spline at a point in time is produced by fitting two jerk-minimizing quintic polynomials, one for the s-coordinate and one for the d-coorinate. These have to be converted back to Cartesian coordinates before feeding into the simulator.

And now onto code! The main function is GenerateTrajectory in trajectory_generator.h. If there is a lot of previous path left, this function will return immediately. Otherwise, it will do the following:
1.  It will first compute the starting state of the car for quintic polynomial generation (line 185-195).
2.  Then it will compute the coarse target state (lines 198-231) from the target speed and lane passed to it from the behavior plannet. Note that the target speed may be damped down if the steering angle is very large (lines 198-220).
3.  Next the target state is handed off to GenerateCandidateTargets, which produces other potential target state by adding noise to the given ones.
4.  The start state and all of the candidate target states will have s- and d-coordinate quintic jerk-minimizing polynomials fit to them. (lines 238-253) The coefficients of the polynomials and valid time horizon are saved (lines 255-257).
5.  Next BestTrajectory evaluates all the polynomials and times and selects the "best" one according to a set of cost functions (defined in cost_fns.h) and weights that are configured when the TrajectoryGenerator is constructed (line 73 of main.cpp). Weights can be found in lines 28-52 of trajectory_generator.h.
6.  Finally, DiscretizeTrajectory first fits a spline (lines 374-498), then iterates through the valid time of the best trajectory's jerk-minimizing polynomials, performing a conversion back to global Cartesian coordinates, incorporating the spline and jerk-minimizing polynomials. (lines 504-527).

## Other Files

Cost functions are defined in cost_fns.h. A description of each cost function can be found at the top of each cost function's class.

Various constants used by all the files can be found in sdc_constants.h.

Various helper functions used by all the files can be found in sdc_utils.h.

## Addressing Rubric Points

*  The code compiles correctly.

Yes, just follow the directions in REAADME.md.

*  The car is able to drive at least 4.32 miles without incident.

It doesn't all the time, but it is able to within a few tries.

* The car drives according to the speed limit.

This was a little tough to accomplish, since I was also trying to make the car go as fast as possible. First, I set the car's maximum speed to 45.0 mph (SPEED_LIMIT in sdc_constants.h), below the posted speed limit of 50 mph, since the simulator experiences some jitter. Second, the Frenet-to-Cartesian conversion I implemented is only a coarse approximation, that gets worse with with increasing distance from the double-yellow line where the waypoints are defined. I damped down the car's maximum speed in the right lanes to prevent the simulator jitter from registering the car as exceeding 50 mph.

* Max acceleration and jerk are not exceeded.

Acceleration was kept below the cap by giving a large path calculation distance (NUM_POINTS_IN_PATH - PATH_RECALCULATION_SIZE in sdc_constants.h) to the trajectory generator. Jerk was minimizes by fitting a jerk-minizing polynomial to produce smooth trajectories. Also, cost functions were defined for total acceleration, total jerk, and whether the maximums were exceeded.

* Car does not have collisions.

Collisions were avoided by only having the behavior planner tell the car to change lanes when it was safe to do so (i.e. there is no one nearby in the Frenet s-coordinate direction in the target lane).

* The car stays in its lane, except for the time between changing lanes.

This was accomplished by having the trajectory generator always have the car target the center of the lane the behavior planner passed it, and by the behavior planner never asking the car to go off the road or into oncoming traffic.

* The car is able to change lanes.

Yes. The behavior planner is responsible for lane change decisions.