# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Here's the result

This video shows in the simulator how the car chooses to change lane and overtake other vehicle,
as well as brake to avoid collisions and wait before taking the decision.
Different trajectory are evaluated and then the minimum cost trajectory is taken.
(Speed, distance from back and ahead vehicle, avoid to change lane to drive in the most smoothly way are amongst the metric to evaluate a trajectory cost)
Although those metric are naive the result seems to be quite effective.

[![IMAGE ALT TEXT](http://img.youtube.com/vi/Et5Mgc2JX3w/0.jpg)](https://youtu.be/Et5Mgc2JX3w)


### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Implementation Details
The code can be divided in two sections: behaviour planning and then trajectory generation.

# Path planner
A specific class was written in order to decide the behaviour of the car. (behaviour_planner.cpp and behaviour_planner.hpp)
In the constructor (behaviour_planner.cpp lines 8 - 17) is it possible to specify some parameters that will influence
the behaviour calculation.
(i.e. guide style in order to have a more aggressive car behaviou that will arrive closer to the front car before changing
lane), then max speed, number of lanes of the highway, width of the lane in meter, the speed limit, the current lane where the car is,
and the time period of the update of the simulator 0.02 seconds)
The core of the behaviour planner is in the method:
BehaviourPlanner::next_action (behaviour_planner.cpp lines  134 - 200) and
BehaviourPlanner::can_change_lane (behaviour_planner.cpp lines 46 - 96).

In the next_action method sensor_fusion data are used in order to detect if our car will get too close to the other cars.
In this case we want to see if we can change_lane or we have to slow down and wait keeping the same lane.
Before calling can_change_lane we have to figure out wich line are we in order not to go outside the road.
So can_change_lane will use "next_line" parameter to see where we can go. It will calculate if the lane
were we want to move is empty (line 71) and then for other car except the one we can potentially crash into (i.e. is in front
of us but too close) it will check if we have enough buffer in front and rear direction.
If noone of the oter cars will violate this, then we can change lane.
When we have multiple choice for changin lane..ie the car is in the center lane so can go left or right we
choose the lane that has the biggest buffer in front..and in case of equality on the rear.
The path planner so will choose which lane the car will be, and the speed...it will accelerate
gradually (in order not to have a violation) until we reach the target speed and if we are getting
too close to another vehicle and cannot change lane it will provide a negative value of speed in order
to slow down the car.

# Trajectory planner
This was the most complicated and difficult part of the project..and the walkthrough video was
heavily used in order to reach the solution.
All the code is in file main.cpp
I decided to generate a fix length path composed of 50 points.
At first when the simulator is starting we can use only the current point of the car
and estimate the previous position of the car...(main.cpp line 270-278) and then we
will use previous_path_x ad previous_path_y point in order to have a smooth transition
between a path and the other "recycling 2 points of the previous path in the new one) (main.cpp lines 279-292)
Then we will some points at constant distance of 30 getting their XY coordinate and added to the points
calculated, doing this we will create our "long" and smooth trajectory (lines 294-311).
Then we convery our points in absolute coordinates in order to have orientation of 0 angle (lines 312 - 320)
We then create a nice interpolated path using spline library of the points calculated (lines 323 - 325)

The simulator will consume(use) 1 point of the trajectory every 0.02 seconds. So as we want to create
a path of 50 points every time we will have some not yet used points..in our previous_path_x and previous_path_y.
We want to reuse this points (that we have calculated in previous iteration) in order not to have
discontinuity in the trajectory and have a smooth path, adding only necessary points in order to rebuild
a 50 points trajectory. This is done in lines 328 - 229.

Now we have to add the points of the new trajectory calculated. How can we do this?
We have a spline that fitted our points and of course we want to use it.
(we pass the x coordinate of the point to the spline library and we will get the corresponding y)
But we want to have a smooth path too, so we want to use points that are equally distributed
in the space...so we fix a distance i.e 30 meter...for x axis oriented segment.
We calculate the corrisponding y with spline and then the hypotenuse of the triangle with
side (x, 0), (0, x). (all this in lines 334-336)
At this point we want to divide our hypotenuse in an equal way, keeping in mind
the speed of the car (in km/h hence we divided by 2.24) and also the time frame of the simulator...and  we will get
the number of points N we had) (line 346). The we divide our X trajectory by this number of points
and calculate interpolated y (lines 347-348).
At this point we have only to convert back the coordinate from car coordinate to global map
system (lines 355-361) and to add to our path (364-365) and we are done.
