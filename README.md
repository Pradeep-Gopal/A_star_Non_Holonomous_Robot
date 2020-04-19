# Astar_algorithm

Implementation of Astar algorithm in python for Non holonomous Turtlebot-2 robot

## Authors

 1. Pradeep Gopal
 2. Sri Manika Makam


## Dependencies

 1. numpy library
 2. matplotlib library
 3. queue library
 4. math library
 5. time library
 6. argparse library
 7. Python 3.5
 8. ubuntu 16.04
 
## Instructions to run

The inputs from user are coordinates of start point, orientation of start point (in degrees), coordinates of goal point, two RPM values and clearance. 

Robot radius is taken by default as 0.08 (which is half the distance between wheels of the robot). Theta (the angle between the action at each node) is taken as 15. The orientation of goal point is taken by default as 0. 

The total clearance is clearance given by user + 0.16 (minimum clearance).
Wheel radius is 33mm and distance between the wheels is 160mm for turtlebot3.

Go to the directory where code is present and run the following command

```
python a_star.py --user_input 0 --animation 0
```
If 'user_input' is 1, then user is allowed to give inputs of his wish. 
If 'user_input' is 0:

start point = (-4.5,-4.5,0), theta = 15.0, goal point = (4.5, 4.5, 0), robot_radius = 0.08, clearance = 0.16, RPM1 = 9 and RPM2 = 11.

If animation = 1, you can observe the exploration of nodes, but please note that this will take a lot of time.

If animation = 0, the program displays the exploration space and optimal path at once. 

The coordinates are given as per right-handed coordinate system (Gazebo coordinate system).

## Output

The time taken by the algorithm to find the shortest path for default inputs is approximately 0.33 seconds.

The figure showing exploration space and shortest path is saved as 'output.png'.

The video output is saved as 'Phase3_video.mp4'.

