## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
The script `planning_utils.py` provides functions to create the 2.5D map and later to create the shortest path using a A* algorithm. The `motion_planning.py` file provides all the necessary steps to make the drone controlable and make it fly through all the waypoints, landing in the end.

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
In `motion_planning.py`, lines `147 - 153` the starting position set in the map is loaded from the txt file.


#### 2. Set your current local position
In the same file as above, line `159`, the global positions is converted to the local position using the function `global_to_local`


#### 3. Set grid start position from local position
In line `177` it is defined the start position for the path. Basically, the initial point where the drone is started in the simulation

#### 4. Set grid goal position from geodetic coords
In lines `184-185`, there is a conversion from the geodetic coords to local enabling the search algorithm to find a path from start to finish.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
To have diagonals as a possible values, I have changed the `Action` class adding lines `104-107` the represent moving in Diagonals and the cost associated with it. I have also removed the valid actions if replacing with a easier to understand for loop (lines `128-132`)

#### 6. Cull waypoints 
To cull waypoints, I have created the file `prunning.py` that tests for collinearity tests, using numpy's determinant. The function `prune_path` check if 3 points in the path are in collinearity, removing the middle point from the way.

### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

I have also added 2 options to plan the route. One is using regular grid and a* and another option is to use sampling points. A command line can be used to decide that as well as the goal point. More information is found in file `motion_planning.py`, lines `255-256`


