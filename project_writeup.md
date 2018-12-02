## Project: 3D Motion Planning

The 3D Motion Planning project is the second project in the Udacity Flying Car Nanodegree. In this project, our goal is to create a path planning algorithm that enables an autonomous quadcopter to plan and navigate through a complex urban environment to any desired goal location.

The sections below correspond to the project [rubric](https://review.udacity.com/#!/rubrics/1534/view)  and outline how each requirement was satisfied.

![Quad Image](./misc/enroute.png)

---

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

In the starter code in 'motion_planning.py', an additional Planning state has been added to the backyard flyer impplementation. This state occursafter arming and before takeoff. Then, as an event handler for that state, the method plan_path() has been added.

The plan_path method does the following things:

* Sets the flight state to Planning
* Provides a hardcoded target altitude and safety distance
* Creates a grid representation of the configuration space with the target altitude and safety distance
* Reads in the obstacle data from a file
* Establishes a start and goal position
* Uses A* to plan a path through the configuration space from the start state to the goal state using the provided heuristic as a cost function

The included 'planning_utils.py' has several utility features that are used by the path_plan method. These utility features are:

* **create_grid:** creates a grid based representation in 2.5D based on the obstable data (in a format similar to colliders.csv) based on a given drone alititude and safety distance
* **Action:** an enumeration class that defines a 3 tuple for each of the 4 movement directions on a grid based representation (North, South, East, West). For each of the enumeration elements, a grid-based position delta (N, E) and a cost are defined
* **valid_actions:** determines all valid actions for a specified grid cell, given a grid and current node
* **a_star:** implements an a* path planning algorithm for a grid-based representation, given start and goal states, and a specified heuristic and grid
* **heuristic:** provides a cost heuristic to be used by a* for approximating the relative cost between two given points

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position

I created a method (self.get_latlon_fromfile) that parsed the first line of 'colliders.csv' to obtain the latitude and longitude of the current start position in the geodetic frame. I extracted the values, then cast them to type float. 

```
lat0, lon0 = get_latlon_fromfile('colliders.csv')
```

Then, I set the current home position using those values with self.set_home_position().

```
self.set_home_position(lon0, lat0, 0)
```

#### 2. Set your current local position

In order the get the local position (which is relative to a given global home position (established in the last step), I am ready to get the current local position.

First, I retrieve the current global position in the geodetic frame.

```
curr_global_pos = (self._longitude, self._latitude, self._altitude)
```

Then, I convert the global position to the local ECEF frame using the global_to_local() method imported from 'planning_utils.py'.

```
curr_local_pos = global_to_local(curr_global_pos, self.global_home)
```

#### 3. Set grid start position from local position

Get the current start position and use this as the starting point for our upcoming path planning task. For the conversion between the local frame and the grid relative frame (relative to map center), I defined a method to provide this conversion called get_gridrelative_position().

```
grid_start = self.get_gridrelative_position(curr_local_pos[0:2], offsets)
```

Previously, I saved the offsets returned from the create_grid method to be used for the NED to grid conversion.

```
offsets = (north_offset, east_offset)
```

#### 4. Set grid goal position from geodetic coords

In order to add flexibility to the goal location, I added 2 program input parameters (--goal_longitude and --goal_latitude).

```
parser.add_argument('--goal_longitude', type=float, default=-122.398414, help='Goal Longitude')
parser.add_argument('--goal_latitude', type=float, default=37.7939265, help='Goal Latitude')
```

Then, I used the input parameter values to set the goal position, convert them to NED, then obtain the grid-relative position.

```
custom_goal_pos = (-122.398414, 37.7939265, 0)
goal_local = global_to_local(custom_goal_pos, self.global_home)[0:2]
grid_goal = get_gridrelative_position(goal_local, offsets)
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)

In 'planning_utils.py', I modified the Action enumeration to include the additional action and  cost:

```
NORTH_WEST = (-1, -1, np.sqrt(2))
NORTH_EAST = (-1, 1, np.sqrt(2))
SOUTH_WEST = (1, -1, np.sqrt(2))
SOUTH_EAST = (1, 1, np.sqrt(2))
````
Additionally, I added the related logic in the valid actions method:

```
# Diagonal Actions
if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
    valid_actions.remove(Action.NORTH_WEST)
if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
    valid_actions.remove(Action.NORTH_EAST)
if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
    valid_actions.remove(Action.SOUTH_WEST)  
if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
    valid_actions.remove(Action.SOUTH_EAST)   
```

#### 6. Cull waypoints

I created a path pruning algo called prune_path, which used a collinearity test to determine if a given set of points are collinear within a given threshold. 

```
def collinearity_test(self, p1, p2, p3, epsilon=1e-6):   
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon
```

Then, in the pruning algorithm, if the points were collinear, then the middle point was removed from the plan.

Then I ran it on the path returned from A*:

```
path = self.prune_path(path)
```

### Execute the flight

#### 1. Does it work?

My program enabled the drone to navigate the urban environment successfully. Since it is able to start at its current location, and be directed to any desired goal waypoint, the program can be run multiple times.

#### Path Planning v1 (Diagonal Paths)

Planner specs:
* Single phase (global-plan only)
* Grid-based representation
* No vehicle dynamics
* Straight and diagonal paths