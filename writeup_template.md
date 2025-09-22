## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---

# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.  
2. Discretize the environment into a grid or graph representation.  
3. Define the start and goal locations.  
4. Perform a search using A* or other search algorithm.  
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.  
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0]).  
7. Write it up.  
8. Congratulations! You’re Done!  

---

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  
You’re reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

---

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

- **motion_planning.py**:  
  Provides a finite state machine (MANUAL, ARMING, PLANNING, TAKEOFF, WAYPOINT, LANDING, DISARMING) that manages the mission flow automatically. It defines the `plan_path()` function, which reads the environment, sets start and goal positions, runs the path planning algorithm, and sends waypoints to the simulator.  

- **planning_utils.py**:  
  Provides helper functions including `create_grid()` (to build the 2.5D grid), the `Action` Enum (which defines motion primitives and their costs), `valid_actions()`, the `a_star()` search algorithm, and `heuristic()`.  

Compared to the starter `backyard_flyer.py`, which only executed a square trajectory with hardcoded waypoints, this new implementation builds a real search-based planner that adapts to any start and goal.

![Top Down View](./misc/high_up.png)

---

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
I read the first line of `colliders.csv` to extract `lat0` and `lon0` as floating-point values and used `self.set_home_position()` to set the global home:

```python
with open("colliders.csv", 'r') as f:
    lat0, lon0 = f.readline().split(",")
    lat0 = float(lat0.split(" ")[1])
    lon0 = float(lon0.strip().split(" ")[1])
self.set_home_position(lon0, lat0, 0)
```

---

#### 2. Set your current local position

I retrieved the drone’s current geodetic position (`self.global_position`) and converted it to local coordinates with `global_to_local()`:

```python
local_position = global_to_local(self.global_position, self.global_home)
```

---

#### 3. Set grid start position from local position

Instead of using the grid center as in the starter code, the start is computed dynamically from the drone’s actual location:

```python
grid_start = (int(-north_offset + local_position[0]),
              int(-east_offset + local_position[1]))
```

---

#### 4. Set grid goal position from geodetic coords

I defined the goal as an arbitrary latitude/longitude coordinate. This makes the goal flexible:

```python
goal_lat = 37.7922263
goal_lon = -122.3974193
local_goal = global_to_local((goal_lon, goal_lat, TARGET_ALTITUDE), self.global_home)
grid_goal = (int(-north_offset + local_goal[0]),
             int(-east_offset + local_goal[1]))
```

By modifying `goal_lat` and `goal_lon`, the drone can be sent to any location within the map.

---

#### 5. Modify A* to include diagonal motion (or replace A* altogether)

The `Action` Enum in `planning_utils.py` was extended to include diagonal moves (SW, NW, NE, SE). Each diagonal motion has a cost of `sqrt(2)`:

```python
SOUTHWEST = (1, -1, np.sqrt(2))
NORTHWEST = (-1, -1, np.sqrt(2))
NORTHEAST = (-1, 1, np.sqrt(2))
SOUTHEAST = (1, 1, np.sqrt(2))
```

This ensures shorter and more natural paths compared to the original 4-directional planner.

---

#### 6. Cull waypoints

To reduce unnecessary intermediate points, I implemented a path pruning method using a **collinearity check**:

```python
def prune_path(self, path):
    pruned_path = [p for p in path]
    i = 0
    while i < len(pruned_path) - 2:
        p1, p2, p3 = self.point(pruned_path[i]), self.point(pruned_path[i+1]), self.point(pruned_path[i+2])
        if self.collinearity_check(p1, p2, p3):
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path
```

This greatly reduces the number of waypoints and creates smoother trajectories.

---

### Execute the flight

#### 1. Does it work?

Yes! After loading the map, setting start and goal dynamically, running A*, and pruning the path, the drone successfully flies from start to goal while avoiding obstacles.

I also added a **safety distance of 5 meters** when building the grid to ensure safe clearance around obstacles.

---

### Extra: Improving Transition Between Waypoints

During testing, I noticed that the drone sometimes **paused for 20–40 seconds at a waypoint** before moving to the next.

This delay occurs because of strict conditions in `local_position_callback()`:

- The distance threshold to consider “arrival” was `< 1.0 meter`.
    
- Additionally, the drone had to reduce its local velocity to below `1.0 m/s` before transitioning.
    

In practice, this made the drone wait too long even if it was already close enough to the waypoint.

**Solution:**

- Relaxed the position threshold (e.g., `< 2.0 or < 3.0 meters`).
    
- Relaxed or removed the velocity check between waypoints (only keeping it for final landing).
    

This made the transitions smoother and significantly reduced unnecessary pauses.

---

## Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.

- Global home set from `colliders.csv`. ✅
    
- Start from current drone position. ✅
    
- Goal defined from arbitrary lat/lon coordinates. ✅
    
- A* search with diagonals (cost `sqrt(2)`). ✅
    
- Path pruning with collinearity. ✅
    
- Waypoints sent to simulator. ✅
    

---

# Extra Challenges: Real World Planning

For future improvements, I could explore:

- Probabilistic Roadmap (PRM) or Rapidly-exploring Random Trees (RRT).
    
- Replanning in case of unexpected obstacles.
    
- Using a vehicle dynamics model for smoother trajectories.
    
- Incorporating velocity commands instead of only position commands.
    

---
