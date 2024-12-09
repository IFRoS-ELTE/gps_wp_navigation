# GPS Waypoint Navigation


## Part 1 : 
### Task 1: Reading waypoints from geojson file.


### Task 2: Transform to robot coordinate
- Detail description about conversion here( what is utm conversion , axis representation )
- In this task we implement function to  transforms geographic coordinates (latitude and longitude) of waypoints into a robot's local Cartesian coordinate frame (X, Y coordinates).
The transformation is done using the utm , which is a  method to convert geographic coordinates into Cartesian coordinates. 

- Visualization waypoints in robot frame 

    <div style="display: flex; justify-content: center;">
      <div style="flex: 1; margin-right: 10px;">
          <img src="./paltech_assignment/imgs/waypoints.png" alt="Figure 1" width="400"/>
          <p style="text-align: center;">Waypoints in robot frame</p>
      </div>
    </div>

### Task3: Calibration  Issue 
  - detail explanation , figure , how we calibrate the robot 

## Part 2: Efficient Waypoint Traversing Path Planning 
### Method 1 - Nearest Neighbor Algorithm with Dubins Path Constraints
To implement the nearest neighbor algorithm, we begin at robot position. From there, we find the closest unvisited waypoint and add it to the sequence. Then, we move to the next node and repeat the process of finding the nearest unvisited node until all nodes are included in the tour.To find the neareast unvisited node we use The `KDTree` class from `scipy.spatial` to find the k nearest neighbors of the current waypoint instead of searching the whole waypoint.

#### Summary of the Nearest Neighbor 

  1. **Start at a robot position**.
  2. **Find k neareast neighbors waypoints  to the current waypoint**.
  3. **Remove neareast waypoint with robot non holonomic constrain**:omit waypoints inside the turning radius  of the current waypoint** 
  4. **Calculate Dubins path to each selected waypoint.** 
  5. **Select waypoint with minimum dubins  path lenght.**
  6. **Add to Path and Repeat**: Add the nearest point to the path, mark it as visited, and move to this point. Repeat the process until all points are visited.**


### Method 2 : Using Traveling Salesman Problem (TSP) approximate Algorithm
This method uses the `NetworkX` library to build a weighted graph between waypoints, allowing for efficient path planning. It then calculates an approximate solution to the Traveling Salesman Problem (TSP) using this graph. Finally, it utilizes the TSP solution to compute new Dubins paths between waypoints, ensuring adherence to non-holonomic constraints.

#### steps
 
  1. **Add Nodes**: Add a node to the graph for each waypoint, including the robot's starting position. 

  2. **Add Edges**: Adds edges to the graph with the calculated path length as the weight. instead of create a graph with the whole node i use the `KD-tree` for efficient neighbor searching.

  3. **Find an approximate solution to the Traveling Salesman Problem (TSP) starting from a start waypoint.**:uses the NetworkX approximation algorithm to solve the TSP for the graph 
  
  4. **Calculate dubins path**: calculate the dubins path from the approximate tps path , omit points that does not adhere the non holonomic constrain of te robot. 


### Obstacle Avoidance Planner and controller  
   
   #### Method1 : Custome Planner

   1. **RRT_Pllaner** : This takes the start and the next waypoint and gives us a efficient  path between the two waypoints considering the obstacle(uses the `StateValidityChecker` to check for state of position and path). 
   2. **State_Validity_Checker**: This class, `StateValidityChecker`, is responsible for checking the validity of individual positions and paths (sequences of positions) with respect to obstacel list.
   3. **Move Controller**:

   #### Method2:  MoveBase ros package Planner 
   1. A* star 
   2. DWA
