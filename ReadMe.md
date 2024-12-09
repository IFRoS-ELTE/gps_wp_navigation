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

### Task3: Calibration of Magnetometer sensor
For the GPS navigation to work correctly, initial robot orientation must be aligned with the global(earth's) coordinate frame. According to [REP 105](https://www.ros.org/reps/rep-0105.html), when using GPS for navigation, the robot's x-axis should point towards the east, and y-axis should point towrads the north.  

The reading from the Magnetometer sensor can be used to calibrate the robot's orientation. The magnetometer sensor provides the magnetic field vector in the robot's local coordinate frame. However, the magnetometer itself is distorted by magnetic field from the sensor circuit board, and the surounding environment.The two most common distortion are hard iron and soft iron distortion as shown in the following figure.
<p>
  <img src="./soft-and-hard.png" height="300" width="auto"/> &nbsp;&nbsp;
</p><br>

To correct these distortions, the magnetometer sensor readings can be calibrated using the following steps:
1. **Collect Data**: Collect data from the magnetometer sensor while rotating the robot in all possible directions. Figure-eight motion is a common method to collect data from the magnetometer sensor.
2. **Compute calibration coefficients**: Use the collected data to compute the calibration coefficients. The calibration coefficients are used to correct the distortions in the magnetometer sensor readings. Different tools such as [magcal](https://www.mathworks.com/help/nav/ref/magcal.html) can be used to compute the calibration coefficients.

The equation is:

$$ 
C = (D - b) \cdot A
$$
where `C` is the calibrated magnetometer sensor readings, `D` is the raw magnetometer sensor readings, `A` is the soft iron calibration matrix, and `b` is the hard iron calibration vector.

 2.1. **Soft Iron Calibration**: The soft iron calibration corrects the ellipsoidal distortion in the magnetometer sensor readings. The calibration coefficients are used to scale the magnetometer sensor readings along each axis. These coefficients are represented as a matrix(`A` in the above equation) and multiplied with the magnetometer readings.

 2.2. **Hard Iron Calibration**: The hard iron calibration corrects the offset in the magnetometer sensor readings. The calibration coefficients are used to shift the magnetometer sensor readings along each axis. This coefficents are represented as vector(`b` in the above equation) and added to the magnetometer readings.  

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