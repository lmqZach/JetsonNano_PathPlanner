# JetsonNano_PathPlanner
## File Structure
This projects shares the same ROS Jetson configuration root file from: https://github.com/lmqZach/JetsonNano_PoseEstimation

The new algorithm for this progress file resides under root/navigation_dev/src
```
|-- ROOT
  |-- README.md
  |-- CMakeLists.txt
  |-- init.sh
  |-- jetbot_ROS
  |   |-- ```
  |-- jetson-inference
  |   |-- ```
  |-- navigation_dev
  |   |-- CMakeLists.txt
  |   |-- package.xml
  |   |-- launch
  |   |-- msg
  |   |-- src
  |          |-- april_detect.py
  |          |-- localization_node.py
  |          |-- planner_node.py
  |          |-- VoroniGraph.py
  |-- ros_deep_learning
      |-- ```
```

## Objective
An important part of robotics is path and task planning. In this phase of project, we designed a Voronoi-based path planner using SciPy to auto-navigate through target locations.

## Detailed Tasks
1. Setup an environment in a 10ft ×10ft area with landmarks at the edges as show in the figure. Measure the position of your landmarks. Place an obstacle of size 1x1 - 2x2 ft in the middle of the workspace. 

<img width="257" alt="Screen Shot 2022-05-09 at 15 28 12" src="https://user-images.githubusercontent.com/92130976/167484904-6d9798a6-1d55-449e-9a2e-a34f59181c96.png">


2. Design a voronoi based path planner to go from start to the 1st and 2nd stops.

3. Provide a report that describes your chosen representation, the associated algorithm, videos/images that document the results. 

# Report
## Logistic: 

Differing from the last assignment which consistently evaluates mapping and localization, this assignment focuses on path planning based on known obstacle locations. The main method associated is the Voronoi diagram which generates viable vertices and path. Our approach is to divide into three sections: localization with known obstacles; Voronoi graph generation and vertices elimination; motion planning.

## Voronoi Application:

Voronoi diagram is widely used in dividing 2-D space into cells, which covers the region closest to a specific point. In this case, obstacle boundaries are represented by continuous lists of points, fitted into the Voronoi algorithm to generate a path that is mostly equidistant between the obstacles closest to the robot. As a result, the route is considered collision-free and relatively safe during motion.

## Code Implementation:

### Localization:

For every tag observed in a frame, it needs to be recognized as which specific one it is in the map. Thus, our group computes its Euclidean distance in world coordinate (this is done by using the previous frame's robot position and current frame's relative tag coordination) to all tags in the map that belong to the same category. Then we assign an observation to the tag with a minimum distance. After we have all ground-truth values of tag coordinates, we can then compute the robot’s world coordinate in the current frame by averaging over all observations.

### Voronoi Graph:

In our environment setup, both environment boundary and obstacle are generally viewed as rectangles, which allows us to simplify the points used to represent the edges. As a result, only the vertices and the midpoints on each edge of the obstacles are listed in the obstacle array. Then, ‘scipy.spatial.Voronoi’ library is imported to generate the raw diagram which has all unfiltered Voronoi vertices. By checking if the vertices are on or within the obstacle boundaries, the rest of the vertices can be effectively viewed as the selectable waypoints in the robot path.

### Motion Planning:

The project has assigned the start point and two stop points in the required path so that Voronoi vertices which are closest to these three points are chosen as the final waypoints. All points and vertices are connected by straight lines, with the control values relying on the robot’s relative position and orientation to waypoints. The localization method can calculate the robot’s pose in the world coordinate based on the relative pose to known AprilTags, which is the prerequisite of obtaining accurate relative positions and orientation. Variable ‘dist1’ is inserted into the algorithm to pause the robot at two given stops momentarily.

## Performance:

The Voronoi diagram is checked and filtered offline into final waypoints, and they are verified manually before importing into the motion. In the actual run, the robot is placed at the starting point, and it followed waypoints correctly with two short stops. It is within our expectation due to the similar motion model from our previous work which was proven accurate and effective.

## Limitations:

Although the project requirements are met, there are few limitations. Starting with the overall smoothness of the path, the path algorithm could use denser waypoints so it could fit into more complicated maps. Also, the waypoints sequence is manually sorted due to the small number of points needed in this project, which does not apply to a wider broad. 

## Potential Improvements:

Our group has some knowledge on the possible ways to improve. More waypoints can be obtained by increasing the density of obstacles. However, it is noteworthy to implement a different vertices elimination algorithm depending on the complexity of obstacles. In addition, automatically searching for the next waypoints among filtered vertices can be achieved by standard search algorithms, like Dijkstra's Algorithm of finding the best path from the subset of the Voronoi diagram connecting starting and stop points. Our other potential approach is to manually assign the first one or two waypoints (depending on the desired path) from the vertices list and repeatedly search for the next vertex with the shortest Euclidian distance from the current waypoint. 
With those improvements being mentioned, we know the steps to implement but did not actually use them in this project due to the simplicity of the test environment and obstacles.

## Demostration Video: 

https://youtu.be/pnAii2hZpVA 

## Voronoi Graph:

<img width="327" alt="image" src="https://user-images.githubusercontent.com/92130976/167485483-c9ac0f4e-93b8-459b-8172-1b6075d4b032.png">

Figure 1: Raw Voronoi Graph (with unfiltered vertices)

<img width="312" alt="image" src="https://user-images.githubusercontent.com/92130976/167485531-4b416e04-90cd-4f96-834a-d06c9288f792.png">

Figure 2: Filtered Voronoi Vertices(yellow) and Obstacle Points(blue)

<img width="324" alt="image" src="https://user-images.githubusercontent.com/92130976/167485553-77d5452d-8afd-43cf-9ea7-88c1639cffc9.png">

Figure 3: Final Waypoints(yellow), Start Point(red), Stop Points(green), and Obstacles(blue)
