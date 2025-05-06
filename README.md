# saveptsgenpath Service -

## Project Context

This project builds on and integrates the room exploration package from the [ipa320/ipa_coverage_planning](https://github.com/ipa320/ipa_coverage_planning) repository developed by the Institute for Anthropomatics and Robotics, KIT.

We use the `ipa_room_exploration` module from that repository to generate coverage paths after performing segmentation.

- **Service Name**: `/saveptsandgenpath`
  - **Request**: `saveptsandgenpath`
    - `float64[] x_pts`: X-coordinates of the points
    - `float64[] y_pts`: Y-coordinates of the points
    - `string file_name`: Name of the segment
  - **Response**: `saveptsandgenpathResponse`
    - `string result`: Result message ("done" if successful)

---

### `saveptsandgenpath_server.py`
#### ROS Params
- **Map Resolution**: `/mapresolution`
- **Map Origin**: `/maporigin`
#### Dependencies
- ROS packages: `std_msgs`, `sensor_msgs`, `nav_msgs`, `cv_bridge`, `actionlib`, `ipa_building_msgs`, `geometry_msgs`, `mb_navigation`

#### Description
1. Node and service initialization
2. Receives x_pts, y_pts and segment name from `/saveptsandgenpath` client 
3. Converts OccupancyGrid to cv Image
4. Segments the Image 
5. Saves the image locally
6. Initiates ipa room exploration on the segment 
7. Gets Action result from ipa room exploration Action Server
8. Write the path onto yaml file

---
### `saveptsandgenpath_client.py`

#### Purpose
This script acts as a ROS client to capture user-clicked points on a map. It converts these points to pixel coordinates and sends them, along with a segment name, to the `saveptsandgenpath_server.py` script.
#### ROS Params
- **Map Resolution**: `/mapresolution`
- **Map Origin**: `/maporigin`
#### Dependencies
- ROS packages: `std_msgs`, `geometry_msgs`, `mb_navigation`

#### Description
1. **ROS Node Initialization**: Initializes a ROS node and waits for the `saveptsandgenpath` service.
2. **Map Dimensions Setup**: Retrieves map dimensions (width and height) from the `/map` topic.
3. **User Interaction**: Captures user-clicked points as ROS PointStamped messages from `/clicked_point` topic.
4. **Pixel Conversion**: Converts metric coordinates to pixel coordinates using map resolution and origin.
5. **Service Call**: Calls the `saveptsandgenpath` service with the collected points and segment name.

---

### Usage :
1. Launch `room_exploration_action_server`
```bash
roslaunch ipa_room_exploration room_exploration_action_server.launch 
```
2. Set `/mapresolution`
```bash
rosparam set /mapresolution <value>
```
3. Set `/maporigin`
```bash
rosparam set /maporigin <value>
```
4. 
```bash
rosrun map_server map_server <map_file>
```
5. Launch rviz
```bash
rviz
```
6. Launch `saveptsandgenpath_server`
```bash
rosrun mb_navigation saveptsandgenpath_server.py
```
7. Launch `saveptsandgenpath_client`
```bash
rosrun mb_navigation saveptsandgenpath_client.py
```
8. Enter segment name
9. Add /Map and /Path on RViz
10. Click required no. of points (using Publish Point) on RViz
11. press enter on the terminal `saveptsandgenpath_client` is running.

---

### Available Path Planning Algorithms - 

Change the room_exploration_algorithm_ parameter in either the launch file or the script

| ID | Algorithm                         |
|----|-----------------------------------|
| 1  | Grid Point Explorator             |
| 2  | Boustrophedon Explorator          |
| 3  | Neural Network Explorator         |
| 4  | Convex SPP Explorator             |
| 5  | Flow Network Explorator           |
| 6  | Energy Functional Explorator      |
| 7  | Voronoi Explorator                |
| 8  | Boustrophedon Variant Explorator  |

