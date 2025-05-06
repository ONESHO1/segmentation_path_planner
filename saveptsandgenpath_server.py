#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge, CvBridgeError

import actionlib
import ipa_building_msgs.msg

import cv2
import numpy as np
from geometry_msgs.msg import Pose, Pose2D, Point32
from std_msgs.msg import String

import yaml
from beginner_tutorials.srv import saveptsandgenpath, saveptsandgenpathResponse

def room_exploration_client():
    global segment_name
    
    bridge = CvBridge()

    ac = actionlib.SimpleActionClient('/room_exploration/room_exploration_server', ipa_building_msgs.msg.RoomExplorationAction)

    robot_radius = rospy.get_param('~robot_radius', 0.3)
    coverage_radius = rospy.get_param('~coverage_radius', 1.0)
    start_pos = rospy.get_param('~starting_position', [0, 0, 0])
    select_area = rospy.get_param('~select_area', False)

    if len(start_pos) != 3:
        rospy.logfatal("starting_position must contain 3 values")
        rospy.signal_shutdown("Invalid starting_position")

    image_path = f"/home/oneshot/catkin_ws/src/beginner_tutorials/maps/{segment_name}.png"

    rospy.loginfo("image path - %s", image_path)
    map_flipped = cv2.imread(image_path, 0)
    map_image = cv2.flip(map_flipped, 0)

    map_image[map_image < 250] = 0
    map_image[map_image >= 250] = 255

    rospy.loginfo("Waiting for action server to start.")
    ac.wait_for_server()
    rospy.loginfo("Action server started, sending goal.")
    
    labeling = bridge.cv2_to_imgmsg(map_image, encoding="mono8")
    
    resolution = rospy.get_param('/mapresolution', 0.05)
    origin = rospy.get_param('/maporigin', [0, 0, 0])

    map_origin = Pose()
    map_origin.position.x, map_origin.position.y, map_origin.position.z = origin

    starting_position = Pose2D()
    starting_position.x, starting_position.y, starting_position.theta = start_pos

    fov_points = [
        Point32(0.04035, -0.136, 0.0),
        Point32(0.04035, 0.364, 0.0),
        Point32(0.54035, 0.364, 0.0),
        Point32(0.54035, -0.136, 0.0)
    ]
    fov_origin = Point32(0.0, 0.0, 0.0)
    
    #parameters changed as it is different to the ones published in the original client (launch file)
    robot_radius = 0.5
    coverage_radius = 0.5
    
    
    goal = ipa_building_msgs.msg.RoomExplorationGoal(
        input_map=labeling,
        map_resolution=resolution,
        map_origin=map_origin,
        robot_radius=robot_radius,
        coverage_radius=coverage_radius,
        field_of_view=fov_points,
        field_of_view_origin=fov_origin,
        starting_position=starting_position,
        planning_mode=2  # viewpoint planning
    )

    ac.send_goal(goal)
    ac.wait_for_result()

    action_result = ac.get_result()

    rospy.loginfo("Got a path with %d nodes.", len(action_result.coverage_path))
    
    #write path to yaml file
    with open(f"/home/oneshot/catkin_ws/src/beginner_tutorials/maps/{segment_name}.yaml", "w") as yaml_file:
        for entry in action_result.coverage_path:
            yaml_file.write(f"{entry.x} {entry.y} {entry.theta}\n")
    
    return 0

def filler(image):
    global x_pts, y_pts
    
    # Combine x_pts and y_pts into main_points_array
    main_points_array = np.column_stack((x_pts, y_pts))
    
    stencil = np.zeros(image.shape).astype(image.dtype)
    contours = [main_points_array]
    color = [255, 255, 255]
    cv2.fillPoly(stencil, contours, color)
    result = cv2.bitwise_and(image, stencil)
    return result

def occupancy_grid_to_image(map_msg):
    # Extract map data
    width = map_msg.info.width
    height = map_msg.info.height
    data = np.array(map_msg.data).reshape((height, width))

    # Convert occupancy values to grayscale values (0-255)
    image_data = np.uint8(255 - 255 * (data.astype(float) / 100.0))

    # Create a grayscale image
    map_image = np.zeros((height, width, 3), dtype=np.uint8)
    map_image[:, :, 0] = image_data
    map_image[:, :, 1] = image_data
    map_image[:, :, 2] = image_data

    return map_image

def segment(msg):
    try:
        # Convert ROS OccupancyGrid message to OpenCV image
        input_image = occupancy_grid_to_image(msg)
        
        #input image flipped
        map_image = cv2.flip(input_image, 0)

        # Perform segmentation on the input image
        result_image = filler(map_image)
        
        # Save the segmented image to disk
        result_image_path = '/home/oneshot/catkin_ws/src/beginner_tutorials/maps/' + segment_name + '.png'
        cv2.imwrite(result_image_path, result_image)
        
        room_exploration_client()
    except CvBridgeError as e:
        rospy.logerr(e)
  
def server_handle(req):
    global x_pts, y_pts, segment_name
    
    x_pts = req.x_pts
    y_pts = req.y_pts
    segment_name = req.file_name
    
    rospy.Subscriber('/map', OccupancyGrid, segment)
    
    return saveptsandgenpathResponse("done")
    
def server_init():
     rospy.init_node('room_exploration_client')
     s = rospy.Service('saveptsandgenpath', saveptsandgenpath, server_handle)
     print("saveptsandgenpath server Ready")
     rospy.spin()

x_pts = []
y_pts = []
segment_name = None

if __name__ == '__main__':
    try:
        server_init()
    except rospy.ROSInterruptException:
        pass

