import rclpy
from rclpy.node import Node
import time
import cv2
import imutils
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from . import tf_transformations as trans
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion, Vector3
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String, ColorRGBA
from vlad_messages.msg import Beacon # custom msg type
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

class my_node(Node):
    def __init__(self):
        super().__init__('demo2_node')

        # QoS Profiles
        qos_policy = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)        

        # REQUIREMENT 1 (REPUBLISH /map to /ecte477/map)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.callback_map, qos_policy)
        self.map_pub = self.create_publisher(OccupancyGrid, '/ecte477/map', qos_policy)
        
        # REQUIREMENT 1 (Plotting path instead of e and r path)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.callback_odom_path, 10)
        self.path_pub = self.create_publisher(Path, '/ecte477/path', 10)

        # REQUIREMENT 1 (Getting Stack_points from demo 1)
        self.stack_points = self.create_subscription(MarkerArray, '/stack_points', self.callback_stack_points, 10)
        
        # REQUIREMENT 1 (Initialise Path object)
        self.path = Path()
        self.path.header.frame_id = 'odom'

        # REQUIREMENT 2 (load beacon parameters from YAML file, lab 9 param_reader)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('beacons.id', rclpy.Parameter.Type.INTEGER_ARRAY),
                ('beacons.top', rclpy.Parameter.Type.STRING_ARRAY),
                ('beacons.bottom', rclpy.Parameter.Type.STRING_ARRAY)
            ]
        )

        self.beacons_id = [0, 1, 2, 3]
        self.beacons_top = ['red', 'blue', 'green', 'blue']
        self.beacons_bottom = ['yellow', 'red', 'yellow', 'green'] # hard coded as beacon_24 wasnt loading correctly


        # REQUIREMENT 2 (Get beacon parameters, im not using them though?)
        self.beacon_ids = self.get_parameter('beacons.id').value
        self.beacon_tops = self.get_parameter('beacons.top').value
        self.beacon_bottoms = self.get_parameter('beacons.bottom').value
        self.beacon_pub = self.create_publisher(Beacon, '/ecte477/beacons', 10)
        self.get_logger().info(f"########LOADED BEACON INFO FROM BEACONS.MSG##########: {self.beacon_ids}")
        
        # REQUIREMENT 2 (Store value for homogenous vector and subscribe to needed channels lab 9)
        self.K = None # Store value for homogenous vector
        self.transform_cam_to_world = None
        self.subscriber_camera_info = self.create_subscription(CameraInfo, '/intel_realsense_r200_depth/camera_info', self.callback_cam_info, qos_policy) # subscribe for intrinsic info
        self.subscriber_odom = self.create_subscription(Odometry, '/odom', self.callback_odom, qos_policy) # subscribe for extrinsic info
        self.subscriber_colour = self.create_subscription(Image, '/intel_realsense_r200_depth/image_raw',self.callback_colour, qos_policy)
        self.subscriber_depth = self.create_subscription(Image, '/intel_realsense_r200_depth/depth/image_raw',self.callback_depth, qos_policy)
        
        self.bridge = CvBridge() # instantiate object of cv_bridge
        self.colour_frame = None # instantiates colour image display
        self.depth_frame = None # instantiates depth image display
        self.num_colour_images = 0 # instantiates no. of colour images (not necessary)
        self.num_depth_images = 0 # instantiates no. of depth images (not necessary)
        self.mask = None # instantiates masked image display

        # REQUIREMENT 3 (publish detected beacons)
        self.beacon_markers = self.create_publisher(MarkerArray, '/ecte477/beacon_markers', 10)
        self.beacon_array = MarkerArray()
        self.marker_id = 0 # unique id for each marker
        
        # loop to spin the node
        while rclpy.ok():
            self.loop() # run loop which shows colour,depth and masked displays
            rclpy.spin_once(self, timeout_sec=1.0)

    # REQUIREMENT 1 (Republish map from /map to /ecte477/map)
    def callback_map(self, data):
        self.map_pub.publish(data) #Publish map to /ecte477/map

    # REQUIREMENT 1 (Subscribe to stack_points and keep track of exploring)
    def callback_stack_points(self, stack_points):
        if len(stack_points.markers) == 0:
            self.get_logger().info("##############DONE EXPLORING##############") # all it does is know when exploring done
    
    # REQUIREMENT 1 (odom path to publish path in RVIZ while exploring)
    def callback_odom_path(self, data):
        path_point = PoseStamped() #Track and publish path to /ecte477/path
        path_point.pose = data.pose.pose
        path_point.header.stamp = self.get_clock().now().to_msg()
        path_point.header.frame_id = 'odom'
        self.path.poses.append(path_point)
        self.path_pub.publish(self.path)

    # REQUIREMENT 2 (camera info to extract intrisic parameters matrik K from lab 9)
    def callback_cam_info(self, camera_info):
        self.K = np.array(camera_info.k).reshape([3, 3])

    # REQUIREMENT 2 (transformations.py user, get transformation from camera to world frame)
    def callback_odom(self, odometry):
        self.transform_cam_to_world = trans.msg_to_se3(odometry.pose.pose)
        
    # REQUIREMENT 2 (callback depth from lab 9)
    def callback_depth(self, depth_image):
        self.get_logger().info('[Image Processing] callback_depth')
        self.depth_frame = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")

    # REQUIREMENT 2 (callback colour from lab 9) Split between this and beacon_process so self.mask doesnt get overriden
    def callback_colour(self, colour_image):
        self.get_logger().info('[Image Processing] callback_colour')
        self.colour_frame = self.bridge.imgmsg_to_cv2(colour_image, "bgr8")  
        blurred = cv2.GaussianBlur(self.colour_frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        self.beacon_process(hsv)

    def beacon_process(self, hsv):
        colour_lower_Red = (0, 90, 100)
        colour_upper_Red = (0, 255, 200)
        colour_lower_yellow = (20, 0, 50)
        colour_upper_yellow = (40, 255, 255)
        colour_lower_blue = (70, 130, 50)
        colour_upper_blue = (130, 255, 120)
        colour_lower_green = (30, 40, 45)
        colour_upper_green = (70, 255, 255)

        detected_colors = [] # Store detected colors and their positions
        combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)  # this mask help combine all colour masks (dan help)
    
        # RED
        mask_red = cv2.inRange(hsv, colour_lower_Red, colour_upper_Red) # binary mask for red
        mask_red = cv2.erode(mask_red, None, iterations=2)
        mask_red = cv2.dilate(mask_red, None, iterations=2)
        combined_mask = cv2.bitwise_or(combined_mask, mask_red)  # Combined with main mask so red is seen via bitwise. Takes pixel vales from 2 pics and stores both in 1 pic
        contours = cv2.findContours(mask_red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea) # get largest red contour
            x, y, w, h = cv2.boundingRect(largest_contour) # make boudning rectangle
            center_y = y + h//2 # get y centre for red region
            center_x = x + w//2 # get x centre for red
            detected_colors.append((center_y, "red", center_x, center_y)) # append list
            self.colour_frame = cv2.rectangle(self.colour_frame, (x, y), (x+w, y+h), (0, 0, 255), 2) # draw bound

        # YELLOW
        mask_yellow = cv2.inRange(hsv, colour_lower_yellow, colour_upper_yellow)
        mask_yellow = cv2.erode(mask_yellow, None, iterations=2)
        mask_yellow = cv2.dilate(mask_yellow, None, iterations=2)
        combined_mask = cv2.bitwise_or(combined_mask, mask_yellow) # Combined with main mask so yellow is seen
        contours = cv2.findContours(mask_yellow.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_y = y + h//2
            center_x = x + w//2
            detected_colors.append((center_y, "yellow", center_x, center_y))
            self.colour_frame = cv2.rectangle(self.colour_frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
    
        # BLUE
        mask_blue = cv2.inRange(hsv, colour_lower_blue, colour_upper_blue)
        mask_blue = cv2.erode(mask_blue, None, iterations=2)
        mask_blue = cv2.dilate(mask_blue, None, iterations=2)
        combined_mask = cv2.bitwise_or(combined_mask, mask_blue) # Combined with main mask so blue is seen
        contours = cv2.findContours(mask_blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_y = y + h//2
            center_x = x + w//2
            detected_colors.append((center_y, "blue", center_x, center_y))
            self.colour_frame = cv2.rectangle(self.colour_frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
        
        # GREEN
        mask_green = cv2.inRange(hsv, colour_lower_green, colour_upper_green)
        mask_green = cv2.erode(mask_green, None, iterations=2)
        mask_green = cv2.dilate(mask_green, None, iterations=2)
        combined_mask = cv2.bitwise_or(combined_mask, mask_green) # Combined with main mask so green is seen
        contours = cv2.findContours(mask_green.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_y = y + h//2
            center_x = x + w//2
            detected_colors.append((center_y, "green", center_x, center_y))
            self.colour_frame = cv2.rectangle(self.colour_frame, (x, y), (x+w, y+h), (0, 0, 255), 2)

        # Update the mask to have all colours together
        self.mask = combined_mask
    
        if len(detected_colors) == 2: # if we have exactly two colors detected 
            detected_colors.sort() # Sort by y position to determine top and bottom colors
            top_color = detected_colors[0] # top colour
            bottom_color = detected_colors[1] # bottom colour
        
            x_center = (top_color[2] + bottom_color[2]) // 2 # Calculate center points for top and bottom colour
            y_center = (top_color[3] + bottom_color[3]) // 2
        
            # Display beacon info top and bottom colours
            cv2.putText(self.colour_frame, f"Top: {top_color[1]}, Bottom: {bottom_color[1]}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Calculate 3D position (lab 9)
            if np.any(self.K) and np.any(self.transform_cam_to_world) and np.any(self.depth_frame):
                    depth = self.depth_frame[y_center, x_center]
                    p_h = np.array([[x_center], [y_center], [1]])  # the homo-vector
                    p3d = depth * np.matmul(np.linalg.inv(self.K), p_h)
                    p3d_h = np.array([[p3d[2][0]], [-p3d[0][0]], [-p3d[1][0]], [1]])  # Note axis swap
                    p3d_w_h = np.matmul(self.transform_cam_to_world, p3d_h) # transform to world coordinates
                    p3d_w = np.array([[p3d_w_h[0][0]/p3d_w_h[3][0]],[p3d_w_h[1][0]/p3d_w_h[3][0]],[p3d_w_h[2][0]/p3d_w_h[3][0]]])
            
                    # Print 3D location
                    self.get_logger().info(f"Beacon detected at: x={p3d_w[0][0]:.2f}, y={p3d_w[1][0]:.2f}, z={p3d_w[2][0]:.2f}")
            
                    # Create marker (lab 9)
                    marker = Marker() # Marker imported with MarkerArray
                    marker.id = self.marker_id
                    self.marker_id += 1
                    marker.header.frame_id = 'map'
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.type = Marker.SPHERE # make markers sphere shaped
                    marker.action = Marker.ADD # added cause markers weren't showing up
                    mk_point = Point()
                    mk_quat = Quaternion()
                    mk_pose = Pose()
        
                    mk_point.x = float(p3d_w[0][0]) #used my values
                    mk_point.y = float(p3d_w[1][0])
                    mk_point.z = float(p3d_w[2][0])
                
                    mk_quat.x, mk_quat.y, mk_quat.z, mk_quat.w = 0.0, 1.0, 0.0, 1.0
                    mk_pose.position, mk_pose.orientation = mk_point, mk_quat
                    marker.pose = mk_pose
                    mk_scale = Vector3()
                    mk_scale.x, mk_scale.y, mk_scale.z = 0.1, 0.1, 0.1 # Choose an appropriate scale
                    marker.scale = mk_scale
                    mk_colour = ColorRGBA()
                    mk_colour.r, mk_colour.g, mk_colour.b, mk_colour.a = 0.0, 1.0, 0.0, 1.0 # Choose an RGBA colour (needs fixing)
                    marker.color = mk_colour
                    self.beacon_array.markers.append(marker)
                    self.beacon_markers.publish(self.beacon_array)

                    # SEND INFO TO BEACON.MSG
                    beacon_message = Beacon()
                    beacon_message.header.stamp = self.get_clock().now().to_msg()
                    beacon_message.header.frame_id = 'map'
                    beacon_message.seq = marker.id
                    beacon_message.top = top_color
                    beacon_message.bottom = bottom_color
                    beacon_message.position.x = float(p3d_w[0][0])
                    beacon_message.position.y = float(p3d_w[1][0])
                    beacon_message.position.z = float(p3d_w[2][0])
                    self.beacon_pub.publish(beacon_message)

    # REQUIREMENT 2 (loop to show displays from lab 9)
    def loop(self):
        if np.any(self.colour_frame) and np.any(self.depth_frame):
            cv2.imshow('Colour Image', self.colour_frame)
            cv2.imshow('Depth_Image', self.depth_frame)
            cv2.imshow('Masked Image', self.mask)
            resp = cv2.waitKey(80)
            if resp == ord('c'):
                self.get_logger().info('[Image Processing] Saving colour')
                cv2.imwrite('colour{}.png'.format(self.num_colour_images), self.colour_frame)
                self.num_colour_images += 1
            if resp == ord('d'):
                self.get_logger().info('[Image Processing] Saving depth')
                cv2.imwrite('depth_{}.png'.format(self.num_depth_images), self.depth_frame)
                self.num_depth_images += 1

def main(args=None):
    rclpy.init(args=args)
    mn = my_node() # unchanged
    rclpy.spin(mn)

if __name__ == '__main__':
    main()