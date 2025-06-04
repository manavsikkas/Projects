import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion, Vector3
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String, ColorRGBA
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

import cv2
import imutils
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from . import tf_transformations as trans
from beacon.msg import Beacon



class my_node(Node):
    def __init__(self):
        
        # DEMONSTRATION 1

        # Initialise subs, pubs, service calls, path object
        super().__init__('demo1_node')
        qos_policy = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=1)      

        # Publisher and subcriber for Requirement 3 
        self.command_pub =self.create_publisher(String,'/start_explore', self.callback_trigger,1)
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.callback_map, qos_policy)       
        self.pubMap = self.create_publisher(OccupancyGrid, '/ecte477/map', qos_policy)

        # Subscriber for Requirement 4
        self.stack_points_subscriber = self.create_subscription(MarkerArray, '/stack_points', self.callback_stack_points, 10)

        # Publishers for Requirement 6
        self.path_pub = self.create_publisher(Path, '/ecte477/path', 10)

        # Create Path objects for Requirement 6
        self.path = Path()
        self.path.header.frame_id = 'odom'

        # Subscriber for Requirement 6
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.callback_odom, 10)

        # IMAGE PROCESSING
        self.bridge = CvBridge()
        self.colour_frame = None
        self.depth_frame = None
        self.num_colour_images = 0
        self.num_depth_images = 0

        qos_policy = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE, reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=5)

        self.subscriber_colour = self.create_subscription(Image, '/intel_realsense_r200_depth/image_raw', self.callback_colour, qos_policy)

        self.subscriber_depth = self.create_subscription(Image, '/intel_realsense_r200_depth/depth/image_raw', self.callback_depth, qos_policy)

        self.K = None
        self.subscriber_camera_info = self.create_subscription(CameraInfo, '/intel_realsense_r200_depth/camera_info', self.callback_cam_info, qos_policy)

        self.transform_cam_to_world = None
        self.subscriber_odom = self.create_subscription(Odometry, '/odom', self.callback_odom, 5)

        # Markers
        self.beacon_markers = self.create_publisher(MarkerArray,'/ecte477/beacon_markers',5)
        self.beacon_array = MarkerArray()

        self.declare_parameters(
            namespace='',
            parameters=[
            ('beacons.id', rclpy.Parameter.Type.INTEGER_ARRAY),
            ('beacons.top', rclpy.Parameter.Type.STRING_ARRAY),
            ('beacons.bottom', rclpy.Parameter.Type.STRING_ARRAY)
            ]
        )

        self.beacons = (self.get_parameter('beacons.id').value, self.get_parameter('beacons.top').value, self.get_parameter('beacons.bottom').value)
       
        self.beaconscustommessage= self.create_publisher(Beacon, '/ecte477/beacons', 5)

        while rclpy.ok():
            self.loop()
            rclpy.spin_once(self, timeout_sec=1.0)

    def callback_trigger(self,msg):
        if msg.data=='start':
            self.started=True

    # Method to publish map to /ecte477/map topic (Requirement 3)
    def callback_map(self, data):
        self.pubMap.publish(data)

    # Method for requirement 4
    def callback_stack_points(self, stack_points):  
        if len(stack_points.markers) == 0:
            self.get_logger().info('EXPLORATION IS FINISHED!')      # Detecting when exploration is finished (Requirement 4)

    # Method to publish paths of the robot (Requirement 6)
    def callback_odom(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

    # IMAGE PROCESSING (DEMO 2 Requirement 2)
    def callback_colour(self, colour_image):
        marker = Marker()
        self.get_logger().info('[Image Processing] callback_colour')
        self.colour_frame = self.bridge.imgmsg_to_cv2(colour_image, "bgr8")
        blurred = cv2.GaussianBlur(self.colour_frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        colour_yellow_lower = (18,135,101)
        colour_yellow_upper = (41,255,205)

        colour_red_lower = (0,90,93)
        colour_red_upper = (0,255,205)

        colour_blue_lower = (69,130,49)
        colour_blue_upper = (130,255,222)

        colour_green_lower = (34,51,49)
        colour_green_upper = (74,255,255)

        self.foundYellow = False
        self.mask_yellow = cv2.inRange(hsv, colour_yellow_lower, colour_yellow_upper)
        #mask_yellow = cv2.erode(mask_yellow, None, iterations=2)
        #mask_yellow = cv2.dilate(mask_yellow, None, iterations=2)
        contours_yellow = cv2.findContours(self.mask_yellow.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_yellow = imutils.grab_contours(contours_yellow)
        if len(contours_yellow) != 0:
            largest_contour_yellow = max(contours_yellow, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour_yellow)
            self.colour_frame = cv2.rectangle(self.colour_frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
            self.foundYellow = True

        self.foundRed = False
        self.mask_red = cv2.inRange(hsv, colour_red_lower, colour_red_upper)
        #mask_red = cv2.erode(mask_red, None, iterations=2)
        #mask_red = cv2.dilate(mask_red, None, iterations=2)
        contours_red = cv2.findContours(self.mask_red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_red = imutils.grab_contours(contours_red)
        if len(contours_red) != 0:
            largest_contour_red = max(contours_red, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour_red)
            self.colour_frame = cv2.rectangle(self.colour_frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
            self.foundRed = True

        self.foundBlue = False
        self.mask_blue = cv2.inRange(hsv, colour_blue_lower, colour_blue_upper)
        #mask_yellow = cv2.erode(mask_yellow, None, iterations=2)
        #mask_yellow = cv2.dilate(mask_yellow, None, iterations=2)
        contours_blue = cv2.findContours(self.mask_blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_blue = imutils.grab_contours(contours_blue)
        if len(contours_blue) != 0:
            largest_contour_blue = max(contours_blue, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour_blue)
            self.colour_frame = cv2.rectangle(self.colour_frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
            self.foundBlue = True

        self.foundGreen = False
        self.mask_green = cv2.inRange(hsv, colour_green_lower, colour_green_upper)
        #mask_red = cv2.erode(mask_red, None, iterations=2)
        #mask_red = cv2.dilate(mask_red, None, iterations=2)
        contours_green = cv2.findContours(self.mask_green.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_green = imutils.grab_contours(contours_green)
        if len(contours_green) != 0:
            largest_contour_green = max(contours_green, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour_green)
            self.colour_frame = cv2.rectangle(self.colour_frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
            self.foundGreen = True
        
        testvariable = 5

        


        if self.foundRed and self.foundYellow:
            marker.id = self.beacons[0][0]
            testvariable = 0
            topcolour = "red"
            bottomcolour = "yellow"
            colour_r = 1.0
            colour_g = 0.0
            colour_b = 1.0
            colour_a = 1.0
        elif self.foundBlue and self.foundRed:
            marker.id = self.beacons[0][1]
            testvariable = 1
            topcolour = "blue"
            bottomcolour = "red"
            colour_r = 0.0
            colour_g = 1.0
            colour_b = 1.0
            colour_a = 1.0            
        elif self.foundGreen and self.foundYellow:
            marker.id = self.beacons[0][2]
            testvariable = 2
            topcolour = "green"
            bottomcolour = "yellow"
            colour_r = 1.0
            colour_g = 1.0
            colour_b = 0.0
            colour_a = 1.0            
        elif self.foundBlue and self.foundGreen:
            marker.id = self.beacons[0][3]  
            testvariable = 3
            topcolour = "blue"
            bottomcolour = "green"   
            colour_r = 1.0
            colour_g = 0.0
            colour_b = 0.0
            colour_a = 1.0                   

        if np.any(self.K) and np.any(self.transform_cam_to_world) and np.any(self.depth_frame) and (self.foundGreen or self.foundBlue or self.foundRed or self.foundYellow) and testvariable != 5 :
            depth = self.depth_frame[y, x]
            p_h = np.array([[x], [y], [1]]) # homogeneous vector
            p3d = depth * np.matmul(np.linalg.inv(self.K), p_h)
            p3d_h = np.array([[p3d[2][0]], [-p3d[0][0]], [-p3d[1][0]], [1]]) # Note
            p3d_w_h = np.matmul(self.transform_cam_to_world, p3d_h)
            p3d_w = np.array([[p3d_w_h[0][0]/p3d_w_h[3][0]], [p3d_w_h[1][0]/p3d_w_h[3][0]], [p3d_w_h[2][0]/p3d_w_h[3][0]]])
            self.get_logger().info(f'CALLER DETECTED AT 3D LOCATION: {p3d_w}')
            point_x = float(p3d_w[0])
            point_y = float(p3d_w[1]) 
            

            
            
            #   Choose a unique ID, maybe from param file
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            mk_point = Point()
            mk_quat = Quaternion()
            mk_pose = Pose()
            mk_point.x, mk_point.y, mk_point.z = point_x, point_y, 0.0
            # Use your values
            mk_quat.x, mk_quat.y, mk_quat.z, mk_quat.w = 0.0, 1.0, 0.0, 1.0
            mk_pose.position, mk_pose.orientation = mk_point, mk_quat
            marker.pose = mk_pose
            mk_scale = Vector3()
            mk_scale.x, mk_scale.y, mk_scale.z = 0.1, 0.1, 0.1
            # Choose an appropriate scale
            marker.scale = mk_scale
            mk_colour = ColorRGBA()
            mk_colour.r, mk_colour.g,mk_colour.b, mk_colour.a = colour_r, colour_g, colour_b, colour_a
            # Choose an RGBA colour
            marker.color = mk_colour
            self.beacon_array.markers.append(marker)
            self.beacon_markers.publish(self.beacon_array)

            testvariable = 5

            beacon_message = Beacon()
            beacon_message.header.frame_id = 'map'
            beacon_message.header.stamp = self.get_clock().now().to_msg()
            beacon_message.seq = marker.id
            beacon_message.top = topcolour
            beacon_message.bottom = bottomcolour
            beacon_message.position.x,beacon_message.position.y,beacon_message.position.z = float(p3d_w[0]), float(p3d_w[1]), float(p3d_w[2])
            self.beaconscustommessage.publish(beacon_message)


        

    def callback_depth(self, depth_image):
        self.get_logger().info('[Image Processing] callback_depth')
        self.depth_frame = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")

    def loop(self):
        if np.any(self.colour_frame) and np.any(self.depth_frame):
            cv2.imshow('Colour Image', self.colour_frame)
            cv2.imshow('Depth_Image', self.depth_frame)
            cv2.imshow('Masked Image', self.mask_red)
            resp = cv2.waitKey(80)
            if resp == ord('c'):
                self.get_logger().info('[Image Processing] Saving colour')
                cv2.imwrite('colour{}.png'.format(self.num_colour_images), self.colour_frame)
                self.num_colour_images += 1
            if resp == ord('d'):
                self.get_logger().info('[Image Processing] Saving depth')
                cv2.imwrite('depth_{}.png'.format(self.num_depth_images), self.depth_frame)
                self.num_depth_images += 1

    def callback_cam_info(self, camera_info):
        self.K = np.array(camera_info.k).reshape([3,3])

    def callback_odom(self, odometry):
        self.transform_cam_to_world = trans.msg_to_se3(odometry.pose.pose)

# Main function
def main(args=None):
    rclpy.init(args=args)
    mn = my_node()
    rclpy.spin(mn)

if __name__ == '__main__':
    main()
