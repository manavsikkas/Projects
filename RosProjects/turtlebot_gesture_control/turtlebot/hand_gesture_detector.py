#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
import cv2
import mediapipe as mp
import numpy as np


class HandGestureDetector(Node):
    def __init__(self):
        super().__init__('hand_gesture_detector')
        
        self.gesture_pub = self.create_publisher(String, 'hand_gesture', 10)
        # Publish TwistStamped for TurtleBot3 bridge compatibility
        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils
        
        self.cap = cv2.VideoCapture(0)
        
        self.timer = self.create_timer(0.033, self.detect_gesture)
        
        self.get_logger().info('Hand Gesture Detector initialized for TurtleBot3 Waffle')
        self.get_logger().info('Gestures: Index=Forward, Peace=Backward, Thumb=Left/Right, Open=Stop')
    
    def detect_gesture(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)
        
        gesture = "STOP"
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"
        
        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            
            self.mp_drawing.draw_landmarks(
                frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
            )
            
            gesture = self.classify_gesture(hand_landmarks)
            
            # TurtleBot3 Waffle speed limits
            # Max linear: 0.26 m/s, Max angular: 1.82 rad/s
            if gesture == "FORWARD":
                twist_msg.twist.linear.x = 0.2  # Safe forward speed
                twist_msg.twist.angular.z = 0.0
            elif gesture == "BACKWARD":
                twist_msg.twist.linear.x = -0.2  # Safe backward speed
                twist_msg.twist.angular.z = 0.0
            elif gesture == "LEFT":
                twist_msg.twist.linear.x = 0.0
                twist_msg.twist.angular.z = 0.5  # Moderate turn speed
            elif gesture == "RIGHT":
                twist_msg.twist.linear.x = 0.0
                twist_msg.twist.angular.z = -0.5  # Moderate turn speed
            elif gesture == "STOP":
                twist_msg.twist.linear.x = 0.0
                twist_msg.twist.angular.z = 0.0
        
        gesture_msg = String()
        gesture_msg.data = gesture
        self.gesture_pub.publish(gesture_msg)
        self.cmd_vel_pub.publish(twist_msg)
        
        # Display information on frame
        cv2.putText(frame, "TurtleBot3 Waffle Control", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(frame, f"Gesture: {gesture}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, f"Linear: {twist_msg.twist.linear.x:.2f} m/s", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)
        cv2.putText(frame, f"Angular: {twist_msg.twist.angular.z:.2f} rad/s", (10, 115),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)
        cv2.putText(frame, "Press 'q' to quit", (10, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.imshow('TurtleBot3 Gesture Control', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()
    
    def classify_gesture(self, hand_landmarks):
        landmarks = hand_landmarks.landmark
        
        thumb_tip = landmarks[4]
        thumb_ip = landmarks[3]
        index_tip = landmarks[8]
        index_pip = landmarks[6]
        middle_tip = landmarks[12]
        middle_pip = landmarks[10]
        ring_tip = landmarks[16]
        ring_pip = landmarks[14]
        pinky_tip = landmarks[20]
        pinky_pip = landmarks[18]
        
        fingers_up = []
        
        if thumb_tip.x > thumb_ip.x:
            fingers_up.append(1)
        else:
            fingers_up.append(0)
        
        for tip, pip in [(index_tip, index_pip), (middle_tip, middle_pip),
                        (ring_tip, ring_pip), (pinky_tip, pinky_pip)]:
            if tip.y < pip.y:
                fingers_up.append(1)
            else:
                fingers_up.append(0)
        
        total_fingers = sum(fingers_up)
        
        if total_fingers == 5:
            return "STOP"
        elif total_fingers == 1 and fingers_up[1] == 1:
            return "FORWARD"
        elif total_fingers == 2 and fingers_up[1] == 1 and fingers_up[2] == 1:
            return "BACKWARD"
        elif total_fingers == 1 and fingers_up[0] == 1:
            return "LEFT"
        elif total_fingers == 1 and fingers_up[4] == 1:
            return "RIGHT"
        else:
            return "STOP"
    
    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = HandGestureDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()