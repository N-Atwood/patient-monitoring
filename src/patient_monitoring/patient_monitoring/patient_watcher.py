"""
patient_watcher.py
authors: aadi, jack, null
"""

import rclpy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from my_robot_interfaces.msg import Person
from numpy import asarray
import mediapipe as mp
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from message_filters import ApproximateTimeSynchronizer, Subscriber
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from message_filters import Subscriber, TimeSynchronizer

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        self.bridge = CvBridge()
        self.publisher1 = self.create_publisher(Person, 'person', 1)
        
        color_sub = Subscriber(self, Image, '/camera/color/image_rect_raw')
        depth_sub = Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw')

        ts = TimeSynchronizer([color_sub, depth_sub], 10)
        ts.registerCallback(self.callback)

    def callback(self, msg: Image, msg_depth: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(msg_depth, msg_depth.encoding)

        h, w, c = cv_image.shape
        
        person_detected = Person()
        person_detected.x = -1
        person_detected.y = -1
        person_detected.w = -1
        person_detected.h = -1
        person_detected.depth = 0.

        person_detected.landmarks = []
        
        # Detect objects in the image using mediapipe
        with mp_pose.Pose(
                min_detection_confidence=0.8,   #TODO: Adjust this until false positives are eliminated and false negatives are low
                min_tracking_confidence=0.8) as pose:

               	results = pose.process(cv_image)
               	landmarks=results.pose_landmarks

               	if (landmarks):
                    landmarks = landmarks.landmark
                    x_max = -1
                    y_max = -1
                    x_min = w
                    y_min = h
                    for i in range(len(landmarks)):
                        person_detected.landmarks[i].x = landmarks[i].x
                        person_detected.landmarks[i].y = landmarks[i].y
                        person_detected.landmarks[i].z = landmarks[i].z
                        person_detected.landmarks[i].visibility = landmarks[i].visibility
                        x, y = int(landmarks[i].x * w), int(landmarks[i].y *h)
                        if 0 <= x and x < w:
                            if x > x_max:
                                x_max = x
                            if x < x_min:
                                x_min = x
                        if 0 <= y and y < h:
                            if y > y_max:
                                y_max = y
                            if y < y_min:
                                y_min = y

                    # depth_x and depth_y are coordinates we want to get depth from
                    #depth_x, depth_y
                    # person_detected.depth is the actual depth
                    person_detected.depth = 0.
                    
                    #print(x_min, x_max, y_min, y_max)
                    person_detected.x = (x_max + x_min)//2
                    person_detected.y = (y_max + y_min)//2
                    person_detected.w = x_max - x_min + 1
                    person_detected.h = y_max - y_min + 1
                    if(person_detected.x < 424 and person_detected.y < 240):
                        person_detected.depth = float(depth_image[person_detected.y, person_detected.x])
                        line = '\rPerson detected! (%3d, %3d): %7.1f(mm).' % (person_detected.x, person_detected.y, depth_image[person_detected.y, person_detected.x])
                        print(line)

                    cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
               	    cv_image.flags.writeable = True
                    
               	mp_drawing.draw_landmarks(
                       cv_image,
                       results.pose_landmarks,
                       mp_pose.POSE_CONNECTIONS,
        			   landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
						 
                self.publisher1.publish(person_detected)
                #self.publisher2.publish(person_detected)
                
               	cv2.imshow('cv_image', cv_image)
               	cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    node = MyNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
