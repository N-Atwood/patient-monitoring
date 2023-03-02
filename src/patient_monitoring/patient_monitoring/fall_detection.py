# -*- coding: utf-8 -*-
"""
Created on Tue Feb 21 17:17:13 2023

@author: Aditya, Null
"""


import cv2
import mediapipe as mp
import numpy as np
import math
import time
import csv
import os


# For webcam input:
cap = cv2.VideoCapture(0)
_, frame = cap.read()
h, w, c = frame.shape

# intialise variables
start_time = time.time()
interval = 1

prev_centroid = None
prev_time = None
prev_w = None
prev_h = None

folder_path = 'C:/Users/user'
file_name = 'pose_csv_file.csv'
file_path = os.path.join(folder_path, file_name)

class FallDetector:
    def __init__(self):
        self.node = rclpy.create_node('patient_subscriber')
        self.subscription = self.node.create_subscription(
            Person,
            'person',
            self.person_callback,
            1)

        self.publisher = self.node.create_publisher(Bool, 'fall', 10)

    def person_callback(self,msg_in):
        current_time = time.time()

        if msg_in.x == -1:
            print("Ignoring frame with no person detected.")
        else:
            # Extract the coordinates of the shoulder, hip, and mid-point landmarks
            shoulder_left = (landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x, landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y)
            shoulder_right = (landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y)
            hip_left = (landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].x, landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].y,landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].z)
            hip_right = (landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].y,landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].z)
            mid_point = ((shoulder_left[0] + shoulder_right[0])/2, (shoulder_left[1] + shoulder_right[1])/2)
        
            # Calculate the centroid of the person
            centroid = ((shoulder_left[0] + shoulder_right[0] + mid_point[0] + hip_left[0] + hip_right[0])/5, (shoulder_left[1] + shoulder_right[1] + mid_point[1] + hip_left[1] + hip_right[1])/40*9,(hip_left[2]+hip_right[2])/2)
            # Draw a circle at the centroid of the person
            # cv2.circle(image, (int(centroid[0]*frame.shape[1]), int(centroid[1]*frame.shape[0])), 5, (0, 0, 255), -1)
        
            # fall detection
            base = (int(centroid[0]*frame.shape[1]) ,int(y_max))
            cv2.circle(image, base, 10, (0, 0, 255), -1)
            if (current_time - start_time) >= interval:
                if prev_centroid is not None:
                    prev_distance = abs(int(prev_centroid[1]*frame.shape[0]) - y_max)
                    distance = abs(int(centroid[1]*frame.shape[0]) - y_max)
                    # print("prev",round(prev_distance,2))
                    # print("curr",round(distance,2))
                    if prev_distance != 0:
                        change = distance/prev_distance     #IDEA: multiply by (current_time - start_time) to avoid false positives after losing track of someone
                        if change<0.45:     #IDEA: ... and prev_h/prev_w > 0.20 could avoid false positives when already lying down
                            print("fall")
                            # This will make it easy to create a separate function to react to falls if needed
                            self.publisher.publish(True)
                
                prev_centroid = centroid
                start_time = current_time
                prev_h = msg_in.h
                prev_w = msg_in.w
    
def main():
    rclpy.init()
    patient_sub = PatientSubscriber()
    rclpy.spin(patient_sub.node)

if __name__ == '__main__':
    main()

    # Not sure how/if to integrate this portion:
    #if cv2.waitKey(5) & 0xFF == 27:
    #    cap.release()
    #    cv2.destroyAllWindows()
