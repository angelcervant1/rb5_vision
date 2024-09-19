#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
# from std_msgs.msg import String
# from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
# import torch
import numpy as np

MODEL_PATH = "/home/angelcervant/rb5_vision/src/target_detection_and_localisation/scripts/model/yolov8m.pt"

print("Imports done, loading model...")

model = YOLO(MODEL_PATH)
 
print("Model loaded!")

def publish_results():
    pub = rospy.Publisher('results', Image, queue_size=10)
    rospy.init_node('results_node', anonymous=True)
    rate = rospy.Rate(1)

    video_capture = cv2.VideoCapture(0)

    rospy.loginfo("Camera feed accessed, publisher started...")

    while not rospy.is_shutdown():
        ret, frame = video_capture.read()

        if frame is None or frame.size == 0:
            print("Empty or invalid frame!")
        else:
            cv2.imshow('Video', frame)

        # results = model.predict(frame, conf=0.5)

        # print(results)

        # msg = str(results)

        # pub.publish(msg)
        rate.sleep()
  
if __name__ == '__main__':
    try:
        publish_results()
    except rospy.ROSInterruptException:
       pass
