#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import time
import numpy as np
from depth_estimator.msg import ImageWithCentroids
from geometry_msgs.msg import Point

frame_rate = 30
prev = time.time()  

class YoloDetectionModel():
    def __init__(self):
        # Callback from the tracking image topic 
        rospy.init_node("object_detection")
        self.model = YOLO('/home/angelcervant/rb5_vision/src/depth_estimator/scripts/model/yolov8m.pt')
        self.centroids = []
        self.prediction = None
        self.pub = rospy.Publisher("/yolo_prediction", ImageWithCentroids, queue_size=100)    
        self.cvb = CvBridge()
        self.sub = rospy.Subscriber("/tracking", Image, self.img_cb)
        self.img = None
        self.yolo_msg = ImageWithCentroids()
        self.last_time = rospy.Time.now()
        self.rate = rospy.Rate(10)
        
    def img_cb(self, msg):
        self.img = self.cvb.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # rospy.loginfo(f"Shape: {self.img.shape}")

    def get_centroids(self) -> list:
        return self.centroids

    def yolo_main(self):        
        curr_time = rospy.Time.now()
        self.dt = (curr_time - self.last_time).to_sec() 
        # rospy.loginfo(f"current dt: {self.dt}")
       
        if self.dt>(1/frame_rate) and self.img is not None:
            self.last_time = curr_time
            self.yolo_msg.header.stamp = curr_time
            self.yolo_msg.centroids = []
            try:
                results = self.model.predict(self.img, conf=0.6)
                for result in results:
                    for box in result.boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        self.prediction = results[0].plot(show=False)
                        x_min, y_min = int(x1), int(y1)  # Top-left corner
                        x_max, y_max = int(x2), int(y2)  # Bottom-right corner
                        cx = (x_min + x_max) / 2
                        cy = (y_min + y_max) / 2
                        centroid = Point()
                        centroid.x = cx
                        centroid.y = cy
                        centroid.z = 0  
                        self.yolo_msg.centroids.append(centroid)
                        self.centroids.append((cx, cy))
                        rospy.loginfo(f"Centroid: ({cx}, {cy}) ")
                        cv2.circle(self.prediction, (int(cx), int(cy)), 5, (0, 255, 0), -1)
                        # self.yolo_msg.image = self.cvb.cv2_to_imgmsg(self.prediction, encoding="bgr8")
                        self.yolo_msg.image = self.cvb.cv2_to_imgmsg(self.img, encoding="bgr8")
                        self.pub.publish(self.yolo_msg)
                        if self.prediction is not None:
                            cv2.imshow('drone_video', self.prediction)
                cv2.waitKey(1)
            except Exception as e:
                print(f"Error during prediction: {e}") 
        else:
            rospy.logwarn("Not image received yet..")
            

    def run(self):        
        self.yolo_main()
        self.rate.sleep()
    
if __name__ == "__main__":
   yolo = YoloDetectionModel()
   try: 
    while not rospy.is_shutdown():
        yolo.run()
    cv2.destroyAllWindows()
   except rospy.ROSInterruptException as e:
       print(f"Terminating Process: {e}")
    