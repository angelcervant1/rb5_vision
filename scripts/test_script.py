
import cv2
from ultralytics import YOLO

import time
frame_rate = 5
prev = 0

# video_capture = cv2.VideoCapture("rtsp://192.168.0.170:8901/live")
video_capture = cv2.VideoCapture(0)

model = YOLO('/home/angelcervant/rb5_vision/src/target_detection_and_localisation/scripts/model/yolov8m.pt')

while(True):
    
    time_elapsed = time.time() - prev
    ret, frame = video_capture.read()

    if time_elapsed>(1/frame_rate):
        
        prev = time.time()
        cv2.imshow('original_video', frame)
        results = model.predict(frame, conf=0.7)
        # print(results)
        for result in results:
            for box in result.boxes:
        # Extract the bounding box in the format (x1, y1, x2, y2)
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                annotaed_frame = results[0].plot()
                # Convert to (x, y) format for top-left and bottom-right corners
                x_min, y_min = int(x1), int(y1)  # Top-left corner
                x_max, y_max = int(x2), int(y2)  # Bottom-right corner
                cx = (x_min + x_max) / 2
                cy = (y_min + y_max) / 2
                # Draw bounding box on the image (optional)
                # cv2.rectangle(annotaed_frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

                # Print or use the coordinates as needed
                print(f"Centroid: ({cx}, {cy}) ")
        
        cv2.imshow('drone_video', annotaed_frame)
        cv2.waitKey(1)
