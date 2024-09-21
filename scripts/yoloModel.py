
import cv2
from ultralytics import YOLO
import time
frame_rate = 5
prev = time.time()

# video_capture = cv2.VideoCapture("rtsp://192.168.0.170:8901/live")
video_capture = cv2.VideoCapture(0)

model = YOLO('/home/angelcervant/rb5_vision/src/target_detection_and_localisation/scripts/model/yolov8m.pt')
centroids = []
annotaed_frame = None

def get_centroids() -> list:
    return centroids

def yolo_main():
    global prev
    global annotaed_frame

    ret, frame = video_capture.read()
    
    if ret:
        time_elapsed = time.time() - prev

    if time_elapsed>(1/frame_rate):
        
        prev = time.time()
        # cv2.imshow('original_video', frame)
        try:
            results = model.predict(frame, conf=0.6)
        except Exception as e:
            print(f"Error during prediction: {e}")
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                annotaed_frame = results[0].plot()
                
                x_min, y_min = int(x1), int(y1)  # Top-left corner
                x_max, y_max = int(x2), int(y2)  # Bottom-right corner
                cx = (x_min + x_max) / 2
                cy = (y_min + y_max) / 2
                centroids.append((cx, cy))
                print(f"Centroid: ({cx}, {cy}) ")
                cv2.circle(annotaed_frame, (int(cx), int(cy)), 5, (0, 255, 0), -1)
        cv2.imshow('drone_video', annotaed_frame)
        cv2.waitKey(1)
        
    return annotaed_frame, centroids

    
