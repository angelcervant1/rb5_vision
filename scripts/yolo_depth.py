import multiprocessing as mp
import time
from yoloModel import yolo_main
from depthEstimator import depth_main

def run_yolo(queue):
    print("Starting YOLO detection...")

    while(True):
        annotaed_frame, centroids = yolo_main()    
        if centroids and annotaed_frame is not None:
            queue.put((annotaed_frame, centroids))
        time.sleep(0.1)  

def run_depth_estimator(queue):
    print("Starting Depth Estimator...")
    
    while True:
        if not queue.empty():
            annotaed_frame, centroids = queue.get()
            depth_main(centroids, annotaed_frame)
    

    
if __name__ == "__main__":
    queue = mp.Queue()
    yolo_process = mp.Process(target=run_yolo, args=(queue,))
    depth_process = mp.Process(target=run_depth_estimator, args=(queue,))
    
    yolo_process.start()
    depth_process.start()

    try:
        yolo_process.join()
        depth_process.join()
    except KeyboardInterrupt:
        print("Stopping processes...")
        yolo_process.terminate()
        depth_process.terminate()
        yolo_process.join()
        depth_process.join()
