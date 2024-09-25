Header header                        # Includes a timestamp and frame ID

sensor_msgs/Image image              # The image frame

geometry_msgs/Point[] centroids      # List of centroids
string[] class_labels                # Class labels (e.g., 'person', 'car')
float32[] confidence_scores          # Confidence scores for each detected object
int32[] object_ids                   # Unique object IDs for tracking (optional)
