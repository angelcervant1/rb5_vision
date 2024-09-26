from transformers import GLPNImageProcessor, GLPNForDepthEstimation

# Define the model name and local cache directory
model_name = "vinvino02/glpn-nyu"
cache_dir = "/ws/rb5_vision/src/depth_estimator/scripts/models"  # Define a custom cache directory if needed

# Download the processor and model
processor = GLPNImageProcessor.from_pretrained(model_name, cache_dir=cache_dir)
model = GLPNForDepthEstimation.from_pretrained(model_name, cache_dir=cache_dir)