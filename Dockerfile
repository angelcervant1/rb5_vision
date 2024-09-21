# Use the NVIDIA base image with CUDA support
FROM nvidia/cuda:12.1.0-base-ubuntu20.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV HOME=/ws/rb5_vision

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    python3-tk \
    git \
    curl \
    libgl1-mesa-glx \
    libglu1-mesa \  
    libxi-dev \
    libxmu-dev \
    libglib2.0-0 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /ws

# Upgrade pip
RUN python3 -m pip install --upgrade pip

# Copy the requirements.txt file into the container
COPY requirements.txt /tmp/requirements.txt

# Copy the entire `rb5_vision` folder into the container
COPY . /ws/rb5_vision

# Install Python dependencies from requirements.txt
RUN pip install -r /tmp/requirements.txt

# Ensure the script is executable (optional, in case it needs execution permission)
RUN chmod +x /ws/rb5_vision/scripts/depthEstimator.py

# Set the default command to run the depthEstimator.py script
CMD ["bash", "-c", "python3 /ws/rb5_vision/scripts/depthEstimator.py && python3 /ws/rb5_vision/scripts/test_script.py"]
