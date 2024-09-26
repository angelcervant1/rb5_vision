# Use the NVIDIA base image with CUDA support
FROM nvidia/cuda:12.1.0-base-ubuntu20.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV HOME=/ws/rb5_vision

# Install dependencies
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq wget curl git build-essential vim sudo lsb-release locales bash-completion

# Add ROS Noetic repository
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -k https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -

# Install ROS Noetic desktop and development tools
RUN apt-get update -q && \
    apt-get install -y ros-noetic-ros-base python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin-tools && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep and locale
RUN rosdep init && rosdep update
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8

# Install additional ROS dependencies (ensure Noetic versions)
RUN apt-get update && apt-get install -y \
    ros-noetic-cv-bridge \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

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
WORKDIR /ws/rb5_vision

# Upgrade pip

# Copy the requirements.txt file into the container
COPY requirements.txt /tmp/requirements.txt

# Copy the entire `rb5_vision` folder into the container
COPY . .

# Clean any existing build or devel directories
# Install Python dependencies from requirements.txt
RUN pip3 install -r /tmp/requirements.txt

# Ensure the script is executable (optional, in case it needs execution permission)
RUN chmod +x /ws/rb5_vision/src/depth_estimator/scripts/depthEstimator.py
RUN chmod +x /ws/rb5_vision/src/depth_estimator/scripts/yoloModel.py

# Build the Catkin workspace
RUN rm -rf build/ devel/

# Build the Catkin workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"


# Set up the environment for running commands
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /ws/rb5_vision/devel/setup.bash --extend" >> ~/.bashrc

CMD ["/bin/bash", "-c", "roslaunch depth_estimator depth_model.launch"]