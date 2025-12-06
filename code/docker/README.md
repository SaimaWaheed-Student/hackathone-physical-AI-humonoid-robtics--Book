# Docker Development Environment Setup

This directory contains the `Dockerfile` and associated scripts to set up a consistent development environment for the "Physical AI and Humanoid Robotics" book. This Docker image provides all the necessary tools and libraries, including Ubuntu 22.04, ROS 2 Humble, Gazebo, and NVIDIA container toolkit dependencies.

## 1. Build the Docker Image

Navigate to this directory (`code/docker`) in your terminal and build the Docker image:

```bash
docker build -t physical-ai-robotics:latest .
```

This process might take some time as it downloads and installs all the required software.

## 2. Run the Docker Container

Once the image is built, you can run a container from it. It's recommended to mount your local project directory into the container to easily access your code.

```bash
docker run -it --rm \
  --name physical-ai-robotics_container \
  --privileged \
  --net=host \
  -v "$(pwd)/../..:/workspace" \
  physical-ai-robotics:latest \
  bash
```

**Explanation of Flags:**
*   `-it`: Grants interactive access to the container and allocates a pseudo-TTY.
*   `--rm`: Automatically removes the container when it exits.
*   `--name physical-ai-robotics_container`: Assigns a readable name to your container.
*   `--privileged`: Gives the container elevated privileges, sometimes necessary for robotics simulations or hardware access.
*   `--net=host`: Allows the container to share the host's network stack, simplifying ROS 2 discovery.
*   `-v "$(pwd)/../..:/workspace"`: Mounts the parent directory of `code/docker` (i.e., your entire project root) to `/workspace` inside the container. This means any changes you make locally will be reflected inside the container, and vice-versa.
*   `physical-ai-robotics:latest`: Specifies the name and tag of the Docker image to use.
*   `bash`: Runs the bash shell inside the container.

**Important Note for NVIDIA GPU Users:**
If you plan to use NVIDIA Isaac Sim or other GPU-accelerated tools (covered in later modules), you need to ensure the NVIDIA Container Toolkit is correctly installed on your **host machine**. The `Dockerfile` includes commands to install the necessary drivers and toolkit *inside* the container, but your host system also needs to be configured.

After installing the NVIDIA Container Toolkit on your host, you would typically run the container with the `--gpus all` flag:

```bash
docker run -it --rm \
  --gpus all \
  --name physical-ai-robotics_container \
  --privileged \
  --net=host \
  -v "$(pwd)/../..:/workspace" \
  physical-ai-robotics:latest \
  bash
```

## 3. Inside the Container

Once inside the container, you will be in the `/workspace` directory. You can then source your ROS 2 environment and build your ROS 2 packages:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/code/module1
colcon build
source install/setup.bash
ros2 run pub_sub_py publisher
ros2 run pub_sub_py subscriber
```

## 4. Gazebo Fortress Installation Guide

Gazebo Fortress is already included in the Docker image via `ros-humble-gazebo-ros-pkgs`. Once you are inside the Docker container, you can launch Gazebo by simply sourcing your ROS 2 environment and running:

```bash
source /opt/ros/humble/setup.bash
gazeboo
```

You can also launch Gazebo worlds defined in ROS 2 launch files. Examples will be provided in Module 2.
