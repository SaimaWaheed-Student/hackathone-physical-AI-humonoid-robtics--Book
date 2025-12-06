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
gazebo
```

You can also launch Gazebo worlds defined in ROS 2 launch files. Examples will be provided in Module 2.

## 5. NVIDIA Isaac Sim Installation and Setup

NVIDIA Isaac Sim is a powerful robotics simulation platform built on Omniverse, enabling high-fidelity, physically accurate simulations for developing, testing, and training AI-powered robots. Setting it up within a Dockerized environment, especially for GPU acceleration, requires careful configuration.

**Important Host Requirements:**
*   **NVIDIA GPU:** An NVIDIA RTX series GPU is highly recommended for optimal performance.
*   **NVIDIA Drivers:** Ensure you have the latest proprietary NVIDIA drivers installed on your **host machine**.
*   **NVIDIA Container Toolkit:** This *must* be installed on your **host machine** to allow Docker containers to access your GPU. Refer to the official NVIDIA Container Toolkit documentation for installation on your specific Linux distribution: [https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
*   **Internet Connection:** Isaac Sim requires downloading substantial assets (several GBs).

### 5.1. Isaac Sim Installation Method

There are two primary ways to run Isaac Sim:
1.  **Natively on Host:** Download and install via NVIDIA Omniverse Launcher. This is often simpler for initial setup but might conflict with specific Docker setups.
2.  **Via Docker (Recommended for this Book):** Run Isaac Sim itself in a separate Docker container. This provides a clean, reproducible environment aligned with the rest of the book's setup. Our main Dockerfile includes **dependencies for Isaac Sim**, but Isaac Sim itself typically runs in its own container or natively.

For this book, we will focus on running Isaac Sim in its own Docker container, which will then interact with our main ROS 2 Docker container via the host network.

### 5.2. Downloading Isaac Sim Docker Image

NVIDIA provides Isaac Sim as a Docker image via their NGC (NVIDIA GPU Cloud) Catalog. You will need an NVIDIA developer account to access it.

1.  **Login to NGC:**
    ```bash
    docker login nvcr.io
    # Enter your NGC API Key as password
    ```
    If you don't have an API Key, generate one from the NGC website ([https://ngc.nvidia.com/setup/api-key](https://ngc.nvidia.com/setup/api-key)).

2.  **Pull the Isaac Sim Image:** Replace `VERSION` with the desired Isaac Sim version (e.g., `2023.1.1`). Check the NGC Catalog for available versions: [https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim)
    ```bash
    docker pull nvcr.io/nvidia/isaac-sim:VERSION
    ```
    This download can take a long time due to the image size (tens of GBs).

### 5.3. Running Isaac Sim Docker Container

Running Isaac Sim requires a specific set of parameters to enable GPU access, display forwarding, and shared memory.

```bash
# Example for a typical Linux desktop setup. Adjust display if needed.
docker run --name isaac-sim \
  -e "ACCEPT_EULA=Y" \
  --network=host \
  --gpus all \
  -e "PRIVACY_CONSENT=Y" \
  -e "OMNI_PBR_NATIVE_PATH=/opt/nvidia/omniverse/kit/exts/omni.usd.libs.pbr_native.linux-x86_64" \
  -e "OMNI_COLLAB_WHITELIST=file:///tmp/,file:///home/${USER}/" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim/cache/compute_cache:/root/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/documents:/root/.local/share/ov/documents:rw \
  -v ~/docker/isaac-sim/settings:/root/.nvidia-omniverse/config:rw \
  -v /path/to/your/workspace/code/module3:/workspace/code/module3:rw \ # Mount your code for Isaac Sim
  -p 8888:8888 \ # Jupyter Lab
  -p 8080:8080 \ # Web UI
  -p 50000-50010:50000-50010/udp \ # ROS/ROS2 Bridge
  nvcr.io/nvidia/isaac-sim:VERSION
```

**Key Flags Explained:**
*   `-e "ACCEPT_EULA=Y"`: Required to accept NVIDIA's End User License Agreement.
*   `--network=host`: Allows Isaac Sim to communicate directly with your ROS 2 container (if running on the same host) and other network services.
*   `--gpus all`: Grants access to all available GPUs.
*   `-v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY`: Essential for displaying Isaac Sim's GUI on your host X server.
*   `-v ~/docker/isaac-sim/cache/...`: Mounts host directories to persist Isaac Sim's cache, logs, and data, preventing re-downloading large assets every time. **Adjust `~/docker/isaac-sim` to your preferred host path.**
*   `-v /path/to/your/workspace/code/module3:/workspace/code/module3:rw`: **CRITICAL!** Mount your `code/module3` directory from this book project into the Isaac Sim container. This allows you to work on Isaac Sim-specific examples directly. **Replace `/path/to/your/workspace/` with your actual project path.**
*   `-p 50000-50010:50000-50010/udp`: Ports for ROS/ROS2 Bridge communication.

### 5.4. Interacting with Isaac Sim

Once the Isaac Sim container is running:
*   **GUI:** The Isaac Sim application GUI should appear on your host machine.
*   **JupyterLab:** You can access JupyterLab at `http://localhost:8888` for Python scripting.
*   **ROS 2 Bridge:** Isaac Sim includes a ROS 2 bridge to facilitate communication with external ROS 2 nodes (e.g., our main ROS 2 Docker container).

### 5.5. Running Isaac ROS Applications

For Isaac ROS (covered in later lessons), you will typically run these components within your main ROS 2 Docker container, and they will interact with Isaac Sim over the ROS 2 Bridge. Your main Dockerfile already includes NVIDIA container toolkit dependencies to prepare for this.

This comprehensive setup ensures you have a robust environment for exploring NVIDIA Isaac Sim's capabilities for AI and robotics development.

