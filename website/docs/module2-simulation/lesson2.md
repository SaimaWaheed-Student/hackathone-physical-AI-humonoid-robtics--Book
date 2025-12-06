---
sidebar_position: 2
---

# Module 2: Simulation - Lesson 2: ROS-Unity Integration in 30 Minutes

This lesson will guide you through setting up Unity for ROS integration, enabling communication between your ROS 2 robot control stack and a high-fidelity Unity simulation environment.

## Introduction to Unity for Robotics

Unity is a powerful cross-platform game engine that is increasingly being adopted in robotics for high-fidelity simulation, data generation, and visualization. Its advanced rendering capabilities, physics engine, and extensive asset store make it an excellent choice for creating realistic digital twins of your robots and environments.

Integrating Unity with ROS 2 allows you to:
*   Develop and test ROS 2 algorithms in a visually rich and interactive simulation.
*   Generate synthetic sensor data (e.g., camera, LiDAR) that closely mimics real-world scenarios.
*   Create complex and dynamic environments that are difficult or costly to replicate physically.
*   Visualize robot behavior and sensor feedback in real-time.

## Prerequisites

Before you begin, ensure you have:
*   Unity Hub and Unity Editor (version 2020.3 LTS or newer recommended) installed on your system.
*   A basic understanding of Unity's interface and C# scripting.
*   Your Dockerized ROS 2 environment (from Module 1) up and running.

## Step 1: Create a New Unity Project

1.  Open Unity Hub and create a new 3D project. Give it a meaningful name like `ROSUnitySimulation`.
2.  Select the **Universal Render Pipeline (URP)** template for better performance and visual customization, especially for robotics applications.

## Step 2: Install the ROS-TCP-Connector

The `ROS-TCP-Connector` is a Unity package that facilitates communication between Unity and ROS 2 over TCP.

1.  In your Unity project, go to `Window > Package Manager`.
2.  Click the `+` icon in the top-left corner and select `Add package from git URL...`.
3.  Enter the URL for the `ROS-TCP-Connector` package: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.ros_tcp_connector`
4.  Click `Add`. Unity will download and install the package.

## Step 3: Configure ROS 2 IP Address (ROS-TCP-Endpoint)

For Unity to communicate with your ROS 2 environment (likely running in a Docker container or on a different machine), you need to specify the IP address of your ROS 2 endpoint.

1.  In your Unity Project window, navigate to `Assets > ROS-TCP-Connector > Scripts`.
2.  Locate the `ROSTCPConnector.cs` script and attach it to an empty GameObject in your scene (e.g., create an empty GameObject named `ROSConnection`).
3.  In the Inspector window for the `ROSConnection` GameObject, you will see a field for `ROS IP Address`. Enter the IP address of your ROS 2 host.
    *   **If ROS 2 is in Docker on the same machine:** You might use `172.17.0.1` (the default Docker gateway IP for Linux hosts) or the actual IP address of your host machine.
    *   **If ROS 2 is on a different machine:** Use that machine's IP address.
4.  Ensure the `ROS Port` is set to `10000` (default for ROS-TCP-Connector).

## Step 4: Test Basic Communication (Optional but Recommended)

To quickly test if your Unity project can communicate with ROS 2, you can use the `ROS-TCP-Connector`'s basic publisher and subscriber examples.

1.  In Unity, import the `ROS-TCP-Connector`'s `Tutorials` samples (`Window > Package Manager > ROS-TCP-Connector > Samples > Import all`).
2.  Open the `RosPublisherTutorial` scene (`Assets > Samples > ROS TCP Connector > 0.X.X > Tutorials > RosPublisherTutorial`).
3.  Run the scene in Unity.
4.  In your ROS 2 Docker container, run `ros2 topic echo /unity_chatter`. You should see messages being published from Unity.

## Step 5: Preparing Your Robot Model

If you have a robot model (e.g., from URDF/XACRO) that you want to use in Unity, you will typically:
1.  **Export:** Convert your URDF/XACRO model into a format Unity can import, such as FBX or glTF. Tools like `urdf_to_scene` or `blender` can assist with this.
2.  **Import:** Import the converted model into your Unity project (`Assets > Import New Asset...`).
3.  **Configure:** Add colliders, rigidbodies, and scripts to your robot model in Unity to enable physics simulation and control.

This guide provides a quick setup to get you started. Future lessons will delve into more advanced topics like simulating sensors, controlling joints, and generating synthetic data from Unity.