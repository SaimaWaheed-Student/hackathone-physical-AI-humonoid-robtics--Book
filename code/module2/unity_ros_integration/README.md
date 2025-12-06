# Unity ROS-TCP-Connector Integration Guide

This guide provides instructions on how to configure your Unity project to communicate with the ROS 2 network using the `ROS-TCP-Connector` package. This setup is crucial for enabling high-fidelity simulations in Unity that interact with your ROS 2 robot control stack.

## Prerequisites

*   A Unity project with the `ROS-TCP-Connector` package installed (refer to `website/docs/module2-simulation/lesson2.md` for installation steps).
*   Your Dockerized ROS 2 environment (from Module 1) running and accessible from your host machine.

## 1. Configure ROS 2 IP Address in Unity

The `ROS-TCP-Connector` needs to know the IP address of your ROS 2 host to establish communication.

1.  **Identify ROS 2 Host IP:**
    *   If your ROS 2 environment is running in a Docker container on the same machine, the IP address of your host machine will typically be accessible from within the container. You might use `172.17.0.1` (the default Docker gateway IP for Linux hosts) or your actual host machine's IP address.
    *   If ROS 2 is on a different machine, use that machine's IP address.
    *   You can find your host machine's IP address by running `ip addr show` (Linux/macOS) or `ipconfig` (Windows) in your host terminal.

2.  **Unity Configuration:**
    *   Open your Unity project.
    *   Create an empty GameObject in your scene (e.g., name it `ROSConnection`).
    *   Add the `ROS-TCP-Connector/Scripts/ROSTCPConnector.cs` script to this GameObject.
    *   In the Inspector panel for the `ROSConnection` GameObject, locate the `ROS IP Address` field. Enter the IP address of your ROS 2 host that you identified in step 1.
    *   Ensure the `ROS Port` is set to `10000` (this is the default for `ROS-TCP-Connector`).

## 2. Verify Basic Communication

To test if Unity can successfully communicate with your ROS 2 network:

1.  **Import Unity Tutorials:**
    *   In Unity, go to `Window > Package Manager`.
    *   Select `ROS-TCP-Connector` and then click `Samples`.
    *   Import the `Tutorials` samples.
2.  **Run Unity Publisher Example:**
    *   Open the `RosPublisherTutorial` scene located at `Assets/Samples/ROS TCP Connector/0.X.X/Tutorials/RosPublisherTutorial`.
    *   Run this scene in the Unity Editor. It will start publishing messages to the `/unity_chatter` ROS 2 topic.
3.  **Monitor ROS 2 Topic:**
    *   In your ROS 2 Docker container (or directly in your ROS 2 environment), source your ROS 2 setup:
        ```bash
        source /opt/ros/humble/setup.bash
        ```
    *   Then, echo the topic:
        ```bash
        ros2 topic echo /unity_chatter
        ```
    *   You should see messages being published from your Unity Editor in your ROS 2 terminal. This confirms successful communication.

## 3. Basic ROS 2 Subscriber in Unity

You can also set up a basic ROS 2 subscriber in Unity to receive messages from your ROS 2 network.

1.  **Unity Subscriber Script (Example C#):**
    ```csharp
    using RosMessageTypes.Std; // Example for String message type
    using Unity.Robotics.ROSTCPConnector;
    using UnityEngine;

    public class RosSubscriberExample : MonoBehaviour
    {
        ROSConnection ros;
        public string topicName = "/ros_chatter";

        void Start()
        {
            ros = ROSConnection.Get = Instance();
            ros.Subscribe<StringMsg>(topicName, HandleRosMessage);
            Debug.Log($"Subscribing to {topicName}");
        }

        void HandleRosMessage(StringMsg msg)
        {
            Debug.Log($"Received ROS Message: {msg.data}");
            // You can process the received message here
        }
    }
    ```
2.  Attach this script to a GameObject in your Unity scene.
3.  **ROS 2 Publisher:** In your ROS 2 environment, publish a message to the `/ros_chatter` topic:
    ```bash
    ros2 topic pub /ros_chatter std_msgs/msg/String "data: 'Hello from ROS!'"
    ```
4.  You should see the message logged in the Unity Editor's console.
