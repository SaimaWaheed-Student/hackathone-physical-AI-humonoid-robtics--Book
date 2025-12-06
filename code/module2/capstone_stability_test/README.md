# Capstone: Robot Stability Test in Gazebo

This capstone project allows you to test the stability of the humanoid robot in various Gazebo environments with different physics configurations.

## Setup

Before running these tests, ensure:
1.  Your Docker development environment is running (from Module 1).
2.  All ROS 2 packages in your workspace (`code/`) are built using `colcon build`.
3.  The `humanoid_description` package and this `capstone_stability_test` package are correctly sourced.

## Running the Stability Tests

You can launch the humanoid robot in different Gazebo worlds using the `stability_test.launch.py` launch file.

1.  **Launch the flat world:**
    ```bash
    ros2 launch capstone_stability_test stability_test.launch.py world:=flat_world.world
    ```

2.  **Launch the uneven terrain world:**
    ```bash
    ros2 launch capstone_stability_test stability_test.launch.py world:=uneven_terrain.world
    ```

3.  **Launch the high friction world:**
    ```bash
    ros2 launch capstone_stability_test stability_test.launch.py world:=high_friction_world.world
    ```

## Testing Procedure

Once Gazebo is launched with the robot:
*   **Observe:** Carefully observe the robot's behavior. Does it stand upright? Does it fall?
*   **Interact (Optional):** You can manually apply forces or torques to the robot within Gazebo (using the GUI's "Force" or "Torque" tools) to test its resilience.
*   **Movements:** Later, you can integrate simple commands (e.g., from the voice controller in Module 1) to make the robot attempt to walk or perform other actions in these environments.

The goal is to analyze how different environmental physics (e.g., uneven ground, high friction) impact the robot's ability to maintain balance. This will be a foundation for developing more sophisticated balance control algorithms in later modules.
