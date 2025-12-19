# Predator Turtle (ROS 2 Humble)

## Description
This project demonstrates the use of **TF2 (Transform Library)** and geometric math in ROS 2. It implements a "predator-prey" behavior where an autonomous turtle (the predator) relentlessly chases another turtle (controlled by the user) using real-time coordinate transforms.

## Key Concepts
* **TF2 Broadcasting:** Broadcasting dynamic frames for multiple agents (`turtle1` and `predator`) to the `world` frame.
* **TF2 Listening:** Calculating relative transforms (distance and angle) between two dynamic frames.
* **Math:** Converting Cartesian coordinates ($x, y$) into velocity commands ($v, \omega$) using `atan2` and Euclidean distance.
* **Launch System:** Automating the startup of 5 separate nodes (Simulation, Spawner, 2x Broadcasters, Listener) with a single command.

## How to Run
1.  **Clone the repo:**
    ```bash
    git clone [https://github.com/RishabhShahIITGN/predator_turtle.git](https://github.com/RishabhShahIITGN/predator_turtle.git)
    ```
2.  **Build the package:**
    ```bash
    colcon build --packages-select predator_turtle
    source install/setup.bash
    ```
3.  **Launch the simulation:**
    ```bash
    ros2 launch predator_turtle predator_launch.py
    ```
4.  **Control the prey (in a new terminal):**
    ```bash
    ros2 run turtlesim turtle_teleop_key
    ```