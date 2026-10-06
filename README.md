# ROS 2 Fuzzy Right-Wall Following

A Python controller that uses fuzzy logic and laser scan measurements to guide a mobile robot along a wall on its right-hand side.

## Overview

The controller subscribes to `/scan`, processes two selected laser scan sectors, and publishes linear and angular velocity commands to `/cmd_vel`.

## Features

- Near, medium, and far distance membership functions.
- Nine fuzzy rules using two distance inputs.
- Weighted-average defuzzification.
- Velocity commands published at 5 Hz.
- Stop command on keyboard interruption.

## Main File

`right_edge_F.py` — laser processing, fuzzy inference, and robot motion control.

## ROS Interfaces

| Topic | Message type | Purpose |
|-------|--------------|---------|
| `/scan` | `sensor_msgs/msg/LaserScan` | Laser distance measurements |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Robot velocity commands |

## Control Process

1. Read laser scan ranges from indices `200:220` and `320:340`.
2. Select the nearest positive distance in each sector.
3. Cap each distance at 1 metre.
4. Calculate near, medium, and far membership values.
5. Evaluate the nine fuzzy rules.
6. Calculate linear and angular velocities.
7. Publish the resulting motion command.

## Requirements

- A configured ROS 2 installation.
- Python 3.
- ROS 2 packages providing `rclpy`, `sensor_msgs`,
  `geometry_msgs`, `nav_msgs`, and `tf2_ros`.
- A robot or simulation publishing `/scan` and accepting `/cmd_vel`.

## Running

Start the robot or simulation first.

In a terminal with your ROS 2 environment sourced, navigate to
the directory containing the script and run:

```bash
python3 right_edge_F.py
```

Press `Ctrl+C` to stop the controller.

## Configuration Notes

The scan sectors are selected using fixed array indices.
Verify that these indices correspond to the intended right-side
directions for your laser scanner.

Distance membership functions, speed values, and steering values
are defined near the beginning of the script and can be adjusted
for the robot and environment.

