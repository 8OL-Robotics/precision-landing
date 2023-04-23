# Precision Landing

This is a collection of Python scripts that enable a drone to perform precise landings on an ArUco marker. The suite includes decoupled PID controllers, IO, and estimators that can be easily swapped out for different methods of control and sources of pose.

## Flight Controller Support

The main branch of the repository supports the PX4 flight controller, while the ardupilot branch supports precision landing for the ardupilot flight controller. This is because mavSDK does not support ardupilot, so we use dronekit for ardupilot.

## Simulation Setup

To use the simulation setup, you can use a simulated drone with either the ardupilot or the PX4 flight controller. The only requirement is that the drone has a downward-facing or gimbal-mounted camera that is pointing downwards during offboard control. Additionally, you need to set up the ports appropriately for the relevant IO from the IO directory and run the Python package.

## 3DR Solo Setup

Similar to the simulation setup, the 3DR Solo setup streams the ArUco stream over UDP and over the same port as default gazebo cameras. To set up the 3DR Solo, follow these steps:
- Start the Solo controller and drone
- Connect to the Solo hotspot
- Use the command `solo video acquire` to enable GoPro streaming on the 3DR Solo. The drone should restart.
- Use the command `nc 10.1.1.1 5502` to simulate a mobile app connected to the controller
- Run the Python package

## Demo

### Simulation

Check out this video of Precision Landing in action in a simulated environment:

[![SIM](https://img.youtube.com/vi/zFaq3G3E5Ek/0.jpg)](https://www.youtube.com/watch?v=zFaq3G3E5Ek)

### Real-Life Demonstration on 3DR Solo

Check out this video of Precision Landing in action on a 3DR Solo:

[![REAL](https://img.youtube.com/vi/YOW82ZHeRjg/0.jpg)](https://www.youtube.com/watch?v=YOW82ZHeRjg)
