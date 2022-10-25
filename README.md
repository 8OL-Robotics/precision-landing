# precision landing

A suite of python scripts to perform precision landing of a drone on an aruco marker, the PID controller, IO, estimators are all decoupled for hotswapping methods of control and sources of pose.

## flight controller support 
The main branch has code which supports the PX4 flight controller, whereas the ardupilot branch has code which supports precision landing for the ardupilot flight controller, this is because mavSDK does not support ardupilot, so for ardupilot we use dronekit

## simulation setup 
-  you can use a simulated drone having either the ardupilot or the PX4 flight controller, the only requirements are that there is a downward facing or gimble mounted camera, pointing downards for the duration of offboard control.
-  setup the ports appropriately for the relavant IO from the IO directory
-  run the python package
## 3DR solo setup
Just like the simulation setup, the 3dr solo setup also streams the aruco stream over UDP and over the same port as default gazebo cameras
 - start solo controller and drone
 - connect to solo hotspot
 - use the command `solo video acquire` to enable go pro streaming on the 3dr solo, the drone should restart.
 - use the command `nc 10.1.1.1 5502` to simulate a mobile app connected to the controller
 - run the python package
 
 ## DEMO
 
 ### Simulation 
 [![SIM](https://img.youtube.com/vi/zFaq3G3E5Ek/0.jpg)](https://www.youtube.com/watch?v=zFaq3G3E5Ek)
 
 ### Real-life demonstration on 3DR solo
 
 [![SIM](https://img.youtube.com/vi/CtwPBNmoWCg/0.jpg)](https://www.youtube.com/watch?v=CtwPBNmoWCg)
