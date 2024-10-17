# ROS2_Dist_Mutex

This repository contains a basic ROS2 implementation of the algorithm proposed in [Ricart_and_Agrawala \[1981\]](https://dl.acm.org/doi/pdf/10.1145/358527.358537 "Article link").
There are 2 ROS2 packages in the repository:
## 1. Test_msgs
This package only contains the definition of a signed and timestamped message for a ROS2 service.
## 2. Lamport_clks
This package contains a ROS2 node that uses 2 services and several clients to periodically run the code inside its critical_section function.
The implementation of the mentioned algorithm theoretically guarantees that at a given point in time, only one instance of this node is running their critical_section function.

