# ROS 2 Autonomous and Collaborative Robotics

This directory holds the source code for the unity setup. Please make sure to have Unity Hub installed and the Unity Editor with version 6000.0.29f1.

## Prepare Unity

Open the Unity Project from disk and load it. A double click on the sample scene should load the full scene with all robots and warehouse assets. 

If there is no **Robotics** in the navigation options at the top of the Unity Editor, please make sure to install it following this [tutorial](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/quick_setup.md).

Then, make sure to select ROS2 and leave the rest as it is in the configuration. Use the CustomROSEndpoint directory as the folder for the unity messages and generate all message types. This enables Unity to have the correct types and automatically generates the associated classes for it.