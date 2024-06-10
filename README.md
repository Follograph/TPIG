# TPIG
## Contents
- [Install](#install)
- [Simulation](#simulation)
- [Trajectory Planning](#trajectory-planning)
- [Intelligent Grasping](#intelligent-grasping)

## Install

**System Requirements**
- ubuntu22.04.3 LTS
- CUDA11.7
- Pytorch1.13.0
- python3.10

**Library Installation**
- [cuRobo](https://curobo.org/get_started/1_install_instructions.html)
- [Flexiv RDK](https://github.com/flexivrobotics/flexiv_rdk/tree/main)

*The version of cuRobo applied is v0.6.2, and the version of Flexiv RDK is v0.9.*

**Using in Isaac Sim**
- [Isaac Sim 2022.2.1](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html)

*:grey_exclamation:[System requirements](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html).*

**Installation for intelligent grasping**
- [ZED SDK](https://www.stereolabs.com/docs/app-development/python/install)
- OpenAI (pip)

## Simulation

- [cuRobo instruction](https://curobo.org/get_started/2b_isaacsim_examples.html)  
- [Configuring a new robot](https://curobo.org/tutorials/1_robot_configuration.html)  

[Example code](https://github.com/Follograph/TPIG/tree/main/trajectory%20planning/simulation)  
Explanation:  
In single_complete.py, a flexiv robot picks a cube to another place.  
In novisual.py, the robot with extended finger tips picks the tube from the box to another box, and withplot.py records changes in joints during one of the motion.

## Trajectory-Planning

If you are not using Linux

## Intelligent-Grasping

Use GPT-4o to recognize images of grasping scenarios. Images are captured by ZED2 camera, which is not recommended.  


1. Install openAI and ZED SDK.
2. Run comtest.py to recognize and apply correct robot actions (robot_actions.py).
