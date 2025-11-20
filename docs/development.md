---
layout: default
title: Development & Extensibility
nav_order: 8
---

# Development & Extensibility

Rero Core’s audio, speech recognition, and NLU modules are implemented in C++ and integrated with ROS. You can modify them and rebuild the system.

---

## Modifying core modules (C++)

On prebuilt images, the C++ source for core modules is in a ROS catkin workspace, typically:

- `~/rero_ros_ws/src/rero_ros/src`
- or `~/catkin_ws/src/rero_ros/src`

depending on the image version.

After editing C++ files:

```bash
cd ~/rero_ros_ws    # or ~/catkin_ws
catkin_make
````

This recompiles the modified modules and updates the ROS nodes accordingly. 

---

## Creating new distributables

When building from the standalone Rero Core source repository, you can produce new distributable packages from your `build` directory using:

```bash
sudo cpack
```

Be sure to bump the project version numbers to reflect your changes before packaging. 

---

## Licensing and citation

Rero Core distributables are released under the **GPL-2.0** license. Commercial licensing under a separate license is available on request. 

If you use this project in academic work, please cite:

```bibtex
@article{grasse_tata_2021_platform, 
  title   = {An End-to-End Platform for Human-Robot Speech Interaction}, 
  url     = {https://r00binson.wixsite.com/soundinhri/submissions}, 
  journal = {Sound in HRI Workshop. 16th Annual Conference for Basic and Applied Human-Robot Interaction.}, 
  author  = {Grasse, Lukas S. and Tata, Matthew S.}, 
  year    = {2021},
  month   = {Mar}
}
```
