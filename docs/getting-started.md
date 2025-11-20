---
layout: default
title: Getting Started
nav_order: 2
---

# Getting Started

This page summarizes what you need before running Rero Core and Rero ROS.

## Supported platforms

The prebuilt Rero Core distributables currently target:

- **Ubuntu (x86_64) 18.04+**
- **Raspbian (ARM) on Raspberry Pi**

A prebuilt Raspberry Pi image is also available with Rero Core and ROS already configured. 

---

## System dependencies

### Audio libraries

Rero Core uses ALSA and PortAudio. Install them first:

```bash
sudo apt-get update
sudo apt-get install libasound2-dev libsndfile-dev
````

Then install PortAudio:

```bash
wget http://files.portaudio.com/archives/pa_stable_v190700_20210406.tgz
tar -zxvf pa_stable_v190700_20210406.tgz
cd portaudio/
./configure --with-alsa
make
sudo make install
```



If you plan to **compile from source**, also install:

```bash
sudo apt-get install libasound2-dev libsndfile1-dev libsystemd-dev libcurl4-openssl-dev
sudo apt-get install cmake cmake-curses-gui git git-lfs build-essential
git lfs install
```



---

## ROS / ROS2 requirements

* For **ROS1 integration**, use the prebuilt images or a ROS1 catkin workspace (e.g., `~/rero_ros_ws` or `~/catkin_ws`). 
* For **ROS2 integration**, you need:

  * A ROS2 distribution (e.g., Humble/Foxy) with `colcon` installed.
  * gRPC C++ installed and discoverable by CMake. 

---

## Next steps

* To install using prebuilt packages or images, see **[Installing Rero Core](./installing-rero-core)**.
* To build everything from source, see **[Building from Source](./building-from-source)**.