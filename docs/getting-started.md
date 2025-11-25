---
layout: default
title: Getting Started
nav_order: 2
---

# Getting Started

This page summarizes what you need before running Rero Core and Rero ROS.

## Supported platforms

The prebuilt Rero Core distributables currently target:

- **Ubuntu (x86_64) 24.04+**
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

---

## ROS 2 requirements
 
* For **ROS 2 integration**, you need:

  * A ROS2 distribution (e.g., Humble/Foxy) with `colcon` installed.
  * gRPC C++ installed and discoverable by CMake. 

---

## Next steps

* To install using prebuilt packages or images, see **[Installing Rero Core](./installing-rero-core)**.
* Alternatively, if you are using the Rero Single Board Computer, you can skip to **[Setting Up the Rero Board](./setting-up-rero-board)**