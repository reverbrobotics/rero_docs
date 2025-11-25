---
layout: default
title: ROS Integration
nav_order: 7
---

# ROS Integration

Rero integrates with **ROS 2** via a dedicated package.

Before launching ROS nodes, ensure a `rero_server` instance is running and reachable at the host/port specified in `config.ini`.

---

## ROS 2: building the `rero_ros` package

### 1. Install gRPC

Follow the official gRPC C++ quickstart to build and install gRPC to a prefix such as `~/grpc_install`. 

### 2. Build with `colcon`

In your ROS2 workspace (e.g., `~/ros2_ws`):

```bash
colcon build --symlink-install \
  --cmake-args -DGRPC_FETCHCONTENT=OFF \
               -DCMAKE_PREFIX_PATH=~/grpc_install
```

This tells CMake to use your existing gRPC installation rather than downloading it via FetchContent. 

Source the workspace after build:

```bash
source install/setup.bash
```

---

## ROS2 nodes and launch files

All commands below assume your workspace has been sourced.

### Audio stream publisher

Streams audio from the microphone array into ROS2 topics:

```bash
ros2 launch rero_ros audio_stream.launch.xml
```

### Audio stream subscriber

Subscribes to the audio stream and writes audio to `.wav` files:

```bash
ros2 launch rero_ros audio_subscriber.launch.xml
```

### Direction of arrival (DOA)

Estimates the direction of incoming sound sources:

```bash
ros2 launch rero_ros doa.launch.xml
```

### NLU node

Converts recognized text into intents:

```bash
ros2 launch rero_ros nlu.launch.xml
```

### Speech recognition

Turns microphone audio into text:

```bash
ros2 launch rero_ros speech_recognition.launch.xml
```

### Text-to-speech (TTS)

Synthesizes speech from text messages:

```bash
ros2 launch rero_ros text_to_speech.launch.xml
```

### Voice activity detection (VAD)

Detects when speech is present in the audio stream:

```bash
ros2 launch rero_ros vad.launch.xml
```



---

## Testing the ROS2 NLU pipeline

1. In a new terminal, echo the NLU results:

   ```bash
   ros2 topic echo /rero_ros/nlu_result
   ```

2. Publish a test utterance to the speech recognition topic:

   ```bash
   ros2 topic pub /rero_ros/speech_recognition std_msgs/String \
     "data: Please play the beatles"
   ```

You should see the corresponding NLU result on `/rero_ros/nlu_result`, verifying the full speech → NLU pipeline. 
