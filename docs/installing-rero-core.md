---
layout: default
title: Installing Rero Core
nav_order: 3
---

# Installing Rero Core

You can install Rero Core in two main ways:

1. **Using prebuilt distributables** (Ubuntu & Raspbian).  
2. **Flashing the prebuilt Raspberry Pi image**.

This step can also be skipped if using the Reverb Robotics single board computer which already has Rero Core preinstalled, and a system service that automatically starts Rero Core on boot up. For info on setting up the board, see **[Setting up the Rero Board](./setting-up-rero-board)**.

## Using prebuilt distributables

Download the appropriate archive for your platform (Ubuntu x86_64 or Raspbian ARM) from the Rero Core distributables release page or table.

Once downloaded:

```bash
tar -xvf rero_core_<platform>.tar.gz
cd rero_core_<platform>
````

To start the server, run:

```bash
./bin/rero_server ./conf/config.ini
```

If everything is configured correctly, you should see a message similar to `Server listening on 0.0.0.0:50052`. 

---

## Using the prebuilt Raspberry Pi image

A preconfigured Raspberry Pi image is also provided. Flash it to a microSD card using your preferred imaging tool.

* Default credentials: `ubuntu:rero123.123`. 

After boot:

1. Log in via HDMI/keyboard or SSH.
2. Confirm that the `rerocore` service is running (or start `./bin/rero_server` manually).
3. Edit `./conf/config.ini` as needed (see **Configuration** docs).

---

## Verifying the installation

From the extracted Rero Core directory, run:

```bash
./bin/speech_recognition_cmdline ./conf/config.ini
```

Speak into the microphone and verify that recognized text appears in the terminal. 

You can also test:

```bash
./bin/nlu_cmdline ./conf/config.ini
./bin/audio_player ./conf/config.ini
./bin/tts_cmdline ./conf/config.ini
```

to verify NLU, audio playback, and TTS respectively. 
