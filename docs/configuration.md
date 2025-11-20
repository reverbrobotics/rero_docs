---
layout: default
title: Configuration
nav_order: 5
---

# Configuration

Rero Core is configured via a single `config.ini` file passed to each executable:

```bash
./bin/rero_server ./conf/config.ini
````

Executables expect the path to this file as their first argument. 

---

## Default configuration

A typical `config.ini` looks like:

```ini
[server]
host=0.0.0.0
port=50052
paDeviceIndex=-1

[model]
path="./models/vosk-rpi-model/"

[nlu]
model_path="./models/nlu/nlu_engine_music/"
```

* `server.host` / `server.port`: gRPC bind address and port.
* `server.paDeviceIndex`: PortAudio device index. `-1` selects the system default device. 

---

## Selecting the audio device

To list audio devices and their indices:

```bash
python3 -m sounddevice
```

Pick the desired input device index and set `paDeviceIndex` accordingly, then restart the Rero Core service (for systemd-based installs):

```bash
sudo service rerocore restart
```



> **Important:** Any change to `config.ini` requires a restart of the core service for the changes to take effect. 

---

## Custom speech recognition vocabulary

To improve recognition accuracy in a narrow domain, define a custom vocabulary file:

1. Create `models/vocab.txt` with one term per line.

2. Update the `model` section:

   ```ini
   [model]
   path="./models/vosk-rpi-model/"
   vocab_path="./models/vocab.txt"
   ```

3. Restart the service:

   ```bash
   sudo service rerocore restart
   ```



---

## Custom NLU models

You can train a new NLU model using the Snips NLU toolkit, following their CLI documentation. Once trained, point Rero Core to the model directory:

```ini
[nlu]
model_path="/path/to/trained/model"
```

Models trained on x86 can be copied to ARM devices (like Raspberry Pi) and used directly for inference. 
