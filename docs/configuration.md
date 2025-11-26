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

-----

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

-----

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

-----

## Custom speech recognition vocabulary

To improve recognition accuracy in a narrow domain, define a custom vocabulary file:

1.  Create `models/vocab.txt` with one term per line.

2.  Update the `model` section:

    ```ini
    [model]
    path="./models/vosk-rpi-model/"
    vocab_path="./models/vocab.txt"
    ```

3.  Restart the service:

    ```bash
    sudo service rerocore restart
    ```

-----

## Custom NLU models

You can train a new NLU model using the Snips NLU toolkit, following their CLI documentation. Once trained, point Rero Core to the model directory:

```ini
[nlu]
model_path="/path/to/trained/model"
```

Models trained on x86 can be copied to ARM devices (like Raspberry Pi) and used directly for inference.

-----

## Configuration Parameters

The following table lists all available configuration parameters, their section, and default values as read by the `rero_server` executable.

| Section | Parameter | Type | Default Value | Description |
| :--- | :--- | :--- | :--- | :-------------------------------------------------------------------------------------------------------------------------- |
| `server` | **host** | string | `0.0.0.0` | gRPC bind address. |
| `server` | **port** | string | `50051` | gRPC port. Note: The example above uses `50052`. |
| `server` | **headless** | bool | `false` | If `true`, the Audio and Text-to-Speech services are not registered. |
| `server` | **paDeviceIndex** | int | `-1` | PortAudio input device index. `-1` selects the system default input device. |
| `server` | **paOutputDeviceIndex** | int | `-1` | PortAudio output device index. |
| `server` | **denoising** | bool | `false` | Enable audio denoising on the input stream. |
| `server` | **storeAudio** | bool | `false` | Enable storing audio streams to disk. |
| `server` | **storeAudioPath** | string | `""` | Path to store audio files if `storeAudio` is `true`. |
| `model` | **path** | string | `./models/vosk-rpi-model` | Path to the speech recognition model (e.g., Vosk model). |
| `model` | **use\_whisper** | bool | `false` | If `true`, use the Whisper model for speech recognition. If using whisper, model path must be set to a whisper.cpp model. |
| `model` | **vad\_path** | string | `./models/vad/silero_vad.onnx` | Path to the Voice Activity Detection (VAD) model. |
| `model` | **vocab\_path** | string | `""` | Path to a custom vocabulary file for better recognition accuracy. |
| `nlu` | **model\_path** | string | `./models/nlu/nlu_engine_music` | Path to the trained Natural Language Understanding (NLU) model directory. |
| `tts` | **voice\_path** | string | `./models/tts/mycroft_voice_4.0.flitevox` | Path to the Text-to-Speech (TTS) voice model. |
| `tts` | **use\_ssml** | bool | `false` | Enable SSML (Speech Synthesis Markup Language) support. |
| `tts` | **use\_larynx** | bool | `false` | Use an external Larynx server for TTS. |
| `tts` | **larynx\_address** | string | `http://localhost:5002` | Address of the Larynx TTS server. |
| `tts` | **noise\_scale** | float | `0.667` | Parameter to control sample variability (noise/speaker difference). |
| `tts` | **length\_scale** | float | `1.0` | Parameter to control speaking speed (1.0 is normal). |
| `hotword` | **mel\_model\_path** | string | `./models/hotword/logmelcalc.tflite` | Path to the hotword detection mel-spectrogram TFLite model. |
| `hotword` | **base\_model\_path** | string | `./models/hotword/baseModel.tflite` | Path to the hotword detection base TFLite model. |
| `hotword` | **folder** | string | `./models/hotword/hotwords` | Folder containing trained hotword embeddings. |
| `hotword` | **threshold** | float | `0.8` | Confidence threshold for a detected hotword to be accepted. |