---
layout: default
title: Running Core Tools
nav_order: 6
---

# Running Core Tools

These CLI tools are useful for debugging and validating your Rero Core configuration. Each takes `config.ini` as its first argument.   

---

## Starting the ReroCore server

```bash
./bin/rero_server ./conf/config.ini
````

Wait until you see a message indicating the server is listening on the configured port (e.g., `0.0.0.0:50052`). 

---

## Speech recognition (CLI)

```bash
./bin/speech_recognition_cmdline ./conf/config.ini
```

Speak into the microphone; recognized text will print in the terminal.

---

## Natural language understanding (CLI)

```bash
./bin/nlu_cmdline ./conf/config.ini
```

Type test sentences (e.g., “Please play the Beatles”) and observe the parsed intents/slots. 

---

## Raw audio playback

To monitor microphone audio directly:

```bash
./bin/audio_player ./conf/config.ini
```

Use headphones to avoid feedback loops. 

---

## Text-to-speech (CLI)

```bash
./bin/tts_cmdline ./conf/config.ini
```

Provide text as prompted and listen to the generated speech. 
