---
layout: default
title: CLI Tools
nav_order: 6
---

# CLI Tools

These CLI tools are useful for debugging and validating your Rero Core configuration. Each takes `config.ini` as its first argument.   

---

## Starting the ReroCore Server

```bash
./bin/rero_server ./conf/config.ini
````

Wait until you see a message indicating the server is listening on the configured port (e.g., `0.0.0.0:50052`). 

---

## Speech Recognition

```bash
./bin/speech_recognition_cmdline ./conf/config.ini
```

Speak into the microphone; recognized text will print in the terminal.

---

## Natural Language Understanding

```bash
./bin/nlu_cmdline ./conf/config.ini
```

Type test sentences (e.g., “Please play the Beatles”) and observe the parsed intents/slots. 

---

## Raw Audio Playback

To monitor microphone audio directly:

```bash
./bin/audio_player ./conf/config.ini
```

This usually works best when running the cli-tools from another computer (with `host=0.0.0.0` set to the ip of the Rero Board in the other computer's config.ini). Use headphones to avoid feedback loops. 

---

## Text-To-Speech

```bash
./bin/tts_cmdline ./conf/config.ini
```

Provide text as prompted and listen to the generated speech. 
