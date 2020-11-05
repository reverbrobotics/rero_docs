# ReRo Speech Recognition

This package performs speech recognition and publishes the resulting recognized text over ROS. Speech recognition is performed using the [Vosk](https://github.com/alphacep/vosk-api) api for [Kaldi](https://kaldi-asr.org/), which enables the easy configuration of custom vocabularies.

## Installing Package and Dependencies
This package requires [ROS](https://www.ros.org/). Once ROS is installed, clone the package to the src folder of your catkin workspace and run ```catkin_make``` to build the package.
 
 All python dependencies can then be installed using pip, e.g:

```
~$ pip install -r requirements.txt
```


Once this is complete, the final step is to download a vosk speech recognition model to the *models* folder in the package. The recommended model to use is the [vosk-model-small-en-us-0.4](http://alphacephei.com/vosk/models/vosk-model-small-en-us-0.4.zip) model. Simply download this to the models folder and unzip it. e.g. 

```
~$ cd src/rero_speech_recognition/models
~$ wget http://alphacephei.com/vosk/models/vosk-model-small-en-us-0.4.zip
~$ unzip vosk-model-small-en-us-0.4.zip
```

Now the speech recognition package is ready to use.

## Usage
The speech recognition package can now be launched using the provided launch file:

```~$ roslaunch rero_speech_recognition speech_recognition.launch```

## Custom Vocabulary
A custom vocabulary can be used by setting the ```vocab_path``` to point to a text file containing the vocabulary to used, where the text file contains one vocabulary entry per line. The default vocabulary is a set of words for instructing a robot to hand over different objects. This default vocabulary works in conjunction with the example model in the [ReRo NLU](https://github.com/reverbrobotics/rero_nlu) package.


## Disclaimer
The provided software is still in BETA and is provided as is. 


