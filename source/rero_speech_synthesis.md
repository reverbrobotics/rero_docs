# ReRo Speech Synthesis
This package performs text-to-speech synthesis given text input on an input topic. The speech synthesis is performed using [eSpeak NG](https://github.com/espeak-ng/espeak-ng).


## Installing Package and Dependencies
This package requires [ROS](https://www.ros.org/). Once ROS is installed, clone the package to the src folder of your catkin workspace and run ```catkin_make``` to build the package.

The package also requires [eSpeak NG](https://github.com/espeak-ng/espeak-ng) and [py-espeak-ng](https://pypi.org/project/py-espeak-ng/) to be installed.

eSpeak NG can be installed using apt as shown below: 

```
~$ sudo apt-get install espeak-ng
```

and then py-espeak-ng can then be installed using pip:

```
~$ pip install py-espeak-ng
```

## Usage

### Launching the Package
The speech synthesis package can now be launched using the provided launch file:

```~$ roslaunch rero_speech_synthesis speech_synthesis.launch```

Various arguments can be used to customize the synthesised voice. The ```pitch``` argument can be used to set the voice pitch, and defaults to *32*. The ```speed``` argument controls the voice speed and defaults to *150*. The voice can also be set using the ```voice``` parameter and defaults to *en*. A list of available voices can be found [here](http://espeak.sourceforge.net/languages.html).

### Testing the Speech Synthesis
To test the speech synthesis, simply publish a *std_msgs/String* message to the **/rero_speech_synthesis/speech_synthesis** topic. 

For example:
```
~$ rostopic pub /rero_speech_synthesis/speech_synthesis std_msgs/String "Hello world!"
```

If everything is working you should hear the synthesized voice say the phrase *Hello world!*.


## Disclaimer
The provided software is still in BETA and is provided as is. 