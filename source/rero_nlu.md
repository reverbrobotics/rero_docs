# ReRo Natural Language Understanding (NLU)
This package performs natural language understanding on the output of the [ReRo Speech Recognition](https://github.com/reverbrobotics/rero_speech_recognition) package. NLU is performed using the [Snips NLU](https://github.com/snipsco/snips-nlu) library. 


## Installing Package and Dependencies
This package requires [ROS](https://www.ros.org/). Once ROS is installed, clone the package to the src folder of your catkin workspace and run ```catkin_make``` to build the package.

The package also requires the [snips-nlu](https://github.com/snipsco/snips-nlu) library for natural language understanding. This can be installed on x86-64 systems using

```
~$ pip install snips-nlu
```

For ARM-based systems we provide pre-built wheels for the following packages:

|      Package      | Python Version | Architecture |                                                           Wheel                                                          |
|:-----------------:|:--------------:|:------------:|:------------------------------------------------------------------------------------------------------------------------:|
|       numpy       |       3.6      |    aarch64   |        [link](https://github.com/reverbrobotics/rero_nlu/releases/download/v0.1/numpy-1.19.3-cp36-cp36m-linux_aarch64.whl)       |
|       scipy       |       3.6      |    aarch64   |       [link]( https://github.com/reverbrobotics/rero_nlu/releases/download/v0.1/scipy-1.5.3-cp36-cp36m-linux_aarch64.whl)        |
|  python-crfsuite  |       3.6      |    aarch64   |   [link](https://github.com/reverbrobotics/rero_nlu/releases/download/v0.1/python_crfsuite-0.9.7-cp36-cp36m-linux_aarch64.whl)   |
|    scikit-learn   |       3.6      |    aarch64   | [link](https://github.com/reverbrobotics/rero_nlu/releases/download/v0.1/scikit_learn-0.22.2.post1-cp36-cp36m-linux_aarch64.whl) |
|  snips-nlu-utils  |       3.6      |    aarch64   |   [link](https://github.com/reverbrobotics/rero_nlu/releases/download/v0.1/snips_nlu_utils-0.9.1-cp36-cp36m-linux_aarch64.whl)   |
| snips-nlu-parsers |       3.6      |    aarch64   |  [link](https://github.com/reverbrobotics/rero_nlu/releases/download/v0.1/snips_nlu_parsers-0.4.3-cp36-cp36m-linux_aarch64.whl)  |
|     snips-nlu     |       3.6      |    aarch64   |            [link](https://github.com/reverbrobotics/rero_nlu/releases/download/v0.1/snips_nlu-0.20.2-py3-none-any.whl)           |

The wheels can be installed using pip. Once the python packages are installed, language resources must be installed. This can be done using the command

```~$ python -m snips_nlu download <language>```

For example, the following command will download the resources for english:

```~$ python -m snips_nlu download en```

## Usage

### Launching the Package
The nlu package can now be launched using the provided launch file:

```~$ roslaunch rero_nlu nlu.launch```

The ```input_topic_name``` parameter corresponds to the input topic from the speech recognition system, and the ```output_topic_name``` parameter defines the topic that the nlu package publishes results on. The ```dataset_path``` parameter defines a path to a snips dataset json file. The default example dataset included with the repo is called ```robot_dataset.json``` and recognizes user intents related to the robot handing over objects, such as **"Robot, hand me the wrench"**.

### NLU Concepts

The ReRo NLU packages classifies input sentences into different *intents* and also extracts pieces of structured information from the sentence, that are called *slots*. Intents are predefined categories of possible intents the user can have when interacting with the robot. 

For example, imagine a speech interface for a robot that enables it to hand over various objects. 

If the ReRo speech recognition system recognised the sentence **"Robot, hand me the wrench"**, this would be classified as the **handObject** intent, and the intent would have a single slot with the name ***object*** and a corresponding value of ***wrench***.

### ReRo Message Types

The intent recognition results are published on the output topic using two custom rero message types: *Intent* and *Slot*

#### Intent

The intent message type corresponds to an intent that the system has recognized from the speech recognition output. It is defined as follows:

```
string inputText
string intentName
float32 probability
Slot[] slots
```

**inputText** is the input text from the speech recognition system, **intentName** is the recognized intent, **probability** is the probability that the recognized intent is the correct one, and **slots** is a list of Slot messages. If the system could not recognize an intent from the speech recognition output, then the ***intentName*** field is be set to ```null```.

#### Slot
Each slot message object corresponds to a parsed slot in the intent. The messages are structured as follows:

```
string rawValue
string entity
string slotName
uint16 rangeStart
uint16 rangeEnd
string typeName
string typeValue
```

e.g. the parsed slot for the value **wrench** from the previously described example would look like so:

```
rawValue: "wrench"
entity: "object"
slotName: "object"
rangeStart: 19
rangeEnd: 25
typeName: "Custom"
typeValue: "wrench"
```

## Custom Datasets
Custom snips training datasets can be created by following the instructions [here](https://snips-nlu.readthedocs.io/en/latest/tutorial.html#training-data). Once a training dataset json file has been created, the nlu package can be configured to use it by setting the ```dataset_path``` parameter. Note: it is highly recommended to also configure a custom vocabulary for the speech recognition system in order to improve accuracy. 


## Disclaimer
The provided software is still in BETA and is provided as is. 