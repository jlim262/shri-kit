# Multimodal Expression Engine

## Package Summary

Multimodal Expression Engine(Social HRI Software Framework) is the modularized HRI software framework for developing the social robots.

-   Maintainer status: maintained
-   Maintainer: Ho Seok Ahn, JongYoon Lim, Chris Lee
-   Author: Byeong-Kyu Ahn
-   License: Apache License 2.0
-   Source: git https://github.com/jlim262/social_behavior.git

## Overview

Multimodal Expression Engine is the ROS-based software framework for developing SHRI(Social Human Robotics Interface) robots quickly. This framework consists of extensible modules such as sensory perception modules, dialog manager, motion generator.Each module communicates using the ROS topic, service, and actionlib. Data can be stored in the database as well if necessary. Multimodal Expression Engine also provides reflex social skills such as gazing, blinking and turn-taking. Tag and Gesture List for Robots typically can find in each robot's render_gesutre package: motions.yaml

<center><a href="./doc/social_mind_block_diagram.png"><img src="./doc/social_mind_block_diagram.png" width="800px"></a></center>

### perception_base node

perception_base is an base object for various sensory perception nodes. This provides the following base methods.

-   init(node_name): Read yaml type configuration file and setup database. Also enables subscribers for start/stop topics.
-   handle_start_perception(msg): Callback function of ros subscriber. This is called when '{node_name}/start' topic is received.
-   handle_stop_perception(msd): Callback function of ros subscriber. This is called when '{node_name}/stop' topic is received.
-   raise_event(perception_item, event, data): Publish ROS message with 'event' having 'data' by 'perception_item'.
-   register_data_to_memory(memory_name, perception_name, data): Register data into the memory using '{node_name}/register_data'. 'data' will be dumped as json type.
-   save_to_memory(perception_name, data={}): Save data into internal database. 'data' will be dumped as json type.
-   read_from_memory(target_memory, data): Read data['data'] from internal database.

### working_memory node

working_memory provides methods to store and read data from database. It uses 'pymongo' internally and exposes ROS services such as read_data, write_data, register_data, and get_data_list. Any modules can use this database and share with other ROS modules.

### motion_speech_generator(motion_arbiter) node

motion_speech_generator node generates a sort of script which robot should express motions and dialog to speech. Currently, it supports 8 tags such as SAY, GAZE, POINTING, SCREEN, MOBILITY, SM, EXPRESSION for generating a script. Also emotions are supported(happy, neutral, sad, surprise, angry).

### expression(motion_renderer) node

expression node is a mediator between motion_speech_generator and physical motion managing nodes for a specific robot. It consists of a set of ROS action clients. It parses the script from motion_speech_generator and call action servers from a robot motion nodes.

### reflex_behavior node

reflex_behavior node is responsible for the reflex motions such as gazing, turn-taking, and so on.

## Hardware requirements

Multimodal Expression Engine can be used for any hardware platforms which support ROS Indigo or above versions.

## Quick start

        $ cd ~/catkin_ws/src
        $ git clone https://github.com/jlim262/social_behavior.git
        $ rosdep install --from-paths social_behavior --ignore-src -r -y
        $ cd social_behavior
        $ find -name 'requirements.txt' | xargs -L 1 sudo pip install -U -r
        $ cd ~/catkin_ws
        $ catkin_make
        $ source ./devel/setup.bash
        $ roslaunch social_behavior bringup.launch

## Input/Subscribed Topics

### /raising_events [RaisingEvents.msg] via [events_multiplexer](./events_multiplexer)

-   Type

          Header header
          string recognized_word
          string[] events
          string[] data

    -   Example: Speech Recognition

            header:
                seq: 12
                stamp:
                secs: 1505344152
                nsecs: 513442993
                frame_id: ''
            recognized_word: Hello
            events: ['speech_recognized']
            data: ['{"confidence": 1.0, "recognized_word": "Hello"}']

    -   Example: Button Input

            header:
                seq: 52
                stamp:
                secs: 1505344319
                nsecs:  13427972
                frame_id: ''
            recognized_word: ''
            events: ['button_pressed']
            data: ['{"text": "yes"}']

## Output/Published Topics

### /reply [Reply.msg] received from Scheduler Engine

-   Type

          Header header
          string reply

-   Format

    -   Say

            header:
                seq:
                stamp:
                secs:
                nsecs:
                frame_id: ''
            reply: "Hello. My name is Silbot"

    -   Say with gesture by tag

            header:
                seq:
                stamp:
                secs:
                nsecs:
                frame_id: ''
            reply: "<sm=tag:neutral> Hello. My name is Silbot"

    -   Say with gesture by specific motion

            header:
                seq:
                stamp:
                secs:
                nsecs:
                frame_id: ''
            reply: "<sm=play:sm/open/1> Hello. My name is Silbot"

    -   Say with expession

            header:
                seq:
                stamp:
                secs:
                nsecs:
                frame_id: ''
            reply: "<expression=happiness> Hello. My name is Silbot"

## Parameters

-   use_v1_arbiter: "true" for using arbiter version 1. Default value is "false"
-   gaze: "true" for gazing an object(human face). Default value is "true".
-   gaze_default_height: Offset for the gazing height.
-   show_generated_timeline: "true" shows visualized timeline of generated motions. Default value is "false".
-   fake_render_speech: Text based simple renderer of speech. Default value is "true".
-   fake_render_gesture: Text based simple renderer of gesture. Default value is "true".
-   fake_render_screen: Text based simple renderer of screen. Default value is "true".
-   fake_render_facial_expression: Text based simple renderer of facial. Default value is "true".
-   fake_render_sound: Text based simple renderer of sound. Default value is "true".
-   fake_render_mobility: Text based simple renderer of mobility. Default value is "true".
