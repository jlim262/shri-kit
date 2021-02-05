#!/usr/bin/env python
#-*- encoding: utf8 -*-

import json
import rospy
import actionlib
import random
import re

from sensor_msgs.msg import JointState
from social_msgs.srv import SocialMotion, SocialMotionResponse, SocialMotionRequest
from mind_msgs.msg import RenderItemAction, RenderItemResult, RenderItemFeedback
from mind_msgs.srv import GetInstalledGestures, GetInstalledGesturesResponse


import json
import rospkg
import os

from std_msgs.msg import ColorRGBA, String, Float64, Empty #, Bool, UInt16, Empty
from robocare_msgs.srv import TTSMake, SoundPlay, SoundPlayResponse

class FakeMotionRender:

    def __init__(self):
        rospy.init_node('fake_renderer', anonymous=True)

        try:
            topic_name = rospy.get_param('~action_name')
        except KeyError as e:
            print('[ERROR] Needed parameter for (topic_name)...')
            quit()

        if 'render_gesture' in rospy.get_name():
            self.GetInstalledGesturesService = rospy.Service(
                "get_installed_gestures",
                GetInstalledGestures,
                self.handle_get_installed_gestures
            )

            self.motion_list = {
                'neutral': ['neutral_motion1'],
                'encourge': ['encourge_motion1'],
                'attention': ['attention_motion1'],
                'consolation': ['consolation_motion1'],
                'greeting': ['hello', 'hello2', 'polite_hello', 'korean_greeting', 'right_hand_hello'],
                'hello': ['hello'],
                'waiting': ['waiting_motion1'],
                'advice': ['advice_motion1'],
                'praise': ['praise_motion1'],
                'command': ['command_motion1'],
                'hoho': ['hoho', 'hoho2'],
                'scratch_head': ['scratch_head'],
                'shake_hand': ['question']
            }

        self.server = actionlib.SimpleActionServer(
            topic_name, RenderItemAction, self.execute_callback, False)
        self.server.start()

        # self.pub_silbot_execution = rospy.Publisher('/reply_deprecated', Reply, queue_size=10)
        # self.cmd_pos_publisher = rospy.Publisher('/cmd_pos', JointState, queue_size=10)
        
    
        
        
        self.social_motion = rospy.ServiceProxy('/social_motion_player/play_motion', SocialMotion)
        self.social_motion.wait_for_service()

        self.__make_tts_service = rospy.ServiceProxy('/robocare_tts/make', TTSMake)
        self.__play_sound_service = rospy.ServiceProxy('/robocare_sound/play', SoundPlay)
        self.__play_sound_stop_publisher = rospy.Publisher('/robocare_sound/stop', Empty, queue_size=10)

        self.__tts_count = -1
        self.__tts_file_path = self.get_wav_path()
        
        rospy.loginfo('[%s] initialized...' % rospy.get_name())
        rospy.spin()

    def get_wav_path(self):
        path = '/tmp/robocare_tts_from_uoa' + str(self.__tts_count) + '.wav'
        return path

    def request_tts(self, text):
        self.__tts_count = self.__tts_count + 1        
        if self.__tts_count > 50:
            self.__tts_count = 0

        self.__tts_file_path = self.get_wav_path()
        self.__make_tts_service(text, self.__tts_file_path)

        self.__play_sound_service(self.__tts_file_path)

    def handle_get_installed_gestures(self, req):
        result = json.dumps(self.motion_list)
        return GetInstalledGesturesResponse(result)


    def execute_callback(self, goal):
        rospy.loginfo('\033[95m%s\033[0m rendering requested...' % rospy.get_name())
        result = RenderItemResult()
        feedback = RenderItemFeedback()

        success = True
        loop_count = 0

        if 'render_gesture' in rospy.get_name():
            (gesture_category, gesture_item) = goal.data.split('=')

            if gesture_category == 'pointing':
                parse_data = json.loads(gesture_item)
                rospy.loginfo('\033[94m[%s]\033[0m rendering pointing to xyz:%s, frame_id [%s]...'%(rospy.get_name(),
                    parse_data['xyz'], parse_data['frame_id']))

            elif gesture_category == 'gesture':
                (cmd, item_name) = gesture_item.split(':')
                rospy.loginfo(gesture_item)
                if cmd == 'tag':
                    match = re.search(r'\[(.+?)\]', item_name)
                    if match:
                        item_name = item_name.replace(match.group(0), '')
                        emotion = match.group(1)
                        rospy.loginfo(emotion)
                        try:
                            rospy.loginfo('\033[94m[%s]\033[0m match> rendering gesture cmd [%s], name [%s]...'%(rospy.get_name(),
                                cmd,
                                self.motion_list[item_name][emotion][random.randint(0, len(self.motion_list[item_name]) - 1)]))

                            
                        except (KeyError, TypeError):
                            rospy.loginfo('\033[94m[%s]\033[0m match> except) rendering gesture cmd [%s], name [%s]...'%(rospy.get_name(),
                                cmd,
                                self.motion_list[item_name][random.randint(0, len(self.motion_list[item_name]) - 1)]))
                    else:
                        try:
                            rospy.loginfo('\033[94m[%s]\033[0m ! rendering gesture cmd [%s], name [%s]...'%(rospy.get_name(),
                                cmd,
                                self.motion_list[item_name][random.randint(0, len(self.motion_list[item_name]) - 1)]))
                                
                            req = SocialMotionRequest()
                            # req.file_name = self.motion_list[item_name][random.randint(0, len(self.motion_list[item_name]) - 1)]
                            req.file_name = item_name
                            req.text = ''
                            self.social_motion(req)

                        except KeyError:
                            rospy.logwarn('\033[94m[%s]\033[0m except) rendering gesture cmd [%s], name [%s]...'%(rospy.get_name(),
                                cmd,
                                self.motion_list['neutral'][random.randint(0, len(self.motion_list['neutral']) - 1)]))
                            req = SocialMotionRequest()
                            # req.file_name = 'hello2'
                            req.file_name = item_name
                            req.text = ''
                            self.social_motion(req)

                elif cmd == 'play':
                    find_result = False
                    for k, v in self.motion_list.items():
                        if item_name in v:
                            find_result = True

                    if find_result:
                        rospy.loginfo('\033[94m[%s]\033[0m rendering gesture cmd [%s], name [%s]...'%(rospy.get_name(),
                            cmd, item_name))
                    else:
                        rospy.logwarn('\033[94m[%s]\033[0m rendering gesture cmd [%s], name [%s]...'%(rospy.get_name(),
                            cmd,
                            self.motion_list['neutral'][random.randint(0, len(self.motion_list['neutral']) - 1)]))

            loop_count = 10
        if 'render_speech' in rospy.get_name():
            rospy.loginfo('\033[94m[%s]\033[0m rendering speech [%s]...'%(rospy.get_name(), goal.data))
            loop_count = 10
            self.request_tts(goal.data)

        if 'render_screen' in rospy.get_name():
            rospy.loginfo('\033[94m[%s]\033[0m rendering screen [%s]...'%(rospy.get_name(), goal.data))
            loop_count = 10

        if 'render_mobility' in rospy.get_name():
            rospy.loginfo('\033[94m[%s]\033[0m rendering mobility [%s]...'%(rospy.get_name(), goal.data))
            loop_count = 10

        if 'render_facial_expression' in rospy.get_name():
            rospy.loginfo('\033[94m[%s]\033[0m rendering expression [%s]...'%(rospy.get_name(), goal.data))
            loop_count = 5

        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                success = False
                break

            feedback.is_rendering = True
            self.server.publish_feedback(feedback)
            rospy.sleep(0.1)

            loop_count = loop_count - 1
            if loop_count == 0:
                break

        if success:
            result.result = True
            self.server.set_succeeded(result)
            rospy.loginfo('\033[95m%s\033[0m rendering completed...' % rospy.get_name())
        else:
            rospy.loginfo('\033[95m%s\033[0m rendering canceled...' % rospy.get_name())


if __name__ == '__main__':
    m = FakeMotionRender()
