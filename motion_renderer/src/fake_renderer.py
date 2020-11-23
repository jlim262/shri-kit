#!/usr/bin/env python
#-*- encoding: utf8 -*-

import json
import rospy
import actionlib
import random
import re

import moveit_commander
from actionlib import SimpleActionClient
from moveit_msgs.msg import *

from sensor_msgs.msg import JointState
from social_msgs.srv import SocialMotion, SocialMotionResponse, SocialMotionRequest
from mind_msgs.msg import RenderItemAction, RenderItemResult, RenderItemFeedback
from mind_msgs.srv import GetInstalledGestures, GetInstalledGesturesResponse

import json
import rospkg
import os

from std_msgs.msg import ColorRGBA, String, Float64, Empty #, Bool, UInt16, Empty
from robocare_msgs.srv import TTSMake, SoundPlay, SoundPlayResponse

class TestMoveAction(object):
    def __init__(self):
        super(TestMoveAction, self).__init__()

        # self.robot = moveit_commander.RobotCommander()        
        # self.scene = moveit_commander.PlanningSceneInterface()  
        # rospy.sleep(2)

        # # create a action client of move group
        # self._move_client = SimpleActionClient('move_group', MoveGroupAction)
        # self._move_client.wait_for_server()

        # create a action client of move group
        self._move_client = SimpleActionClient('/execute_trajectory', ExecuteTrajectoryAction)
        self._move_client.wait_for_server()

    def execute(self, motion_name):
        # set joint name
        joint_names = ['Waist_Roll', 'Waist_Pitch', 'LShoulder_Pitch', 'LShoulder_Roll', 'LElbow_Pitch', 'LElbow_Yaw', 'LWrist_Pitch', 'LWrist_Roll']
        # joint_names = ['Waist_Roll', 'Waist_Pitch', 'RShoulder_Pitch', 'RShoulder_Roll', 'RElbow_Pitch', 'RElbow_Yaw', 'RWrist_Pitch', 'RWrist_Roll']

        # prepare a joint trajectory
        goal = ExecuteTrajectoryGoal()
        goal.trajectory.joint_trajectory.joint_names = joint_names
        
        self.create_trajectory_points(goal, motion_name)
        
        self._move_client.send_goal(goal)

    def json_to_trajectory(self):
        json_path = rospkg.RosPack().get_path('motion_renderer') + '/src'
        # json_path = os.path.join(json_path, 'hoho2.json')
        # json_path = os.path.join(json_path, 'hand_shake.json')
        json_path = os.path.join(json_path, 'question.json')

        with open(json_path) as f:
            data = json.load(f)

            trajectory = []

            for d in data['frames']:
                trajectory.append(d['head_waist'][2:] + d['arm'][:6])
                
                # trajectory.append(d['head_waist'][2:] + d['arm'][6:])
            
            return trajectory

    
    def create_trajectory_points(self, goal, motion_name):       
        # joint_trajectory = self.json_to_trajectory()
        
        # json_path = os.path.join(json_path, 'hoho2.json')
        # json_path = os.path.join(json_path, 'hand_shake.json')
        joint_trajectory = []
        
        json_path = rospkg.RosPack().get_path('motion_renderer') + '/src'
        json_path = os.path.join(json_path, motion_name + '.json')        
        with open(json_path) as f:
            data = json.load(f)

            if motion_name in ['question', 'hand_shake']:
                    arm = [x * -1 for x in d['arm'][6:]]
                else:
                    arm = d['arm'][:6]

                joint_trajectory.append(d['head_waist'][2:] + arm)

        print(len(joint_trajectory))
        print(joint_trajectory)
        

        for i,positions in enumerate(joint_trajectory):
            pt = trajectory_msgs.msg.JointTrajectoryPoint()
            pt.positions = positions
            pt.velocities = [0.1 for j in range(len(positions))]
            pt.accelerations = [0.0 for j in range(len(positions))]
            pt.time_from_start = rospy.Duration(0.1 * i)
            goal.trajectory.joint_trajectory.points.append(pt)

    def send(self, joint_values=[0, 0, 0, 0, 0, 0, 0, 0]):
        # set group        
        group_name = 'left_arm'
        group = moveit_commander.MoveGroupCommander(group_name)

        # prepare a joint goal
        goal = MoveGroupGoal()
        goal.request.group_name = group_name
        goal.planning_options.plan_only = False
        
        joint_names = group.get_active_joints()
        # joint_values = [0, 0, 0, 0, 0, 0, 0, 0]

        goal_constraint = Constraints()
        for i in range(len(joint_names)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_names[i]
            joint_constraint.position = joint_values[i]
            joint_constraint.weight = 1.0
            goal_constraint.joint_constraints.append(joint_constraint)
        goal.request.goal_constraints.append(goal_constraint)

        # send the goal
        self._move_client.send_goal(goal)

class FakeMotionRender:

    def __init__(self):
        rospy.init_node('fake_renderer', anonymous=True)
        self.moveit_action = TestMoveAction() 
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
                'greeting': ['hello2', 'hello', 'polite_hello', 'korean_greeting', 'right_hand_hello'],
                'hoho' : ['hoho2'],
                'handshake' : ['hand_shake'],
                'question' : ['question'],
                'zero_pose' : ['zero_pose'],
                'waiting': ['waiting_motion1'],
                'advice': ['advice_motion1'],
                'praise': ['praise_motion1'],
                'command': ['command_motion1'],
            }

        self.server = actionlib.SimpleActionServer(
            topic_name, RenderItemAction, self.execute_callback, False)
        self.server.start()

        self.__make_tts_service = rospy.ServiceProxy('/robocare_tts/make', TTSMake)
        self.__play_sound_service = rospy.ServiceProxy('/robocare_sound/play', SoundPlay)
        self.__play_sound_stop_publisher = rospy.Publisher('/robocare_sound/stop', Empty, queue_size=10)

        self.__tts_count = -1
        self.__tts_file_path = '/tmp/robocare_tts'+str(self.__tts_count)+'.wav'

        rospy.loginfo('[%s] initialized...' % rospy.get_name())
        rospy.spin()

    def request_tts(self, text):
        self.__tts_count = self.__tts_count + 1
        if self.__tts_count > 10:
            self.__tts_count = 0
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
                                
                            # req = SocialMotionRequest()
                            # req.file_name = self.motion_list[item_name][random.randint(0, len(self.motion_list[item_name]) - 1)]
                            # req.text = ''
                            # self.social_motion(req)
                            
                            self.moveit_action.execute(self.motion_list[item_name][0])

                        except KeyError:
                            rospy.logwarn('\033[94m[%s]\033[0m except) rendering gesture cmd [%s], name [%s]...'%(rospy.get_name(),
                                cmd,
                                self.motion_list['neutral'][random.randint(0, len(self.motion_list['neutral']) - 1)]))
                            # req = SocialMotionRequest()
                            # req.file_name = 'hello2'
                            # req.text = ''
                            # self.social_motion(req)
                            
                            # moveit_action.send()
                            self.moveit_action.execute('hello2')

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
            rospy.loginfo('\033[94m[%s]\033[0m rendering speech [%s]...' % (rospy.get_name(), goal.data))
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
