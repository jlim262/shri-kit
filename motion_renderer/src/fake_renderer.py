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

    def execute(self):
        # set joint name
        joint_names = ['Waist_Roll', 'Waist_Pitch', 'LShoulder_Pitch', 'LShoulder_Roll', 'LElbow_Pitch', 'LElbow_Yaw', 'LWrist_Pitch', 'LWrist_Roll']

        # prepare a joint trajectory
        goal = ExecuteTrajectoryGoal()
        goal.trajectory.joint_trajectory.joint_names = joint_names
        
        self.create_trajectory_points(goal)
        
        self._move_client.send_goal(goal)

    def json_to_trajectory(self):
        json_path = rospkg.RosPack().get_path('motion_renderer') + '/src'
        json_path = os.path.join(json_path, 'hello2.json')

        with open(json_path) as f:
            data = json.load(f)

            trajectory = []

            for d in data['frames']:
                trajectory.append(d['head_waist'][2:] + d['arm'][:6])
            
            return trajectory

    
    def create_trajectory_points(self, goal):
        
        joint_trajectory = [[0.0, 0.0, 0.0, -1.45, 0.0, 0.0, 0.0, 0.0],
        [1.331517426717457e-05, 5.589214870188798e-06, 3.9801453376033634e-06, -1.3722135761824332, -7.377725225847421e-06, -3.2476645797335854e-06, 1.7768139683175826e-05, -5.6452930503939506e-08],
        [2.663034853434914e-05, 1.1178429740377597e-05, 7.960290675206727e-06, -1.2944271523648665, -1.4755450451694842e-05, -6.495329159467171e-06, 3.553627936635165e-05, -1.1290586100787901e-07],
        [3.994552280152371e-05, 1.6767644610566392e-05, 1.1940436012810091e-05, -1.2166407285472998, -2.2133175677542263e-05, -9.742993739200756e-06, 5.330441904952748e-05, -1.6935879151181853e-07],
        [5.326069706869828e-05, 2.2356859480755193e-05, 1.5920581350413454e-05, -1.1388543047297333, -2.9510900903389684e-05, -1.2990658318934341e-05, 7.10725587327033e-05, -2.2581172201575802e-07],
        [6.657587133587286e-05, 2.794607435094399e-05, 1.990072668801682e-05, -1.0610678809121665, -3.688862612923711e-05, -1.623832289866793e-05, 8.884069841587915e-05, -2.8226465251969757e-07],
        [7.989104560304742e-05, 3.3535289221132785e-05, 2.3880872025620182e-05, -0.9832814570945998, -4.4266351355084525e-05, -1.9485987478401512e-05, 0.00010660883809905497, -3.3871758302363706e-07],
        [9.320621987022199e-05, 3.912450409132159e-05, 2.7861017363223547e-05, -0.9054950332770331, -5.164407658093195e-05, -2.27336520581351e-05, 0.0001243769777822308, -3.9517051352757655e-07],
        [0.00010652139413739655, 4.4713718961510386e-05, 3.184116270082691e-05, -0.8277086094594664, -5.902180180677937e-05, -2.5981316637868683e-05, 0.0001421451174654066, -4.5162344403151605e-07],
        [0.00011983656840457113, 5.0302933831699184e-05, 3.5821308038430275e-05, -0.7499221856418997, -6.63995270326268e-05, -2.9228981217602272e-05, 0.00015991325714858246, -5.080763745354556e-07],
         [0.00011966242163907737, 5.007498475606553e-05, 3.560578988981433e-05, -0.7499224543571472, -6.66150517645292e-05, -2.944451080111321e-05, 0.0001596977235749364, -7.275779694282392e-07],
        [0.00011091530174534355, 4.9751434956366814e-05, 3.882189755741921e-05, -0.8277044928792346, -6.150354611583881e-05, -1.7372543093127513e-05, 0.00014459900290498303, 6.747138048214968e-06],
        [0.00010216818185160971, 4.94278851566681e-05, 4.203800522502408e-05, -0.9054865314013221, -5.639204046714844e-05, -5.300575385141812e-06, 0.00012950028223502968, 1.4221854065858175e-05],
        [9.34210619578759e-05, 4.9104335356969384e-05, 4.525411289262896e-05, -0.9832685699234095, -5.1280534818458065e-05, 6.77139232284389e-06, 0.00011440156156507632, 2.169657008350138e-05],
        [8.467394206414206e-05, 4.878078555727067e-05, 4.847022056023383e-05, -1.061050608445497, -4.616902916976769e-05, 1.8843360030829587e-05, 9.930284089512296e-05, 2.917128610114459e-05],
        [7.592682217040823e-05, 4.845723575757195e-05, 5.1686328227838714e-05, -1.1388326469675845, -4.10575235210773e-05, 3.09153277388153e-05, 8.42041202251696e-05, 3.66460021187878e-05],
        [6.717970227667442e-05, 4.813368595787324e-05, 5.4902435895443584e-05, -1.2166146854896718, -3.594601787238693e-05, 4.298729544680099e-05, 6.910539955521625e-05, 4.4120718136431e-05],
        [5.843258238294059e-05, 4.781013615817452e-05, 5.8118543563048455e-05, -1.2943967240117593, -3.083451222369655e-05, 5.50592631547867e-05, 5.400667888526289e-05, 5.159543415407421e-05],
        [4.968546248920677e-05, 4.748658635847581e-05, 6.133465123065333e-05, -1.3721787625338466, -2.572300657500617e-05, 6.713123086277239e-05, 3.8907958215309534e-05, 5.9070150171717416e-05],
        [4.093834259547294e-05, 4.716303655877709e-05, 6.455075889825821e-05, -1.4499608010559342, -2.061150092631579e-05, 7.92031985707581e-05, 2.3809237545356163e-05, 6.654486618936063e-05]]
        
        joint_trajectory = self.json_to_trajectory()
        print(len(joint_trajectory))
        print(joint_trajectory)
        

        for i,positions in enumerate(joint_trajectory):
            pt = trajectory_msgs.msg.JointTrajectoryPoint()
            pt.positions = positions
            pt.velocities = [0.1 for j in range(len(positions))]
            pt.accelerations = [0.0 for j in range(len(positions))]
            pt.time_from_start = rospy.Duration(0.2 * i)
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
                'greeting': ['hello', 'hello2', 'polite_hello', 'korean_greeting', 'right_hand_hello'],
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
                            
                            self.moveit_action.execute()

                        except KeyError:
                            rospy.logwarn('\033[94m[%s]\033[0m except) rendering gesture cmd [%s], name [%s]...'%(rospy.get_name(),
                                cmd,
                                self.motion_list['neutral'][random.randint(0, len(self.motion_list['neutral']) - 1)]))
                            # req = SocialMotionRequest()
                            # req.file_name = 'hello2'
                            # req.text = ''
                            # self.social_motion(req)
                            
                            # moveit_action.send()
                            self.moveit_action.execute()

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
