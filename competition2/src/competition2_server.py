#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse 
from competition2.srv import BanditStep, BanditStepResponse, BanditAnswer, BanditAnswerResponse
from competition2.srv import ShapesAnswer, ShapesAnswerResponse
from competition2.srv import MazeAnswer, MazeAnswerResponse
import numpy as np

shapes_count = rospy.get_param('/competition2_server/shapes_count', 6)
shapes_type = rospy.get_param('/competition2_server/shapes_type', "red cube")
shapes_room = rospy.get_param('/competition2_server/shapes_room', 19)

bandit_room = rospy.get_param('/competition2_server/bandit_room', 6)
bandit_passcode = rospy.get_param('/competition2_server/bandit_passcode', 42)
bandit_num_arms = rospy.get_param('/competition2_server/bandit_num_arms', 2)
bandit_probs = rospy.get_param('/competition2_server/bandit_probs', "0.8, 0.5")
bandit_probs = [float(i) for i in bandit_probs.split(',')]
bandit_max_idx = bandit_probs.index(max(bandit_probs))

maze_room = rospy.get_param('/competition2_server/maze_room', 3)
maze_passcode = rospy.get_param('/competition2_server/maze_passcode', 37)

final_room = rospy.get_param('/competition2_server/final_room', 8)

def shapes_answer_callback(request):
	count = request.count
	response = ShapesAnswerResponse()
	response.room = bandit_room
	if count == shapes_count:
		response.how = "Revolver"
	else:
		response.how = ""
	return response

def sample_reward(action):
	# stochastic reward return 1 for success and 0 for failure
	return 1 if (np.random.random() < bandit_probs[action]) else 0

def bandit_step_callback(request):
	action = request.action - 1
	response = BanditStepResponse()
	if request.passcode != bandit_passcode or action < 0 or action >= bandit_num_arms:
		response.valid = False
		response.reward = -np.inf
	else:
		response.valid = True
		response.reward = sample_reward(action)
	return response

def bandit_answer_callback(request):
	arm = request.arm - 1
	response = BanditAnswerResponse()
	response.room = maze_room
	if arm == bandit_max_idx:
		response.where = "Lounge"
	else:
		response.where = ""
	return response

def maze_answer_callback(request):
	response = MazeAnswerResponse()
	response.room = final_room
	if request.passcode == maze_passcode:
		response.who = "Dr. Taylor"
	else:
		response.who = ""
	return response

rospy.init_node('competition2_server') 

shapes_answer_srv = rospy.Service('/shapes_answer', ShapesAnswer, shapes_answer_callback)

bandit_step_srv = rospy.Service('/bandit_step', BanditStep, bandit_step_callback)
bandit_answer_srv = rospy.Service('/bandit_answer', BanditAnswer, bandit_answer_callback)

maze_answer_srv = rospy.Service('/maze_answer', MazeAnswer, maze_answer_callback)

rospy.spin()