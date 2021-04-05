#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse 
from competition2.srv import BanditStep, BanditStepResponse, BanditAnswer, BanditAnswerResponse
from competition2.srv import ShapesAnswer, ShapesAnswerResponse
import numpy as np

shapes_count = rospy.get_param('/competition2_server/shapes_count', 1)
shapes_type = rospy.get_param('/competition2_server/shapes_type', "blue cube")
shapes_room_a = rospy.get_param('/competition2_server/shapes_room_a', 1)
shapes_room_b = rospy.get_param('/competition2_server/shapes_room_b', 2)

# Parameters for bandit room
bandit_room_a = rospy.get_param('/competition2_server/bandit_room_a', 1)
bandit_room_b = rospy.get_param('/competition2_server/bandit_room_b', 2)
bandit_passcode = rospy.get_param('/competition2_server/bandit_passcode', 42)
bandit_num_arms = rospy.get_param('/competition2_server/bandit_num_arms', 2)
bandit_probs = rospy.get_param('/competition2_server/bandit_probs', "0.8, 0.5")
bandit_probs = [float(i) for i in bandit_probs.split(',')]
bandit_max_idx = bandit_probs.index(max(bandit_probs))

def shapes_answer_callback(request):
	count = request.count
	response = ShapesAnswerResponse()
	if count == shapes_count:
		response.room = shapes_room_a
	else:
		response.room = shapes_room_b
	return response

def sample_reward(action):
	# stochastic reward return 1 for success and 0 for failure
	return 1 if (np.random.random() < bandit_probs[action]) else 0

def bandit_step_callback(request):
	action = request.action - 1
	response = BanditStepResponse()
	if request.code != bandit_passcode or action < 0 or action >= bandit_num_arms:
		response.valid = False
		response.reward = -np.inf
	else:
		response.valid = True
		response.reward = sample_reward(action)
	return response

def bandit_answer_callback(request):
	arm = request.arm - 1
	response = BanditAnswerResponse()
	if arm == bandit_max_idx:
		response.room = bandit_room_a
	else:
		response.room = bandit_room_b
	return response

rospy.init_node('competition2_server') 

shapes_answer_srv = rospy.Service('/shapes_answer', ShapesAnswer, shapes_answer_callback)

# Servives for bandit room
bandit_step_srv = rospy.Service('/bandit_step', BanditStep, bandit_step_callback)
bandit_answer_srv = rospy.Service('/bandit_answer', BanditAnswer, bandit_answer_callback)

rospy.spin()