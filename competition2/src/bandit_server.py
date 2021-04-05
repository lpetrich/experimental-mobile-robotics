#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse 
from competition2.srv import BanditStep, BanditStepResponse, BanditChoice, BanditChoiceResponse
import numpy as np

room_a = rospy.get_param('/bandit_server/room_a', 1)
room_b = rospy.get_param('/bandit_server/room_b', 2)
passcode = rospy.get_param('/bandit_server/passcode', 42)
num_arms = rospy.get_param('/bandit_server/num_arms', 2)
probs = rospy.get_param('/bandit_server/probs', "0.8, 0.5")
probs = [float(i) for i in probs.split(',')]
max_idx = probs.index(max(probs))
# print(max_idx)
# print("num_arms " + str(num_arms))
# print("passcode " + str(passcode))
# print("probs " + str(probs))

def sample_reward(action):
	# stochastic reward return 1 for success and 0 for failure
	return 1 if (np.random.random() < probs[action]) else 0

def bandit_step_callback(request):
	action = request.action - 1
	response = BanditStepResponse()
	if request.code != passcode or action < 0 or action >= num_arms:
		response.valid = False
		response.reward = -np.inf
	else:
		response.valid = True
		response.reward = sample_reward(action)
	return response

def bandit_choice_callback(request):
	arm = request.arm - 1
	response = BanditChoiceResponse()
	if arm == max_idx:
		response.room = room_a
	else:
		response.room = room_b
	return response

rospy.init_node('bandit_server') 

my_service = rospy.Service('/bandit_step', BanditStep , bandit_step_callback)
my_service = rospy.Service('/bandit_choice', BanditChoice , bandit_choice_callback)

rospy.spin()