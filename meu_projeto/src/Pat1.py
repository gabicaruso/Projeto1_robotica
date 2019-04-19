#! /usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import UInt8


bump = -1

def BumpNumber(dado):
	global bump
	bump = dado.data
	print(dado)
	print('')
	

	
if __name__=="__main__":

	rospy.init_node("Pat1")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/bumper", UInt8, BumpNumber)

#manobras evasivas com o robô após a deteção de uma colisão no sensor *bumper*

	while not rospy.is_shutdown():


		if bump == 1:

			

			vel = Twist(Vector3(-0.06, 0, 0), Vector3(0, 0, 0.0))
			velocidade_saida.publish(vel) 
			rospy.sleep(2)

			vel = Twist(Vector3(0.0, 0, 0), Vector3(0, 0, -0.2))
			velocidade_saida.publish(vel) 
			rospy.sleep(1)

			bump = -1

		if bump == 2: 

			

			vel = Twist(Vector3(-0.06, 0, 0), Vector3(0, 0, 0.0))
			velocidade_saida.publish(vel) 
			rospy.sleep(2)

			vel = Twist(Vector3(0.0, 0, 0), Vector3(0, 0, 0.2))
			velocidade_saida.publish(vel) 
			rospy.sleep(1)

			bump = -1	


		if bump == 3 :

		    vel = Twist(Vector3(0.13, 0, 0), Vector3(0, 0, 0.0))
		    velocidade_saida.publish(vel) 
		    rospy.sleep(2.0) 

		    bump = -1

		if bump == 4 :

		    vel = Twist(Vector3(0.13, 0, 0), Vector3(0, 0, 0.0))
		    velocidade_saida.publish(vel) 
		    rospy.sleep(2.0) 

		    bump = -1    

		else:

			vel = Twist(Vector3(0.05, 0, 0), Vector3(0, 0, 0.0))
			velocidade_saida.publish(vel) 
			rospy.sleep(1) 
			





		print(bump)


		
