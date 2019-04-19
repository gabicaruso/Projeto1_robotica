#! /usr/bin/env python
# -*- coding:utf-8 -*-

#Usando o simulador e o LIDAR simulado, faça um robô avançar quando o obstáculo bem à sua frente estiver a menos de 1.0m e recuar quando estiver a mais de 1.02 m.

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

distancia = 0
def scaneou(dado):
	print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	print("Leituras:")
	print(np.array(dado.ranges).round(decimals=2))

	global distancia

	distancia = dado.ranges[0]
	print(dado)
	print('')
	# ranges =  
	

	


if __name__=="__main__":

	rospy.init_node("le_scan")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)



	while not rospy.is_shutdown():

		if distancia > 1:
            		vel = Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))
            		velocidade_saida.publish(vel)

        	if distancia < 1.02:
            		vel = Twist(Vector3(-0.5, 0, 0), Vector3(0, 0, 0))
            		velocidade_saida.publish(vel)
		
		
