#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import UInt8
import cormodule
import visao_module
from sensor_msgs.msg import LaserScan

#02

bridge = CvBridge()
viu_bottle = False

cv_image = None
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

centro_caixa = []
media = []
distancia = []


area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

aa = 0

def auto_canny(image, sigma=0.33):
	# compute the median of the single channel pixel intensities
	v = np.median(image)

	# apply automatic Canny edge detection using the computed median
	lower = int(max(0, (1.0 - sigma) * v))
	upper = int(min(255, (1.0 + sigma) * v))
	edged = cv2.Canny(image, lower, upper)

	# return the edged image
	return edged

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
	global cv_image
	global centro
	
	global centro_caixa
	global media
	global area

	global viu_bottle


	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")

		centro, imagem, resultados =  visao_module.processa(cv_image)

		media, centro_caixa, area =  cormodule.identifica_cor(cv_image)
		# print(media)
		# print("q")
		# print(centro_caixa)
		# print("s")


		for r in resultados:
						
            # print(r) - print feito para documentar e entender
			if r[0] == "bottle":
				viu_bottle = True

		depois = time.clock()
        # Desnecessário - Hough e MobileNet já abrem janelas
        #cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)

distancia = []
def scaneou(dado):
	# print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	# print("Leituras:")
	print(np.array(dado.ranges).round(decimals=2))

	global distancia

	distancia = dado.ranges
#	print(dado)
#	print('')

def BumpNumber(dado):
	global bump
	bump = dado.data
	print("BUMPER: {}".format(bump))



if __name__=="__main__":

	rospy.init_node("Projeto1")
	topico_imagem = "/raspicam_node/image/compressed"

	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
	print("Usando ", topico_imagem)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/bumper", UInt8, BumpNumber)
	laser = rospy.Subscriber("/scan", LaserScan, scaneou)

	bump = -1
	try:

		while not rospy.is_shutdown():
			aa += 1
			if bump == 3:

				vel = Twist(Vector3(0.06, 0, 0), Vector3(0, 0, 0.0))
				velocidade_saida.publish(vel) 
				rospy.sleep(2)


				bump = -1
				continue

			if bump == 4: 

				vel = Twist(Vector3(0.06, 0, 0), Vector3(0, 0, 0.0))
				velocidade_saida.publish(vel) 
				rospy.sleep(2)

				bump = -1	
				continue


			if bump == 2:

				vel = Twist(Vector3(-0.07, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel) 
				rospy.sleep(1)



				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.2))
				velocidade_saida.publish(vel) 
				rospy.sleep(2.0) 

				vel = Twist(Vector3(0.07, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel) 
				rospy.sleep(1)

				bump = -1
				continue

			if bump == 1:

				vel = Twist(Vector3(-0.07, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel) 
				rospy.sleep(1)


				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.2))
				velocidade_saida.publish(vel) 
				rospy.sleep(2.0) 

				vel = Twist(Vector3(0.07, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel) 
				rospy.sleep(1)

				bump = -1  
				continue  

			# else:

			# 	vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.0))
			# 	velocidade_saida.publish(vel) 
			# 	rospy.sleep(1) 


			if viu_bottle:

				vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/3))
				velocidade_saida.publish(vel)
				rospy.sleep(1)

				vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
				velocidade_saida.publish(vel)
				rospy.sleep(0.5)


				vel = Twist(Vector3(0.08,0,0), Vector3(0,0,0))
				velocidade_saida.publish(vel)
				rospy.sleep(1.2)

				viu_bottle = False

			if len(media) != 0 and len(centro_caixa) != 0 and area > 50000:
				# print("Média dos vermelhos: {0}, {1}".format(media[0], media[1]))
				# print("Centro dos vermelhos: {0}, {1}".format(centro_caixa[0], centro_caixa[1]))
				
				if media[0] < centro_caixa[1] + 30:
					vel = Twist(Vector3(0.08,0,0), Vector3(0,0,math.pi/5))
					velocidade_saida.publish(vel)
					rospy.sleep(0.1)

				elif media[0] > centro_caixa[1] - 30:
					vel = Twist(Vector3(0.08,0,0), Vector3(0,0,-math.pi/5))
					velocidade_saida.publish(vel)
					rospy.sleep(0.1)

				else:
					vel = Twist(Vector3(0.06,0,0), Vector3(0,0,0))
					velocidade_saida.publish(vel)
					rospy.sleep(0.1)

			for i in range(len(distancia)):
				if distancia[i] != 0 and distancia[i] <= 0.2:
					if i > 0 and i <= 90:

						vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						vel = Twist(Vector3(-0.08,0,0), Vector3(0,0,-0.2))
						velocidade_saida.publish(vel)
						rospy.sleep(2.0)
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(2.0)
						continue
					
					if i > 90 and i <= 180:

						vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						vel = Twist(Vector3(0.08,0,0), Vector3(0,0,-0.2))
						velocidade_saida.publish(vel)
						rospy.sleep(2.0)
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(2.0)
						continue

					if i > 180 and i <= 270:

						vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						vel = Twist(Vector3(0.08,0,0), Vector3(0,0,0.2))
						velocidade_saida.publish(vel)
						rospy.sleep(2.0)
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(2.0)
						continue

					if i > 270 and i <= 360:

						vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						vel = Twist(Vector3(-0.08,0,0), Vector3(0,0,0.2))
						velocidade_saida.publish(vel)
						rospy.sleep(2.0)
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(2.0)
						continue
				if aa < 6:
					distancia = []
					aa = 0

	except rospy.ROSInterruptException:
		print("Ocorreu uma exceção com o rospy")