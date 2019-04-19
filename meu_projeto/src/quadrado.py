#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Twist, Vector3

v = 0.2# Velocidade linear
w = math.pi/2 # Velocidade angular

if __name__ == "__main__":
    rospy.init_node("roda_exemplo")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)

    try:
        while not rospy.is_shutdown():
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            pub.publish(vel)
            rospy.sleep(0.5)

            vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
            pub.publish(vel)
            rospy.sleep(5.0)

            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            pub.publish(vel)
            rospy.sleep(0.5)

            vel = Twist(Vector3(0,0,0), Vector3(0,0,w))
            pub.publish(vel)
            rospy.sleep(0.9788)

            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            pub.publish(vel)
            rospy.sleep(0.5)


    except rospy.ROSInterruptException:
        print("Ocorreu uma exceÃ§Ã£o com o rospy")
