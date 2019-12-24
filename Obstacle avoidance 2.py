#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose,Twist
from math import atan2
from math import sqrt

from std_msgs.msg import Float64


x = 0.0
y = 0.0
x_linear=0
angle_to_goal = 0.0
p_error=0
z_angular=0
dis = 0

gps_flag=0


out=Twist()

def callback(msg,pub):

	
	global angle_to_goal
	global p_error
	global z_angular
	global x_linear
	global gps_flag
	theta = msg.data
	theta = theta

	
	error=angle_to_goal-theta
	#print("error")
	#print(error)
	if(error>180):
		error=error-360



	
	#print("error_updated")
	#print(error)
	kp=0.35
	kd=0.0	


        if(error>20 or error<-20):
        	differential=error-p_error
        	z_angular=kp*error+kd*differential
        	p_error=z_angular
        	#print(kp)
        	if(z_angular>10):
        	    z_angular=10
		    x_linear=0
        	    #print("greater than 10")
        	if(z_angular<-10):
        	    #print("less than -10")
        	    z_angular=-10
		    x_linear=0
        	#print("Setting Angle")
        	#print(z_angular)
        	#print("************")
	else:
		print("Going Forward")

        	if(dis>1):
            		z_angular=0
            		x_linear=10
        	else:
			z_angular=0
            		x_linear=0




    	#print("publishing")
	if(z_angular!=0):
        	out.angular.z=z_angular
        	out.linear.x=0
		pub.publish(out)
	else:
		out.angular.z=0
        	out.linear.x=x_linear
		#out.linear.x=0
		pub.publish(out)


def newOdom(msg,pub):
    global x
    global y
    global dis
    global angle_to_goal
    global gps_flag

    x = msg.position.x
    #transform from west to north
    y =msg.position.y

    x=-x
    y=-y

    #print("calculating distance")
    dis = ((x*x)+(y*y))

    dis=sqrt(dis)
    print("************")		
    print("{}Angle to goal".format(angle_to_goal))		
    print("{}distance to goal".format(dis))
    #print(y)
    #print(x)
    if(gps_flag==1):
	while(3>2):
		out.angular.z=0
        	out.linear.x=0

		pub.publish(out)

		print("GOAL REACHED SUCCESSFULLY!!!!!!!")

    if(dis<1):
	print("GOAL REACHED SUCCESSFULLY!!!!!!!")
	gps_flag=1
    #print("angle")
    #print(atan2(y,x))
    #wrt north clockwise
 
    if(x>0 and y>0):
		angle_to_goal=((atan2(y,x)*180)/3.141519)
		angle_to_goal=90-angle_to_goal
    elif(x>0 and y<0):
	        angle_to_goal=((atan2((-1*y),x)*180)/3.141519)
		angle_to_goal=90+angle_to_goal
    elif(x<0 and y>0):
	        angle_to_goal=((atan2(y,(-1*x))*180)/3.141519)
		angle_to_goal=270+angle_to_goal
    else:
	        angle_to_goal=((atan2((-1*y),(-1*x))*180)/3.141519)
		angle_to_goal=270-angle_to_goal
    if(z_angular!=0):
        	out.angular.z=z_angular
        	out.linear.x=0
		pub.publish(out)
    else:
		out.angular.z=0
        	out.linear.x=x_linear
		#out.linear.x=0
		pub.publish(out)

    #angle_to_goal =
    #if(angle_to_goal < 0):
    #	angle_to_goal = 360 + angle_to_goal

    #kp=0.5




if __name__ =='__main__' :

	rospy.init_node("speed_controller")

	pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
	sub1 = rospy.Subscriber("/distance", Pose, newOdom,pub)
	sub2 = rospy.Subscriber("/imu_degree", Float64,callback,pub)

	#r = rospy.Rate(100)
	rospy.spin()
