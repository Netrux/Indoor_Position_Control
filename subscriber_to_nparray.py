import rospy 
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import math as m

global vertex
#vertex=0
incremet = 0.00435422640294
rospy.init_node("scanner")
rospy.Rate(10)

f = open("slam1.ply","w")
ply_header = '''ply
format ascii 1.0
element vertex 
property float x
property float y
property float z
end_header
'''
f.write(ply_header)
comp = np.ones([1,1440])*30

def callback_imu(msg):
	global x,y,z
	#print("hello",msg.pose.position)
	x = msg.pose.position.x
	y = msg.pose.position.y
	z = msg.pose.position.z
	qx = msg.pose.orientation.x
	qy = msg.pose.orientation.y
	qz = msg.pose.orientation.z
	qw = msg.pose.orientation.w
	#print("callback",x,y,z,qx)

def callback_scan(msg):
    #print("callback")
	k=np.array(msg.ranges)
	print("scan",k.shape)
	#print(type(k[27]))
	# if k[27]>25:
	# 	print("infinity")
	#np.savetxt("a.txt",k[27:29])
	# for i in range(0,1440,1):
	#     r = k[i]
	#     #print(i,r)
	#     ang = i*incremet
	#     if r<30:
	#         #print(1,ang,m.sin(ang),r)
	#         print(i,r)
	#         fx = r*m.sin(ang)# + x 
	#         fy = r*m.cos(ang)# + y
	#         #z = z
	#         f.write(str(fx)+" " + str(y)+" "+str(z)+"\n")
	for i in range(0,1440,1):
	    r = k[i]
	    ang = i*incremet
	    if r <30:
	        #print(i,ang,m.sin(ang),r)
	        #vertex+=1
	        f.write(str(r*m.sin(ang)+x)+" " + str(r*m.cos(ang)+y )+" "+str(z)+"\n")
	    else:
	        continue
def callback_vect(msg):

     #print("callback")
	print("scan")
	k=np.array(msg.ranges)
	k = k*(k<comp)*1
	k = k.reshape(1440,1)
	print(k.shape)
	ang = np.ones(k.shape)
	print(ang.shape)
	for i in range(0,1440,1):
	     #r = k[i]
	     ang[i,0] = i*incremet

	fx = k*(np.sin(ang))
	fy = k*(np.cos(ang))
	fz = np.ones(k.shape)*z
	
	final = np.array([fx,fy,fz])
	f.write(str(final))
	
rospy.Subscriber("/mavros/local_position/pose",PoseStamped,callback_imu,queue_size=10)
rospy.Subscriber("/scan",LaserScan,callback_scan,queue_size=10)

print("hello")

while (not rospy.is_shutdown()) and KeyboardInterrupt:
	rospy.spin()
f.close()
print("closed")#,vertex)

