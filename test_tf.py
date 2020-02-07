import tf
import tf2_msgs
import numpy
from geometry_msgs.msg import TransformStamped
import rospy
trans_x=1
trans_y=1
trans_z=1
rot_x=1
rot_y=1
rot_w=1
rot_z=1
from tf2_msgs import msg

def callback(data):
    global trans_x,trans_y,trans_z,rot_w,rot_x,rot_y,rot_z
    #rospy.loginfo(rospy.get_caller_id() + "check %s",data)
    trans_x = data.transforms[0].transform.translation.x
    trans_y = data.transforms[0].transform.translation.y
    trans_z = data.transforms[0].transform.translation.z

    rot_x = data.transforms[0].transform.rotation.x
    rot_y = data.transforms[0].transform.rotation.y
    rot_z = data.transforms[0].transform.rotation.z
    rot_w = data.transforms[0].transform.rotation.w

def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z
    
def listener():

    rospy.init_node('listener',anonymous=True)
    rospy.Subscriber("/tf",tf2_msgs.msg.TFMessage,callback)
    return [trans_x,trans_y,trans_z]

if __name__ == '__main__':
    #listener()
    while not rospy.is_shutdown():
        listener()
        print("printing_tranalation")
        print("X:",trans_x)
        print("Y:",trans_y)
        print("Z:",trans_z)
        print("printing_rotation")
        print("rot_X:",rot_x)
        print("rot_Y:",rot_y)
        print("rot_Z:",rot_z)
        print("rot_W:",rot_w)
        rot_x,rot_y,rot_z =  quaternion_to_euler(rot_x,rot_y,rot_z,rot_w)
    rospy.spin()
