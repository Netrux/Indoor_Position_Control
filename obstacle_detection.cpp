import pyrealsense2 as rs
import mavros
import rospy
from std_msgs.msg import Int32,Int32MultiArray,MultiArrayDimension
import numpy as np
import cv2
import time
areaObj = 1000
frame_cx = 320
frame_cy = 240
clipping_distance_in_meters = 1 #meters
vectors = []
a = Int32MultiArray()
a.layout.dim.append(MultiArrayDimension())
#a.layout.dim[0].label = "x"
a.data = [0,0]
z = np.zeros(2)
avg_x = 0
avg_y = 0


pipeline = rs.pipeline()

config = rs.config()
config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
lower_blue = np.array([5,5,5])
upper_blue = np.array([255,255,255])
profile = pipeline.start(config)

# Getting the depth sensor's depth scale(0.001)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

kernel = np.ones((5,5),np.uint8)
kernel1 = np.ones((5,5),np.uint8)
pub = rospy.Publisher('object_found', Int32MultiArray, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(30) # 10hz

# Streaming loop
try:
    while True:
        start = time.time()
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame :
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        # print(depth_image.shape)
        # print(color_image.shape)
        grey_color = 3000

        depth_image2 = np.where((depth_image > clipping_distance) | (depth_image <= 0), grey_color, depth_image)
        sorted_depth_vector = np.argpartition(depth_image2,25,None)
        sorted_depth_vector2 = sorted_depth_vector[:20]
        min_depth_col = sorted_depth_vector2/640
        min_depth_row = sorted_depth_vector2%640
        col_centroid = (np.sum(min_depth_col))/20
        row_centroid = (np.sum(min_depth_row))/20

        grey_color = 0
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        bg_removed1 = bg_removed

        hsv = cv2.cvtColor(bg_removed1, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_blue,upper_blue)

        add = 0
        i = j = 0
        add_x = add_y = 0
        #print(depth_image[479,639])
        while i < len(mask):
            while j < len(mask[0]) :
                add_x = add_x + (mask[i][j]/250) * (j-320)
                j += 1
            add_y = add_y + (mask[i][j-1]/255) * (i-240)
            i += 1
        #print(add_x)
        add_x = float(add_x)/640.0
        add_y = float(add_y)/480.0
        print(add_x,add_y)

        lineThickness = 2
        cv2.line(mask, (320, 240), (int(add_x) + 320, int(add_y) + 240), (255,255,0), lineThickness)
        cv2.circle(mask,(int(add_x) + 320,int(add_y) +240),7,(255,0,0),-1)

        a.data = [add_x,add_y]

        cv2.imshow('color_image',mask)
        rospy.loginfo(a)
        pub.publish(a)
        rate.sleep()
        key = cv2.waitKey(1)
        #print(1/(time.time()-start))
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()
