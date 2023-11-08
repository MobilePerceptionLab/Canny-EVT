#coding:utf-8

import roslib;  
import rosbag
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

path='/xxx/rgb/' #path for saving rgb

class ImageCreator():


   def __init__(self):
       self.bridge = CvBridge()
       with rosbag.Bag('/xxx/xxx.bag', 'r') as bag:   #path of rosbag
           for topic,msg,t in bag.read_messages():
               if topic == "":  #topic of rgb image
                       try:
                           cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
                       except CvBridgeError as e:
                           print e
                       timestr = "%.6f" %  msg.header.stamp.to_sec()
   
                       image_name = timestr+ ".png"
                       cv2.imwrite(path+image_name, cv_image) 

                       with open("rgb.txt", "a") as f1:
                           f1.write(timestr + ' ' + 'rgb/' + image_name + '\n')


if __name__ == '__main__':

   try:
       image_creator = ImageCreator()
   except rospy.ROSInterruptException:
       pass
