import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import cv2
import numpy as np
import mediapipe as mp
from std_msgs.msg import String
mp_pose = mp.solutions.pose
import math

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

class GetData(object):

  def __init__(self):
        ##Initialize the publishers and subscribers
        self._bridge = CvBridge()
        rospy.Subscriber("/ridgeback/bumblebee/left/image_rect", Image, self.callback_color)
        # rospy.Subscriber("/usb_cam/image_raw", Image, self.callback_color)
        self.pub = rospy.Publisher('/delivery_state', String, queue_size=10)
        
        # self.show_img()

  
  def show_img(self,img):
      """ 
      Function:
          Displays image of the camera with POSE landmarks
      Args:
          img- The color video stream
      Returns:
          None
      """

      # Show video
      # cap = cv2.VideoCapture(0)
      # # pub = rospy.Publisher('status_hand', String, queue_size=10)
      # # rospy.init_node('talker', anonymous=True)
      # rate = rospy.Rate(10) # 10
      with mp_pose.Pose(
          min_detection_confidence=0.5,
          min_tracking_confidence=0.5) as pose:
          # To improve performance, optionally mark the image as not writeable to
          # pass by reference.
          img.flags.writeable = False
          image_height, image_width, _ = img.shape
          results = pose.process(img)
          if results.pose_landmarks:
            l_wrist_x = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST].x * image_width
            l_wrist_y = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST].y * image_height
            l_shoulder_x = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].x * image_width
            l_shoulder_y = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].y * image_height
            # print(
            #     f'Right Shoulder coordinates: ('
            #     f'{r_shoulder_x}, '
            #     f'{r_shoulder_y})'
            # )
            
            # print(
            #     f'Right Wrist coordinates: ('
            #     f'{r_wrist_x}, '
            #     f'{r_wrist_y})'
            # )
            delta_x = l_wrist_x-l_shoulder_x
            delta_y = l_wrist_y-l_shoulder_y
            distance = math.sqrt(pow(delta_x,2)+pow(delta_y,2))
            angle = abs(math.atan2(delta_y,delta_x) * (360/(2*3.1415)))
            # print(f'{distance}') #get distance between points
            print(f'{angle}')
            if (angle > 60.0 and angle < 80.0):
                print(f'RED')
                
            if (angle > 0.0 and angle < 5.0):
                print(f'BLUE')
                
            # Draw the pose annotation on the image.
            img.flags.writeable = True
            mp_drawing.draw_landmarks(
                img,
                results.pose_landmarks,
                mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
          # else:
          #   self.pub.publish("no hands!")
          # Flip the image horizontally for a selfie-view display.
          cv2.imshow('MediaPipe Hands', cv2.flip(img, 1))
          # cv2.namedWindow("Image", 1)
          # cv2.imshow("Image", img)
          cv2.waitKey(3)
    
  def callback_color(self,data):
    """ 
    Function: Callback function for geting color image from camera
    Args:
        data (sensor_msgs/Image)
    Returns:
        None
    """
    #Get the color image from camera
    color_img = self._bridge.imgmsg_to_cv2(data, "passthrough")
    #convert to RGB
    color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
    self.show_img(color_img)
    
    
if __name__=="__main__":
    #Initialize the perceoption node
    rospy.init_node("perception")
    #Start GetData
    c = GetData()
    #Prevent the node from stopping
    rospy.spin()