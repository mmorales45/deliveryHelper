import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import cv2
import numpy as np
import mediapipe as mp
from std_msgs.msg import String

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

class GetData(object):

  def __init__(self):
        self._bridge = CvBridge()
        rospy.Subscriber("/ridgeback/bumblebee/left/image_rect", Image, self.callback_color)
        self.pub = rospy.Publisher('/delivery_state', String, queue_size=10)
        rospy.sleep(2.0)
        self.pub.publish("Follow")
        # self.show_img()


  def show_img(self,img):
      """ 
      Function:
          Displays a top down view of the robot and ball
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
      with mp_hands.Hands(
        model_complexity=0,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as hands:
        # while cap.isOpened():
        #   success, img = cap.read()
        #   if not success:
        #     print("Ignoring empty camera frame.")
        #     # If loading a video, use 'break' instead of 'continue'.
        #     continue
          img.flags.writeable = False
          results = hands.process(img)
          image_height, image_width, _ = img.shape
          img.flags.writeable = True
          # img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
          # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
          if results.multi_hand_landmarks:
            self.pub.publish("deliver")
            # print("found hands")
            for hand_landmarks in results.multi_hand_landmarks:
              # print('hand_landmarks:', hand_landmarks)
              print(
              f'Index finger tip coordinates: (',
              f'{hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x * image_width}, '
              f'{hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y * image_height})'
              )
              mp_drawing.draw_landmarks(
                img,
                hand_landmarks,
                mp_hands.HAND_CONNECTIONS,
                mp_drawing_styles.get_default_hand_landmarks_style(),
                mp_drawing_styles.get_default_hand_connections_style())
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