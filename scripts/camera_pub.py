import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

import numpy as np

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)


print(cap.isOpened())
bridge = CvBridge()

def talker():
    pub = rospy.Publisher('/webcam/compressed', CompressedImage, queue_size=1)
    rospy.init_node('image', anonymous=False)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break
        
        frame = cv2.resize(frame, (640, 640))
        # Compress the image
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 20]  # Adjust quality here (0-100)
        _, jpg_data = cv2.imencode('.jpg', frame, encode_param)
        
        # Create a CompressedImage message and publish
        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = np.array(jpg_data).tobytes()
        pub.publish(msg)

        if cv2.waitKey(1) == ord('q'):
            break
        if rospy.is_shutdown():
            break

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
