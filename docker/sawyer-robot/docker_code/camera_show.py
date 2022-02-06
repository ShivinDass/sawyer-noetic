import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


def callback(data):
	br = CvBridge()
	rospy.loginfo('recieving image')
	cv2.imshow("camera", cv2.cvtColor(br.imgmsg_to_cv2(data), cv2.COLOR_BGR2RGB))
	cv2.waitKey(1)

def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('/usb_cam1/image_raw', Image, callback)
	rospy.spin()
	cv2.destroyAllWindows()

if __name__=='__main__':
	listener()