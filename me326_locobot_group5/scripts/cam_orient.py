from std_msgs.msg import Float64
import rospy


class OrientCamera(object):
	"""docstring for OrientCamera"""
	def __init__(self, tilt_topic = "/locobot/tilt_controller/command", pan_topic = "/locobot/pan_controller/command"):		
		self.orient_pub = rospy.Publisher(tilt_topic, Float64, queue_size=1, latch=True)
		self.pan_pub = rospy.Publisher(pan_topic, Float64, queue_size=1, latch=True)

	def tilt_camera(self,angle=0.5):
		msg = Float64()
		msg.data = angle
		self.orient_pub.publish(msg)
		# print("cause orientation, msg: ", msg)

	def pan_camera(self,angle=0.5):
		msg = Float64()
		msg.data = angle
		self.pan_pub.publish(msg)