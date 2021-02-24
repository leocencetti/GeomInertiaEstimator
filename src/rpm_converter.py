# File created by Leonardo Cencetti on 2/14/21

import rospy
from geom_inertia_estimator.msg import MotorRPM
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from mavros_msgs.msg import RCOut
import numpy as np

class Transformer:
    def __init__(self):
        self.node = rospy.init_node('RPM_converter')
        self.sub1 = rospy.Subscriber("/mavros/rc/out", RCOut, callback=self.transformRPM)
        self.sub2 = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.transformPose)
        self.pub1 = rospy.Publisher('/rpm', MotorRPM, queue_size=10)
        self.pub2 = rospy.Publisher('/mavros/local_position/pose_cov', PoseWithCovarianceStamped, queue_size=10)
        self.cov = np.eye(6) * 1e-4

    def transformRPM(self, msg: RCOut):
        out = MotorRPM()
        out.header = msg.header
        out.rpm = [*msg.channels[:4]]
        self.pub1.publish(out)

    def transformPose(self, msg: PoseStamped):
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.pose.pose = msg.pose
        out.pose.covariance = list(self.cov.flat)
        self.pub2.publish(out)


if __name__ == '__main__':
    node = Transformer()
    while not rospy.is_shutdown():
        rospy.spin()
