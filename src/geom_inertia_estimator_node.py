import sys

import rospy
from geom_inertia_estimator.msg import ParameterEstimates, MotorRPM
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu

from estimator import InertiaEstimator

if __name__ == '__main__':
    rospy.init_node("geom_inertia_estimator")
    rospy.myargv(argv=sys.argv)

    estimator = InertiaEstimator()
    estimator.onInit()
    estimator.sub_rpm_ = rospy.Subscriber("rpm", MotorRPM, estimator.rpm_callback)
    estimator.sub_odom_ = rospy.Subscriber("/mavros/local_position/pose_cov", PoseWithCovarianceStamped,
                                           estimator.pose_callback)
    estimator.sub_imu_ = rospy.Subscriber("/mavros/imu/data", Imu, estimator.imu_callback)

    estimator.pub_estimates_ = rospy.Publisher("param_estimates", ParameterEstimates, queue_size=10)
    while not rospy.is_shutdown():
        rospy.sleep(0.0001)

