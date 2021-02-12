import sys

import rospy

from .estimator import InertiaEstimator
from geom_inertia_estimator import ParameterEstimates
if __name__ == '__main__':
    rospy.init_node("geom_inertia_estimator")
    rospy.myargv(argv=sys.argv)

    estimator = InertiaEstimator()
    estimator.sub_rpm_ = rospy.Subscriber("rpm", 10, estimator.rpm_callback)
    estimator.sub_odom_ = rospy.Subscribe("pose", 10, estimator.pose_callback)
    estimator.sub_imu_ = rospy.Subscribe("imu", 10, estimator.imu_callback)

    estimator.pub_estimates_ = rospy.Publisher("param_estimates", ParameterEstimates, 10)

    rospy.spin()
