##########  MAIN FUNCTIONS  ##########
import logging
from math import sqrt
from queue import Queue

import rospy
from geom_inertia_estimator import MotorRPM
from geom_inertia_estimator import ParameterEstimates
from geometry_msgs import PoseWithCovarianceStamped
from numpy.linalg import cholesky
from scipy.linalg import block_diag, inv
from sensor_msgs import Imu

from .calc_EKF_F_optimized import calc_EKF_F_optimized
from .calc_EKF_H_imu_optimized_simple import calc_EKF_H_imu_optimized_simple
from .calc_EKF_H_odom_optimized_simple import calc_EKF_H_odom_optimized_simple
from .estimator_header import *
from .tools import *


# estimator class
class InertiaEstimator:
    def __init__(self):
        self.logger = logging.getLogger()
        # subscribers and publisher
        self._sub_imu: rospy.topics.Subscriber = None
        self._sub_odom: rospy.topics.Subscriber = None
        self._sub_rpm: rospy.topics.Subscriber = None

        self._pub_estimates: rospy.topics.Publisher = None

        # member variables
        self._consts: Consts = Consts()
        self._init_vars: InitVars = InitVars()
        self._state: StateWithCov = StateWithCov()
        self._input: Input = Input()
        self._inputList: Queue = Queue()
        self._measImuList: Queue = Queue()
        self._F_temp = np.empty(NUM_STATES_TANGENT * NUM_STATES_TANGENT)
        self._propagatedSigmaPointsMat = np.empty((NUM_STATES, NUM_SIGMAS))

        # status flags
        self._initialized: bool = False
        self._useEKF: bool = True

    def onInit(self, nh):
        # initialize often used variables as consts
        self._consts.zero3 = np.zeros((3, 3))
        self._consts.eye3 = np.eye(3)
        self._consts.eye_NST = np.eye(NUM_STATES_TANGENT)
        self._consts.rotate3 = np.diag((1, -1, -1))
        # self._consts.rotate6 << self._consts.rotate3, self._consts.zero3, self._consts.zero3, self._consts.rotate3
        self._consts.e_z = np.array((0, 0, 1))
        self._consts.g = 9.807

        self._init_vars.P0 = np.zeros((NUM_STATES_TANGENT, NUM_STATES_TANGENT))

        # read flag to see whether to use EKF or UKF
        tempd = rospy.get_param("useEKF", 1)
        self._useEKF = bool(tempd)

        # temp variables for faster reads
        temp30 = np.empty((30, 1))
        temp6 = np.empty((6, 1))
        temp3 = np.empty((3, 1))
        temp2 = np.empty((2, 1))
        temp = 0

        # read attitude of IMU
        temp6[0] = rospy.get_param("RImu/a/x", 5)
        temp6[1] = rospy.get_param("RImu/a/y", 5)
        temp6[2] = rospy.get_param("RImu/a/z", 5)
        temp6[3] = rospy.get_param("RImu/Omega/x", 1)
        temp6[4] = rospy.get_param("RImu/Omega/y", 1)
        temp6[5] = rospy.get_param("RImu/Omega/z", 1)
        self._consts.RImu = np.diag(temp6)

        # read process covariance
        Qx_temp = rospy.get_param("Qx", np.empty(NUM_STATES_TANGENT))
        self._consts.Qx = np.diag(Qx_temp)

        # read initial values
        # moment of inertia
        temp3[0] = rospy.get_param("x0/I/x", 1e-3)
        temp3[1] = rospy.get_param("x0/I/y", 1e-3)
        temp3[2] = rospy.get_param("x0/I/z", 1e-3)
        self._init_vars.I_tensor = np.diag(temp3)
        # mass
        rospy.get_param("x0/m", self._init_vars.m, 0.5)
        # position center of mass
        temp3[0] = rospy.get_param("x0/r_BM/x", 0)
        temp3[1] = rospy.get_param("x0/r_BM/y", 0)
        temp3[2] = rospy.get_param("x0/r_BM/z", 0)
        self._init_vars.r_BM = temp3
        # position pose sensor
        temp3[0] = rospy.get_param("x0/r_BP/x", 0)
        temp3[1] = rospy.get_param("x0/r_BP/y", 0)
        temp3[2] = rospy.get_param("x0/r_BP/z", 0)
        self._init_vars.r_BP = temp3
        # position IMU
        temp2[0] = rospy.get_param("x0/r_BI/x", 0)
        temp2[1] = rospy.get_param("x0/r_BI/y", 0)
        self._init_vars.r_BI = temp2

        # read initial state covariance
        # velocity
        temp30[6] = rospy.get_param("P0/v/x", 1e-3)
        temp30[7] = rospy.get_param("P0/v/y", 1e-3)
        temp30[8] = rospy.get_param("P0/v/z", 1e-3)
        # mass
        temp30[12] = rospy.get_param("P0/m", 0.5)
        # moment of inertia
        temp30[13] = rospy.get_param("P0/I/x", 1e-6)
        temp30[14] = rospy.get_param("P0/I/y", 1e-6)
        temp30[15] = rospy.get_param("P0/I/z", 1e-6)
        # position center of mass
        temp30[16] = rospy.get_param("P0/r_BM/x", 0.1)
        temp30[17] = rospy.get_param("P0/r_BM/y", 0.1)
        temp30[18] = rospy.get_param("P0/r_BM/z", 0.1)
        # position pose sensor
        temp30[19] = rospy.get_param("P0/r_BP/x", 0.1)
        temp30[20] = rospy.get_param("P0/r_BP/y", 0.1)
        temp30[21] = rospy.get_param("P0/r_BP/z", 0.1)
        # position IMU sensor
        temp30[22] = rospy.get_param("P0/r_BI/x", 0.1)
        temp30[23] = rospy.get_param("P0/r_BI/y", 0.1)
        # IMU bias acceleration
        temp30[24] = rospy.get_param("P0/b_a/x", 4)
        temp30[25] = rospy.get_param("P0/b_a/y", 4)
        temp30[26] = rospy.get_param("P0/b_a/z", 4)
        # IMU bias angular velocity
        temp30[27] = rospy.get_param("P0/b_Omega/x", 1)
        temp30[28] = rospy.get_param("P0/b_Omega/y", 1)
        temp30[29] = rospy.get_param("P0/b_Omega/z", 1)
        self._init_vars.P0 = np.diag(temp30)

        # read UKF tuning parameters
        temp = rospy.get_param("UKF/alpha", 0.4)
        self._consts.alpha = float(temp)
        temp = rospy.get_param("UKF/beta", 2)
        self._consts.beta = float(temp)
        temp = rospy.get_param("UKF/kappa", 0)
        self._consts.kappa = float(temp)

        # read attitude of pose sensor and IMU
        R_temp = rospy.get_param("R_BP", np.empty((9, 1)))
        self._consts.R_BP = R_temp.reshape((3, 3))
        self._consts.R_BP_6D = block_diag(self._consts.R_BP, self._consts.R_BP)
        self._consts.q_RB = Quat(self._consts.R_BP).normalized()
        R_temp = rospy.get_param("R_BI", np.empty((9, 1)))
        self._consts.R_BI = R_temp.reshape((3, 3))
        temp66 = block_diag(self._consts.R_BI, self._consts.R_BI)
        self._consts.RImu = temp66 @ self._consts.RImu @ temp66.T

        # calculate matrices for force & moment calculation for the case where M & B coincide
        P_f = np.empty((3, 20))
        P_M = np.empty((3, 20))
        num_rotors = 0
        for num_rotors in range(20):
            # read thrust coeff, drag moment coeff, attitude and position of rotors
            k_f = rospy.get_param("multirotor/rotor" + str(num_rotors + 1) + "/kf", 5.0e-9)
            k_M = rospy.get_param("multirotor/rotor" + str(num_rotors + 1) + "/km", 4.0e-11)
            R_temp = rospy.get_param("multirotor/rotor" + str(num_rotors + 1) + "/R", np.empty((9, 1)))
            t_temp = rospy.get_param("multirotor/rotor" + str(num_rotors + 1) + "/t", np.empty((3, 1)))
            R = R_temp.reshape((3, 3))
            t = t_temp

            Qx_temp = rospy.get_param("Qx", np.empty(NUM_STATES_TANGENT))
            self._consts.Qx = np.diag(Qx_temp)

            if R.norm() == 3:  # not defined rotor
                break
            else:
                # TODO save individual kf/km for each rotor
                self._consts.k_f = k_f
                self._consts.k_M = k_M
                P_f[:, num_rotors] = self._consts.k_f * R @ self._consts.e_z
                P_M[:, num_rotors] = self._consts.k_M * R @ self._consts.e_z + t.cross(
                    self._consts.k_f * R @ self._consts.e_z)
        self._consts.P_f = P_f[:3, :num_rotors]
        self._consts.P_M = P_M[:3, :num_rotors]
        self._consts.num_rotors = num_rotors

        # calculate the lambda and weights for unscented transform
        self._consts.lambd = self._consts.alpha * self._consts.alpha * (
                NUM_STATES_TANGENT + self._consts.kappa) - NUM_STATES_TANGENT
        self._consts.Wm = np.concatenate([
            self._consts.lambd / (NUM_STATES_TANGENT + self._consts.lambd),
            np.ones((2 * NUM_STATES_TANGENT, 1)) * (1 / (2 * (NUM_STATES_TANGENT + self._consts.lambd)))
        ], axis=0)
        self._consts.Wc = np.concatenate([
            self._consts.lambd / (NUM_STATES_TANGENT + self._consts.lambd) + (
                    1 - self._consts.alpha * self._consts.alpha + self._consts.beta),
            np.ones((2 * NUM_STATES_TANGENT, 1)) * (1 / (2 * (NUM_STATES_TANGENT + self._consts.lambd)))
        ], axis=0)

    def initializeFilter(self, init: InitVars):
        # set state value, state covariance values and time stamp
        self._state.X.r = init.r
        self._state.X.q = init.q
        self._state.X.Omega = init.Omega

        self._state.X.I_tensor = init.I_tensor
        self._state.X.m = init.m
        self._state.X.r_BM = init.r_BM
        self._state.X.r_BP = init.r_BP
        self._state.X.r_BI = init.r_BI

        self._state.X.b_a = np.zeros(3)
        self._state.X.b_Omega = np.zeros(3)

        self._state.t = init.t

        self._state.P = init.P0

        self._init_vars = InitVars()
        self._initialized = True

        self.publishEstimates(self._state)

        self.logger.info("filter initialized")

    def resetFilter(self):
        self._init_vars = InitVars()
        self._initialized = False
        self.logger.warning("reset {}".format(
            self._init_vars.rpmInitialized or self._init_vars.imuInitialized or self._init_vars.poseInitialized))

    def predictEKF(self, state: StateWithCov, input: Input):
        # calculate time diff to predicted state
        dt = (input.t - state.t).to_sec()
        if dt > 1.0e-5:
            orientation = qLog(state.X.q) * 2
            # calculate state transition matrix F using matlab generated function
            F = np.empty([NUM_STATES_TANGENT, NUM_STATES_TANGENT])
            calc_EKF_F_optimized(
                orientation[0], orientation[1], orientation[2],
                state.X.r[0], state.X.r[1], state.X.r[2],
                state.X.v[0], state.X.v[1], state.X.v[2],
                state.X.Omega[0], state.X.Omega[1], state.X.Omega[2],
                state.X.m,
                state.X.I_tensor[0, 0], state.X.I_tensor[1, 1], state.X.I_tensor[2, 2],
                state.X.r_BM[0], state.X.r_BM[1], state.X.r_BM[2],
                state.X.r_BP[0], state.X.r_BP[1], state.X.r_BP[2],
                state.X.r_BI[0], state.X.r_BI[1],
                state.X.b_a[0], state.X.b_a[1], state.X.b_a[2],
                state.X.b_Omega[0], state.X.b_Omega[1], state.X.b_Omega[2],
                sqrt(input.rpm_sq[0]), sqrt(input.rpm_sq[1]), sqrt(input.rpm_sq[2]), sqrt(input.rpm_sq[3]),
                dt,
                0,
                0.08, 0.071, 0.0095, 4.179e-9, F.flat)

            # update time stamp
            state.t = input.t
            # predicted mean state with non-linear model
            self.processModel(state.X, input, dt)
            # predicted state covariance with linearized model
            state.P = F * state.P @ F.T + self._consts.Qx * dt

    def predictUKF(self, state: StateWithCov, input: Input):
        # calculate time diff to predicted state
        dt: float = (input.t - state.t).to_sec()

        if dt > 1.0e-5:
            # create and propagate sigma points
            sigmaPoints: List[State] = []
            self.getSigmaPoints(sigmaPoints, state)
            for i in range(NUM_SIGMAS):
                self.processModel(sigmaPoints[i], input, dt)

            # calculate the mean
            self.calculateMeanStateSigma(state.X, sigmaPoints)

            # calculate the covariance
            diff = np.empty((NUM_STATES_TANGENT, NUM_SIGMAS))
            for i in range(NUM_SIGMAS):
                diff[:, i] = sigmaPoints[i].boxminus(state.X)

            state.P = diff @ np.diag(self._consts.Wc) @ diff.T + self._consts.Qx * dt

            # update time stamp and
            state.t = input.t

    def measUpdatePoseEKF(self, state: StateWithCov, meas: MeasPose):
        # since orientation measurement is linear, we use linear Kalman Filter update for orientation measurement
        # allocate matrices
        H = np.zeros((3, NUM_STATES_TANGENT))
        H[:3, :3] = np.eye(3)
        # innovation
        Xhat = np.empty((3, 1))
        Xhat = qBoxMinus(meas.q, state.X.q)
        # innovation covariance
        S_KF = meas.cov[:3, :3] + H @ state.P @ H.T
        # optimal kalman gain
        K_KF = state.P * H.T * inv(S_KF)
        # updated state estimate and state covariance estimate (a posteriori)
        dX = K_KF @ Xhat
        state.X.boxplus(dX)
        state.P = (self._consts.eye_NST - K_KF * H) * state.P * (self._consts.eye_NST - K_KF * H).T + K_KF * meas.cov[
                                                                                                             :3,
                                                                                                             :3] * K_KF.T

        # non-linear update for position measurement
        # prep calculations for the calculation of linearized observation model (H)
        orientation = qLog(state.X.q) * 2
        H_EKF = np.empty([3, NUM_STATES_TANGENT])
        # calculate linearized observation model using Matlab generated function
        calc_EKF_H_odom_optimized_simple(
            orientation[0], orientation[1], orientation[2],
            state.X.r[0], state.X.r[1], state.X.r[2],
            state.X.v[0], state.X.v[1], state.X.v[2],
            state.X.Omega[0], state.X.Omega[1], state.X.Omega[2],
            state.X.m,
            state.X.I_tensor[0, 0], state.X.I_tensor[1, 1], state.X.I_tensor[2, 2],
            state.X.r_BM[0], state.X.r_BM[1], state.X.r_BM[2],
            state.X.r_BP[0], state.X.r_BP[1], state.X.r_BP[2],
            state.X.r_BI[0], state.X.r_BI[1],
            state.X.b_a[0], state.X.b_a[1], state.X.b_a[2],
            state.X.b_Omega[0], state.X.b_Omega[1], state.X.b_Omega[2],
            0, 0, 0, 0,
            0,
            0,
            0.08, 0.071, 0.0095, 4.179e-9, H_EKF.flat)

        # mean of expected measurement using non-linear measurement model
        zpred = self.measurementModelPosition(state.X)
        # innovation covariance
        S_EKF = H_EKF @ state.P @ H_EKF.T + meas.cov[3:6, 3:6]
        # kalman gain
        K_EKF = state.P @ H_EKF.T @ inv(S_EKF)
        # updated state estimate (a posteriori)
        d = K_EKF @ (meas.r - zpred)
        state.X.boxplus(d)
        # updated state estimate covariance (a posteriori)
        state.P = (np.eye(NUM_STATES_TANGENT) - K_EKF @ H_EKF) @ state.P

    def measUpdatePoseUKF(self, state: StateWithCov, meas: MeasPose):
        # since orientation measurement is linear, we use linear Kalman Filter update for orientation measurement
        # allocate matrices
        H = np.zeros((3, NUM_STATES_TANGENT))
        H[:3, :3] = np.eye(3)
        # innovation
        Xhat = np.empty((3, 1))
        Xhat = qBoxMinus(meas.q, state.X.q)
        # innovation covariance
        S_KF = meas.cov[:3, :3] + H @ state.P @ H.T
        # optimal kalman gain
        K_KF = state.P @ H.T @ inv(S_KF)
        # updated state estimate and state covariance estimate (a posteriori)
        dX = K_KF @ Xhat
        state.X.boxplus(dX)
        state.P = (self._consts.eye_NST - K_KF @ H) @ state.P @ (self._consts.eye_NST - K_KF @ H).T + K_KF @ meas.cov[
                                                                                                             :3,
                                                                                                             :3] @ K_KF.T

        # Unscented Kalman Filter update step for position measurement
        # allocating sigma points
        L = 2 * NUM_STATES_TANGENT + 1
        sigmaPoints: List[State] = []
        propagatedSigmaPointsMat = np.empty((3, L))
        # calculate sigma points
        self.getSigmaPoints(sigmaPoints, state)
        # sigmapoint transformation through non-linear measurement model
        for i in range(L):
            propagatedSigmaPointsMat[:, i] = self.measurementModelPosition(sigmaPoints[i])
        # mean of transformed sigma points
        zhat = propagatedSigmaPointsMat @ self._consts.Wm
        # innovation
        diffZ = np.empty((3, L))
        diffZ = propagatedSigmaPointsMat - zhat.tile((1, L))
        # calculate innovation covariance
        diffX = np.empty((NUM_STATES_TANGENT, L))
        for i in range(L):
            diffX[:, i] = sigmaPoints[i].boxminus(state.X)
        S_UKF = diffZ @ np.diag(self._consts.Wc) @ diffZ.T + meas.cov[3:6, 3:6]
        # kalman gain
        Pcross = diffX @ np.diag(self._consts.Wc) @ diffZ.T
        K_UKF = Pcross @ inv(S_UKF)
        # updated covariance estimate (a posteriori)
        state.P = state.P - K_UKF @ S_UKF @ K_UKF.T
        # updated estimate (a posteriori)
        d = K_UKF @ (meas.r - zhat)
        state.X.boxplus(d)

    def measUpdateImuEKF(self, state: StateWithCov, input: Input, meas: MeasImu):
        # since angular velocity measurement is linear, we use linear Kalman Filter update for orientation measurement
        # allocate matrices
        H_P = np.zeros((3, NUM_STATES_TANGENT))
        H_P[:3, 9:12] = np.eye(3)
        H_P[:3, 27:30] = np.eye(3)
        # innovation
        Xhat = np.empty((3, 1))
        Xhat = meas.Omega - (state.X.Omega + state.X.b_Omega)
        # innovation covariance
        S_KF = H_P @ state.P @ H_P.T + self._consts.RImu[3:6, 3:6]
        # optimal kalman gain
        K_KF = state.P @ H_P.T @ inv(S_KF)
        # updated state estimate and state covariance estimate (a posteriori)
        dX = K_KF @ Xhat
        state.X.boxplus(dX)
        state.P = (self._consts.eye_NST - K_KF @ H_P) @ state.P @ (
                self._consts.eye_NST - K_KF @ H_P).T + K_KF @ self._consts.RImu[3:6, 3:6] @ K_KF.T

        # non-linear update for acceleration measurement
        # prep caluclaitons for the calculation of linearized observation model (H)
        orientation = qLog(state.X.q) * 2
        dt = (input.t - state.t).to_sec()
        H_EKF = np.empty([3, NUM_STATES_TANGENT])
        # calculate linearized observation model using Matlab generated function
        calc_EKF_H_imu_optimized_simple(
            orientation[0], orientation[1], orientation[2],
            state.X.r[0], state.X.r[1], state.X.r[2],
            state.X.v[0], state.X.v[1], state.X.v[2],
            state.X.Omega[0], state.X.Omega[1], state.X.Omega[2],
            state.X.m,
            state.X.I_tensor[0, 0], state.X.I_tensor[1, 1], state.X.I_tensor[2, 2],
            state.X.r_BM[0], state.X.r_BM[1], state.X.r_BM[2],
            state.X.r_BP[0], state.X.r_BP[1], state.X.r_BP[2],
            state.X.r_BI[0], state.X.r_BI[1],
            state.X.b_a[0], state.X.b_a[1], state.X.b_a[2],
            state.X.b_Omega[0], state.X.b_Omega[1], state.X.b_Omega[2],
            sqrt(input.rpm_sq[0]), sqrt(input.rpm_sq[1]), sqrt(input.rpm_sq[2]), sqrt(input.rpm_sq[3]),
            dt,
            0,
            0.08, 0.071, 0.0095, 4.179e-9, H_EKF.flat)

        # mean of expected measurement using non-linear measurement model
        zpred = self.measurementModelAcceleration(state.X, input)
        # innovation covariance
        S_EKF = H_EKF @ state.P @ H_EKF.T + self._consts.RImu[:3, :3]
        # kalman gain
        K_EKF = state.P @ H_EKF.T @ (inv(S_EKF))
        # updated state estimate (a posteriori)
        yhat = meas.a - zpred
        d = K_EKF @ yhat
        state.X.boxplus(d)
        # updated state estimate covariance (a posteriori)
        state.P = (np.eye(NUM_STATES_TANGENT) - K_EKF @ H_EKF) @ state.P

    def measUpdateImuUKF(self, state: StateWithCov, input: Input, meas: MeasImu):
        # since angular velocity measurement is linear, we use linear Kalman Filter update for orientation measurement
        # allocate matrices
        H_P = np.zeros((3, NUM_STATES_TANGENT))
        H_P[:3, 9:12] = np.eye(3)
        H_P[:3, 27:30] = np.eye(3)
        # innovation
        Xhat = np.empty((3, 1))
        Xhat = meas.Omega - (state.X.Omega + state.X.b_Omega)
        # innovation covvariance
        S_KF = self._consts.RImu[3:6, 3:6] + H_P @ state.P @ H_P.T
        # optimal kalman gain
        K_KF = state.P @ H_P.T @ inv(S_KF)
        # updated state estimate and state covariance estimate (a posteriori)
        dX = K_KF @ Xhat
        state.X.boxplus(dX)
        state.P = (self._consts.eye_NST - K_KF @ H_P) @ state.P @ (
                self._consts.eye_NST - K_KF @ H_P).T + K_KF @ self._consts.RImu[3:6, 3:6] @ K_KF.T

        # Unscented Kalman Filter update step for acceleration measurement
        # allocating sigma points
        L = 2 * (NUM_STATES_TANGENT) + 1
        sigmaPoints = List[State]
        propagatedSigmaPointsMat = np.empty((3, L))
        # calculate sigma points
        self.getSigmaPoints(sigmaPoints, state)
        # sigmaPoint transformation through non-linear measurement model
        for i in range(L):
            propagatedSigmaPointsMat[:, i] = self.measurementModelAcceleration(sigmaPoints[i], input)
        # mean of transformed sigma points
        zhat = propagatedSigmaPointsMat @ self._consts.Wm
        # innovation
        diffZ = np.empty((3, L))
        diffZ = propagatedSigmaPointsMat - zhat.tile((1, L))
        # calculate innovation covariance
        diffX = np.empty((NUM_STATES_TANGENT, L))
        for i in range(L):
            diffX[:, i] = sigmaPoints[i].boxminus(state.X)

        S_UKF = diffZ @ np.diag(self._consts.Wc) @ diffZ.T + self._consts.RImu[:3, :3]
        # kalman gain
        Pcross = diffX @ np.diag(self._consts.Wc) @ diffZ.T
        K_UKF = Pcross @ inv(S_UKF)
        # updated covariance estimate (a posteriori)
        state.P = state.P - K_UKF @ S_UKF @ K_UKF.T
        # updated estimate (a posteriori)
        d = K_UKF @ (meas.a - zhat)
        state.X.boxplus(d)

    ##########  IMPLEMENTATION FUNCTIONS  ##########

    ##########  process and measurement models  ##########

    def processModel(self, X: State, U: Input, dt: float):
        # correct wrench calculation matrix for the case of the center of mass not being in the geometric center of rotors
        P_M_CoGcorrected = self._consts.P_M + self._consts.k_f * np.array([[-X.r_BM(1), X.r_BM(0), 0]]).T.tile(
            (1, self._consts.num_rotors))
        # calculate delta: delta = xdot*dt
        delta = np.empty((NUM_STATES_TANGENT, 1))
        delta = np.concatenate([
            X.Omega * dt,
            X.v * dt,
            (1 / X.m * qRotateVec(X.q, self._consts.P_f @ U.rpm_sq) - self._consts.g * self._consts.e_z) * dt,
            (inv(X.I_tensor) * (P_M_CoGcorrected @ U.rpm_sq - X.Omega.cross(X.I_tensor @ X.Omega))) * dt,
            np.zeros((18, 1))], axis=0)
        # add delta: x_new = x_old + delta
        X.boxplus(delta)

    def measurementModelPosition(self, X: State):
        # measurement model position
        meas = X.r + qRotateVec(X.q, X.r_BP - X.r_BM)
        return meas

    def measurementModelAcceleration(self, X: State, U: Input):
        # vector from center of mass to IMU
        r_MI = np.array([X.r_BI.x(), X.r_BI.y(), 0]) - X.r_BM
        # correct wrench calculation matrix for the case of the center of mass not being in the geometric center of rotors
        P_M_CoGcorrected = self._consts.P_M + self._consts.k_f * np.array([[-X.r_BM(1), X.r_BM(0), 0]]).T.tile(
            (1, self._consts.num_rotors))
        # applied force and moment based on rotor rpms
        F = self._consts.P_f @ U.rpm_sq
        M = P_M_CoGcorrected @ U.rpm_sq
        # measurement model acceleration
        Omega_dot = inv(X.I_tensor) @ (M - X.Omega.cross(X.I_tensor @ X.Omega))
        a_meas = 1 / X.m * F + Omega_dot.cross(r_MI) + X.Omega.cross(X.Omega.cross(r_MI)) + X.b_a

        return a_meas

    ##########  UKF functions  ##########

    def getSigmaPoints(self, sigmaPoints: List[State], state: StateWithCov):
        # Cholesky factorization
        factorization = cholesky(state.P)
        c = sqrt(self._consts.lambd + NUM_STATES_TANGENT)
        factorization *= c

        # calculate sigma points
        for i in range(2 * NUM_STATES_TANGENT + 1):
            sigmaPoints[i] = state.X
        for i in range(2 * NUM_STATES_TANGENT + 1):
            sigmaPoints[i].boxplus(factorization[:, i - 1])
            sigmaPoints[i + NUM_STATES_TANGENT].boxplus(-factorization[:, i - 1])

    def calculateMeanStateSigma(self, mean: State, sigmaPoints: List[State]):
        # function to calculate the mean of sigma points
        # calculate mean on SO(3)
        qList = np.empty([4, NUM_SIGMAS])
        for i in range(NUM_SIGMAS):
            qList[:, i] = sigmaPoints[i].q
        mean.q = qMean(qList, self._consts.Wm)
        # calulate mean on Euclidean Space variables
        mean.r = self._consts.Wm[0] * sigmaPoints[0].r
        mean.v = self._consts.Wm[0] * sigmaPoints[0].v
        mean.Omega = self._consts.Wm[0] * sigmaPoints[0].Omega
        mean.m = self._consts.Wm[0] * sigmaPoints[0].m
        mean.r_BM = self._consts.Wm[0] * sigmaPoints[0].r_BM
        mean.r_BP = self._consts.Wm[0] * sigmaPoints[0].r_BP
        mean.r_BI = self._consts.Wm[0] * sigmaPoints[0].r_BI
        mean.b_a = self._consts.Wm[0] * sigmaPoints[0].b_a
        mean.b_Omega = self._consts.Wm[0] * sigmaPoints[0].b_Omega
        for i in range(1, NUM_SIGMAS):
            mean.r += self._consts.Wm[i] * sigmaPoints[i].r
            mean.v += self._consts.Wm[i] * sigmaPoints[i].v
            mean.Omega += self._consts.Wm[i] * sigmaPoints[i].Omega
            mean.m += self._consts.Wm[i] * sigmaPoints[i].m
            mean.r_BM += self._consts.Wm[i] * sigmaPoints[i].r_BM
            mean.r_BP += self._consts.Wm[i] * sigmaPoints[i].r_BP
            mean.r_BI += self._consts.Wm[i] * sigmaPoints[i].r_BI
            mean.b_a += self._consts.Wm[i] * sigmaPoints[i].b_a
            mean.b_Omega += self._consts.Wm[i] * sigmaPoints[i].b_Omega

    ##########  message callback functions  ##########

    def rpm_callback(self, msg: MotorRPM):
        # read msg
        self.rpmMsg2input(self._input, msg)
        if self._initialized:
            # predict state with either EKF or UKF
            if self._useEKF:
                self.predictEKF(self._state, self._input)
            else:
                self.predictUKF(self._state, self._input)
            # publish updated estimates
            self.publishEstimates(self._state)
        else:

            # if not initialized, save msg, set flag that pose initialized
            if self._inputList.qsize() >= 30:
                self._inputList.get()
                self._inputList.task_done()
            self._init_vars.rpmInitialized = True
            self._init_vars.t = msg.header.stamp
            # check whether ready to initialize or not
            self._init_vars.readyToInitialize = self._init_vars.rpmInitialized and \
                                                self._init_vars.imuInitialized and \
                                                self._init_vars.poseInitialized
            if self._init_vars.readyToInitialize:
                self.initializeFilter(self._init_vars)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        # read msg
        meas_pose: MeasPose = self.poseMsg2measPose(msg)
        if self._initialized:
            # update state with either EKF or UKF
            if self._useEKF:
                self.measUpdatePoseEKF(self._state, meas_pose)
            else:
                self.measUpdatePoseUKF(self._state, meas_pose)
            # publish updated estimates
            self.publishEstimates(self._state)
        else:
            # if not initialized, save msg, set flag that pose initialized
            self._init_vars.r = meas_pose.r
            self._init_vars.q = meas_pose.q
            self._init_vars.P0[:3, :3] = meas_pose.cov[:3, :3]
            self._init_vars.P0[3:6, 3:6] = meas_pose.cov[3:6, 3:6]
            self._init_vars.poseInitialized = True
            # check whether ready to initialize or not
            self._init_vars.readyToInitialize = self._init_vars.rpmInitialized and \
                                                self._init_vars.imuInitialized and \
                                                self._init_vars.poseInitialized
            if self._init_vars.readyToInitialize:
                self.initializeFilter(self._init_vars)

    def imu_callback(self, msg: Imu):
        # read message
        meas: MeasImu = self.imuMsg2measImu(msg)
        if self._initialized:
            # update state with either EKF or UKF
            if self._useEKF:
                self.measUpdateImuEKF(self._state, self._input, meas)
            else:
                self.measUpdateImuUKF(self._state, self._input, meas)
            # publish updated estimates
            self.publishEstimates(self._state)
        else:
            # if not initialized, save msg, set flag that imu initialized
            meas = self.imuMsg2measImu(msg)
            self._init_vars.Omega = meas.Omega
            self._init_vars.P0[9:12, 9:12] = self._consts.RImu[3:6, 3:6]
            self._init_vars.imuInitialized = True
            # check whether ready to initialize
            self._init_vars.readyToInitialize = self._init_vars.rpmInitialized and \
                                                self._init_vars.imuInitialized and \
                                                self._init_vars.poseInitialized
            if self._init_vars.readyToInitialize:
                self.initializeFilter(self._init_vars)

    ##########  message conversion functions  ##########

    def rpmMsg2input(self, returnVar: Input, msg: MotorRPM):
        # TODO adapt to arbitrary number of rotors
        # square rpms so we only need to multiply by the rotor thrust coeff
        for i in range(4):
            returnVar.rpm_sq[i] = msg.rpm[i] ** 2
        returnVar.t = msg.header.stamp
        return returnVar

    def poseMsg2measPose(self, msg: PoseWithCovarianceStamped):
        # TODO allow any rotation
        meas_pose = MeasPose()
        # read position
        meas_pose.r = self._consts.R_BP @ np.array([msg.pose.pose.position.x,
                                                    msg.pose.pose.position.y,
                                                    msg.pose.pose.position.z])
        # read attitude
        meas_pose.q = self._consts.q_RB * Quat([msg.pose.pose.orientation.w,
                                                msg.pose.pose.orientation.x,
                                                msg.pose.pose.orientation.y,
                                                msg.pose.pose.orientation.z]).normalized() * self._consts.q_RB.inverse()

        # rotate s.t. z is up instead of down
        covPose = np.empty((6, 6))
        getCovInMsg(msg.pose.covariance, covPose)
        covPose = self._consts.R_BP_6D @ covPose @ self._consts.R_BP_6D.T
        meas_pose.cov = covPose

        return meas_pose

    def imuMsg2measImu(self, msg: Imu):
        meas_imu = MeasImu()

        meas_imu.a = self._consts.R_BI @ np.array([msg.linear_acceleration.x,
                                                   msg.linear_acceleration.y,
                                                   msg.linear_acceleration.z])

        meas_imu.Omega = self._consts.R_BI @ np.array([msg.angular_velocity.x,
                                                       msg.angular_velocity.y,
                                                       msg.angular_velocity.z])

        return meas_imu

    def publishEstimates(self, estimate: StateWithCov):
        # function to publish estimates and covariances
        msg = ParameterEstimates()
        # set stamp
        msg.header.stamp = estimate.t
        # set trajectory states
        msg.q = quat2msg(estimate.X.q)
        msg.r = vec2msg(estimate.X.r)
        msg.v = vec2msg(estimate.X.v)
        msg.Omega = vec2msg(estimate.X.Omega)
        # set inertia states
        msg.m = estimate.X.m
        msg.I_tensor = vec2msg(estimate.X.I_tensor.diagonal())
        msg.r_BM = vec2msg(estimate.X.r_BM)
        # set geometric states
        msg.r_BP = vec2msg(estimate.X.r_BP)
        msg.r_BI.x = estimate.X.r_BI(0)
        msg.r_BI.y = estimate.X.r_BI(1)
        # set biases states
        msg.b_a = vec2msg(estimate.X.b_a)
        msg.b_Omega = vec2msg(estimate.X.b_Omega)
        # set covariance matrix
        for row in range(NUM_STATES_TANGENT):
            msg.covariance[row] = estimate.P(row, row)
        # publish
        self._pub_estimates.publish(msg)
