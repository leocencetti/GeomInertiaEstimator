from dataclasses import dataclass
from functools import singledispatchmethod

import numpy as np
import rospy

# define state & measurement sizes
NUM_STATES = 31
NUM_STATES_TANGENT = 30
NUM_MEAS_POSE = 7
NUM_MEAS_IMU = 6
NUM_SIGMAS = 2 * NUM_STATES_TANGENT + 1
RT_PI = 3.14159265358979323846
RT_PIF = 3.1415927


# structs/classes to cleanly save sates, inputs, measurements, and initialization values
@dataclass
class InitVars:
    # variables needed for filter initialization
    # flags
    readyToInitialize = False
    poseInitialized = False
    imuInitialized = False
    rpmInitialized = False
    # time
    t: rospy.rostime.Time = rospy.rostime.Time()
    # pose
    r = np.empty([3, 1])
    q = np.empty([4, 1])
    # imu
    Omega = np.empty([3, 1])
    # biases
    b_a = np.empty([3, 1])
    b_Omega = np.empty([3, 1])
    # from param file - user set
    m: float = 0
    I_tensor = np.empty([3, 3])
    r_BM = np.empty([3, 1])
    r_BP = np.empty([3, 1])
    r_BI = np.empty([2, 1])
    # uncertainty covariance
    P0 = np.empty([NUM_STATES_TANGENT, NUM_STATES_TANGENT])


class State:
    # state variables
    # trajectory
    r = np.empty([3, 1])
    v = np.empty([3, 1])
    q = np.empty([4, 1])
    Omega = np.empty([3, 1])
    # inertia
    m: float = 0
    I_tensor = np.empty([3, 3])
    r_BM = np.empty([3, 1])
    # geometric
    r_BP = np.empty([3, 1])
    r_BI = np.empty([2, 1])
    # biases
    b_a = np.empty([3, 1])
    b_Omega = np.empty([3, 1])

    ##########  state addition/subtraction functions  ##########

    def boxplus(self, delta: np.ndarray):
        # adding a delta (element of the tangent space of the state space) to a state (element of the state space)
        # "addition" on SO(3)
        self.q = qBoxPlus(self.q, delta[:3])
        # addition in the Euclidean Space
        self.r += delta[3, 6]
        self.v += delta[6, 9]
        self.Omega += delta[9, 12]
        self.m += delta[12]
        self.I_tensor += np.diag(delta[13, 16])
        self.r_BM += delta[16, 19]
        self.r_BP += delta[19, 22]
        self.r_BI += delta[22, 24]
        self.b_a += delta[24, 27]
        self.b_Omega += delta[27, 30]

    def boxminus(self, other: State):
        # calculating the delta (lying on the tangent space of the state space) of two state
        delta = np.empty([NUM_STATES_TANGENT, 1])
        # delta on SO(3)
        delta.segment[0:3] = qBoxMinus(self.q, other.q)
        # delta on the Euclidean Space
        delta[3:6] = self.r - other.r
        delta[6:9] = self.v - other.v
        delta[9:12] = self.Omega - other.Omega
        delta[12] = self.m - other.m
        delta[13:6] = self.I_tensor.diagonal() - other.I_tensor.diagonal()
        delta[16:19] = self.r_BM - other.r_BM
        delta[19:22] = self.r_BP - other.r_BP
        delta[22:24] = (self.r_BI - other.r_BI)[:2]
        delta[24:27] = self.b_a - other.b_a
        delta[27:30] = self.b_Omega - other.b_Omega
        return delta


@dataclass
class StateWithCov:
    # complete state with covariance and timestamp
    X: State = State()
    P = np.empty([NUM_STATES_TANGENT, NUM_STATES_TANGENT])
    t: rospy.rostime.Time = rospy.rostime.Time()


@dataclass
class Input:
    # inputs for prediction step
    rpm_sq = np.empty([4, 1])
    t: rospy.rostime.Time = rospy.rostime.Time()


@dataclass
class MeasPose:
    # measurements for pose measurement update
    r = np.empty([3, 1])
    q = np.empty([4, 1])
    cov = np.empty([6, 6])


@dataclass
class MeasImu:
    # measurements for IMU measurement update
    a = np.empty([3, 1])
    Omega = np.empty([3, 1])


class Quat:
    @singledispatchmethod
    def __init__(self, *args, **kwargs):
        return NotImplemented

    @__init__.register
    def _(self, w: float, x: float, y: float, z: float):
        self.w: float = w
        self.x: float = x
        self.y: float = y
        self.z: float = z

    @__init__.register
    def _(self, q: Quat):
        self.w = q.w
        self.x = q.x
        self.y = q.y
        self.z = q.z

    def __str__(self):
        return '{} {} {} {}'.format(self.w, self.x, self.y, self.z)

    def inverse(self):
        return Quat(self.w, -self.x, -self.y, -self.z)

    @singledispatchmethod
    def __mul__(self, other):
        return NotImplemented

    @__mul__.register
    def _(self, other: Quat):
        return Quat(-other.x * self.x - other.y * self.y - other.z * self.z + other.w * self.w,
                    other.x * self.w + other.y * self.z - other.z * self.y + other.w * self.x,
                    -other.x * self.z + other.y * self.w + other.z * self.x + other.w * self.y,
                    other.x * self.y - other.y * self.x + other.z * self.w + other.w * self.z)

    @__mul__.register
    def _(self, other: Number):
        return Quat(other * self.w, other * self.x, other * self.y, other * self.z)

    def normalized(self):
        return self / sqrt(self.w ** 2 + self.x ** 2 + self.y ** 2 + self.z ** 2)


@dataclass
class Consts:
    # often used constants
    # covariance IMU
    RImu = np.empty([6, 6])

    # process model covariance
    Qx = np.empty([NUM_STATES_TANGENT, NUM_STATES_TANGENT])

    # rotor thrust and drag moment coefficients
    k_f: float = 0
    k_M: float = 0
    # matrices for wrench calculation from rotor rpms
    P_f = np.empty([3, 4])
    P_M = np.empty([3, 4])
    num_rotors: int = 0

    # attitude IMU & pose sensor
    R_BI = np.empty([3, 3])
    R_BP = np.empty([3, 3])
    q_RB = np.empty([4, 1])

    # UKF tuning parameters
    alpha: float = 0
    kappa: float = 0
    beta: float = 0
    lambd: float = 0

    # UKF weight matrices
    Wm = np.empty([2 * NUM_STATES_TANGENT + 1, 1])
    Wc = np.empty([2 * NUM_STATES_TANGENT + 1, 1])

    # generally often used constants
    e_z = np.empty([3, 1])
    g: float = 0
    # matrices to speed up calculation
    rotate3 = np.empty([3, 3])
    R_BP_6D = np.empty([6, 6])
    zero3 = np.empty([3, 3])
    eye3 = np.empty([3, 3])
    eye_NST = np.empty([NUM_STATES_TANGENT, NUM_STATES_TANGENT])
