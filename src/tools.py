import sys
from functools import singledispatch
from math import cos, sin, atan2
from typing import List

import numpy as np
from geometry_msgs.msg import Vector3, Quaternion
from numpy.linalg import norm

from estimator_header import Quat

doPrint = True


def varray(*args):
    return np.array([[*args]]).T


def getCovInMsg(covariance: np.ndarray, matrix: np.ndarray):
    for idx in range(covariance.size):
        matrix.flat[idx] = covariance.flat[idx]
    return matrix


def setCovInMsg(covariance: np.ndarray, matrix: np.ndarray):
    for idx in range(matrix.size):
        covariance.flat[idx] = matrix.flat[idx]
    return covariance

def msg2vec(t: Vector3) -> np.ndarray:
    return np.array([[t.x, t.y, t.z]]).T


def vec2msg(t: np.ndarray) -> Vector3:
    msg = Vector3()
    msg.x = t[0]
    msg.y = t[1]
    msg.z = t[2]
    return msg


def ivec2msg(t: np.ndarray) -> Vector3:
    msg = Vector3()
    msg.x = t[0]
    msg.y = -t[1]
    msg.z = -t[2]
    return msg


def msg2quat(q: Quaternion) -> Quat:
    return Quat(q.w, q.x, q.y, q.z)


def quat2msg(q: Quat) -> Quaternion:
    msg = Quaternion()
    msg.w = q.w
    msg.x = q.x
    msg.y = q.y
    msg.z = q.z
    return msg


def iquat2msg(q: Quat) -> Quaternion:
    msg = Quaternion()
    msg.w = q.w
    msg.x = q.x
    msg.y = -q.y
    msg.z = -q.z
    return msg


def qExp(aA: np.ndarray) -> Quat:
    q = Quat()
    phi = norm(aA)
    if phi == 0:
        q = Quat(1, 0, 0, 0)
    else:
        u = aA / phi
        q.w = cos(phi)
        q.x = sin(phi) * u[0]
        q.y = sin(phi) * u[1]
        q.z = sin(phi) * u[2]
    return q


def qLog(q: Quat) -> np.ndarray:
    qv = varray(q.x, q.y, q.z)
    qvNorm = norm(qv)
    phi = atan2(qvNorm, q.w)
    if phi == 0:
        u = varray(0, 0, 1)
    elif phi < 1e-6:
        u = qv / q.w * (1 - qvNorm * qvNorm / (3 * q.w * q.w))
    else:
        u = qv / qvNorm
    return phi * u


def qBoxPlus(q: Quat, delta: np.ndarray) -> Quat:
    return Quat(q * qExp(delta / 2))


def qBoxMinus(q1: Quat, q2: Quat) -> np.ndarray:
    res = 2 * qLog(q2.inverse() * q1)
    return varray(*res)


@singledispatch
def qMean(qList, weights):
    return NotImplemented


@qMean.register
def _(qList: list, weights: np.ndarray) -> Quat:
    n = weights.shape[0]
    mu: Quat = qList[0]
    error = np.empty([3, n])
    k = 1
    while True:
        meanError = np.zeros([3, 1])
        for i in range(n):
            error[:, i] = qBoxMinus(qList[i], mu)
            meanError += error[i]
        meanError = meanError / n
        mu = qBoxPlus(mu, meanError)
        if k > 2:
            # print("mean quat calc too long, done steps",k);
            break

        if meanError.norm() < sys.float_info.epsilon * 1.0e3:
            break
        k += 1

    return mu


@qMean.register
def _(qMatrix: np.ndarray, weights: np.ndarray) -> Quat:
    c = qMatrix.shape[1]
    qList = [qMatrix[:i] for i in range(c)]
    for i in range(c):
        qList[i] = Quat(qMatrix[0, i], qMatrix[1, i], qMatrix[2, i], qMatrix[3, i]);
    qMu = qMean(qList, weights)
    return Quat(qMu.w, qMu.x, qMu.y, qMu.z)


def qRotateVec(q: Quat, vec: np.ndarray) -> np.ndarray:
    qvec = Quat()
    qvec.x = vec[0]
    qvec.y = vec[1]
    qvec.z = vec[2]
    qvec = q * qvec * q.inverse()
    return varray(qvec.x, qvec.y, qvec.z)
