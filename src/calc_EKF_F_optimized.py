#
# Academic License - for use in teaching, academic research, and meeting
# course requirements at degree granting institutions only.  Not for
# government, commercial, or other organizational use.
# File: calc_EKF_F_optimized.cpp
#
# MATLAB Coder version            : 3.4
# C/C++ source code generated on  : 26-Feb-2019 22:17:41
#

# CALC_EKF_F_OPTIMIZED
#     F = CALC_EKF_F_OPTIMIZED(QLX,QLY,QLZ,RX,RY,RZ,VX,VY,VZ,OMEGAX,OMEGAY,OMEGAZ,MTOT,ITOTXX,ITOTYY,ITOTZZ,T_GEOMCOGX,T_GEOMCOGY,T_GEOMCOGZ,T_GEOMODOMX,T_GEOMODOMY,T_GEOMODOMZ,T_GEOMIMUX,T_GEOMIMUY,IMUBIASACCX,IMUBIASACCY,IMUBIASACCZ,IMUBIASANGVELX,IMUBIASANGVELY,IMUBIASANGVELZ,MOTOR1,MOTOR2,MOTOR3,MOTOR4,DT,T_GEOMIMUZ,W,L,C,K_F)
# Arguments    : double qLx
#                double qLy
#                double qLz
#                double rx
#                double ry
#                double rz
#                double vx
#                double vy
#                double vz
#                double Omegax
#                double Omegay
#                double Omegaz
#                double mtot
#                double Itotxx
#                double Itotyy
#                double Itotzz
#                double t_GeomCogX
#                double t_GeomCogY
#                double t_GeomCogZ
#                double t_GeomOdomX
#                double t_GeomOdomY
#                double t_GeomOdomZ
#                double t_GeomImuX
#                double t_GeomImuY
#                double ImuBiasAccX
#                double ImuBiasAccY
#                double ImuBiasAccZ
#                double ImuBiasAngVelX
#                double ImuBiasAngVelY
#                double ImuBiasAngVelZ
#                double motor1
#                double motor2
#                double motor3
#                double motor4
#                double dt
#                double t_GeomImuZ
#                double w
#                double l
#                double c
#                double k_f
#                double F[900]
# Return Type  : void
#
import numpy as np
from numpy import sin, cos, sign, sqrt
from math import atan2


def calc_EKF_F_optimized(qLx: float, qLy: float, qLz: float,
                         rx: float, ry: float, rz: float,
                         vx: float, vy: float, vz: float,
                         Omegax: float, Omegay: float, Omegaz: float,
                         mtot: float,
                         Itotxx: float, Itotyy: float, Itotzz: float,
                         t_GeomCogX: float, t_GeomCogY: float, t_GeomCogZ: float, t_GeomOdomX: float,
                         t_GeomOdomY: float, t_GeomOdomZ: float,
                         t_GeomImuX: float, t_GeomImuY: float,
                         ImuBiasAccX: float, ImuBiasAccY: float, ImuBiasAccZ: float,
                         ImuBiasAngVelX: float, ImuBiasAngVelY: float, ImuBiasAngVelZ: float,
                         motor1: float, motor2: float, motor3: float, motor4: float,
                         dt: float,
                         t_GeomImuZ: float,
                         w: float, l: float, c: float, k_f: float, F: np.ndarray):

    b_t55 = np.zeros(900)

    t2 = abs(qLx)
    t3 = abs(qLy)
    t4 = abs(qLz)
    t12 = Omegax * dt
    t5 = abs(t12)
    t14 = Omegay * dt
    t6 = abs(t14)
    t16 = Omegaz * dt
    t7 = abs(t16)
    t11 = (t2 * t2 + t3 * t3) + t4 * t4
    t18 = (t5 * t5 + t6 * t6) + t7 * t7
    t142 = t18
    t142 = sqrt(t142)
    t20 = t142 * 0.5
    t142 = t11
    t142 = sqrt(t142)
    t22 = t142 * 0.5
    t23 = sin(t20)
    t24 = sin(t22)
    t142 = t18
    t142 = sqrt(t142)
    t25 = 1 / t142
    t142 = t11
    t142 = sqrt(t142)
    t26 = 1 / t142
    t28 = cos(t20)
    t29 = cos(t22)
    t27 = ((qLx * t24 * t26 * t28 + Omegax * dt * t23 * t25 * t29) + Omegaz * dt *
           qLy * t23 * t24 * t25 * t26) - Omegay * dt * qLz * t23 * t24 * t25 * t26

    t30 = ((qLy * t24 * t26 * t28 + Omegay * dt * t23 * t25 * t29) - Omegaz * dt *
           qLx * t23 * t24 * t25 * t26) + Omegax * dt * qLz * t23 * t24 * t25 * t26
    t31 = ((qLz * t24 * t26 * t28 + Omegaz * dt * t23 * t25 * t29) + Omegay * dt *
           qLx * t23 * t24 * t25 * t26) - Omegax * dt * qLy * t23 * t24 * t25 * t26
    t36 = t27 * t27
    t41 = t30 * t30
    t46 = t31 * t31
    t47 = (t36 + t41) + t46
    t48 = qLx
    t48 = sign(t48)
    t49 = 1 / pow(t11, 1.5)
    t50 = 1 / t11
    t51 = t47
    t51 = sqrt(t51)
    t20 = t28 * t29
    t22 = Omegax * dt * qLx * t23 * t24 * t25 * t26
    t81 = Omegay * dt * qLy * t23 * t24 * t25 * t26
    t82 = Omegaz * dt * qLz * t23 * t24 * t25 * t26
    t55 = atan2(t51, ((t20 - t22) - t81) - t82)
    t56 = t24 * t26 * t28
    t60 = ((((((t56 + qLx * t2 * t28 * t29 * t48 * t50 * 0.5) + Omegay * dt * qLz *
               t2 * t23 * t24 * t25 * t48 * t49) + Omegaz * dt * qLy * t2 * t23 *
              t25 * t29 * t48 * t50 * 0.5) - qLx * t2 * t24 * t28 * t48 * t49) -
            Omegax * dt * t2 * t23 * t24 * t25 * t26 * t48 * 0.5) - Omegaz * dt *
           qLy * t2 * t23 * t24 * t25 * t48 * t49) - Omegay * dt * qLz * t2 * t23 * t25 * t29 * t48 * t50 * 0.5
    t142 = t47
    t142 = sqrt(t142)
    t61 = 1 / t142
    t110 = Omegay * dt * t23 * t24 * t25 * t26
    t66 = ((((((qLz * t2 * t24 * t28 * t48 * t49 + Omegaz * dt * t2 * t23 * t24 *
                t25 * t26 * t48 * 0.5) + Omegay * dt * qLx * t2 * t23 * t24 * t25 *
               t48 * t49) + Omegax * dt * qLy * t2 * t23 * t25 * t29 * t48 * t50 *
              0.5) - t110) - qLz * t2 * t28 * t29 * t48 * t50 * 0.5) - Omegax * dt
           * qLy * t2 * t23 * t24 * t25 * t48 * t49) - Omegay * dt * qLx * t2 * t23 * t25 * t29 * t48 * t50 * 0.5
    t69 = Omegaz * dt * t23 * t24 * t25 * t26
    t73 = ((((((qLy * t2 * t24 * t28 * t48 * t49 + t69) + Omegay * dt * t2 * t23 *
               t24 * t25 * t26 * t48 * 0.5) + Omegax * dt * qLz * t2 * t23 * t24 *
              t25 * t48 * t49) + Omegaz * dt * qLx * t2 * t23 * t25 * t29 * t48 *
             t50 * 0.5) - qLy * t2 * t28 * t29 * t48 * t50 * 0.5) - Omegaz * dt *
           qLx * t2 * t23 * t24 * t25 * t48 * t49) - Omegax * dt * qLz * t2 * t23 * t25 * t29 * t48 * t50 * 0.5
    t79 = (t31 * t66 * 2 + t30 * t73 * 2) - t27 * t60 * 2
    t20 = ((-t20 + t22) + t81) + t82
    t84 = t20 * t20
    t85 = qLy
    t85 = sign(t85)
    t86 = 1 / pow(t47, 1.5)
    t87 = Omegax * dt * t23 * t24 * t25 * t26
    t91 = ((((((t69 + qLx * t3 * t28 * t29 * t50 * t85 * 0.5) + Omegay * dt * qLz *
               t3 * t23 * t24 * t25 * t49 * t85) + Omegaz * dt * qLy * t3 * t23 *
              t25 * t29 * t50 * t85 * 0.5) - qLx * t3 * t24 * t28 * t49 * t85) -
            Omegax * dt * t3 * t23 * t24 * t25 * t26 * t85 * 0.5) - Omegaz * dt *
           qLy * t3 * t23 * t24 * t25 * t49 * t85) - Omegay * dt * qLz * t3 * t23 * t25 * t29 * t50 * t85 * 0.5
    t96 = ((((((t87 + qLz * t3 * t24 * t28 * t49 * t85) + Omegaz * dt * t3 * t23 *
               t24 * t25 * t26 * t85 * 0.5) + Omegay * dt * qLx * t3 * t23 * t24 *
              t25 * t49 * t85) + Omegax * dt * qLy * t3 * t23 * t25 * t29 * t50 *
             t85 * 0.5) - qLz * t3 * t28 * t29 * t50 * t85 * 0.5) - Omegax * dt *
           qLy * t3 * t23 * t24 * t25 * t49 * t85) - Omegay * dt * qLx * t3 * t23 * t25 * t29 * t50 * t85 * 0.5
    t105 = ((((((t56 + qLy * t3 * t28 * t29 * t50 * t85 * 0.5) + Omegaz * dt * qLx
                * t3 * t23 * t24 * t25 * t49 * t85) + Omegax * dt * qLz * t3 * t23
               * t25 * t29 * t50 * t85 * 0.5) - qLy * t3 * t24 * t28 * t49 * t85)
             - Omegay * dt * t3 * t23 * t24 * t25 * t26 * t85 * 0.5) - Omegax * dt
            * qLz * t3 * t23 * t24 * t25 * t49 * t85) - Omegaz * dt * qLx * t3 * t23 * t25 * t29 * t50 * t85 * 0.5
    t107 = 1 / t20
    t108 = (t27 * t91 * 2 + t30 * t105 * 2) - t31 * t96 * 2
    t109 = 1 / (t20 * t20)
    t112 = 1 / (((t36 + t41) + t46) + t84)
    t113 = qLz
    t113 = sign(t113)
    t118 = ((((((t110 + qLx * t4 * t24 * t28 * t49 * t113) + Omegax * dt * t4 *
                t23 * t24 * t25 * t26 * t113 * 0.5) + Omegaz * dt * qLy * t4 * t23
               * t24 * t25 * t49 * t113) + Omegay * dt * qLz * t4 * t23 * t25 *
              t29 * t50 * t113 * 0.5) - qLx * t4 * t28 * t29 * t50 * t113 * 0.5) -
            Omegay * dt * qLz * t4 * t23 * t24 * t25 * t49 * t113) - Omegaz * dt * qLy * t4 * t23 * t25 * t29 * t50 * t113 * 0.5
    t119 = qLy * t4 * t28 * t29 * t50 * t113 * 0.5
    t120 = Omegaz * dt * qLx * t4 * t23 * t24 * t25 * t49 * t113
    t121 = Omegax * dt * qLz * t4 * t23 * t25 * t29 * t50 * t113 * 0.5
    t232 = qLy * t4 * t24 * t28 * t49 * t113
    t233 = Omegay * dt * t4 * t23 * t24 * t25 * t26 * t113 * 0.5
    t234 = Omegax * dt * qLz * t4 * t23 * t24 * t25 * t49 * t113
    t235 = Omegaz * dt * qLx * t4 * t23 * t25 * t29 * t50 * t113 * 0.5
    t130 = ((((((t56 + qLz * t4 * t28 * t29 * t50 * t113 * 0.5) + Omegax * dt *
                qLy * t4 * t23 * t24 * t25 * t49 * t113) + Omegay * dt * qLx * t4 *
               t23 * t25 * t29 * t50 * t113 * 0.5) - qLz * t4 * t24 * t28 * t49 *
              t113) - Omegaz * dt * t4 * t23 * t24 * t25 * t26 * t113 * 0.5) -
            Omegay * dt * qLx * t4 * t23 * t24 * t25 * t49 * t113) - Omegax * dt * qLy * t4 * t23 * t25 * t29 * t50 * t113 * 0.5
    t132 = (t30 * (((((((t87 + t119) + t120) + t121) - t232) - t233) - t234) -
                   t235) * 2 + t31 * t130 * 2) - t27 * t118 * 2
    t47 = dt * dt
    t12 = sign(t12)
    t36 = 1 / t18
    t81 = 1 / pow(t18, 1.5)
    t20 = dt * t23 * t25 * t29
    t141 = ((((((t20 + Omegax * t5 * t28 * t29 * t47 * t12 * t36 * 0.5) + Omegaz *
                qLy * t5 * t24 * t26 * t28 * t47 * t12 * t36 * 0.5) + Omegay * qLz
               * t5 * t23 * t24 * t26 * t47 * t12 * t81) - Omegax * t5 * t23 * t29
              * t47 * t12 * t81) - dt * qLx * t5 * t23 * t24 * t25 * t26 * t12 *
             0.5) - Omegay * qLz * t5 * t24 * t26 * t28 * t47 * t12 * t36 * 0.5) - Omegaz * qLy * t5 * t23 * t24 * t26 * t47 * t12 * t81
    t142 = dt * qLy * t23 * t24 * t25 * t26
    t147 = ((((((t142 + Omegaz * t5 * t23 * t29 * t47 * t12 * t81) + dt * qLz * t5
                * t23 * t24 * t25 * t26 * t12 * 0.5) + Omegax * qLy * t5 * t24 *
               t26 * t28 * t47 * t12 * t36 * 0.5) + Omegay * qLx * t5 * t23 * t24 *
              t26 * t47 * t12 * t81) - Omegaz * t5 * t28 * t29 * t47 * t12 * t36 *
             0.5) - Omegay * qLx * t5 * t24 * t26 * t28 * t47 * t12 * t36 * 0.5) - Omegax * qLy * t5 * t23 * t24 * t26 * t47 * t12 * t81
    t82 = dt * qLz * t23 * t24 * t25 * t26
    t152 = ((((((t82 + Omegay * t5 * t28 * t29 * t47 * t12 * t36 * 0.5) + Omegax *
                qLz * t5 * t24 * t26 * t28 * t47 * t12 * t36 * 0.5) + Omegaz * qLx
               * t5 * t23 * t24 * t26 * t47 * t12 * t81) - Omegay * t5 * t23 * t29
              * t47 * t12 * t81) - dt * qLy * t5 * t23 * t24 * t25 * t26 * t12 *
             0.5) - Omegaz * qLx * t5 * t24 * t26 * t28 * t47 * t12 * t36 * 0.5) - Omegax * qLz * t5 * t23 * t24 * t26 * t47 * t12 * t81
    t159 = (t30 * t152 * 2 + t27 * t141 * 2) - t31 * t147 * 2
    t14 = sign(t14)
    t22 = dt * qLx * t23 * t24 * t25 * t26
    t166 = ((((((t82 + Omegax * t6 * t23 * t29 * t47 * t81 * t14) + dt * qLx * t6 *
                t23 * t24 * t25 * t26 * t14 * 0.5) + Omegay * qLz * t6 * t24 * t26
               * t28 * t47 * t36 * t14 * 0.5) + Omegaz * qLy * t6 * t23 * t24 *
              t26 * t47 * t81 * t14) - Omegax * t6 * t28 * t29 * t47 * t36 * t14 *
             0.5) - Omegaz * qLy * t6 * t24 * t26 * t28 * t47 * t36 * t14 * 0.5) - Omegay * qLz * t6 * t23 * t24 * t26 * t47 * t81 * t14
    t170 = ((((((t22 + Omegaz * t6 * t28 * t29 * t47 * t36 * t14 * 0.5) + Omegay *
                qLx * t6 * t24 * t26 * t28 * t47 * t36 * t14 * 0.5) + Omegax * qLy
               * t6 * t23 * t24 * t26 * t47 * t81 * t14) - Omegaz * t6 * t23 * t29
              * t47 * t81 * t14) - dt * qLz * t6 * t23 * t24 * t25 * t26 * t14 *
             0.5) - Omegax * qLy * t6 * t24 * t26 * t28 * t47 * t36 * t14 * 0.5) - Omegay * qLx * t6 * t23 * t24 * t26 * t47 * t81 * t14
    t178 = ((((((t20 + Omegay * t6 * t28 * t29 * t47 * t36 * t14 * 0.5) + Omegax *
                qLz * t6 * t24 * t26 * t28 * t47 * t36 * t14 * 0.5) + Omegaz * qLx
               * t6 * t23 * t24 * t26 * t47 * t81 * t14) - Omegay * t6 * t23 * t29
              * t47 * t81 * t14) - dt * qLy * t6 * t23 * t24 * t25 * t26 * t14 *
             0.5) - Omegaz * qLx * t6 * t24 * t26 * t28 * t47 * t36 * t14 * 0.5) - Omegax * qLz * t6 * t23 * t24 * t26 * t47 * t81 * t14
    t180 = (t31 * t170 * 2 + t30 * t178 * 2) - t27 * t166 * 2
    t16 = sign(t16)
    t185 = ((((((t142 + Omegax * t7 * t28 * t29 * t47 * t36 * t16 * 0.5) + Omegaz *
                qLy * t7 * t24 * t26 * t28 * t47 * t36 * t16 * 0.5) + Omegay * qLz
               * t7 * t23 * t24 * t26 * t47 * t81 * t16) - Omegax * t7 * t23 * t29
              * t47 * t81 * t16) - dt * qLx * t7 * t23 * t24 * t25 * t26 * t16 *
             0.5) - Omegay * qLz * t7 * t24 * t26 * t28 * t47 * t36 * t16 * 0.5) - Omegaz * qLy * t7 * t23 * t24 * t26 * t47 * t81 * t16
    t190 = ((((((t22 + Omegay * t7 * t23 * t29 * t47 * t81 * t16) + dt * qLy * t7 *
                t23 * t24 * t25 * t26 * t16 * 0.5) + Omegaz * qLx * t7 * t24 * t26
               * t28 * t47 * t36 * t16 * 0.5) + Omegax * qLz * t7 * t23 * t24 *
              t26 * t47 * t81 * t16) - Omegay * t7 * t28 * t29 * t47 * t36 * t16 *
             0.5) - Omegax * qLz * t7 * t24 * t26 * t28 * t47 * t36 * t16 * 0.5) - Omegaz * qLx * t7 * t23 * t24 * t26 * t47 * t81 * t16
    t199 = ((((((t20 + Omegaz * t7 * t28 * t29 * t47 * t36 * t16 * 0.5) + Omegay *
                qLx * t7 * t24 * t26 * t28 * t47 * t36 * t16 * 0.5) + Omegax * qLy
               * t7 * t23 * t24 * t26 * t47 * t81 * t16) - Omegaz * t7 * t23 * t29
              * t47 * t81 * t16) - dt * qLz * t7 * t23 * t24 * t25 * t26 * t16 *
             0.5) - Omegax * qLy * t7 * t24 * t26 * t28 * t47 * t36 * t16 * 0.5) - Omegay * qLx * t7 * t23 * t24 * t26 * t47 * t81 * t16
    t201 = (t27 * t185 * 2 + t31 * t199 * 2) - t30 * t190 * 2
    t216 = t61 * t79 * t107 * 0.5 + t51 * t109 * (((((((t87 + t2 * t24 * t26 * t28
                                                        * t48 * 0.5) + Omegax * dt * qLx * t2 * t23 * t25 * t29 * t48 * t50 * 0.5) +
                                                      Omegay * dt * qLy * t2 * t23 * t25 * t29 * t48 * t50 * 0.5) + Omegaz * dt *
                                                     qLz * t2 * t23 * t25 * t29 * t48 * t50 * 0.5) - Omegax * dt * qLx * t2 * t23
                                                    * t24 * t25 * t48 * t49) - Omegay * dt * qLy * t2 * t23 * t24 * t25 * t48 *
                                                   t49) - Omegaz * dt * qLz * t2 * t23 * t24 * t25 * t48 * t49)
    t231 = t61 * t107 * t108 * 0.5 - t51 * t109 * (((((((t110 + t3 * t24 * t26 *
                                                         t28 * t85 * 0.5) + Omegax * dt * qLx * t3 * t23 * t25 * t29 * t50 * t85 *
                                                        0.5) + Omegay * dt * qLy * t3 * t23 * t25 * t29 * t50 * t85 * 0.5) + Omegaz *
                                                      dt * qLz * t3 * t23 * t25 * t29 * t50 * t85 * 0.5) - Omegax * dt * qLx * t3 *
                                                     t23 * t24 * t25 * t49 * t85) - Omegay * dt * qLy * t3 * t23 * t24 * t25 *
                                                    t49 * t85) - Omegaz * dt * qLz * t3 * t23 * t24 * t25 * t49 * t85)
    t247 = t61 * t107 * t132 * 0.5 - t51 * t109 * (((((((t69 + t4 * t24 * t26 *
                                                         t28 * t113 * 0.5) + Omegax * dt * qLx * t4 * t23 * t25 * t29 * t50 * t113 *
                                                        0.5) + Omegay * dt * qLy * t4 * t23 * t25 * t29 * t50 * t113 * 0.5) + Omegaz
                                                      * dt * qLz * t4 * t23 * t25 * t29 * t50 * t113 * 0.5) - Omegax * dt * qLx *
                                                     t4 * t23 * t24 * t25 * t49 * t113) - Omegay * dt * qLy * t4 * t23 * t24 *
                                                    t25 * t49 * t113) - Omegaz * dt * qLz * t4 * t23 * t24 * t25 * t49 * t113)
    t262 = t61 * t107 * t159 * 0.5 - t51 * t109 * (((((((t22 + dt * t5 * t23 * t25
                                                         * t29 * t12 * 0.5) + Omegax * qLx * t5 * t24 * t26 * t28 * t47 * t12 * t36 *
                                                        0.5) + Omegay * qLy * t5 * t24 * t26 * t28 * t47 * t12 * t36 * 0.5) + Omegaz
                                                      * qLz * t5 * t24 * t26 * t28 * t47 * t12 * t36 * 0.5) - Omegax * qLx * t5 *
                                                     t23 * t24 * t26 * t47 * t12 * t81) - Omegay * qLy * t5 * t23 * t24 * t26 *
                                                    t47 * t12 * t81) - Omegaz * qLz * t5 * t23 * t24 * t26 * t47 * t12 * t81)
    t278 = t61 * t107 * t180 * 0.5 - t51 * t109 * (((((((t142 + dt * t6 * t23 *
                                                         t25 * t29 * t14 * 0.5) + Omegax * qLx * t6 * t24 * t26 * t28 * t47 * t36 *
                                                        t14 * 0.5) + Omegay * qLy * t6 * t24 * t26 * t28 * t47 * t36 * t14 * 0.5) +
                                                      Omegaz * qLz * t6 * t24 * t26 * t28 * t47 * t36 * t14 * 0.5) - Omegax * qLx *
                                                     t6 * t23 * t24 * t26 * t47 * t81 * t14) - Omegay * qLy * t6 * t23 * t24 *
                                                    t26 * t47 * t81 * t14) - Omegaz * qLz * t6 * t23 * t24 * t26 * t47 * t81 *
                                                   t14)
    t5 = t61 * t107 * t201 * 0.5 - t51 * t109 * (((((((t82 + dt * t7 * t23 * t25 *
                                                       t29 * t16 * 0.5) + Omegax * qLx * t7 * t24 * t26 * t28 * t47 * t36 * t16 *
                                                      0.5) + Omegay * qLy * t7 * t24 * t26 * t28 * t47 * t36 * t16 * 0.5) + Omegaz
                                                    * qLz * t7 * t24 * t26 * t28 * t47 * t36 * t16 * 0.5) - Omegax * qLx * t7 *
                                                   t23 * t24 * t26 * t47 * t81 * t16) - Omegay * qLy * t7 * t23 * t24 * t26 *
                                                  t47 * t81 * t16) - Omegaz * qLz * t7 * t23 * t24 * t26 * t47 * t81 * t16)
    t51 = t24 * t24
    t14 = t29 * t29
    t6 = motor1 * motor1
    t28 = motor2 * motor2
    t16 = motor3 * motor3
    t7 = motor4 * motor4
    t23 = ((k_f * t6 + k_f * t28) + k_f * t16) + k_f * t7
    t25 = qLx * qLx
    t330 = qLy * qLy
    t332 = qLz * qLz
    t20 = ((t14 + t50 * t51 * t25) + t50 * t51 * t330) + t50 * t51 * t332
    t335 = 1 / t20
    t12 = 1 / (t11 * t11)
    t20 = 1 / (t20 * t20)
    t22 = ((((((qLx * t50 * t51 * 2 + t2 * t24 * t29 * t48 * t49 * t25) + t2 *
               t24 * t29 * t48 * t49 * t330) + t2 * t24 * t29 * t48 * t49 * t332)
             - t2 * t24 * t26 * t29 * t48) - t2 * t48 * t51 * t25 * t12 * 2) -
           t2 * t48 * t51 * t330 * t12 * 2) - t2 * t48 * t51 * t332 * t12 * 2
    t81 = 1 / mtot
    t82 = ((((((qLy * t50 * t51 * 2 + t3 * t24 * t29 * t49 * t85 * t25) + t3 *
               t24 * t29 * t49 * t85 * t330) + t3 * t24 * t29 * t49 * t85 * t332)
             - t3 * t24 * t26 * t29 * t85) - t3 * t85 * t51 * t25 * t12 * 2) -
           t3 * t85 * t51 * t330 * t12 * 2) - t3 * t85 * t51 * t332 * t12 * 2
    t47 = ((((((qLz * t50 * t51 * 2 + t4 * t24 * t29 * t49 * t113 * t25) + t4 *
               t24 * t29 * t49 * t113 * t330) + t4 * t24 * t29 * t49 * t113 * t332)
             - t4 * t24 * t26 * t29 * t113) - t4 * t113 * t51 * t25 * t12 * 2) -
           t4 * t113 * t51 * t330 * t12 * 2) - t4 * t113 * t51 * t332 * t12 * 2
    t36 = 1 / (mtot * mtot)
    t41 = qLy * t50 * t51 * t23 * t335 * 2
    t46 = qLz * t50 * t51 * t23 * t335 * 2
    t56 = 1 / Itotxx
    t18 = Omegay * Omegaz * dt * t56
    t142 = 1 / Itotyy
    t109 = Itotyy * Omegay
    t107 = 1 / Itotzz
    t110 = Itotxx * Omegax
    t69 = Omegax * Omegay * dt * t107
    b_t55[0] = (t55 * t60 * t61 * 2 + t27 * t55 * t79 * t86) + t27 * t61 * t84 * t112 * t216 * 2
    b_t55[1] = (t55 * t61 * t73 * -2 + t30 * t55 * t79 * t86) + t30 * t61 * t84 * t112 * t216 * 2
    b_t55[2] = (t55 * t61 * t66 * -2 + t31 * t55 * t79 * t86) + t31 * t61 * t84 * t112 * t216 * 2
    b_t55[3] = 0
    b_t55[4] = 0
    b_t55[5] = 0
    b_t55[6] = -dt * t81 * (((((((qLz * t50 * t51 * t23 * t335 * -2 + qLx * qLz *
                                  t50 * t51 * t23 * t20 * t22 * 2) + qLy * t2 * t48 * t50 * t51 * t23 * t335)
                                - qLy * t2 * t48 * t50 * t14 * t23 * t335) + qLy * t24 * t26 * t29 * t23 *
                               t20 * t22 * 2) + qLx * qLz * t2 * t48 * t51 * t23 * t335 * t12 * 4) +
                             qLy * t2 * t24 * t29 * t48 * t49 * t23 * t335 * 2) - qLx * qLz * t2 * t24 *
                            t29 * t48 * t49 * t23 * t335 * 2)
    b_t55[7] = -dt * t81 * (((((((t24 * t26 * t29 * t23 * t335 * 2 + qLy * qLz *
                                  t50 * t51 * t23 * t20 * t22 * 2) - qLx * t2 * t48 * t50 * t51 * t23 * t335)
                                + qLx * t2 * t48 * t50 * t14 * t23 * t335) - qLx * t24 * t26 * t29 * t23 *
                               t20 * t22 * 2) + qLy * qLz * t2 * t48 * t51 * t23 * t335 * t12 * 4) -
                             qLx * t2 * t24 * t29 * t48 * t49 * t23 * t335 * 2) - qLy * qLz * t2 * t24 *
                            t29 * t48 * t49 * t23 * t335 * 2)
    b_t55[8] = -dt * t81 * (((((((((((t14 * t23 * t20 * t22 + qLx * t50 * t51 *
                                      t23 * t335 * 2) - t50 * t51 * t23 * t25 * t20 * t22) - t50 * t51 * t23 *
                                    t330 * t20 * t22) + t50 * t51 * t23 * t332 * t20 * t22) + t2 * t24 * t26 *
                                  t29 * t48 * t23 * t335) - t2 * t48 * t51 * t23 * t25 * t335 * t12 * 2) -
                                t2 * t48 * t51 * t23 * t330 * t335 * t12 * 2) + t2 * t48 * t51 * t23 *
                               t332 * t335 * t12 * 2) + t2 * t24 * t29 * t48 * t49 * t23 * t25 * t335) +
                             t2 * t24 * t29 * t48 * t49 * t23 * t330 * t335) - t2 * t24 * t29 * t48 * t49
                            * t23 * t332 * t335)
    b_t55[9:9 + 21] = 0
    b_t55[30] = (t55 * t61 * t91 * 2 - t27 * t55 * t86 * t108) - t27 * t61 * t84 * t112 * t231 * 2
    b_t55[31] = (t55 * t61 * t105 * 2 - t30 * t55 * t86 * t108) - t30 * t61 * t84 * t112 * t231 * 2
    b_t55[32] = (t55 * t61 * t96 * -2 - t31 * t55 * t86 * t108) - t31 * t61 * t84 * t112 * t231 * 2
    b_t55[33] = 0
    b_t55[34] = 0
    b_t55[35] = 0
    b_t55[36] = -dt * t81 * (((((((t24 * t26 * t29 * t23 * t335 * -2 + qLx * qLz
                                   * t50 * t51 * t23 * t20 * t82 * 2) + qLy * t24 * t26 * t29 * t23 * t20 *
                                  t82 * 2) + qLy * t3 * t50 * t85 * t51 * t23 * t335) - qLy * t3 * t50 * t85
                                * t14 * t23 * t335) + qLx * qLz * t3 * t85 * t51 * t23 * t335 * t12 * 4) +
                              qLy * t3 * t24 * t29 * t49 * t85 * t23 * t335 * 2) - qLx * qLz * t3 * t24 *
                             t29 * t49 * t85 * t23 * t335 * 2)
    b_t55[37] = dt * t81 * (((((((t46 - qLy * qLz * t50 * t51 * t23 * t20 * t82 *
                                  2) + qLx * t24 * t26 * t29 * t23 * t20 * t82 * 2) + qLx * t3 * t50 * t85
                                * t51 * t23 * t335) - qLx * t3 * t50 * t85 * t14 * t23 * t335) - qLy * qLz *
                              t3 * t85 * t51 * t23 * t335 * t12 * 4) + qLx * t3 * t24 * t29 * t49 * t85 *
                             t23 * t335 * 2) + qLy * qLz * t3 * t24 * t29 * t49 * t85 * t23 * t335 *
                            2)
    b_t55[38] = -dt * t81 * (((((((((((t41 + t14 * t23 * t20 * t82) - t50 * t51 *
                                      t23 * t25 * t20 * t82) - t50 * t51 * t23 * t330 * t20 * t82) + t50 * t51 *
                                    t23 * t332 * t20 * t82) + t3 * t24 * t26 * t29 * t85 * t23 * t335) - t3 *
                                  t85 * t51 * t23 * t25 * t335 * t12 * 2) - t3 * t85 * t51 * t23 * t330 *
                                 t335 * t12 * 2) + t3 * t85 * t51 * t23 * t332 * t335 * t12 * 2) + t3 *
                               t24 * t29 * t49 * t85 * t23 * t25 * t335) + t3 * t24 * t29 * t49 * t85 * t23
                              * t330 * t335) - t3 * t24 * t29 * t49 * t85 * t23 * t332 * t335)
    b_t55[39:39 + 21] = 0
    b_t55[60] = (t55 * t61 * t118 * -2 - t27 * t55 * t86 * t132) - t27 * t61 * t84 * t112 * t247 * 2
    b_t55[61] = (t55 * t61 * (((((((t87 + t119) + t120) + t121) - t232) - t233) -
                               t234) - t235) * 2 - t30 * t55 * t86 * t132) - t30 * t61 * t84 * t112 * t247 * 2
    b_t55[62] = (t55 * t61 * t130 * 2 - t31 * t55 * t86 * t132) - t31 * t61 * t84 * t112 * t247 * 2
    b_t55[63] = 0
    b_t55[64] = 0
    b_t55[65] = 0
    b_t55[66] = -dt * t81 * (((((((qLx * t50 * t51 * t23 * t335 * -2 + qLx * qLz
                                   * t50 * t51 * t23 * t20 * t47 * 2) + qLy * t24 * t26 * t29 * t23 * t20 *
                                  t47 * 2) + qLy * t4 * t50 * t113 * t51 * t23 * t335) - qLy * t4 * t50 *
                                t113 * t14 * t23 * t335) + qLx * qLz * t4 * t113 * t51 * t23 * t335 * t12 *
                               4) + qLy * t4 * t24 * t29 * t49 * t113 * t23 * t335 * 2) - qLx * qLz *
                             t4 * t24 * t29 * t49 * t113 * t23 * t335 * 2)
    b_t55[67] = dt * t81 * (((((((t41 - qLy * qLz * t50 * t51 * t23 * t20 * t47 *
                                  2) + qLx * t24 * t26 * t29 * t23 * t20 * t47 * 2) + qLx * t4 * t50 *
                                t113 * t51 * t23 * t335) - qLx * t4 * t50 * t113 * t14 * t23 * t335) - qLy *
                              qLz * t4 * t113 * t51 * t23 * t335 * t12 * 4) + qLx * t4 * t24 * t29 * t49
                             * t113 * t23 * t335 * 2) + qLy * qLz * t4 * t24 * t29 * t49 * t113 * t23 *
                            t335 * 2)
    b_t55[68] = dt * t81 * (((((((((((t46 - t14 * t23 * t20 * t47) + t50 * t51 *
                                     t23 * t25 * t20 * t47) + t50 * t51 * t23 * t330 * t20 * t47) - t50 * t51 *
                                   t23 * t332 * t20 * t47) - t4 * t24 * t26 * t29 * t113 * t23 * t335) + t4 *
                                 t113 * t51 * t23 * t25 * t335 * t12 * 2) + t4 * t113 * t51 * t23 * t330 *
                                t335 * t12 * 2) - t4 * t113 * t51 * t23 * t332 * t335 * t12 * 2) - t4 *
                              t24 * t29 * t49 * t113 * t23 * t25 * t335) - t4 * t24 * t29 * t49 * t113 *
                             t23 * t330 * t335) + t4 * t24 * t29 * t49 * t113 * t23 * t332 * t335)
    b_t55[69:69 + 24] = 0
    b_t55[93] = 1
    b_t55[94:94 + 30] = 0
    b_t55[124] = 1
    b_t55[125:125 + 30] = 0
    b_t55[155] = 1
    b_t55[156:156 + 27] = 0
    b_t55[183] = dt
    b_t55[184] = 0
    b_t55[185] = 0
    b_t55[186] = 1
    b_t55[187:187 + 27] = 0
    b_t55[214] = dt
    b_t55[215] = 0
    b_t55[216] = 0
    b_t55[217] = 1
    b_t55[218:218 + 27] = 0
    b_t55[245] = dt
    b_t55[246] = 0
    b_t55[247] = 0
    b_t55[248] = 1
    b_t55[249:249 + 21] = 0
    b_t55[270] = (t55 * t61 * t141 * 2 - t27 * t55 * t86 * t159) - t27 * t61 * t84 * t112 * t262 * 2
    b_t55[271] = (t55 * t61 * t152 * 2 - t30 * t55 * t86 * t159) - t30 * t61 * t84 * t112 * t262 * 2
    b_t55[272] = (t55 * t61 * t147 * -2 - t31 * t55 * t86 * t159) - t31 * t61 * t84 * t112 * t262 * 2
    b_t55[273] = 0
    b_t55[274] = 0
    b_t55[275] = 0
    b_t55[276] = 0
    b_t55[277] = 0
    b_t55[278] = 0
    b_t55[279] = 1
    b_t55[280] = -dt * t142 * (Itotxx * Omegaz - Itotzz * Omegaz)
    b_t55[281] = -dt * t107 * (t109 - Itotxx * Omegay)
    b_t55[282:282 + 18] = 0
    b_t55[300] = (t55 * t61 * t166 * -2 - t27 * t55 * t86 * t180) - t27 * t61 * t84 * t112 * t278 * 2
    b_t55[301] = (t55 * t61 * t178 * 2 - t30 * t55 * t86 * t180) - t30 * t61 * t84 * t112 * t278 * 2
    b_t55[302] = (t55 * t61 * t170 * 2 - t31 * t55 * t86 * t180) - t31 * t61 * t84 * t112 * t278 * 2
    b_t55[303] = 0
    b_t55[304] = 0
    b_t55[305] = 0
    b_t55[306] = 0
    b_t55[307] = 0
    b_t55[308] = 0
    b_t55[309] = dt * t56 * (Itotyy * Omegaz - Itotzz * Omegaz)
    b_t55[310] = 1
    b_t55[311] = dt * t107 * (t110 - Itotyy * Omegax)
    b_t55[312:312 + 18] = 0
    b_t55[330] = (t55 * t61 * t185 * 2 - t27 * t55 * t86 * t201) - t27 * t61 * t84 * t112 * t5 * 2
    b_t55[331] = (t55 * t61 * t190 * -2 - t30 * t55 * t86 * t201) - t30 * t61 * t84 * t112 * t5 * 2
    b_t55[332] = (t55 * t61 * t199 * 2 - t31 * t55 * t86 * t201) - t31 * t61 * t84 * t112 * t5 * 2
    b_t55[333] = 0
    b_t55[334] = 0
    b_t55[335] = 0
    b_t55[336] = 0
    b_t55[337] = 0
    b_t55[338] = 0
    b_t55[339] = dt * t56 * (t109 - Itotzz * Omegay)
    b_t55[340] = -dt * t142 * (t110 - Itotzz * Omegax)
    b_t55[341] = 1
    b_t55[342:342 + 24] = 0
    b_t55[366] = -dt * t36 * (qLx * qLz * t50 * t51 * t23 * t335 * 2 + qLy * t24
                              * t26 * t29 * t23 * t335 * 2)
    b_t55[367] = -dt * t36 * (qLy * qLz * t50 * t51 * t23 * t335 * 2 - qLx * t24
                              * t26 * t29 * t23 * t335 * 2)
    b_t55[368] = -dt * t36 * (((t14 * t23 * t335 - t50 * t51 * t23 * t25 * t335) -
                               t50 * t51 * t23 * t330 * t335) + t50 * t51 * t23 * t332 * t335)
    b_t55[369] = 0
    b_t55[370] = 0
    b_t55[371] = 0
    b_t55[372] = 1
    b_t55[373:373 + 26] = 0
    b_t55[399] = 1 / (Itotxx * Itotxx) * dt * (((((((((-Itotyy * Omegay * Omegaz
                                                       + Itotzz * Omegay * Omegaz) + k_f * t6 * t_GeomCogY) + k_f * t28 *
                                                     t_GeomCogY) + k_f * t16 * t_GeomCogY) + k_f * t7 * t_GeomCogY) - k_f * t6 *
                                                  w) - k_f * t28 * w) + k_f * t16 * w) + k_f * t7 * w)
    b_t55[400] = -Omegax * Omegaz * dt * t142
    b_t55[401] = t69
    b_t55[402] = 0
    b_t55[403] = 1
    b_t55[404:404 + 25] = 0
    b_t55[429] = t18
    b_t55[430] = -1 / (Itotyy * Itotyy) * dt * (((((((((-Itotxx * Omegax *
                                                        Omegaz + Itotzz * Omegax * Omegaz) - k_f * l * t6) + k_f * l * t28) + k_f *
                                                     l * t16) - k_f * l * t7) + k_f * t6 * t_GeomCogX) + k_f * t28 * t_GeomCogX)
                                                 + k_f * t16 * t_GeomCogX) + k_f * t7 * t_GeomCogX)
    b_t55[431] = -t69
    b_t55[432] = 0
    b_t55[433] = 0
    b_t55[434] = 1
    b_t55[435:435 + 24] = 0
    b_t55[459] = -t18
    b_t55[460] = Omegax * Omegaz * dt * t142
    b_t55[461] = -1 / (Itotzz * Itotzz) * dt * (((((Itotxx * Omegax * Omegay -
                                                    Itotyy * Omegax * Omegay) + c * k_f * t6) - c * k_f * t28) + c * k_f * t16)
                                                - c * k_f * t7)
    b_t55[462] = 0
    b_t55[463] = 0
    b_t55[464] = 0
    b_t55[465] = 1
    b_t55[466:466 + 24] = 0
    b_t55[490] = dt * t23 * t142
    b_t55[491] = 0
    b_t55[492] = 0
    b_t55[493] = 0
    b_t55[494] = 0
    b_t55[495] = 0
    b_t55[496] = 1
    b_t55[497:497 + 22] = 0
    b_t55[519] = -dt * t23 * t56
    b_t55[520] = 0
    b_t55[521] = 0
    b_t55[522] = 0
    b_t55[523] = 0
    b_t55[524] = 0
    b_t55[525] = 0
    b_t55[526] = 0
    b_t55[527] = 1
    b_t55[528:528 + 30] = 0
    b_t55[558] = 1
    b_t55[559:559 + 30] = 0
    b_t55[589] = 1
    b_t55[590:590 + 30] = 0
    b_t55[620] = 1
    b_t55[621:621 + 30] = 0
    b_t55[651] = 1
    b_t55[652:652 + 30] = 0
    b_t55[682] = 1
    b_t55[683:683 + 30] = 0
    b_t55[713] = 1
    b_t55[714:714 + 30] = 0
    b_t55[744] = 1
    b_t55[745:745 + 30] = 0
    b_t55[775] = 1
    b_t55[776:776 + 30] = 0
    b_t55[806] = 1
    b_t55[807:807 + 30] = 0
    b_t55[837] = 1
    b_t55[838:838 + 30] = 0
    b_t55[868] = 1
    b_t55[869:869 + 30] = 0
    b_t55[899] = 1
    for i0 in range(900):
        F[i0] = b_t55[i0]
    return F
