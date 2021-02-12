#
# Academic License - for use in teaching, academic research, and meeting
# course requirements at degree granting institutions only.  Not for
# government, commercial, or other organizational use.
# File: calc_EKF_H_imu_optimized_simple.cpp
#
# MATLAB Coder version            : 3.4
# C/C++ source code generated on  : 26-Feb-2019 22:45:59
#

# Include Files
# include "rt_nonfinite.h"
# include "calc_EKF_H_imu_optimized_simple.h"

# Function Definitions

#
# CALC_EKF_H_IMU_OPTIMIZED_SIMPLE
#     HIMU = CALC_EKF_H_IMU_OPTIMIZED_SIMPLE(QLX,QLY,QLZ,RX,RY,RZ,VX,VY,VZ,OMEGAX,OMEGAY,OMEGAZ,MTOT,ITOTXX,ITOTYY,ITOTZZ,T_GEOMCOGX,T_GEOMCOGY,T_GEOMCOGZ,T_GEOMODOMX,T_GEOMODOMY,T_GEOMODOMZ,T_GEOMIMUX,T_GEOMIMUY,IMUBIASACCX,IMUBIASACCY,IMUBIASACCZ,IMUBIASANGVELX,IMUBIASANGVELY,IMUBIASANGVELZ,MOTOR1,MOTOR2,MOTOR3,MOTOR4,DT,T_GEOMIMUZ,W,L,C,K_F)
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
#                double Himu[90]
# Return Type  : void
#
import numpy as np


def calc_EKF_H_imu_optimized_simple(qLx: float, qLy: float, qLz: float,
                                    rx: float, ry: float, rz: float,
                                    vx: float, vy: float, vz: float,
                                    Omegax: float, Omegay: float, Omegaz: float,
                                    mtot: float,
                                    Itotxx: float, Itotyy: float, Itotzz: float,
                                    t_GeomCogX: float, t_GeomCogY: float, t_GeomCogZ: float,
                                    t_GeomOdomX: float, t_GeomOdomY: float, t_GeomOdomZ: float,
                                    t_GeomImuX: float, t_GeomImuY: float,
                                    ImuBiasAccX: float, ImuBiasAccY: float, ImuBiasAccZ: float,
                                    ImuBiasAngVelX: float, ImuBiasAngVelY: float, ImuBiasAngVelZ: float,
                                    motor1: float, motor2: float, motor3: float, motor4: float,
                                    dt: float,
                                    t_GeomImuZ: float, w: float, l: float, c: float, k_f: float,
                                    Himu: np.ndarray):
    dv0: np.ndarray = np.zeros(90)

    t2 = 1 / Itotzz
    t3 = t_GeomCogY - t_GeomImuY
    t4 = t_GeomCogX - t_GeomImuX
    t5 = 1 / Itotyy
    t6 = t_GeomCogZ - t_GeomImuZ
    t7 = motor1 * motor1
    t8 = motor2 * motor2
    t9 = motor3 * motor3
    t10 = motor4 * motor4
    t11 = Omegax * Omegay * t2 * t3
    t12 = Omegax * Omegaz * t5 * t6
    t16 = ((((Itotxx * Omegax * Omegay + c * k_f * t7) + c * k_f * t9) - Itotyy * Omegax * Omegay) - c * k_f *
           t8) - c * k_f * t10
    t17 = k_f * l * t8
    t18 = k_f * l * t9
    t23 = Itotzz * Omegax * Omegaz
    t56 = k_f * l * t7
    t57 = k_f * l * t10
    t58 = Itotxx * Omegax * Omegaz
    t24 = ((((((((t17 + t18) + k_f * t7 * t_GeomCogX) + k_f * t8 * t_GeomCogX) + k_f * t9 *
               t_GeomCogX) + k_f * t10 * t_GeomCogX) + t23) - t56) - t57) - t58
    t25 = Omegay * Omegay
    t26 = Omegaz * Omegaz
    t30 = t2 * t16
    t31 = Itotxx - Itotyy
    t32 = Itotzz * Omegaz * t_GeomCogZ
    t33 = 1 / Itotxx
    t34 = 1 / (Itotzz * Itotzz)
    t35 = Omegay * Omegaz * t6 * t33
    t36 = Omegax * Omegay
    t37 = ((t7 + t8) + t9) + t10
    t42 = k_f * t9 * w
    t43 = k_f * t10 * w
    t44 = Itotzz * Omegay * Omegaz
    t52 = k_f * t7 * w
    t53 = k_f * t8 * w
    t54 = Itotyy * Omegay * Omegaz
    t45 = ((((((((k_f * t7 * t_GeomCogY + k_f * t8 * t_GeomCogY) + k_f * t9 * t_GeomCogY) + k_f * t10 *
                t_GeomCogY) + t42) + t43) + t44) - t52) - t53) - t54
    t46 = Omegax * Omegax
    t47 = Itotxx - Itotzz
    t48 = Itotyy - Itotzz
    t49 = Itotxx * Omegax * t_GeomCogX
    t50 = Itotyy * Omegay * t_GeomCogY
    t51 = 1 / (Itotxx * Itotxx)
    t55 = 1 / (Itotyy * Itotyy)
    dv0[0:27] = 0
    dv0[27] = -t2 * t5 * ((-Itotxx + Itotyy) + Itotzz) * (
            ((t32 + t50) - Itotyy * Omegay * t_GeomImuY) - Itotzz * Omegaz * t_GeomImuZ)
    dv0[28] = (Omegax * t3 * 2 - Omegay * t4) - Omegay * t2 * t4 * t31
    dv0[29] = (Omegax * t6 * 2 - Omegaz * t4) - Omegaz * t4 * t5 * t47
    dv0[30] = (-Omegax * t3 + Omegay * t4 * 2) + Omegax * t2 * t3 * t31
    dv0[31] = -t2 * t33 * ((Itotxx - Itotyy) + Itotzz) * (
            ((t32 + t49) - Itotxx * Omegax * t_GeomImuX) - Itotzz * Omegaz * t_GeomImuZ)
    dv0[32] = (-Omegaz * t3 + Omegay * t6 * 2) - Omegaz * t3 * t33 * t48
    dv0[33] = (-Omegax * t6 + Omegaz * t4 * 2) + Omegax * t5 * t6 * t47
    dv0[34] = (Omegaz * t3 * 2 - Omegay * t6) + Omegay * t6 * t33 * t48
    dv0[35] = -t5 * t33 * ((Itotxx + Itotyy) - Itotzz) * (
            ((t49 + t50) - Itotxx * Omegax * t_GeomImuX) - Itotyy * Omegay * t_GeomImuY)
    dv0[36] = 0
    dv0[37] = 0
    dv0[38] = -k_f / (mtot * mtot) * t37
    dv0[39] = t11 + t12
    dv0[40] = t6 * t45 * t51 - Omegax * Omegay * t2 * t4
    dv0[41] = -t3 * t45 * t51 - Omegax * Omegaz * t4 * t5
    dv0[42] = -t11 + t6 * t24 * t55
    dv0[43] = t35 + Omegax * Omegay * t2 * t4
    dv0[44] = -t4 * t24 * t55 - Omegay * Omegaz * t3 * t33
    dv0[45] = -t12 - t3 * t16 * t34
    dv0[46] = -t35 + t4 * t16 * t34
    dv0[47] = Omegax * Omegaz * t4 * t5 + Omegay * Omegaz * t3 * t33
    dv0[48] = (t25 + t26) - k_f * t5 * t6 * t37
    dv0[49] = -t30 - t36
    dv0[50] = -t5 * ((((((((((((((-t17 - t18) - t23) + t56) + t57) + t58) + Itotyy * Omegax * Omegaz) - k_f *
                            t7 * t_GeomCogX * 2) - k_f * t8 * t_GeomCogX * 2) - k_f * t9 * t_GeomCogX * 2) - k_f *
                         t10 * t_GeomCogX * 2) + k_f * t7 * t_GeomImuX) + k_f * t8 * t_GeomImuX) + k_f * t9 *
                      t_GeomImuX) + k_f * t10 * t_GeomImuX)
    dv0[51] = t30 - Omegax * Omegay
    dv0[52] = (t26 + t46) - k_f * t6 * t33 * t37
    dv0[53] = -t33 * ((((((((((((((-t42 - t43) - t44) + t52) + t53) + t54) +
                              Itotxx * Omegay * Omegaz) - k_f * t7 * t_GeomCogY * 2) - k_f * t8 *
                            t_GeomCogY * 2) - k_f * t9 * t_GeomCogY * 2) - k_f * t10 * t_GeomCogY *
                          2) + k_f * t7 * t_GeomImuY) + k_f * t8 * t_GeomImuY) +
                       k_f * t9 * t_GeomImuY) + k_f * t10 * t_GeomImuY)
    dv0[54] = -t5 * t24 - Omegax * Omegaz
    dv0[55] = -t33 * t45 - Omegay * Omegaz
    dv0[56] = t25 + t46
    dv0[57:66] = 0
    dv0[66] = -t25 - t26
    dv0[67] = t30 + t36
    dv0[68] = -t5 * t24 + Omegax * Omegaz
    dv0[69] = -t30 + t36
    dv0[70] = -t26 - t46
    dv0[71] = -t33 * t45 + Omegay * Omegaz
    dv0[72] = 1
    dv0[73] = 0
    dv0[74] = 0
    dv0[75] = 0
    dv0[76] = 1
    dv0[77] = 0
    dv0[78] = 0
    dv0[79] = 0
    dv0[80] = 1
    dv0[81:90] = 0
    for i0 in range(90):
        Himu[i0] = dv0[i0]
    return Himu
