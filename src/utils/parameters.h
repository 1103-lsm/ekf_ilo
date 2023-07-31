/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <map>

extern std::string IMU_TOPIC;
extern std::string POSE_TOPIC;
extern std::string TWIST_TOPIC;
extern std::string LEG_TOPIC;
extern double LEG_OFFSET_X;
extern double LEG_OFFSET_Y;
extern double THIGH_OFFSET;
extern double THIGH_LENGTH;
extern double CALF_LENGTH;

void readParameters(std::string config_file);

#define NUM_OF_LEG 4
#define NUM_OF_DOF 12
// parameters in the leg kinematics  and imu_leg_integration_base 
#define RHO_OPT_SIZE 1  // 单个关节参数的维度大小
#define TOTAL_RHO_OPT_SIZE 4 // 4xRHO_OPT_SIZE 所有关节参数的总维度大小
#define RHO_FIX_SIZE 4  // 固定关节参数的维度大小
#define D_FK_DRHO_SIZE 3       // 3xRHO_OPT_SIZE 正向运动学中关于关节参数的雅可比矩阵的维度大小
#define D_J_DRHO_SIZE 9        // 9xRHO_OPT_SIZE 雅可比矩阵中关于关节参数的雅可比矩阵的维度大小
#define RESIDUAL_STATE_SIZE 31 // 3*9 + 4xRHO_OPT_SIZE 残差状态的维度大小，包括9个关节位置误差和4个关节参数
#define NOISE_SIZE 46          // 3*14 + 4xRHO_OPT_SIZE 噪声向量的维度大小，包括14个关节位置噪声和4个关节参数

typedef Eigen::Matrix<double, 12, 1> Vector12d; // 4xRHO_OPT_SIZE
typedef Eigen::Matrix<double, 4, 1> Vector_rho; // 4xRHO_OPT_SIZE

typedef Eigen::Matrix<double, NUM_OF_LEG, 1> Vector_leg;
typedef Eigen::Matrix<double, NUM_OF_DOF, 1> Vector_dof;
