#pragma once

#include <mutex>

#include <Eigen/Dense>
#include <casadi/casadi.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include "KF.hpp"
#include "../legKinematics/kinematics.h"
#include "../utils/utility.h"

#define EKF_STATE_SIZE 18   // 状态变量维度
#define MEAS_SIZE 28        // 观测变量维度
#define PROCESS_NOISE_PIMU 0.01 // 位置预测协方差
#define PROCESS_NOISE_VIMU 0.01 // 速度预测协方差
#define PROCESS_NOISE_PFOOT 0.01    // foot位置预测协方差
#define SENSOR_NOISE_PIMU_REL_FOOT 0.001    // 足端位置测量协方差
#define SENSOR_NOISE_VIMU_REL_FOOT 0.1  // 足端速度测量协方差
#define SENSOR_NOISE_ZFOOT 0.001    // 足端高度测量协方差

class BasicEKF : public A1KF
{
public:
    BasicEKF ();
    void init_filter(SensorData data, Eigen::Vector3d _init_pos ,Eigen::Quaterniond _init_orientation);
    void update_filter(SensorData data);

    Eigen::Matrix<double, EKF_STATE_SIZE, 1> get_state() { return x; }

    Eigen::Quaterniond get_orientation() { return now_orientation; }
private:
    // state
    // 0 1 2 pos 3 4 5 vel 6 7 8 foot pos FR 9 10 11 foot pos FL 12 13 14 foot pos RR 15 16 17 foot pos RL
    Eigen::Matrix<double, EKF_STATE_SIZE, 1> x; // 后验估计
    Eigen::Matrix<double, EKF_STATE_SIZE, 1> xhat; // 先验估计
    Eigen::Matrix<double, EKF_STATE_SIZE, EKF_STATE_SIZE> P; // 后验估计协方差矩阵
    Eigen::Matrix<double, EKF_STATE_SIZE, EKF_STATE_SIZE> Phat; // 先验估计协方差矩阵
    Eigen::Matrix<double, EKF_STATE_SIZE, EKF_STATE_SIZE> A; // 状态转移矩阵
    Eigen::Matrix<double, EKF_STATE_SIZE, 3> B; // 输入矩阵
    Eigen::Matrix<double, EKF_STATE_SIZE, EKF_STATE_SIZE> Q; // 状态转移协方差矩阵

    // observation 
    // 0 1 2    世界坐标系下，质心位置和足端位置之差
    // 3 4 5   
    // 6 7 8  
    // 9 10 11 
    // 12 13 14  全局坐标系下的速度
    // 15 16 17 
    // 18 19 20 
    // 21 22 23 
    // 24 25 26 27 足端的高度 
    Eigen::Matrix<double, MEAS_SIZE, 1> z; //  实际测量
    Eigen::Matrix<double, MEAS_SIZE, 1> zhat; // 测量矩阵×状态先验估计
    Eigen::Matrix<double, MEAS_SIZE, 1> error_z; 
    Eigen::Matrix<double, MEAS_SIZE, 1> Serror_y; // S^-1*error_z
    Eigen::Matrix<double, MEAS_SIZE, EKF_STATE_SIZE> H; // 测量矩阵
    Eigen::Matrix<double, MEAS_SIZE, EKF_STATE_SIZE> SC; // S^-1*H
    Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> R; // 测量噪声协方差矩阵
   
    Eigen::Matrix<double, 3, 3> eye3; // 3x3 单位矩阵
    Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> S;
    Eigen::Matrix<double, EKF_STATE_SIZE, MEAS_SIZE> K; // 卡尔曼增益 

    Eigen::Matrix3d rotation_matrix_diff;
    Eigen::Quaterniond now_orientation;
    
    bool assume_flat_ground = false;

    Kinematics kinematics;
    double leg_offset_x[4] = {};
    double leg_offset_y[4] = {};
    double motor_offset[4] = {};
    double upper_leg_length[4] = {};
    double lower_leg_length[4] = {};
    std::vector<Eigen::VectorXd> rho_fix_list;
    std::vector<Eigen::VectorXd> rho_opt_list;
};