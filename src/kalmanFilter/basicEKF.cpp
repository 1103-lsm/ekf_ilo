#include "basicEKF.h"

BasicEKF::BasicEKF () : A1KF()
{
    KF_initialized = false;
}

void BasicEKF::init_filter(SensorData data, Eigen::Vector3d _init_pos,Eigen::Quaterniond _init_orientation) {
    KF_initialized = true;
    // 0-FR  1-FL  2-RR  3-RL
    leg_offset_x[0] = LEG_OFFSET_X;
    leg_offset_x[1] = LEG_OFFSET_X;
    leg_offset_x[2] = -LEG_OFFSET_X;
    leg_offset_x[3] = -LEG_OFFSET_X;

    leg_offset_y[0] = -LEG_OFFSET_Y;
    leg_offset_y[1] = LEG_OFFSET_Y;
    leg_offset_y[2] = -LEG_OFFSET_Y;
    leg_offset_y[3] = LEG_OFFSET_Y;

    motor_offset[0] = -THIGH_OFFSET;
    motor_offset[1] = THIGH_OFFSET;
    motor_offset[2] = -THIGH_OFFSET;
    motor_offset[3] = THIGH_OFFSET;

    upper_leg_length[0] = upper_leg_length[1] = upper_leg_length[2] = upper_leg_length[3] = THIGH_LENGTH;
    lower_leg_length[0] = lower_leg_length[1] = lower_leg_length[2] = lower_leg_length[3] = CALF_LENGTH;

    for (int i = 0; i < NUM_LEG; i++)
    {
        Eigen::VectorXd rho_fix(5);
        rho_fix << leg_offset_x[i], leg_offset_y[i], motor_offset[i], upper_leg_length[i], lower_leg_length[i];
        Eigen::VectorXd rho_opt(3);
        rho_opt << 0.0, 0.0, 0.0;
        rho_fix_list.push_back(rho_fix);
        rho_opt_list.push_back(rho_opt);
    }

    // 计算矩阵 B 相对于矩阵 A 的四元数差值
    Eigen::Quaterniond Q_diff = _init_orientation * data.root_quat.inverse();

    // 将四元数差值转换回旋转矩阵表示
    rotation_matrix_diff = Q_diff.toRotationMatrix();

    now_orientation = data.root_quat*rotation_matrix_diff;
    data.root_rot_mat = now_orientation.toRotationMatrix();

    // 3x3单位矩阵
    eye3.setIdentity();
    // 测量矩阵H 固定
    H.setZero();
    for (int i=0; i<NUM_LEG; ++i) {
        // 足底位置
        H.block<3,3>(i*3,0) = eye3;
        H.block<3,3>(i*3,6+i*3) = -eye3; 
        // 质心速度
        H.block<3,3>(NUM_LEG*3+i*3,3) = eye3;  
        // 足底高度
        H(NUM_LEG*6+i,6+i*3+2) = 1; 
    }

    // 预测协方差矩阵Q 固定
    Q.setIdentity();
    Q.block<3,3>(0,0) = PROCESS_NOISE_PIMU*eye3;              
    Q.block<3,3>(3,3) = PROCESS_NOISE_VIMU*eye3;             
    for (int i=0; i<NUM_LEG; ++i) {
        Q.block<3,3>(6+i*3,6+i*3) = PROCESS_NOISE_PFOOT*eye3;  
    }
    // 测量协防差矩阵R 固定
    R.setIdentity();
    for (int i=0; i<NUM_LEG; ++i) {
        R.block<3,3>(i*3,i*3) = SENSOR_NOISE_PIMU_REL_FOOT*eye3;                       
        R.block<3,3>(NUM_LEG*3+i*3,NUM_LEG*3+i*3) = SENSOR_NOISE_VIMU_REL_FOOT*eye3;     
        R(NUM_LEG*6+i,NUM_LEG*6+i) = SENSOR_NOISE_ZFOOT;                              
    }

    A.setIdentity();
    B.setZero();

    // 后验估计协方差矩阵初始化
    P.setIdentity();
    P = P * 3;

    // 状态后验估计
    x.setZero();
    // 质心位置初始化
    x.segment<3>(0) = _init_pos;
    // 足底位置初始化
    for (int i = 0; i < NUM_LEG; i++)
    {
        Eigen::Vector3d init_foot_pos = kinematics.fk(data.joint_pos.segment<3>(i * 3), rho_opt_list[i], rho_fix_list[i]);
        x.segment<3>(6 + i * 3) = x.segment<3>(0) + data.root_rot_mat*init_foot_pos;
    }
}

void BasicEKF::update_filter(SensorData data) {
    now_orientation = data.root_quat*rotation_matrix_diff;
    data.root_rot_mat = now_orientation.toRotationMatrix();
    // 更新状态转移矩阵和输入矩阵
    A.block<3, 3>(0, 3) = data.dt * eye3;
    B.block<3, 3>(3, 0) = data.dt * eye3;

    // 输入是加速度，为（世界坐标系下）机体加速度和重力加速度之和
    Eigen::Vector3d u = data.root_rot_mat * data.acc + Eigen::Vector3d(0, 0, -9.8);
    // Eigen::Vector3d u = Eigen::Vector3d(0, 0, -9.8);

    // 更新预测协方差Q
    Q.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * data.dt / 20.0 * eye3;
    Q.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * data.dt * 9.8 / 20.0 * eye3;
    // 降低摆动足对运动学的影响
    for (int i = 0; i < NUM_LEG; ++i) 
    { 
        Q.block<3, 3>(6 + i * 3, 6 + i * 3) = (1 + (1 - data.plan_contacts[i]) * 1e3) * data.dt * PROCESS_NOISE_PFOOT * eye3;  
        R.block<3, 3>(i * 3, i * 3) = (1 + (1 - data.plan_contacts[i]) * 1e3) * SENSOR_NOISE_PIMU_REL_FOOT * eye3;             
        R.block<3, 3>(NUM_LEG * 3 + i * 3, NUM_LEG * 3 + i * 3) = (1 + (1 - data.plan_contacts[i]) * 1e3) * SENSOR_NOISE_VIMU_REL_FOOT * eye3;      
        R(NUM_LEG * 6 + i, NUM_LEG * 6 + i) = (1 + (1 - data.plan_contacts[i]) * 1e3) * SENSOR_NOISE_ZFOOT; 
    }

    xhat = A*x + B*u ;
    Phat = A * P * A.transpose() + Q;

    zhat = H*xhat;

    // 实际测量过程
    for (int i=0; i<NUM_LEG; ++i) 
    {
        Eigen::Vector3d fk_pos = kinematics.fk(data.joint_pos.segment<3>(i * 3), rho_opt_list[i], rho_fix_list[i]);
        z.block<3,1>(i*3,0) = -data.root_rot_mat*fk_pos;

        Eigen::Matrix3d tmp_mtx = kinematics.jac(data.joint_pos.segment<3>(i * 3),rho_opt_list[i], rho_fix_list[i]);
        Eigen::Vector3d tmp_vec = data.joint_vel.segment<3>(3 * i);

        Eigen::Vector3d leg_v = tmp_mtx * tmp_vec + Utility::skew(data.ang_vel)*fk_pos;
        z.block<3,1>(NUM_LEG*3+i*3,0) = (1.0-data.plan_contacts[i])*x.segment<3>(3) +  data.plan_contacts[i]*(-data.root_rot_mat*leg_v);
        
        // 接触地面时 foot z 为 0
        z(NUM_LEG*6+i) = (1.0-data.plan_contacts[i])*(x(2)+fk_pos(2)) + data.plan_contacts[i]*0;
    }

    S = H * Phat *H.transpose() + R;
    S = 0.5*(S+S.transpose());

    error_z = z - zhat; // 测量误差 = 实际测量值 - 测量矩阵×先验状态
    Serror_y = S.fullPivHouseholderQr().solve(error_z); // 这一项是:（z-Hx）*（H×P×HT + R）卡尔曼增益的分母
    // 后验状态估计
    x = xhat + Phat * H.transpose() * Serror_y; 

    // 更新误差协方差
    SC = S.fullPivHouseholderQr().solve(H);
    P = Phat - Phat * H.transpose() * SC * Phat;
    P = 0.5 * (P + P.transpose());

    // 减少位置漂移
    if (P.block<2, 2>(0, 0).determinant() > 1e-6) {
        P.block<2, 16>(0, 2).setZero();
        P.block<16, 2>(2, 0).setZero();
        P.block<2, 2>(0, 0) /= 10.0;
    }
}