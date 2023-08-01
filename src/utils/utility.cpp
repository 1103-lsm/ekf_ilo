#include "utility.h"

Eigen::Matrix3d Utility::skew(Eigen::Vector3d vec) {
    Eigen::Matrix3d rst; rst.setZero();
    rst <<            0, -vec(2),  vec(1),
            vec(2),             0, -vec(0),
            -vec(1),  vec(0),             0;
    return rst;

    
}


Eigen::Vector3d Utility::quat_to_euler(Eigen::Quaterniond quat) {
    Eigen::Vector3d rst;

    Eigen::Matrix<double, 4, 1> coeff = quat.coeffs();
    double x = coeff(0);
    double y = coeff(1);
    double z = coeff(2);
    double w = coeff(3);

    double y_sqr = y*y;

    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y_sqr);

    rst[0] = atan2(t0, t1);

    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > +1.0 ? +1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    rst[1] = asin(t2);

    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y_sqr + z * z);
    rst[2] = atan2(t3, t4);
    return rst;
}


// 计算位置误差
PositionError Utility::calculatePositionError(const geometry_msgs::PoseStamped& true_pose,
                                     const geometry_msgs::PoseStamped& estimated_pose) {
    PositionError error;
    error.x = std::fabs(true_pose.pose.position.x - estimated_pose.pose.position.x);
    error.y = std::fabs(true_pose.pose.position.y - estimated_pose.pose.position.y);
    error.z = std::fabs(true_pose.pose.position.z - estimated_pose.pose.position.z);
    return error;
}

// 计算姿态误差
OrientationError Utility::calculateOrientationError(const geometry_msgs::PoseStamped& true_pose,
                                           const geometry_msgs::PoseStamped& estimated_pose) {
    OrientationError error;
    error.roll = std::fabs(true_pose.pose.orientation.x - estimated_pose.pose.orientation.x);
    error.pitch = std::fabs(true_pose.pose.orientation.y - estimated_pose.pose.orientation.y);
    error.yaw = std::fabs(true_pose.pose.orientation.z - estimated_pose.pose.orientation.z);
    return error;
}

// 计算速度误差
double Utility::calculateVelocityError(const geometry_msgs::TwistStamped& true_twist,
                              const geometry_msgs::TwistStamped& estimated_twist) {
    double vx_error = std::fabs(true_twist.twist.linear.x - estimated_twist.twist.linear.x);
    double vy_error = std::fabs(true_twist.twist.linear.y - estimated_twist.twist.linear.y);
    double vz_error = std::fabs(true_twist.twist.linear.z - estimated_twist.twist.linear.z);
    return std::sqrt(vx_error * vx_error + vy_error * vy_error + vz_error * vz_error);
}
