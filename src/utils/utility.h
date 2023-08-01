/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <cmath>
#include <vector>
#include <deque>
#include <utility>
#include <cassert>
#include <cstring>
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "parameters.h"

// 定义位置误差结构体，用于存储位置误差的 x, y, z 分量
struct PositionError {
    double x;
    double y;
    double z;
};

// 定义姿态误差结构体，用于存储姿态误差的滚动、俯仰、偏航角分量
struct OrientationError {
    double roll;
    double pitch;
    double yaw;
};

class Utility
{
public:
    static PositionError calculatePositionError(const geometry_msgs::PoseStamped& true_pose,
                                     const geometry_msgs::PoseStamped& estimated_pose);
    static OrientationError calculateOrientationError(const geometry_msgs::PoseStamped& true_pose,
            const geometry_msgs::PoseStamped& estimated_pose);
    static double calculateVelocityError(const geometry_msgs::TwistStamped& true_twist,
                              const geometry_msgs::TwistStamped& estimated_twist);
    static Eigen::Matrix3d skew(Eigen::Vector3d vec);
    static Eigen::Vector3d quat_to_euler(Eigen::Quaterniond quat);

};
