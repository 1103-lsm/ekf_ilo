#pragma once
#include <deque>
#include <Eigen/Dense>
#include "filter.hpp"
#include <gram_savitzky_golay/gram_savitzky_golay.h>
#include "../utils/utility.h"

#define NUM_LEG 4
#define NUM_DOF 12

class SensorData
{
public:
    SensorData()
    {
        for (int i = 0; i < 3; ++i)
        {
            acc_filter[i] = MovingWindowFilter(5);
            ang_vel_filter[i] = MovingWindowFilter(5);
        }
        for (int i = 0; i < NUM_DOF; ++i)
        {
            joint_pos_filter[i] = MovingWindowFilter(5);
            joint_vel_filter[i] = MovingWindowFilter(5);
            joint_vel_filter_sgolay[i] = gram_sg::SavitzkyGolayFilter(sgolay_order, sgolay_order, sgolay_order, 1);
        }

        for (int i = 0; i < 4; ++i)
        {
            orientation_filter[i] = MovingWindowFilter(5);
        }

        dt = 0.001;    
    }

    void input_imu(Eigen::Matrix<double, 3, 1> acc, Eigen::Matrix<double, 3, 1> ang_vel)
    {
        for (int i = 0; i < 3; ++i)
        {
            this->acc[i] = acc_filter[i].CalculateAverage(acc[i]);
            this->ang_vel[i] = ang_vel_filter[i].CalculateAverage(ang_vel[i]);
        }
    }

    void input_orientation(Eigen::Quaterniond orientation)
    {
        this->orientation.w() = orientation_filter[0].CalculateAverage(orientation.w());
        this->orientation.x() = orientation_filter[1].CalculateAverage(orientation.x());
        this->orientation.y() = orientation_filter[2].CalculateAverage(orientation.y());
        this->orientation.z() = orientation_filter[3].CalculateAverage(orientation.z());

        this->root_quat = Eigen::Quaterniond(orientation.w(),
                                             orientation.x(),
                                             orientation.y(),
                                             orientation.z());

        this->root_rot_mat = this->root_quat.toRotationMatrix();

        this->root_euler = Utility::quat_to_euler(this->root_quat);
    }

    void input_leg(Eigen::Matrix<double, NUM_DOF, 1> joint_pos, Eigen::Matrix<double, NUM_DOF, 1> joint_vel, Eigen::Matrix<double, NUM_LEG, 1> forces)
    {
        for (int i = 0; i < NUM_DOF; ++i)
        {
            this->joint_pos[i] = joint_pos_filter[i].CalculateAverage(joint_pos[i]);
            // this->joint_vel[i] = joint_vel_filter[i].CalculateAverage(joint_vel[i]);

            if (sgolay_values[i].size() < sgolay_frame)
            {
                this->joint_vel[i] = joint_vel_filter[i].CalculateAverage(joint_vel[i]);
                sgolay_values[i].push_back(joint_pos[i]);
            }
            else
            {
                sgolay_values[i].pop_front();
                sgolay_values[i].push_back(joint_pos[i]);
                this->joint_vel[i] = joint_vel_filter_sgolay[i].filter(sgolay_values[i]) / average_dt;
            }
        }

        for (size_t i = 0; i< NUM_LEG; ++i) {
                if (forces[i] < this-> foot_force_min[i]) {
                    this-> foot_force_min[i] = 
                        0.9 * this-> foot_force_min[i] + 0.1 * forces[i] ;
                }
                if (forces[i]  > this-> foot_force_max[i]) {
                    this-> foot_force_max[i] = 
                        0.9 * this-> foot_force_max[i] + 0.1 * forces[i] ;
                }

                this-> foot_force_min[i] *= 0.9991;
                this-> foot_force_max[i] *= 0.997;
                this-> foot_force_contact_threshold[i] = this-> foot_force_min[i] + 0.6*(this-> foot_force_max[i]-this-> foot_force_min[i]);
                this-> plan_contacts[i] = 
                    forces[i] > this-> foot_force_contact_threshold[i]? 1.0 : 0.0;
            }
    }

    void input_dt(double dt)
    {
        this->dt = dt;
        dt_values.push_back(dt);
        if (dt_values.size() > sgolay_frame)
        {
            dt_values.pop_front();
        }
        average_dt = 0.0;
        for (long unsigned int i = 0; i < dt_values.size(); ++i)
        {
            average_dt += dt_values[i];
        }
        average_dt /= dt_values.size();
    }

    Eigen::Vector3d acc;
    Eigen::Vector3d ang_vel;
    Eigen::Quaterniond orientation;

    Eigen::Matrix3d root_rot_mat;
    Eigen::Quaterniond root_quat;
    Eigen::Vector3d root_euler;


    Eigen::Matrix<double, NUM_DOF, 1> joint_pos;
    Eigen::Matrix<double, NUM_DOF, 1> joint_vel;
    Eigen::Matrix<double, NUM_LEG, 1> plan_contacts;
    double dt;
    double average_dt;

private:
    MovingWindowFilter acc_filter[3];
    MovingWindowFilter ang_vel_filter[3];
    MovingWindowFilter orientation_filter[4];
    MovingWindowFilter joint_pos_filter[NUM_DOF];
    MovingWindowFilter joint_vel_filter[NUM_DOF];
    gram_sg::SavitzkyGolayFilter joint_vel_filter_sgolay[NUM_DOF];
    std::deque<double> sgolay_values[NUM_DOF];
    std::deque<double> dt_values;

    Eigen::Matrix<double, NUM_LEG,1> foot_force_min;
    Eigen::Matrix<double, NUM_LEG,1> foot_force_max;
    Eigen::Matrix<double, NUM_LEG,1> foot_force_contact_threshold;

    const long unsigned int sgolay_order = 7;
    const long unsigned int sgolay_frame = 15; 
};

class A1KF
{
public:
    A1KF(){};
    bool is_inited() { return KF_initialized; }

    bool KF_initialized = false;
};