#ifndef VILO_A1KINEMATICS_H
#define VILO_A1KINEMATICS_H

#include <eigen3/Eigen/Dense>
#include "../utils/parameters.h"

class Kinematics
{

public:
    Kinematics() = default;
    ~Kinematics() = default;

    Eigen::Vector3d fk(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix);

    Eigen::Matrix3d jac(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix);
private:

    void fk_pf_pos(const double in1[3], const double in2[RHO_OPT_SIZE], const double in3[RHO_FIX_SIZE], double p_bf[3]);
    void d_fk_dt(const double in1[3], const double in2[RHO_OPT_SIZE], const double in3[RHO_FIX_SIZE], double jacobian[9]);
};

#endif 
