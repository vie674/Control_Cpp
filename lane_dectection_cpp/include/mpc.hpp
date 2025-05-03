#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include <Eigen/Dense>

float mpcControl(Eigen::VectorXd& x0, Eigen::VectorXd& v_k);
void mpcInit(float Q1Coff, float Q2Coff, float RCoff);

#endif // MPC_CONTROLLER_H
