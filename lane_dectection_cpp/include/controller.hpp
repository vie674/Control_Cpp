#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>

float mpcControl(Eigen::VectorXd& x0, Eigen::VectorXd& v_k);
void mpcInit(float Q1Coff, float Q2Coff, float RCoff);
int stanleyControl(float e, float psi, float v, float k);

#endif // CONTROLLER_H
