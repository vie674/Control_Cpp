#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>

float mpcControl(Eigen::VectorXd& x0, Eigen::VectorXd& v_k);
void mpcInit(float Q1Coff, float Q2Coff, float RCoff);
int stanleyControl(double e, double psi, double v, double k);

#endif // CONTROLLER_H
