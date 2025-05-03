#include "mpc.hpp"
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>
#include <OsqpEigen/OsqpEigen.h>
#include <chrono>
#include <algorithm>

const double Ts = 0.1;
const int N = 15;

Eigen::MatrixXd A_d(4, 4), B1_d(4, 1), B2_d(4, 1);
Eigen::MatrixXd AX, BU, BV, H;
double umin = -30 * M_PI / 180;
double umax = 30 * M_PI / 180;

Eigen::MatrixXd matrixPower(const Eigen::MatrixXd& A, int p) {
    if (p == 0) return Eigen::MatrixXd::Identity(A.rows(), A.cols());
    if (p == 1) return A;
    Eigen::MatrixXd result = A;
    for (int i = 1; i < p; ++i) {
        result = result * A;
    }
    return result;
}

void mpcInit(float Q1Coff, float Q2Coff, float RCoff) {
    double mass = 2.3;
    double Lf = 0.12;
    double Lr = 0.132;
    double Caf = 70;
    double Car = 70;
    double Iz = 0.04;
    double Vx = 0.3;

    Eigen::MatrixXd A_c(4, 4);
    A_c << 0, 1, 0, 0,
           0, -(2*Caf + 2*Car)/(mass*Vx), (2*Caf + 2*Car)/mass, (-2*Caf*Lf + 2*Car*Lr)/(mass*Vx),
           0, 0, 0, 1,
           0, (-2*Caf*Lf + 2*Car*Lr)/(Iz*Vx), (2*Caf*Lf - 2*Car*Lr)/Iz, (-2*Caf*Lf*Lf - 2*Car*Lr*Lr)/(Iz*Vx);

    Eigen::MatrixXd B_c(4, 2);
    B_c << 0, 0,
           2*Caf/mass, (-2*Caf*Lf + 2*Car*Lr)/(mass*Vx) - Vx,
           0, 0,
           2*Caf*Lf/Iz, (-2*Caf*Lf*Lf - 2*Car*Lr*Lr)/(Iz*Vx);

    Eigen::MatrixXd M(6, 6);
    M.setZero();
    M.block(0, 0, 4, 4) = A_c;
    M.block(0, 4, 4, 2) = B_c;
    M = M * Ts;

    Eigen::MatrixXd Md = M.exp();
    A_d = Md.block(0, 0, 4, 4);
    Eigen::MatrixXd B_d = Md.block(0, 4, 4, 2);
    B1_d = B_d.col(0);
    B2_d = B_d.col(1);

    int n = 4;
    int m_dim = 1;

    AX = Eigen::MatrixXd::Zero((N + 1) * n, n);
    for (int i = 0; i <= N; ++i) {
        AX.block(i * n, 0, n, n) = matrixPower(A_d, i);
    }

    BU = Eigen::MatrixXd::Zero((N + 1) * n, N * m_dim);
    for (int i = 1; i <= N; ++i) {
        for (int j = 0; j < i; ++j) {
            BU.block(i * n, j * m_dim, n, m_dim) = matrixPower(A_d, i - j - 1) * B1_d;
        }
    }

    BV = Eigen::MatrixXd::Zero((N + 1) * n, N * m_dim);
    for (int i = 1; i <= N; ++i) {
        for (int j = 0; j < i; ++j) {
            BV.block(i * n, j * m_dim, n, m_dim) = matrixPower(A_d, i - j - 1) * B2_d;
        }
    }

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 4);
    Q(0, 0) = Q1Coff;//40
    Q(2, 2) = Q2Coff; //5
    Eigen::MatrixXd QN = Q;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1) * RCoff; //5;

    Eigen::MatrixXd QX = Eigen::MatrixXd::Zero((N + 1) * n, (N + 1) * n);
    for (int i = 0; i < N; ++i)
        QX.block(i * n, i * n, n, n) = Q;
    QX.block(N * n, N * n, n, n) = QN;

    Eigen::MatrixXd RU = Eigen::MatrixXd::Zero(N * m_dim, N * m_dim);
    for (int i = 0; i < N; ++i)
        RU.block(i * m_dim, i * m_dim, m_dim, m_dim) = R;

    H = Eigen::MatrixXd::Zero((N + 1) * n + N * m_dim, (N + 1) * n + N * m_dim);
    H.block(0, 0, (N + 1) * n, (N + 1) * n) = QX;
    H.block((N + 1) * n, (N + 1) * n, N * m_dim, N * m_dim) = RU;
    H += Eigen::MatrixXd::Identity(H.rows(), H.cols()) * 1e-6;
}

float mpcControl(Eigen::VectorXd& x0, Eigen::VectorXd& v_k) {
    int nx = 4;
    int nu = 1;
    int z_dim = (N + 1) * nx + N * nu;

    Eigen::MatrixXd G = H;
    Eigen::VectorXd g = Eigen::VectorXd::Zero(z_dim);

    Eigen::VectorXd lb = Eigen::VectorXd::Constant(z_dim, -1e20);
    Eigen::VectorXd ub = Eigen::VectorXd::Constant(z_dim, 1e20);
    for (int i = 0; i < N; ++i) {
        lb((N + 1) * nx + i * nu) = umin;
        ub((N + 1) * nx + i * nu) = umax;
    }

    Eigen::MatrixXd Aeq((N + 1) * nx, z_dim);
    Aeq.block(0, 0, (N + 1) * nx, (N + 1) * nx) = Eigen::MatrixXd::Identity((N + 1) * nx, (N + 1) * nx);
    Aeq.block(0, (N + 1) * nx, (N + 1) * nx, N * nu) = -BU;
    Eigen::VectorXd beq = AX * x0 + BV * v_k;

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(z_dim);
    solver.data()->setNumberOfConstraints((N + 1) * nx);

    Eigen::SparseMatrix<double> G_sparse = G.sparseView();
    Eigen::SparseMatrix<double> Aeq_sparse = Aeq.sparseView();

    solver.data()->setHessianMatrix(G_sparse);
    solver.data()->setGradient(g);
    solver.data()->setLinearConstraintsMatrix(Aeq_sparse);
    solver.data()->setLowerBound(beq);
    solver.data()->setUpperBound(beq);

    solver.initSolver();
    solver.solveProblem();

    Eigen::VectorXd z_opt = solver.getSolution();
    double u_cmd = z_opt((N + 1) * nx);
    u_cmd = std::clamp(u_cmd, umin, umax);
    return static_cast<float>(-u_cmd * 180.0 / M_PI);
}
