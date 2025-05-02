#include <iostream>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <osqp++.h>

using namespace Eigen;

const double Ts = 0.1;
const int N = 20;

MatrixXd A_d(4,4), B1_d(4,1), B2_d(4,1);
MatrixXd AX, BU, BV, H;
double umin = -7 * M_PI / 36;
double umax = 7 * M_PI / 36;

MatrixXd matrix_power(const MatrixXd& A, int p) {
    if (p == 0) return MatrixXd::Identity(A.rows(), A.cols());
    if (p == 1) return A;
    MatrixXd result = A;
    for (int i = 1; i < p; ++i) {
        result = result * A;
    }
    return result;
}

void mpc_init() {
    double mass = 2.3;
    double Lf = 0.12;
    double Lr = 0.132;
    double Caf = 70;
    double Car = 70;
    double Iz = 0.04;
    double Vx = 0.3;

    MatrixXd A_c(4,4);
    A_c << 0, 1, 0, 0,
           0, -(2*Caf + 2*Car)/(mass*Vx), (2*Caf + 2*Car)/mass, (-2*Caf*Lf + 2*Car*Lr)/(mass*Vx),
           0, 0, 0, 1,
           0, (-2*Caf*Lf + 2*Car*Lr)/(Iz*Vx), (2*Caf*Lf - 2*Car*Lr)/Iz, (-2*Caf*Lf*Lf - 2*Car*Lr*Lr)/(Iz*Vx);

    MatrixXd B_c(4,2);
    B_c << 0, 0,
           2*Caf/mass, (-2*Caf*Lf + 2*Car*Lr)/(mass*Vx) - Vx,
           0, 0,
           2*Caf*Lf/Iz, (-2*Caf*Lf*Lf - 2*Car*Lr*Lr)/(Iz*Vx);

    MatrixXd M(6, 6);
    M.setZero();
    M.block(0, 0, 4, 4) = A_c;
    M.block(0, 4, 4, 2) = B_c;
    M = M * Ts;

    MatrixXd Md = M.exp();
    A_d = Md.block(0, 0, 4, 4);
    MatrixXd B_d = Md.block(0, 4, 4, 2);
    B1_d = B_d.col(0);
    B2_d = B_d.col(1);

    int n = 4;
    int m_dim = 1;

    AX = MatrixXd::Zero((N+1)*n, n);
    for (int i = 0; i <= N; ++i) {
        AX.block(i*n, 0, n, n) = matrix_power(A_d, i);
    }

    BU = MatrixXd::Zero((N+1)*n, N*m_dim);
    for (int i = 1; i <= N; ++i) {
        for (int j = 0; j < i; ++j) {
            BU.block(i*n, j*m_dim, n, m_dim) = matrix_power(A_d, i-j-1) * B1_d;
        }
    }

    BV = MatrixXd::Zero((N+1)*n, N*m_dim);
    for (int i = 1; i <= N; ++i) {
        for (int j = 0; j < i; ++j) {
            BV.block(i*n, j*m_dim, n, m_dim) = matrix_power(A_d, i-j-1) * B2_d;
        }
    }

    MatrixXd Q = MatrixXd::Zero(4,4);
    Q(0,0) = 10000;
    Q(2,2) = 1;
    MatrixXd QN = Q;
    MatrixXd R = MatrixXd::Identity(1,1) * 50;

    MatrixXd QX = MatrixXd::Zero((N+1)*n, (N+1)*n);
    for (int i = 0; i < N; ++i)
        QX.block(i*n, i*n, n, n) = Q;
    QX.block(N*n, N*n, n, n) = QN;

    MatrixXd RU = MatrixXd::Zero(N*m_dim, N*m_dim);
    for (int i = 0; i < N; ++i)
        RU.block(i*m_dim, i*m_dim, m_dim, m_dim) = R;

    H = MatrixXd::Zero((N+1)*n + N*m_dim, (N+1)*n + N*m_dim);
    H.block(0, 0, (N+1)*n, (N+1)*n) = QX;
    H.block((N+1)*n, (N+1)*n, N*m_dim, N*m_dim) = RU;
    H += MatrixXd::Identity(H.rows(), H.cols()) * 1e-6;
}

float mpc_control(const VectorXd& x0, const VectorXd& v_k) {
    int nx = 4;
    int nu = 1;
    int z_dim = (N+1)*nx + N*nu;

    MatrixXd G = H;
    VectorXd g = VectorXd::Zero(z_dim);

    VectorXd lb = VectorXd::Constant(z_dim, -1e20);
    VectorXd ub = VectorXd::Constant(z_dim,  1e20);
    for (int i = 0; i < N; ++i) {
        lb((N+1)*nx + i*nu) = umin;
        ub((N+1)*nx + i*nu) = umax;
    }

    MatrixXd Aeq((N+1)*nx, z_dim);
    Aeq.block(0, 0, (N+1)*nx, (N+1)*nx) = MatrixXd::Identity((N+1)*nx, (N+1)*nx);
    Aeq.block(0, (N+1)*nx, (N+1)*nx, N*nu) = -BU;
    VectorXd beq = AX * x0 + BV * v_k;

    osqp::OsqpInstance instance;
    instance.objective.matrix = G.sparseView();
    instance.objective.linear = g;
    instance.linear_constraints_matrix = Aeq.sparseView();
    instance.constraints.lower = beq;
    instance.constraints.upper = beq;

    osqp::OsqpSolver solver;
    solver.Init(instance);
    auto result = solver.Solve();

    if (result.status != osqp::OsqpExitCode::kOptimal)
    {
        std::cerr << "[❌] OSQP solver failed to solve" << std::endl;
        return 0.0f;
    }

    VectorXd z_opt = result.primal_solution;
    double u_cmd = z_opt((N+1)*nx);
    return static_cast<float>(-u_cmd * 180.0 / M_PI);
}

int main() {
    mpc_init();

    VectorXd x0(4);
    x0 << 2.0, 0.0, 0.1, 0.0;

    VectorXd v_k = VectorXd::Zero(N * 1);

    float steering_angle_deg = mpc_control(x0, v_k);
    std::cout << "Computed steering angle (deg): " << steering_angle_deg << std::endl;

    return 0;
}
