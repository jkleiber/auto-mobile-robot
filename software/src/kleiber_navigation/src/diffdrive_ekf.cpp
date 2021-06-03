#include "kleiber_navigation/diffdrive_ekf.h"

DiffDriveEKF::~DiffDriveEKF(){}

void DiffDriveEKF::init()
{
    last_time_ = std::chrono::steady_clock::now();

    // Initialize EKF state
    ekf_state_ = Eigen::VectorXd::Zero(5);
    ekf_state_(0) = robot_state_->x(0);
    ekf_state_(1) = robot_state_->x(1);
    ekf_state_(2) = robot_state_->x(2);

    // Covariance and noise/disturbances
    P_ = Eigen::MatrixXd::Identity(5,5);
    S_ = Eigen::MatrixXd::Zero(3,3);
    Q_ = 0.25 * Eigen::MatrixXd::Identity(5,5);
    R_ = 0.4 * Eigen::MatrixXd::Identity(3,3);

    // Jacobians
    F_ = Eigen::MatrixXd::Identity(5,5);
    H_ = Eigen::MatrixXd::Zero(3,5);

    // Kalman Gain
    K_ = Eigen::MatrixXd::Zero(5,3);

    // Identity matrix
    I_ = Eigen::MatrixXd::Identity(5,5);
}

void DiffDriveEKF::update()
{
    // Compute elapsed time in seconds
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(current_time - last_time_).count();
    last_time_ = current_time;

    // Compute state prediction
    Eigen::VectorXd x_predict = dynamics(ekf_state_, ctrl_->u, dt);

    // Compute jacobian F
    dynamics_jacobian(ekf_state_, ctrl_->u, dt);

    // Compute covariance P
    P_ = F_*P_*F_.transpose() + Q_;

    // Collect measurements into a vector
    Eigen::VectorXd measure(3);
    measure << sensor_input_->imu_yaw, sensor_input_->linear_velocity, sensor_input_->angular_velocity;

    // Compute innovation
    Eigen::VectorXd innov = measure - measurement_model(x_predict);
    measurement_jacobian(x_predict);

    // Find innovation covariance
    S_ = H_ * P_ * H_.transpose() + R_;

    // Compute Kalman gain
    K_ = P_ * H_.transpose() * S_.inverse();

    // Update the EKF internal state and covariance
    ekf_state_ = x_predict + K_*innov;
    P_ = (I_ - K_*H_)*P_;

    // Update 3-DOF state
    robot_state_->x(0) = ekf_state_(0);
    robot_state_->x(1) = ekf_state_(1);
    robot_state_->x(2) = ekf_state_(2);
}

Eigen::VectorXd DiffDriveEKF::dynamics(Eigen::VectorXd x, Eigen::Vector2d u, double dt)
{
    // [x y theta]^T
    double X = x(0);
    double Y = x(1);
    double theta = x(2);

    // [v w]^T
    double v = u(0);
    double w = u(1);

    // Robot dynamics
    Eigen::VectorXd x_p = Eigen::VectorXd::Zero(5);
    x_p(0) = X + v*cos(theta)*dt;
    x_p(1) = Y + v*sin(theta)*dt;
    x_p(2) = theta + w*dt;
    x_p(3) = v;
    x_p(4) = w;

    return x_p;
}

void DiffDriveEKF::dynamics_jacobian(Eigen::VectorXd x, Eigen::Vector2d u, double dt)
{
    // [x y theta]^T
    double theta = x(2);

    // [v w]^T
    double v = u(0);

    // X
    F_(0, 2) = -v*sin(theta)*dt;
    F_(0, 3) = cos(theta)*dt;

    // Y
    F_(1, 2) = v*cos(theta)*dt;
    F_(1, 3) = sin(theta)*dt;

    // theta
    F_(2, 4) = dt;
}

Eigen::VectorXd DiffDriveEKF::measurement_model(Eigen::VectorXd x)
{
    // Measurements are: [theta, v, w]^T
    Eigen::VectorXd z_p = Eigen::VectorXd::Zero(3);

    z_p(0) = x(2);
    z_p(1) = x(3);
    z_p(2) = x(4);

    return z_p;
}


void DiffDriveEKF::measurement_jacobian(Eigen::VectorXd x)
{
    H_(0, 2) = 1;
    H_(1, 3) = 1;
    H_(2, 4) = 1;
}