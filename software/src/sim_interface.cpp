#include "sim_interface.h"

SimInterface::~SimInterface()
{

}

void SimInterface::init()
{
    // init the node for communication
    node_->Init();

    // Publish to the drivetrain topic
    pub_ = node_->Advertise<gazebo::msgs::Vector2d>("~/six_wheel_robot/cmd_vel");

    // Subscribe to drivetrain sensors
    imu_sub_ = node_->Subscribe("~/six_wheel_robot/imu", &SimInterface::imu_update, this);
    vel_sub_ = node_->Subscribe("~/six_wheel_robot/wheel_vel", &SimInterface::wheel_vel_update, this);

    // Wait for a subscriber to connect to this publisher
    pub_->WaitForConnection();
}

void SimInterface::update()
{
    // Get control and robot information
    double v = ctrl_->u(0);
    double w = ctrl_->u(1);
    double L = Robot::wheelbase_len;
    double r = Robot::wheel_radius;

    // Calculate left and right wheel velocities
    double left_vel = 2.0*v - (w*L);
    double right_vel = 2.0*v + (w*L);
    double left = left_vel / (2.0*r);
    double right = right_vel / (2.0*r);

    // Publish the velocities to the simulator
    this->publish_message(left, right);

    // simulate communication delay
    usleep(10000);

    // Add sensor inputs from the robot to the sensor input data structure
    double v_in = r*(vel_msg_.x() + vel_msg_.y()) / 2.0;
    double w_in = r*(vel_msg_.y() - vel_msg_.x()) / L;
    this->sensor_data_->linear_velocity = v_in;
    this->sensor_data_->angular_velocity = w_in;
    this->sensor_data_->imu_yaw = imu_msg_.z();
}

void SimInterface::imu_update(ConstVector3dPtr &msg)
{
    // Save IMU message info
    imu_msg_.set_x(msg->x());
    imu_msg_.set_y(msg->y());
    imu_msg_.set_z(msg->z());
}

void SimInterface::wheel_vel_update(ConstVector2dPtr &msg)
{
    // Save wheel velocity
    vel_msg_.set_x(msg->x());
    vel_msg_.set_y(msg->y());
}



void SimInterface::publish_message(double left, double right)
{
    // Create a a vector2 message
    gazebo::msgs::Vector2d msg;

    // Set the velocity in the x-component
    gazebo::msgs::Set(&msg, ignition::math::Vector2d(left, right));

    // Send the message
    pub_->Publish(msg);
}