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

    // Wait for a subscriber to connect to this publisher
    pub_->WaitForConnection();
}

void SimInterface::update()
{
    // Calculate left and right wheel velocities
    double left = 0.0;
    double right = 0.0;

    // Publish the velocities to the simulator
    this->publish_message(left, right);

    // simulate communication delay
    usleep(20000);
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