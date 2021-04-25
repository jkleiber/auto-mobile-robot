#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo as a client
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Publish to the  velodyne topic
  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::Vector2d>("~/six_wheel_robot/cmd_vel");

  // Wait for a subscriber to connect to this publisher
  pub->WaitForConnection();

  // Create a a vector2 message
  gazebo::msgs::Vector2d msg;

  // Set the velocity in the x-component
  gazebo::msgs::Set(&msg, ignition::math::Vector2d(std::atof(_argv[1]), std::atof(_argv[2])));

  // Send the message
  pub->Publish(msg);

  // Make sure to shut everything down.
  gazebo::client::shutdown();

}