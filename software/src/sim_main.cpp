#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include "loop.h"
#include "sim_interface.h"


int main(int argc, char **argv)
{
    // Load gazebo as a client
    gazebo::client::setup(argc, argv);

    // Initialize the robot variables shared memory
    std::shared_ptr<RobotVariables> robot_vars = std::make_shared<RobotVariables>();

    // Initialize the simulation interface
    std::shared_ptr<SimInterface> sim_interface = std::make_shared<SimInterface>(
        &robot_vars->ctrl_out, 
        &robot_vars->ekf_input
        );
    sim_interface->init();

    // Create the robot software loop
    std::shared_ptr<Loop> sim_loop = std::make_shared<Loop>(robot_vars.get());
    sim_loop->init();

    // Run the robot code
    while(true)
    {
        sim_interface->update();
        sim_loop->update();
    }

    // Make sure to shut everything down.
    gazebo::client::shutdown();
}

