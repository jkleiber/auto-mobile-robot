#ifndef SIM_INTERFACE_H
#define SIM_INTERFACE_H

#include <unistd.h>

// Gazebo
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

// GNC
#include "navigation/diffdrive_ekf.h"
#include "control/control_output.h"

// Robot info
#include "constants.h"

class SimInterface
{
    public:
        SimInterface(RamseteOutput *ctrl, 
                     DiffDriveEKFInput *sensor_data) : 
                ctrl_(ctrl), 
                sensor_data_(sensor_data),
                node_(new gazebo::transport::Node()){
                    usleep(5000000);
                }

        void init();
        void update();

        // Sensor updates
        void imu_update(ConstVector3dPtr &msg);
        void wheel_vel_update(ConstVector2dPtr &msg);

        virtual ~SimInterface();
    
    private:
        RamseteOutput *const ctrl_;
        DiffDriveEKFInput *sensor_data_;

        // Gazebo stuff
        gazebo::transport::NodePtr node_;
        gazebo::transport::PublisherPtr pub_;
        gazebo::transport::SubscriberPtr imu_sub_;
        gazebo::transport::SubscriberPtr vel_sub_;

        // Sensors
        gazebo::msgs::Vector3d imu_msg_;
        gazebo::msgs::Vector2d vel_msg_;


        // Publish messages to the simulated drivetrain
        void publish_message(double left, double right);
};

#endif