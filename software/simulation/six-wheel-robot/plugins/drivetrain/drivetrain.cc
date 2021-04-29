
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
  class SixWheelDrivetrainPlugin : public ModelPlugin
  {
    public: 
        SixWheelDrivetrainPlugin(){}

        virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
        {
            std::cerr << "\nSix Wheel robot plugin attached to " << model->GetName() << std::endl;

            // Save model pointer
            this->model_ = model;

            // Save the wheel joints
            this->left_back_wheel_ = model->GetJoint("left_back_wheel_hinge");
            this->left_center_wheel_ = model->GetJoint("left_center_wheel_hinge");
            this->left_front_wheel_ = model->GetJoint("left_front_wheel_hinge");
            this->right_back_wheel_ = model->GetJoint("right_back_wheel_hinge");
            this->right_center_wheel_ = model->GetJoint("right_center_wheel_hinge");
            this->right_front_wheel_ = model->GetJoint("right_front_wheel_hinge");

            // Make identical PID controllers for each wheel since they are chained
            double kp = 0.025;
            double ki = 0.00001;
            double kd = 0.0;
            this->left_back_pid_ = common::PID(kp, ki, kd);
            this->left_center_pid_ = common::PID(kp, ki, kd);
            this->left_front_pid_ = common::PID(kp, ki, kd);
            this->right_back_pid_ = common::PID(kp, ki, kd);
            this->right_center_pid_ = common::PID(kp, ki, kd);
            this->right_front_pid_ = common::PID(kp, ki, kd);

            // Set joint controllers
            this->model_->GetJointController()->SetVelocityPID(this->left_back_wheel_->GetScopedName(), this->left_back_pid_);
            this->model_->GetJointController()->SetVelocityPID(this->left_center_wheel_->GetScopedName(), this->left_center_pid_);
            this->model_->GetJointController()->SetVelocityPID(this->left_front_wheel_->GetScopedName(), this->left_front_pid_);
            this->model_->GetJointController()->SetVelocityPID(this->right_back_wheel_->GetScopedName(), this->right_back_pid_);
            this->model_->GetJointController()->SetVelocityPID(this->right_center_wheel_->GetScopedName(), this->right_center_pid_);
            this->model_->GetJointController()->SetVelocityPID(this->right_front_wheel_->GetScopedName(), this->right_front_pid_);

            // Set velocity limits
            this->vel_limit_ = 100;

            // Create the transport node
            this->node_ = transport::NodePtr(new transport::Node());
            this->node_->Init(this->model_->GetWorld()->Name());

            // Create a topic for simulation
            std::string topic_name = "~/" + this->model_->GetName() + "/cmd_vel";

            // Subscribe to this topic
            this->sub_ = this->node_->Subscribe(topic_name, &SixWheelDrivetrainPlugin::OnMsg, this);

            // Publish sensor data to topics
            this->imu_pub_ = this->node_->Advertise<gazebo::msgs::Vector3d>("~/" + this->model_->GetName() + "/imu");
            this->vel_pub_ = this->node_->Advertise<gazebo::msgs::Vector2d>("~/" + this->model_->GetName() + "/wheel_vel");

            // Connect to world updates
            this->update_conn_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&SixWheelDrivetrainPlugin::OnUpdate, this, _1));
        }

        void SetVelocity(const double &left, const double &right)
        {   
            // Clamp input velocities
            double _left = std::min(vel_limit_, std::max(-vel_limit_, left));
            double _right = std::min(vel_limit_, std::max(-vel_limit_, right));

            // Set speed
            this->model_->GetJointController()->SetVelocityTarget(this->left_back_wheel_->GetScopedName(), _left);
            this->model_->GetJointController()->SetVelocityTarget(this->left_center_wheel_->GetScopedName(), _left);
            this->model_->GetJointController()->SetVelocityTarget(this->left_front_wheel_->GetScopedName(), _left);
            this->model_->GetJointController()->SetVelocityTarget(this->right_back_wheel_->GetScopedName(), _right);
            this->model_->GetJointController()->SetVelocityTarget(this->right_center_wheel_->GetScopedName(), _right);
            this->model_->GetJointController()->SetVelocityTarget(this->right_front_wheel_->GetScopedName(), _right);
        }

        void OnMsg(ConstVector2dPtr &msg)
        {
            this->SetVelocity(msg->x(), msg->y());
        }

        void OnUpdate(const common::UpdateInfo& info)
        {
            PublishIMU();
            PublishWheelSpeeds();
        }

        void PublishIMU()
        {
            // Make a message for roll, pitch, yaw
            gazebo::msgs::Vector3d imu_msg;

            // Get model RPY
            ignition::math::Pose3d robot_pose = this->model_->RelativePose();
            
            // Roll
            imu_msg.set_x(robot_pose.Roll());

            // Pitch
            imu_msg.set_y(robot_pose.Pitch());

            // Yaw 
            imu_msg.set_z(robot_pose.Yaw());

            // Publish the message
            imu_pub_->Publish(imu_msg);

        }

        void PublishWheelSpeeds()
        {
            gazebo::msgs::Vector2d wheel_vel_msg;

            // Left wheel
            double left = (left_back_wheel_->GetVelocity(0) 
                         + left_center_wheel_->GetVelocity(0) 
                         + left_front_wheel_->GetVelocity(0)) / 3.0;
            wheel_vel_msg.set_x(left);

            // Right wheel
            double right = (right_back_wheel_->GetVelocity(0) 
                         + right_center_wheel_->GetVelocity(0) 
                         + right_front_wheel_->GetVelocity(0)) / 3.0;
            wheel_vel_msg.set_y(right);

            // Publish message
            vel_pub_->Publish(wheel_vel_msg);
        }

    private:
        /// \brief Pointer to the model.
        physics::ModelPtr model_;

        /// \brief Transport node and subscriber
        transport::NodePtr node_;
        transport::SubscriberPtr sub_;

        // Publisher and update connection
        event::ConnectionPtr update_conn_;
        transport::PublisherPtr imu_pub_;
        transport::PublisherPtr vel_pub_;

        /// \brief Pointers to the wheel hinges.
        // Left
        physics::JointPtr left_back_wheel_;
        physics::JointPtr left_center_wheel_;
        physics::JointPtr left_front_wheel_;
        // Right
        physics::JointPtr right_back_wheel_;
        physics::JointPtr right_center_wheel_;
        physics::JointPtr right_front_wheel_;

        /// \brief PID controllers for the joints.
        common::PID left_back_pid_;
        common::PID left_center_pid_;
        common::PID left_front_pid_;
        common::PID right_back_pid_;
        common::PID right_center_pid_;
        common::PID right_front_pid_;

        // Velocity limit
        double vel_limit_;
  };
  GZ_REGISTER_MODEL_PLUGIN(SixWheelDrivetrainPlugin)
}