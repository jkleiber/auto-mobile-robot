
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>

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
            double kp = 0.8;
            double ki = 0.0;
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

            // Create the transport node
            this->node = transport::NodePtr(new transport::Node());
            this->node->Init(this->model_->GetWorld()->Name());

            // Create a topic for simulation
            std::string topic_name = "~/" + this->model_->GetName() + "/cmd_vel";

            // Subscribe to this topic
            this->sub = this->node->Subscribe(topic_name, &SixWheelDrivetrainPlugin::OnMsg, this);
        }

        void SetVelocity(const double &left, const double &right)
        {            
            // Set speed
            this->model_->GetJointController()->SetVelocityTarget(this->left_back_wheel_->GetScopedName(), left);
            this->model_->GetJointController()->SetVelocityTarget(this->left_center_wheel_->GetScopedName(), left);
            this->model_->GetJointController()->SetVelocityTarget(this->left_front_wheel_->GetScopedName(), left);
            this->model_->GetJointController()->SetVelocityTarget(this->right_back_wheel_->GetScopedName(), right);
            this->model_->GetJointController()->SetVelocityTarget(this->right_center_wheel_->GetScopedName(), right);
            this->model_->GetJointController()->SetVelocityTarget(this->right_front_wheel_->GetScopedName(), right);
        }

        void OnMsg(ConstVector2dPtr &msg)
        {
            this->SetVelocity(msg->x(), msg->y());
        }

    private:
        /// \brief Pointer to the model.
        physics::ModelPtr model_;

        /// \brief Transport node and subscriber
        transport::NodePtr node;
        transport::SubscriberPtr sub;

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
  };
  GZ_REGISTER_MODEL_PLUGIN(SixWheelDrivetrainPlugin)
}