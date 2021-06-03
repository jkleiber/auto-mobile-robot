#ifndef STATE_SPACE_H
#define STATE_SPACE_H

#include <Eigen/Dense>

class StateSpace
{
    public:
        StateSpace(Eigen::MatrixXd A, 
                   Eigen::MatrixXd B, 
                   Eigen::MatrixXd C, 
                   Eigen::MatrixXd D, 
                   Eigen::VectorXd x0,
                   Eigen::VectorXd *u,
                   Eigen::VectorXd *y) :
            A_(A), B_(B), C_(C), D_(D), x_(x0), u_(u), y_(y) {}

        virtual ~StateSpace();

        void update();

    private:
        // State Matrices
        Eigen::MatrixXd A_;
        Eigen::MatrixXd B_;
        Eigen::MatrixXd C_;
        Eigen::MatrixXd D_;

        // Internal state
        Eigen::VectorXd x_;

        // Input
        Eigen::VectorXd *const u_;

        // Output pointer
        Eigen::VectorXd *y_;

};


#endif