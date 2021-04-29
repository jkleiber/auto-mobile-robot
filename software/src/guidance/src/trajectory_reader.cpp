#include "guidance/trajectory_reader.h"

TrajectoryReader::~TrajectoryReader()
{
    // Close the filestreams
    traj_file_.close();
}

void TrajectoryReader::init()
{
    // Open the file
    std::filesystem::path file = std::filesystem::current_path() / filename_;
    traj_file_.open(file.string());

    // Temporary variables for line by line and cell by cell reading
    std::string cell = "";
    std::string line = "";

    // Don't read if the file isn't open
    if(!traj_file_.good())
    {
        std::cerr << "TrajectoryReader Error: Unable to open trajectory file.\n";
        std::cerr << file << std::endl;
        return;
    }

    // Read the trajectory line by line from a CSV.
    while(getline(traj_file_, line))
    {
        // Split the line into its cells
        std::stringstream ss(line);
        int col = 0;
        double t = 0.0, x = 0.0, y = 0.0, theta = 0.0, v = 0.0, w = 0.0;
        double value = 0.0;
        while(getline(ss, cell, ','))
        {
            // Convert cell to double 
            value = std::atof(cell.c_str());

            // Row Format: t, x, y, theta, v, w
            switch(col)
            {
                case 0:
                    t = value;
                    break;
                case 1:
                    x = value;
                    break;
                case 2:
                    y = value;
                    break;
                case 3:
                    theta = value;
                    break;
                case 4:
                    v = value;
                    break;
                case 5:
                    w = value;
                    break;
                default:
                    std::cerr << "TrajectoryReader Error: cell misalignment. Trajectory may be incorrectly read\n";
                    break;
            }

            // Increment col counter
            col += 1;
        }

        // Create a trajectory point
        TrajectoryPoint traj_pt;
        traj_pt.t = t;
        traj_pt.state(0) = x;
        traj_pt.state(1) = y;
        traj_pt.state(2) = theta;
        traj_pt.linear_velocity = v;
        traj_pt.angular_velocity = w;
        traj_pt.lin_vel_constraints(0) = 0.0;
        traj_pt.lin_vel_constraints(1) = 2.0;
        traj_pt.ang_vel_constraints(0) = -1.0;
        traj_pt.ang_vel_constraints(1) = 1.0;

        // Add the point to the deque
        traj_.push_back(traj_pt);
    }
    
    
}

void TrajectoryReader::update()
{
    if (traj_.size() > 0)
    {
        // std::cout << "norm:\n" << (cur_state_->x - traj_.front().state).norm() << std::endl;

        if((cur_state_->x - traj_.front().state).norm() < 0.2)
        {
            traj_.pop_front();
            std::cout << "Target: \n" << ref_pt_->state << std::endl;
        }

        *ref_pt_ = traj_.front();
    }

    // Only do stuff if the trajectory has at least two elements
    // if(traj_.size() > 1)
    // {
    //     // Remove stale trajectory points until we get a fresh one
    //     while(traj_.front().t < *t_ref_ && traj_.at(1).t < *t_ref_)
    //     {
    //         traj_.pop_front();
    //     }

    //     // Interpolate between the current point and the next point based on time
    //     double t_factor = (*t_ref_ - traj_.front().t) / (traj_.at(1).t - traj_.front().t);

    //     TrajectoryPoint interp_pt1 = traj_.front() * (1 - t_factor);
    //     TrajectoryPoint interp_pt2 = traj_.at(1) * t_factor;
    //     *ref_pt_ = interp_pt1 + interp_pt2;

    //     std::cout << "Target: \n" << ref_pt_->state << std::endl << std::endl;
    // }
    // else if(traj_.size() == 1)
    // {
    //     // Set the active trajectory point as the freshest one available
    //     *ref_pt_ = traj_.front();
    // }
}
