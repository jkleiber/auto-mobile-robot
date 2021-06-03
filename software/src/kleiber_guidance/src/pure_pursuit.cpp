#include "kleiber_guidance/pure_pursuit.h"

PurePursuit::~PurePursuit()
{
    // Close the filestreams
    traj_file_.close();
}

void PurePursuit::init()
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
        std::cerr << "PurePursuit Error: Unable to open trajectory file.\n";
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
                    std::cerr << "PurePursuit Error: cell misalignment. Trajectory may be incorrectly read\n";
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
        traj_pt.lin_vel_constraints(1) = 1;
        traj_pt.ang_vel_constraints(0) = -1.0;
        traj_pt.ang_vel_constraints(1) = 1.0;

        // Add the point to the deque
        traj_.push_back(traj_pt);
    }
    
    
}

void PurePursuit::update()
{
    // Search radius
    // TODO: make configurable
    double radius = 0.6;

    TrajectoryPoint lookahead = *ref_pt_;

    // Look ahead from the most recent target to find the next target
    for(int i = closest_pt_idx_ + 1; i < traj_.size()-1; ++i)
    {
        // Get the segment information
        TrajectoryPoint segment_start = traj_.at(i);
        TrajectoryPoint segment_end = traj_.at(i + 1);

        // Put everything into (x,y)
        Eigen::Vector2d robot_xy, seg_start, seg_end;
        robot_xy << cur_state_->x(0), cur_state_->x(1);
        seg_start << segment_start.state(0), segment_start.state(1);
        seg_end << segment_end.state(0), segment_end.state(1);

        // Find the difference between robot position and the segment endpoints
        Eigen::Vector2d start_delta = seg_start - robot_xy;
        Eigen::Vector2d end_delta = seg_end - robot_xy;
        Eigen::Vector2d seg_delta = seg_end - seg_start;

        // Find the quadratic disciminant
        double seg_dist = sqrt((seg_end - seg_start).norm());
        double D = start_delta(0) * end_delta(1) - end_delta(0) * start_delta(1);
        double discriminant = pow(radius, 2) * pow(seg_dist, 2) - pow(D, 2);

        // If the discriminant is non-negative, then there is an intersection
        if (discriminant >= 0)
        {
            // Take the square root of the discriminant
            double sqrt_disc = sqrt(discriminant);

            // Components of segment differences
            double dx = seg_delta(0);
            double dy = seg_delta(1);

            // Find the intersections
            double sign_dy = dy / fabs(dy);

            // Intersection 1
            double x1 = (D * dy + sign_dy * dx * sqrt_disc) / (pow(seg_dist, 2));
            double y1 = (-D * dx + abs(dy) * sqrt_disc) / (pow(seg_dist, 2));

            // Intersection 2
            double x2 = (D * dy - sign_dy * dx * sqrt_disc) / (pow(seg_dist, 2));
            double y2 = (-D * dx - abs(dy) * sqrt_disc) / (pow(seg_dist, 2));

            bool valid1 = (std::min(start_delta(0), end_delta(0)) < x1 && x1 < std::max(start_delta(0), end_delta(0)))
                        ||(std::min(start_delta(1), end_delta(1)) < y1 && y1 < std::max(start_delta(1), end_delta(1)));

            bool valid2 = (std::min(start_delta(0), end_delta(0)) < x2 && x2 < std::max(start_delta(0), end_delta(0)))
                        ||(std::min(start_delta(1), end_delta(1)) < y2 && y2 < std::max(start_delta(1), end_delta(1)));

            if (valid1)
            {
                // Compute intersection
                Eigen::Vector2d pt = robot_xy;
                pt(0) += x1;
                pt(1) += y1;

                // Update lookahead
                lookahead = segment_start;
                lookahead.state(0) = pt(0);
                lookahead.state(1) = pt(1);

                break;
            }

            bool better = abs(x1 - end_delta(0)) > abs(x2 - end_delta(0)) 
                        || abs(y1 - end_delta(1)) > abs(y2 - end_delta(1));
            if (valid2 && better)
            {
                // Compute intersection
                Eigen::Vector2d pt = robot_xy;
                pt(0) += x2;
                pt(1) += y2;

                // Update lookahead
                lookahead = segment_start;
                lookahead.state(0) = pt(0);
                lookahead.state(1) = pt(1);

                break;
            }
            
        }
    }

    std::cout << lookahead.state << std::endl << std::endl;

    // Set the reference point
    *ref_pt_ = lookahead;

}
