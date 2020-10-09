#include "magnetometer/calibration/calibrator.h"

#include <ros/node_handle.h>

calibrator::calibrator(std::shared_ptr<magnetometer::data_interface>& data_interface)
{
    // Set up variable sets.
    calibrator::m_variables_center = std::make_shared<ifopt::variables_center>();
    calibrator::m_variables_rotation = std::make_shared<ifopt::variables_rotation>();
    calibrator::m_variables_radius = std::make_shared<ifopt::variables_radius>();

    // Set up cost term.
    calibrator::m_cost_objective = std::make_shared<ifopt::cost_objective>(data_interface);

    // Set up problem.
    calibrator::m_problem.AddVariableSet(calibrator::m_variables_center);
    calibrator::m_problem.AddVariableSet(calibrator::m_variables_rotation);
    calibrator::m_problem.AddVariableSet(calibrator::m_variables_radius);
    calibrator::m_problem.AddCostSet(calibrator::m_cost_objective);

    // Set parameters.
    ros::NodeHandle private_handle("~");
    calibrator::m_variables_center->p_max(private_handle.param<double>("calibration_max_center", 50.0));
    calibrator::m_variables_rotation->p_max(private_handle.param<double>("calibration_max_rotation", M_PI));
    calibrator::m_variables_radius->p_max(private_handle.param<double>("calibration_max_radius", 50.0));
    calibrator::m_cost_objective->p_gradient_perturbation(private_handle.param<double>("calibration_gradient_perturbation", 0.000001));
    calibrator::m_solver.SetOption("max_cpu_time", private_handle.param<double>("calibration_max_time", 15.0));

    // Initialize thread.
    calibrator::m_running = false;
}
calibrator::~calibrator()
{
    if(calibrator::m_running)
    {
        calibrator::m_thread.join();
    }
}

bool calibrator::initialize_center(double x, double y, double z)
{
    if(calibrator::m_running)
    {
        return false;
    }

    Eigen::Vector3d initial_value = {x, y, z};
    calibrator::m_variables_center->SetVariables(initial_value);

    return true;
}
bool calibrator::initialize_rotation(double r, double p, double y)
{
    if(calibrator::m_running)
    {
        return false;
    }

    Eigen::Vector3d initial_value = {r, p, y};
    calibrator::m_variables_rotation->SetVariables(initial_value);

    return true;
}
bool calibrator::initialize_radius(double a, double b, double c)
{
    if(calibrator::m_running)
    {
        return false;
    }

    Eigen::Vector3d initial_value = {a, b, c};
    calibrator::m_variables_radius->SetVariables(initial_value);

    return true;
}

bool calibrator::start()
{
    // Start thread.
    if(!calibrator::m_running)
    {
        calibrator::m_thread = boost::thread(&calibrator::thread_worker, this);
        return true;
    }
    else
    {
        return false;
    }
}
Eigen::Matrix<double, 4, 4> calibrator::get_calibration()
{
    
    // desired_radius = 3;
    // s = eye(3);
    // s(1,1) = desired_radius / result(7);
    // s(2,2) = desired_radius / result(8);
    // s(3,3) = desired_radius / result(9);
    // r = rotation_matrix(result(1), result(2), result(3));

    // h = eye(4);
    // h(1:3,1:3) = r*s*r';
    // h(1:3,4) = -result(7:9)';
}


void calibrator::thread_worker()
{
    // Flag thread as running.
    calibrator::m_running = true;

    // Run solver.
    calibrator::m_solver.Solve(calibrator::m_problem);

    // Signal completion.
    calibrator::optimization_completed(calibrator::m_solver.GetReturnStatus() == 0);

    // Flag thread as finished.
    calibrator::m_running = false;
}
