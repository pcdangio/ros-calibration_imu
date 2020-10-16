#include "magnetometer/calibration/calibrator.h"

#include <ros/node_handle.h>

using namespace magnetometer;

calibrator::calibrator(std::shared_ptr<magnetometer::data_interface>& data_interface)
{
    // Initialize true field strength.
    calibrator::m_true_field_strength = 0;

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

bool calibrator::start(const magnetometer::ellipsoid& initial_guess, double true_field_strength)
{
    // Check if thread is running.
    if(!calibrator::m_running)
    {
        // Store ture field strength.
        calibrator::m_true_field_strength = true_field_strength;

        // Initialize values.
        Eigen::Vector3d initial_center;
        initial_guess.get_center(initial_center);
        calibrator::m_variables_center->SetVariables(initial_center);

        Eigen::Vector3d initial_radius;
        initial_guess.get_radius(initial_radius);
        calibrator::m_variables_radius->SetVariables(initial_radius);

        Eigen::Vector3d initial_rotation;
        initial_guess.get_rotation(initial_rotation);
        calibrator::m_variables_rotation->SetVariables(initial_rotation);

        // Start optimization thread to generate fit.
        calibrator::m_thread = boost::thread(&calibrator::thread_worker, this);
        return true;
    }
    else
    {
        return false;
    }
}


void calibrator::get_fit(ellipsoid& ellipse)
{
    // Return ellipsoid parameters.
    ellipse.set_center(calibrator::m_variables_center->GetValues());
    ellipse.set_radius(calibrator::m_variables_radius->GetValues());
    ellipse.set_rotation(calibrator::m_variables_rotation->GetValues());
}
void calibrator::get_truth(ellipsoid &ellipse)
{
    // Build truth ellipse.
    Eigen::Vector3d center, rotation, radius;

    center.setZero();
    rotation.setZero();
    radius.fill(calibrator::m_true_field_strength);

    ellipse.set_center(center);
    ellipse.set_rotation(rotation);
    ellipse.set_radius(radius);
}
void calibrator::get_calibration(Eigen::Matrix3d& m, Eigen::Vector3d& t)
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
    calibrator::calibration_completed(calibrator::m_solver.GetReturnStatus() == 0);

    // Flag thread as finished.
    calibrator::m_running = false;
}
