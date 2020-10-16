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

    // Initialize calibration.
    calibrator::m_calibration_transform.setIdentity();
    calibrator::m_calibration_translation.setZero();

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

        // Reset calibration.
        calibrator::m_calibration_transform.setIdentity();
        calibrator::m_calibration_translation.setZero();

        // Start optimization thread to generate fit.
        calibrator::m_thread = boost::thread(&calibrator::thread_worker, this);
        return true;
    }
    else
    {
        return false;
    }
}

void calibrator::get_fit(ellipsoid& ellipsoid)
{
    // Return ellipsoid parameters.
    ellipsoid.set_center(calibrator::m_variables_center->GetValues());
    ellipsoid.set_radius(calibrator::m_variables_radius->GetValues());
    ellipsoid.set_rotation(calibrator::m_variables_rotation->GetValues());
}
void calibrator::get_truth(ellipsoid &ellipsoid)
{
    // Build truth ellipse.
    Eigen::Vector3d center, rotation, radius;

    center.setZero();
    rotation.setZero();
    radius.fill(calibrator::m_true_field_strength);

    ellipsoid.set_center(center);
    ellipsoid.set_rotation(rotation);
    ellipsoid.set_radius(radius);
}

void calibrator::get_calibration(Eigen::Matrix3d& transform, Eigen::Vector3d& translation)
{
    transform = calibrator::m_calibration_transform;
    translation = calibrator::m_calibration_translation;
}
std::string calibrator::print_calibration()
{
    std::stringstream output;
    output << "transformation:" << std::endl
           << calibrator::m_calibration_transform << std::endl << std::endl
           << "translation:" << std::endl
           << calibrator::m_calibration_translation;

    return output.str();
}
bool calibrator::save_calibration_json(std::string filepath)
{

}
bool calibrator::save_calibration_yaml(std::string filepath)
{

}

void calibrator::thread_worker()
{
    // Flag thread as running.
    calibrator::m_running = true;

    // Run solver.
    calibrator::m_solver.Solve(calibrator::m_problem);

    // Check solve status.
    bool solved = calibrator::m_solver.GetReturnStatus() == 0;

    // If solved, calculate calibration.
    if(solved)
    {
        // Get fit ellipse.
        ellipsoid fit;
        calibrator::get_fit(fit);

        // Calculate scale transform.
        Eigen::Vector3d fit_radius;
        fit.get_radius(fit_radius);
        Eigen::Matrix3d scale;
        scale.setIdentity();
        scale(0,0) = calibrator::m_true_field_strength / fit_radius(0);
        scale(1,1) = calibrator::m_true_field_strength / fit_radius(1);
        scale(2,2) = calibrator::m_true_field_strength / fit_radius(2);

        // Get rotation and transpose matrices.
        Eigen::Matrix3d rotation, rotation_t;
        fit.get_r(rotation);
        fit.get_rt(rotation_t);

        // Calculate and set transform matrix.
        calibrator::m_calibration_transform.noalias() = rotation * scale * rotation_t;

        // Set translation vector.
        Eigen::Vector3d center;
        fit.get_center(center);
        calibrator::m_calibration_translation = -center;
    }

    // Signal completion.
    calibrator::calibration_completed(solved);

    // Flag thread as finished.
    calibrator::m_running = false;
}
