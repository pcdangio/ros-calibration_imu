#include "magnetometer/calibration/calibrator.h"

#include <ros/node_handle.h>

#include <fstream>
#include <boost/property_tree/json_parser.hpp>

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
    calibrator::m_variables_center->p_max(private_handle.param<double>("calibration_max_center", 100.0));
    calibrator::m_variables_rotation->p_max(private_handle.param<double>("calibration_max_rotation", M_PI));
    calibrator::m_variables_radius->p_max(private_handle.param<double>("calibration_max_radius", 100.0));
    calibrator::m_cost_objective->p_gradient_perturbation(private_handle.param<double>("calibration_gradient_perturbation", 0.000001));
    calibrator::m_solver.SetOption("max_cpu_time", private_handle.param<double>("calibration_max_time", 30.0));

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
    // Scale the translation component.
    Eigen::Vector3d scaled_translation = calibrator::m_calibration_translation / calibrator::m_scale_factor;

    // Write to string.
    std::stringstream output;
    output << "calibration units: tesla (T)" << std::endl << std::endl;
    output << std::setprecision(6) << std::fixed;
    output << "transformation:" << std::endl
           << calibrator::m_calibration_transform << std::endl << std::endl;
    output << std::setprecision(10)
           << "translation:" << std::endl
           << scaled_translation;

    return output.str();
}
bool calibrator::save_calibration_json(std::string filepath)
{
    // Use boost property tree to build up JSON file.
    boost::property_tree::ptree json_file;

    // Populate transformation component.
    // Write as row-stepped array.
    boost::property_tree::ptree json_transformation;
    for(uint8_t i = 0; i < 3; ++i)
    {
        for(uint8_t j = 0; j < 3; ++j)
        {
            std::stringstream ss;
            ss << std::setprecision(6) << std::fixed << calibrator::m_calibration_transform(i,j);
            json_transformation.push_back(boost::property_tree::ptree::value_type("", ss.str()));
        }
    }

    // Populate translation component.
    // Apply scaling.
    boost::property_tree::ptree json_translation;
    for(uint8_t i = 0; i < 3; ++i)
    {
        std::stringstream ss;
        ss << std::setprecision(10) << std::fixed << (calibrator::m_calibration_translation(i) / calibrator::m_scale_factor);
        json_translation.push_back(boost::property_tree::ptree::value_type("", ss.str()));
    }

    // Add components to JSON root.
    json_file.add_child("transformation_matrix", json_transformation);
    json_file.add_child("translation_vector", json_translation);

    // Write JSON file.
    try
    {
        boost::property_tree::write_json(filepath, json_file);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("error writing JSON calibration file (" << e.what() << ")");
        return false;
    }

    return true;
}
bool calibrator::save_calibration_yaml(std::string filepath)
{
    // Create stream for writing to file.
    std::ofstream yaml_file(filepath.c_str());

    if(!yaml_file)
    {
        ROS_ERROR_STREAM("error writing YAML calibration file (" << std::strerror(errno) << ")");
        return false;
    }

    // Write transformation component.
    yaml_file << "transformation: [";
    yaml_file << std::setprecision(6) << std::fixed;
    for(uint8_t i = 0; i < 3; ++i)
    {
        for(uint8_t j = 0; j < 3; ++j)
        {
            yaml_file << calibrator::m_calibration_transform(i,j);
            if((i+1)*(j+1) < 9)
            {
                yaml_file << ", ";
            }
        }
    }
    yaml_file << "]" << std::endl;

    yaml_file << std::setprecision(10);
    yaml_file << "translation: [";
    for(uint8_t i = 0; i < 3; ++i)
    {
        yaml_file << (calibrator::m_calibration_translation(i) / calibrator::m_scale_factor);
        if(i < 2)
        {
            yaml_file << ", ";
        }
    }
    yaml_file << "]";

    yaml_file.close();

    return true;
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
