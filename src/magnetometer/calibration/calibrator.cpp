#include "magnetometer/calibration/calibrator.h"

#include <ros/node_handle.h>

#include <fstream>
#include <boost/property_tree/json_parser.hpp>

using namespace magnetometer;

calibrator::calibrator(std::shared_ptr<magnetometer::data_interface>& data_interface)
{
    // Initialize true field strength.
    calibrator::m_true_field_strength = 0;

    // Store data interface.
    calibrator::m_data_interface = data_interface;

    // Initialize calibration.
    calibrator::m_calibration_transform.setIdentity();
    calibrator::m_calibration_translation.setZero();

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

bool calibrator::start(double true_field_strength)
{
    // Check if thread is running.
    if(!calibrator::m_running)
    {
        // Store true field strength.
        calibrator::m_true_field_strength = true_field_strength;

        // Reset fit.
        calibrator::m_fit_center.setZero();
        calibrator::m_fit_radius.setZero();
        calibrator::m_fit_rotation.setZero();

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

void calibrator::get_fit(Eigen::Vector3d& center, Eigen::Vector3d& radius, Eigen::Vector3d& rotation)
{
    // Return ellipsoid parameters.
    center = calibrator::m_fit_center;
    radius = calibrator::m_fit_radius;
    rotation = calibrator::m_fit_rotation;
}
void calibrator::get_calibration(Eigen::Matrix3d& transform, Eigen::Vector3d& translation)
{
    transform = calibrator::m_calibration_transform;
    translation = calibrator::m_calibration_translation;
}

std::string calibrator::print_calibration()
{
    // Scale the translation component.
    Eigen::Vector3d scaled_translation = calibrator::m_calibration_translation;

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
        ss << std::setprecision(10) << std::fixed << (calibrator::m_calibration_translation(i));
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
        yaml_file << (calibrator::m_calibration_translation(i));
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

    // Create a local copy of points to fit from the data interface.
    std::shared_ptr<std::vector<Eigen::Vector3d>> data_points = std::make_shared<std::vector<Eigen::Vector3d>>();

    // Copy the points locally and scale them.
    data_points->clear();
    Eigen::Vector3d new_point;
    for(uint32_t i = 0; i < calibrator::m_data_interface->n_points(); ++i)
    {
        // Grab point from data interface.
        calibrator::m_data_interface->get_point(i, new_point);

        // Scale point.
        new_point *= calibrator::m_field_scale;

        // Add point.
        data_points->push_back(new_point);
    }

    // Calculate the bounding box and it's parameters.
    Eigen::Vector3d bb_min, bb_max;
    bb_min.fill(std::numeric_limits<double>::infinity());
    bb_max.fill(-std::numeric_limits<double>::infinity());
    for(auto point = data_points->cbegin(); point != data_points->cend(); ++point)
    {
        for(uint8_t i = 0; i < 3; ++i)
        {
            if(new_point(i) < bb_min(i))
            {
                bb_min(i) = new_point(i);
            }
            else if(new_point(i) > bb_max(i))
            {
                bb_max(i) = new_point(i);
            }
        }
    }
    // Calculate bounding box center.
    Eigen::Vector3d bb_width = bb_max - bb_min;
    Eigen::Vector3d bb_center = bb_max + bb_min;
    bb_center /= 2.0;

    // Set up the optimization components.
    std::shared_ptr<ifopt::variables_center> variables_center = std::make_shared<ifopt::variables_center>();
    std::shared_ptr<ifopt::variables_rotation> variables_rotation = std::make_shared<ifopt::variables_rotation>();
    std::shared_ptr<ifopt::variables_radius> variables_radius = std::make_shared<ifopt::variables_radius>();
    std::shared_ptr<ifopt::cost_objective> cost_objective = std::make_shared<ifopt::cost_objective>(data_points);

    // Set initial guess and range for each variable.
    // Center
    variables_center->SetVariables(bb_center);
    variables_center->set_range(bb_center - bb_width*0.25, bb_center + bb_width*0.25);
    // Radius
    variables_radius->SetVariables(bb_width / 2.0);
    variables_radius->set_range(bb_width * 0.5, bb_width * 1.5);
    // Rotation
    variables_rotation->set_max(M_PI);

    // Set component parameters.
    cost_objective->p_gradient_perturbation(0.000001);

    // Set up the optimization problem.
    ifopt::Problem problem;
    problem.AddVariableSet(variables_center);
    problem.AddVariableSet(variables_rotation);
    problem.AddVariableSet(variables_radius);
    problem.AddCostSet(cost_objective);

    // Run solver.
    ifopt::IpoptSolver solver;
    solver.SetOption("max_cpu_time", 30.0);
    solver.Solve(problem);

    // Check solve status.
    bool solved = solver.GetReturnStatus() == 0;

    // If solved, calculate calibration.
    if(solved)
    {
        // Extract and scale fit parameters.
        calibrator::m_fit_center = variables_center->GetValues() / calibrator::m_field_scale;
        calibrator::m_fit_radius = variables_radius->GetValues() / calibrator::m_field_scale;
        calibrator::m_fit_rotation = variables_rotation->GetValues();

        // Build up a fit ellipse.
        ellipsoid fit;
        fit.set_center(calibrator::m_fit_center);
        fit.set_radius(calibrator::m_fit_radius);
        fit.set_rotation(calibrator::m_fit_rotation);

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
