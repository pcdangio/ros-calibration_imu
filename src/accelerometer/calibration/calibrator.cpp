#include "accelerometer/calibration/calibrator.h"

#include <ros/node_handle.h>

#include "common/calibration/variables_center.h"
#include "common/calibration/variables_radius.h"
#include "accelerometer/calibration/cost_term.h"

#include <fstream>
#include <boost/property_tree/json_parser.hpp>

using namespace accelerometer;

// CONSTRUCTORS
calibrator::calibrator()
{
    // Initialize calibration.
    calibrator::m_calibration.setIdentity();

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

// METHODS
bool calibrator::start(const Eigen::Matrix<double, 3, 6>& data_set, double true_gravity_vector)
{
    // Check if thread is running.
    if(!calibrator::m_running)
    {
        // Reset calibration.
        calibrator::m_calibration.setIdentity();

        // Start optimization thread to generate fit.
        calibrator::m_thread = boost::thread(boost::bind(&calibrator::thread_worker, this, data_set, true_gravity_vector));
        return true;
    }
    else
    {
        return false;
    }
}

// CALIBRATION SAVING
std::string calibrator::print_calibration()
{
    // Write to string.
    std::stringstream output;
    output << "calibration units: m/s^2" << std::endl << std::endl;
    output << std::setprecision(6) << std::fixed;
    output << calibrator::m_calibration;

    return output.str();
}
bool calibrator::save_calibration_json(std::string filepath)
{
    // Use boost property tree to build up JSON file.
    boost::property_tree::ptree json_file;

    // Write as row-stepped array.
    boost::property_tree::ptree json_calibration;
    for(uint8_t i = 0; i < calibrator::m_calibration.rows(); ++i)
    {
        for(uint8_t j = 0; j < calibrator::m_calibration.cols(); ++j)
        {
            std::stringstream ss;
            ss << std::setprecision(10) << std::fixed << calibrator::m_calibration(i,j);
            json_calibration.push_back(boost::property_tree::ptree::value_type("", ss.str()));
        }
    }

    // Add component to JSON root.
    json_file.add_child("calibration", json_calibration);

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
    yaml_file << "calibration: [";
    yaml_file << std::setprecision(6) << std::fixed;
    for(uint8_t i = 0; i < calibrator::m_calibration.rows(); ++i)
    {
        for(uint8_t j = 0; j < calibrator::m_calibration.cols(); ++j)
        {
            yaml_file << calibrator::m_calibration(i,j);
            if((i+1)*(j+1) < 16)
            {
                yaml_file << ", ";
            }
        }
    }
    yaml_file << "]";

    yaml_file.close();

    return true;
}

// THREADING
void calibrator::thread_worker(Eigen::Matrix<double, 3, 6> data_set, double true_gravity_vector)
{
    // Flag thread as running.
    calibrator::m_running = true;

    // Calculate the bounding box and it's parameters.
    Eigen::Vector3d bb_min, bb_max;
    bb_min.fill(std::numeric_limits<double>::infinity());
    bb_max.fill(-std::numeric_limits<double>::infinity());
    for(uint32_t j = 0; j < data_set.cols(); ++j)
    {
        for(uint32_t i = 0; i < 3; ++i)
        {
            if(data_set(i,j) < bb_min(i))
            {
                bb_min(i) = data_set(i,j);
            }
            if(data_set(i,j) > bb_max(i))
            {
                bb_max(i) = data_set(i,j);
            }
        }
    }
    // Calculate bounding box center.
    Eigen::Vector3d bb_width = bb_max - bb_min;
    Eigen::Vector3d bb_radius = bb_width / 2.0;
    Eigen::Vector3d bb_center = bb_max + bb_min;
    bb_center /= 2.0;

    // Set up the optimization components.
    std::shared_ptr<ifopt::variables_center> variables_center = std::make_shared<ifopt::variables_center>();
    std::shared_ptr<ifopt::variables_radius> variables_radius = std::make_shared<ifopt::variables_radius>();
    std::shared_ptr<ifopt::cost_term> cost_term = std::make_shared<ifopt::cost_term>(data_set);

    // Set initial guess and range for each variable.
    // Center
    variables_center->SetVariables(bb_center);
    variables_center->set_range(bb_center - bb_width*0.25, bb_center + bb_width*0.25);
    // Radius
    variables_radius->SetVariables(bb_radius);
    variables_radius->set_range(bb_radius * 0.5, bb_radius * 1.5);

    // Set component parameters.
    cost_term->p_gradient_perturbation(0.000001);

    // Set up the optimization problem.
    ifopt::Problem problem;
    problem.AddVariableSet(variables_center);
    problem.AddVariableSet(variables_radius);
    problem.AddCostSet(cost_term);

    // Run solver.
    ifopt::IpoptSolver solver;
    solver.SetOption("max_cpu_time", 30.0);
    solver.Solve(problem);

    // Check solve status.
    bool solved = solver.GetReturnStatus() == 0;

    // If solved, calculate calibration.
    if(solved)
    {
        // Extract fit parameters.
        auto fit_center = variables_center->GetValues();
        auto fit_radius = variables_radius->GetValues();

        // Calculate offset transform.
        Eigen::Matrix4d h_offset;
        h_offset.setIdentity();
        h_offset.block(0, 3, 3, 1) = -fit_center;

        // Calculate scale transform.
        Eigen::Matrix4d h_scale;
        h_scale.setIdentity();
        h_scale(0,0) = true_gravity_vector / fit_radius(0);
        h_scale(1,1) = true_gravity_vector / fit_radius(1);
        h_scale(2,2) = true_gravity_vector / fit_radius(2);

        // Calculate homogeneous transform.
        calibrator::m_calibration.noalias() = h_scale * h_offset;

        // Calculate fit points.
        Eigen::Matrix3d fit_points;
        fit_points.row(0) = fit_center - fit_radius;
        fit_points.row(1) = fit_center;
        fit_points.row(2) = fit_center + fit_radius;
        calibrator::new_fit(fit_points);

        // Calcuate calibration points.
        Eigen::Matrix3d calibration_points;
        Eigen::Vector4d p_u, p_c;
        p_u(3) = 1;
        for(uint32_t i = 0; i < 3; ++i)
        {
            p_u.block(0, 0, 3, 1) = fit_points.row(i).transpose();
            p_c.noalias() = calibrator::m_calibration * p_u;
            calibration_points.block(i, 0, 1, 3).noalias() = p_c.block(0, 0, 3, 1).transpose();
        }
        calibrator::new_calibration(calibration_points);

        // Signal new true gravity vector.
        calibrator::new_truth(true_gravity_vector);
    }

    // Signal completion.
    calibrator::calibration_completed(solved);

    // Flag thread as finished.
    calibrator::m_running = false;
}
