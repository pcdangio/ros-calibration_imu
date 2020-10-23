/// \file magnetometer/calibration/calibrator.h
/// \brief Defines the magnetometer::calibrator class.
#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include <QObject>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>

#include <boost/thread.hpp>

#include "magnetometer/data/data_interface.h"

#include "common/calibration/variables_center.h"
#include "common/calibration/variables_radius.h"
#include "magnetometer/calibration/variables_rotation.h"

#include "magnetometer/calibration/cost_objective.h"

namespace magnetometer
{

/// \brief Performs calibration on a set of collected magnetometer data points.
class calibrator
    : public QObject
{
    Q_OBJECT
public:
    // CONSTRUCTORS
    /// \brief Creates a new calibrator instance.
    /// \param data_interface A shared pointer to the magnetometer's data interface.
    calibrator(std::shared_ptr<magnetometer::data_interface>& data_interface);
    ~calibrator();

    // METHODS
    /// \brief Starts a calibration routine on a separate thread.
    /// \param true_field_strength The true field strength to calibrate the points to.
    /// \returns TRUE if the calibration routine started, otherwise FALSE.
    bool start(double true_field_strength);

    // RESULTS
    /// \brief Gets the ellipsoid that was fit to the uncalibrated magnetometer data points.
    /// \param center The ellipsoid center vector to capture the fit into.
    /// \param radius The ellipsoid radius vector to capture the fit into.
    /// \param rotation The ellipsoid rotation vector to capture the fit into.
    void get_fit(Eigen::Vector3d& center, Eigen::Vector3d& radius, Eigen::Vector3d& rotation);
    /// \brief Gets the calculated calibration.
    /// \param transform The calibration's 3x3 transformation matrix.
    /// \param translation The calibration's 3x1 translation vector.
    void get_calibration(Eigen::Matrix3d& transform, Eigen::Vector3d& translation);

    // CALIBRATION SAVING
    /// \brief Prints the calibration as a string.
    /// \returns The printed calibration string.
    std::string print_calibration();
    /// \brief Saves the calibration to a JSON file.
    /// \param filepath The file path to save the file to.
    /// \returns TRUE if the save was successful, otherwise FALSE.
    bool save_calibration_json(std::string filepath);
    /// \brief Saves the calibration to a YAML file.
    /// \param filepath The file path to save the file to.
    /// \returns TRUE if the save was successful, otherwise FALSE.
    bool save_calibration_yaml(std::string filepath);

signals:
    /// \brief Indicates that the calibration routine has completed.
    /// \param success TRUE if the calibration succeeded, otherwise FALSE.
    void calibration_completed(bool success);

private:
    // PARAMETERS
    /// \brief The scale factor for magnetic field strength.
    /// \details This is a multiplier for bringing the field strength (provided in Tesla) within the order of ~10^1
    /// for improved optimization performance.
    const double m_field_scale = 1000000.0;
    /// \brief Stores the true field strength to calibrate the magnetometer data to (in Tesla).
    double m_true_field_strength;

    // COMPONENTS
    /// \brief A shared pointer to the application's magnetometer data interface.
    std::shared_ptr<magnetometer::data_interface> m_data_interface;

    // THREADING
    /// \brief The thread instance for running the calibration routine on.
    boost::thread m_thread;
    /// \brief The calibration thread's worker function.
    void thread_worker();
    /// \brief Thread-safe indicator for if the thread is currently running.
    std::atomic<bool> m_running;

    // FIT RESULTS
    /// \brief The calculated ellipsoid fit center vector (T).
    Eigen::Vector3d m_fit_center;
    /// \brief The calculated ellipsoid fit radius vector (T).
    Eigen::Vector3d m_fit_radius;
    /// \brief The calculated ellipsoid fit rotation vector (rad)
    Eigen::Vector3d m_fit_rotation;

    // CALIBRATION RESULTS
    /// \brief The calculated calibration 3x3 transformation matrix.
    Eigen::Matrix3d m_calibration_transform;
    /// \brief The calculated calibration 3x1 translation vector (T).
    Eigen::Vector3d m_calibration_translation;
};

}

#endif
