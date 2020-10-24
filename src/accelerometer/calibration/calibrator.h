/// \file accelerometer/calibration/calibrator.h
/// \brief Defines the accelerometer::calibrator class.
#ifndef ACCELEROMETER_CALIBRATOR_H
#define ACCELEROMETER_CALIBRATOR_H

#include <QObject>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>

#include <eigen3/Eigen/Dense>

#include <boost/thread.hpp>

namespace accelerometer
{

/// \brief Performs calibration on a set of collected accelerometer data points.
class calibrator
    : public QObject
{
    Q_OBJECT
public:
    // CONSTRUCTORS
    /// \brief Creates a new calibrator instance.
    calibrator();
    ~calibrator();

    // METHODS
    /// \brief Starts a calibration routine on a separate thread.
    /// \param data_set The data to calibrate.
    /// \param true_gravity_vector The true gravity vector to calibrate the points to.
    /// \returns TRUE if the calibration routine started, otherwise FALSE.
    bool start(const Eigen::Matrix<double, 3, 6>& data_set, double true_gravity_vector);

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
    /// \brief Indicates that a new fit has been calculated.
    /// \param fit The newly calculated fit.
    void new_fit(Eigen::Matrix3d fit);
    /// \brief Indicates that a new calibration has been calculated.
    /// \param calibration The newly calculated calibration.
    void new_calibration(Eigen::Matrix3d calibration);
    /// \brief Indicates that the calibration routine has completed.
    /// \param success TRUE if the calibration succeeded, otherwise FALSE.
    void calibration_completed(bool success);

private:
    // THREADING
    /// \brief The thread instance for running the calibration routine on.
    boost::thread m_thread;
    /// \brief The calibration thread's worker function.
    void thread_worker(Eigen::Matrix<double, 3, 6> data_set, double true_gravity_vector);
    /// \brief Thread-safe indicator for if the thread is currently running.
    std::atomic<bool> m_running;

    // CALIBRATION RESULTS
    /// \brief The calculated calibration 3x3 transformation matrix.
    Eigen::Matrix3d m_calibration_transform;
    /// \brief The calculated calibration 3x1 translation vector (T).
    Eigen::Vector3d m_calibration_translation;
};

}

#endif

