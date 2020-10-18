#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include <QObject>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>

#include <boost/thread.hpp>

#include "magnetometer/data/data_interface.h"

#include "magnetometer/calibration/variables_center.h"
#include "magnetometer/calibration/variables_rotation.h"
#include "magnetometer/calibration/variables_radius.h"
#include "magnetometer/calibration/cost_objective.h"

#include "magnetometer/geometry/ellipsoid.h"

namespace magnetometer
{

class calibrator
    : public QObject
{
    Q_OBJECT
public:
    calibrator(std::shared_ptr<magnetometer::data_interface>& data_interface);
    ~calibrator();

    bool start(double true_field_strength);

    void get_fit(Eigen::Vector3d& center, Eigen::Vector3d& radius, Eigen::Vector3d& rotation);
    void get_calibration(Eigen::Matrix3d& transform, Eigen::Vector3d& translation);

    std::string print_calibration();
    bool save_calibration_json(std::string filepath);
    bool save_calibration_yaml(std::string filepath);

signals:
    void calibration_completed(bool success);

private:
    const double m_field_scale = 1000000.0;

    std::shared_ptr<magnetometer::data_interface> m_data_interface;
    double m_true_field_strength;

    boost::thread m_thread;
    void thread_worker();
    std::atomic<bool> m_running;

    Eigen::Vector3d m_fit_center;
    Eigen::Vector3d m_fit_radius;
    Eigen::Vector3d m_fit_rotation;

    Eigen::Matrix3d m_calibration_transform;
    Eigen::Vector3d m_calibration_translation;
};

}

#endif
