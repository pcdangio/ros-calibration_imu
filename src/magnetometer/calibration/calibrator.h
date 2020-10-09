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

class calibrator
    : public QObject
{
    Q_OBJECT
public:
    calibrator(std::shared_ptr<magnetometer::data_interface>& data_interface);
    ~calibrator();

    bool initialize_center(double x, double y, double z);
    bool initialize_rotation(double r, double p, double y);
    bool initialize_radius(double a, double b, double c);

    bool start();

    Eigen::Matrix<double, 4, 4> get_calibration();

signals:
    void optimization_completed(bool success);

private:
    std::shared_ptr<ifopt::variables_center> m_variables_center;
    std::shared_ptr<ifopt::variables_rotation> m_variables_rotation;
    std::shared_ptr<ifopt::variables_radius> m_variables_radius;
    std::shared_ptr<ifopt::cost_objective> m_cost_objective;

    ifopt::Problem m_problem;
    ifopt::IpoptSolver m_solver;

    boost::thread m_thread;
    void thread_worker();
    std::atomic<bool> m_running;
};

#endif
