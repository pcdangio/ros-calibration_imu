#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <QObject>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>

#include <boost/thread.hpp>

#include <ros/node_handle.h>

#include "magnetometer/optimizer/variables_center.h"
#include "magnetometer/optimizer/variables_rotation.h"
#include "magnetometer/optimizer/variables_radius.h"
#include "magnetometer/optimizer/cost_objective.h"

class optimizer
    : public QObject
{
    Q_OBJECT
public:
    optimizer(const ros::NodeHandle& node_handle);
    ~optimizer();

    bool initialize_center(double x, double y, double z);
    bool initialize_rotation(double r, double p, double y);
    bool initialize_radius(double a, double b, double c);

    bool start();

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

#endif // OPTIMIZER_H
