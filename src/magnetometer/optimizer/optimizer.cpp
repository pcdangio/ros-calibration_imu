#include "optimizer.h"

optimizer::optimizer(const ros::NodeHandle& node_handle)
{
    // Set up variable sets.
    optimizer::m_variables_center = std::make_shared<ifopt::variables_center>();
    optimizer::m_variables_rotation = std::make_shared<ifopt::variables_rotation>();
    optimizer::m_variables_radius = std::make_shared<ifopt::variables_radius>();

    // Set up cost term.
    optimizer::m_cost_objective = std::make_shared<ifopt::cost_objective>();

    // Set up problem.
    optimizer::m_problem.AddVariableSet(optimizer::m_variables_center);
    optimizer::m_problem.AddVariableSet(optimizer::m_variables_rotation);
    optimizer::m_problem.AddVariableSet(optimizer::m_variables_radius);
    optimizer::m_problem.AddCostSet(optimizer::m_cost_objective);

    // Set parameters.
    optimizer::m_variables_center->p_max(node_handle.param<double>("fit_max_center", 50.0));
    optimizer::m_variables_rotation->p_max(node_handle.param<double>("fit_max_rotation", M_PI));
    optimizer::m_variables_radius->p_max(node_handle.param<double>("fit_max_radius", 50.0));
    optimizer::m_cost_objective->p_gradient_perturbation(node_handle.param<double>("fit_perturbation", 0.000001));
    optimizer::m_solver.SetOption("max_cpu_time", node_handle.param<double>("fit_max_time", 15.0));

    // Initialize thread.
    optimizer::m_running = false;
}
optimizer::~optimizer()
{
    if(optimizer::m_running)
    {
        optimizer::m_thread.join();
    }
}

bool optimizer::initialize_center(double x, double y, double z)
{
    if(optimizer::m_running)
    {
        return false;
    }

    Eigen::Vector3d initial_value = {x, y, z};
    optimizer::m_variables_center->SetVariables(initial_value);

    return true;
}
bool optimizer::initialize_rotation(double r, double p, double y)
{
    if(optimizer::m_running)
    {
        return false;
    }

    Eigen::Vector3d initial_value = {r, p, y};
    optimizer::m_variables_rotation->SetVariables(initial_value);

    return true;
}
bool optimizer::initialize_radius(double a, double b, double c)
{
    if(optimizer::m_running)
    {
        return false;
    }

    Eigen::Vector3d initial_value = {a, b, c};
    optimizer::m_variables_radius->SetVariables(initial_value);

    return true;
}

bool optimizer::start()
{
    // Start thread.
    if(!optimizer::m_running)
    {
        optimizer::m_thread = boost::thread(&optimizer::thread_worker, this);
        return true;
    }
    else
    {
        return false;
    }
}

void optimizer::thread_worker()
{
    // Flag thread as running.
    optimizer::m_running = true;

    // Run solver.
    optimizer::m_solver.Solve(optimizer::m_problem);

    // Signal completion.
    optimizer::optimization_completed(optimizer::m_solver.GetReturnStatus() == 0);

    // Flag thread as finished.
    optimizer::m_running = false;
}