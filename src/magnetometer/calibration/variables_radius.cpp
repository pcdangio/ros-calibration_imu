#include "magnetometer/calibration/variables_radius.h"

using namespace ifopt;

// CONSTRUCTOR
variables_radius::variables_radius()
    : VariableSet(3, "radius")
{
    variables_radius::a = 30.0;
    variables_radius::b = 30.0;
    variables_radius::c = 30.0;
    variables_radius::max = 50.0;
}

// OVERRIDES
void variables_radius::SetVariables(const Eigen::VectorXd& x)
{
    variables_radius::a = x(0);
    variables_radius::b = x(1);
    variables_radius::c = x(2);
}
Eigen::VectorXd variables_radius::GetValues() const
{
    return Eigen::Vector3d(variables_radius::a,
                           variables_radius::b,
                           variables_radius::c);
}
ifopt::Component::VecBound variables_radius::GetBounds() const
{
    ifopt::Component::VecBound bounds(3);
    bounds.at(0) = ifopt::Bounds(0, variables_radius::max);
    bounds.at(1) = ifopt::Bounds(0, variables_radius::max);
    bounds.at(2) = ifopt::Bounds(0, variables_radius::max);

    return bounds;
}

// METHODS
void variables_radius::radius_matrix(Eigen::Matrix3d& e)
{
    e.setIdentity();
    e(0,0) = 1.0 / std::pow(variables_radius::a, 2.0);
    e(1,1) = 1.0 / std::pow(variables_radius::b, 2.0);
    e(2,2) = 1.0 / std::pow(variables_radius::c, 2.0);
}

// PARAMETERS
void variables_radius::p_max(double value)
{
    variables_radius::max = value;
}