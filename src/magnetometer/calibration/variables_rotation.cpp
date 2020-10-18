#include "magnetometer/calibration/variables_rotation.h"

using namespace ifopt;

// CONSTRUCTORS
variables_rotation::variables_rotation()
    : VariableSet(3, "rotation")
{
    variables_rotation::r = 0.0;
    variables_rotation::p = 0.0;
    variables_rotation::y = 0.0;
    variables_rotation::max = M_PI;
}

// OVERRIDES
void variables_rotation::SetVariables(const Eigen::VectorXd& x)
{
    variables_rotation::r = x(0);
    variables_rotation::p = x(1);
    variables_rotation::y = x(2);
}
Eigen::VectorXd variables_rotation::GetValues() const
{
    return Eigen::Vector3d(variables_rotation::r,
                           variables_rotation::p,
                           variables_rotation::y);
}
ifopt::Component::VecBound variables_rotation::GetBounds() const
{
    ifopt::Component::VecBound bounds(3);
    bounds.at(0) = ifopt::Bounds(-variables_rotation::max, variables_rotation::max);
    bounds.at(1) = ifopt::Bounds(-variables_rotation::max, variables_rotation::max);
    bounds.at(2) = ifopt::Bounds(-variables_rotation::max, variables_rotation::max);

    return bounds;
}

// PARAMETERS
void variables_rotation::set_max(double value)
{
    variables_rotation::max = value;
}
