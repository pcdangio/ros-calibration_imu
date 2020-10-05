#include "variables_center.h"

using namespace ifopt;

// CONSTRUCTOR
variables_center::variables_center()
    : VariableSet(3, "center")
{
    variables_center::x = 0.0;
    variables_center::y = 0.0;
    variables_center::z = 0.0;
    variables_center::max = 100;
}

// OVERRIDES
void variables_center::SetVariables(const Eigen::VectorXd& x)
{
    variables_center::x = x(0);
    variables_center::y = x(1);
    variables_center::z = x(2);
}
Eigen::VectorXd variables_center::GetValues() const
{
    return Eigen::Vector3d(variables_center::x,
                           variables_center::y,
                           variables_center::z);
}
ifopt::Component::VecBound variables_center::GetBounds() const
{
    ifopt::Component::VecBound bounds(3);
    bounds.at(0) = ifopt::Bounds(-variables_center::max, variables_center::max);
    bounds.at(1) = ifopt::Bounds(-variables_center::max, variables_center::max);
    bounds.at(2) = ifopt::Bounds(-variables_center::max, variables_center::max);

    return bounds;
}

// METHODS
void variables_center::center_vector(Eigen::Vector3d& c) const
{
    c(0) = variables_center::x;
    c(1) = variables_center::y;
    c(2) = variables_center::z;
}

// PARAMETERS
void variables_center::p_max(double value)
{
    variables_center::max = value;
}