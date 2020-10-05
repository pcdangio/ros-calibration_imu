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

    variables_rotation::rx = new Eigen::Matrix<double, 3, 3>;
    variables_rotation::ry = new Eigen::Matrix<double, 3, 3>;
    variables_rotation::rz = new Eigen::Matrix<double, 3, 3>;
    variables_rotation::t = new Eigen::Matrix<double, 3, 3>;
}
variables_rotation::~variables_rotation()
{
    delete variables_rotation::rx;
    delete variables_rotation::ry;
    delete variables_rotation::rz;
    delete variables_rotation::t;
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

// METHODS
void variables_rotation::rotation_matrix(Eigen::Matrix3d& r) const
{
    // Get references to the pre-allocated matrices.
    auto& rx = *(variables_rotation::rx);
    auto& ry = *(variables_rotation::ry);
    auto& rz = *(variables_rotation::rz);
    auto& t = *(variables_rotation::t);

    // Create individual rotation matrices.
    rx.setIdentity();
    rx(1,1) = std::cos(variables_rotation::r);
    rx(1,2) = -std::sin(variables_rotation::r);
    rx(2,1) = std::sin(variables_rotation::r);
    rx(2,2) = std::cos(variables_rotation::r);
    ry.setIdentity();
    ry(0,0) = std::cos(variables_rotation::p);
    ry(0,2) = std::sin(variables_rotation::p);
    ry(2,0) = -std::sin(variables_rotation::p);
    ry(2,2) = std::cos(variables_rotation::p);
    rz.setIdentity();
    rz(0,0) = std::cos(variables_rotation::y);
    rz(0,1) = -std::sin(variables_rotation::y);
    rz(1,0) = std::sin(variables_rotation::y);
    rz(1,1) = std::cos(variables_rotation::y);

    // Combine by rz*ry*rx
    t.noalias() = rz * ry;
    r.noalias() = t * rx;
}

// PARAMETERS
void variables_rotation::p_max(double value)
{
    variables_rotation::max = value;
}