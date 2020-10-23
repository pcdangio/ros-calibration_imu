#include "common/geometry/ellipsoid.h"

using namespace common;

// CONSTRUCTOR
ellipsoid::ellipsoid()
{
    // Set default values for ellipsoid.
    ellipsoid::set_center(Eigen::Vector3d(0, 0, 0));
    ellipsoid::set_radius(Eigen::Vector3d(1, 1, 1));
    ellipsoid::set_rotation(Eigen::Vector3d(0, 0, 0));
}

// SET PROPERTIES
void ellipsoid::set_center(const Eigen::Vector3d& center)
{
    // Copy center.
    ellipsoid::m_center = center;
}
void ellipsoid::set_radius(const Eigen::Vector3d& radius)
{
    // Copy radius.
    ellipsoid::m_radius = radius;

    // Create radius matrix.
    ellipsoid::m_a.setZero();
    ellipsoid::m_a(0,0) = 1.0 / std::pow(radius(0), 2.0);
    ellipsoid::m_a(1,1) = 1.0 / std::pow(radius(1), 2.0);
    ellipsoid::m_a(2,2) = 1.0 / std::pow(radius(2), 2.0);
}
void ellipsoid::set_rotation(const Eigen::Vector3d& rotation)
{
    // Copy rotation.
    ellipsoid::m_rotation = rotation;

    // Calculate rotation matrix.

    // Calculate individual matrices.
    ellipsoid::m_rx.setIdentity();
    ellipsoid::m_rx(1,1) = std::cos(ellipsoid::m_rotation(0));
    ellipsoid::m_rx(1,2) = -std::sin(ellipsoid::m_rotation(0));
    ellipsoid::m_rx(2,1) = std::sin(ellipsoid::m_rotation(0));
    ellipsoid::m_rx(2,2) = std::cos(ellipsoid::m_rotation(0));
    ellipsoid::m_ry.setIdentity();
    ellipsoid::m_ry(0,0) = std::cos(ellipsoid::m_rotation(1));
    ellipsoid::m_ry(0,2) = std::sin(ellipsoid::m_rotation(1));
    ellipsoid::m_ry(2,0) = -std::sin(ellipsoid::m_rotation(1));
    ellipsoid::m_ry(2,2) = std::cos(ellipsoid::m_rotation(1));
    ellipsoid::m_rz.setIdentity();
    ellipsoid::m_rz(0,0) = std::cos(ellipsoid::m_rotation(2));
    ellipsoid::m_rz(0,1) = -std::sin(ellipsoid::m_rotation(2));
    ellipsoid::m_rz(1,0) = std::sin(ellipsoid::m_rotation(2));
    ellipsoid::m_rz(1,1) = std::cos(ellipsoid::m_rotation(2));

    // Combine by rz*ry*rx
    ellipsoid::m_t.noalias() = ellipsoid::m_rz * ellipsoid::m_ry;
    ellipsoid::m_r.noalias() = ellipsoid::m_t * ellipsoid::m_rx;

    // Set rotation matrix transpose.
    ellipsoid::m_rt.noalias() = ellipsoid::m_r.transpose();
}

// GET PROPERTIES
void ellipsoid::get_center(Eigen::Vector3d& center) const
{
    center = ellipsoid::m_center;
}
void ellipsoid::get_radius(Eigen::Vector3d& radius) const
{
    radius = ellipsoid::m_radius;
}
void ellipsoid::get_rotation(Eigen::Vector3d& rotation) const
{
    rotation = ellipsoid::m_rotation;
}
void ellipsoid::get_r(Eigen::Matrix3d& matrix) const
{
    matrix = ellipsoid::m_r;
}
void ellipsoid::get_rt(Eigen::Matrix3d& matrix) const
{
    matrix = ellipsoid::m_rt;
}

// GEOMETRY
double ellipsoid::residual(const Eigen::Vector3d& point)
{
    // Create p vector.
    ellipsoid::m_p = ellipsoid::m_center - point;
    ellipsoid::m_pt.noalias() = ellipsoid::m_p.transpose();

    // Evaluate point against ellipsoid equation.
    // p'*r*A*r'*p = 1
    ellipsoid::m_t1.noalias() = ellipsoid::m_pt * ellipsoid::m_r;
    ellipsoid::m_t2.noalias() = ellipsoid::m_t1 * ellipsoid::m_a;
    ellipsoid::m_t1.noalias() = ellipsoid::m_t2 * ellipsoid::m_rt;
    ellipsoid::m_v.noalias() = ellipsoid::m_t1 * ellipsoid::m_p;

    // Return residual.
    return ellipsoid::m_v(0,0) - 1.0;
}
void ellipsoid::draw(QtDataVisualization::QScatterDataArray *points) const
{
    // Set number of angles to a factor of 4.
    uint32_t n_angles = 20;

    // Set up points vector for receiving drawn ellipse.
    points->clear();
    points->reserve(n_angles * n_angles);

    // Use parametric equations for ellipsoid.

    // Iterate over angles.
    double range = 2.0 * M_PI;
    double step = range / static_cast<double>(n_angles);
    Eigen::Vector3d p, pr, pt;
    for(double u = 0; u < range; u += step)
    {
        for(double v = 0; v < range; v+= step)
        {
            // Calculate original point.
            p(0) = ellipsoid::m_radius(0) * std::cos(u) * std::sin(v);
            p(1) = ellipsoid::m_radius(1) * std::sin(u) * std::sin(v);
            p(2) = ellipsoid::m_radius(2) * std::cos(v);

            // Rotate point.
            pr.noalias() = ellipsoid::m_r * p;

            // Translate point.
            pt = pr + ellipsoid::m_center;

            points->append(QVector3D(pt(0), pt(2), pt(1)));
        }
    }
}
