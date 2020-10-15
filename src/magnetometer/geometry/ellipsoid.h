#ifndef ELLIPSOID_H
#define ELLIPSOID_H

#include <eigen3/Eigen/Dense>

#include <QtDataVisualization/QScatterDataArray>

namespace magnetometer
{

class ellipsoid
{
public:
    ellipsoid();

    void set_center(const Eigen::Vector3d& center);
    void set_radius(const Eigen::Vector3d& radius);
    void set_rotation(const Eigen::Vector3d& rotation);

    void get_center(Eigen::Vector3d& center) const;
    void get_radius(Eigen::Vector3d& radius) const;
    void get_rotation(Eigen::Vector3d& rotation) const;
    void get_r(Eigen::Matrix3d& matrix) const;
    void get_rt(Eigen::Matrix3d& matrix) const;

    double residual(const Eigen::Vector3d& point);

    void draw(QtDataVisualization::QScatterDataArray* points) const;

private:
    Eigen::Vector3d m_center;
    Eigen::Vector3d m_radius;
    Eigen::Vector3d m_rotation;

    Eigen::Matrix3d m_a;
    Eigen::Matrix3d m_r, m_rt, m_rx, m_ry, m_rz, m_t;

    Eigen::Vector3d m_p;
    Eigen::Matrix<double, 1, 3> m_pt, m_t1, m_t2;
    Eigen::Matrix<double, 1, 1> m_v;
};

}

#endif
