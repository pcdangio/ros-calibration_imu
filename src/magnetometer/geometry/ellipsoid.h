/// \file magnetometer/geometry/ellipsoid.h
/// \brief Defines the magnetometer::ellipsoid class.
#ifndef ELLIPSOID_H
#define ELLIPSOID_H

#include <eigen3/Eigen/Dense>

#include <QtDataVisualization/QScatterDataArray>

namespace magnetometer
{

/// \brief Represents a 3-dimensional ellipsoid.
class ellipsoid
{
public:
    // CONSTRUCTORS
    /// \brief Creates a new ellipsoid instance.
    ellipsoid();

    // SET PROPERTIES
    /// \brief Sets the center of the ellipsoid.
    /// \param center The new center of the ellipsoid.
    void set_center(const Eigen::Vector3d& center);
    /// \brief Sets the radii of the ellipsoid.
    /// \param radius The new radii of the ellipsoid.
    void set_radius(const Eigen::Vector3d& radius);
    /// \brief Sets the cartesian rotation of the ellipsoid.
    /// \param rotation The new rotation of the ellipsoid.
    void set_rotation(const Eigen::Vector3d& rotation);

    // GET PROPERTIES
    /// \brief Gets the center of the ellipsoid.
    /// \param center The vector to retrieve the center into.
    void get_center(Eigen::Vector3d& center) const;
    /// \brief Gets the radii of the ellipsoid.
    /// \param radius The vector to retrieve the radii into.
    void get_radius(Eigen::Vector3d& radius) const;
    /// \brief Gets the cartesian rotation of the ellipsoid.
    /// \param rotation The vector to retrieve the rotation into.
    void get_rotation(Eigen::Vector3d& rotation) const;
    /// \brief Gets the cartesian rotation of the ellipsoid as a rotation matrix.
    /// \param matrix The matrix to retrive the rotation into.
    void get_r(Eigen::Matrix3d& matrix) const;
    /// \brief Gets the transposed rotation matrix of the ellipsoid.
    /// \param matrix The matrix to retrive the transposed rotation matrix into.
    void get_rt(Eigen::Matrix3d& matrix) const;

    // GEOMETRY
    /// \brief Calculates the residual of a point that is part of the ellipsoid.
    /// \param point The cartesian point to calculate the residual for.
    /// \returns The residual of the point.
    double residual(const Eigen::Vector3d& point);
    /// \brief Draws the ellipsoid into a QScatterDataArray.
    /// \param points The QScatterDataArray to draw the ellipsoid into.
    void draw(QtDataVisualization::QScatterDataArray* points) const;

private:
    // PARAMETERS
    /// \brief Store the cartesian center of the ellipsoid.
    Eigen::Vector3d m_center;
    /// \brief Stores the radii of the ellipsoid.
    Eigen::Vector3d m_radius;
    /// \brief Stores the Euler rotation of the ellipsoid (rad).
    Eigen::Vector3d m_rotation;

    // PREALLOCATIONS
    /// \brief Stores the radius matrix of the ellipsoid.
    Eigen::Matrix3d m_a;
    /// \brief Stores the 3D cartesian rotation matrix of the ellipsoid.
    Eigen::Matrix3d m_r;
    /// \brief Stores the transpose of the ellipsoid's rotation matrix.
    Eigen::Matrix3d m_rt;
    /// \brief A temporary matrix for storing the rotation about the X axis.
    Eigen::Matrix3d m_rx;
    /// \brief A temporary matrix for storing the rotation about the Y axis.
    Eigen::Matrix3d m_ry;
    /// \brief A temporary matrix for storing the rotation about the Z axis.
    Eigen::Matrix3d m_rz;
    /// \brief A temporary 3D matrix for calculations.
    Eigen::Matrix3d m_t;
    /// \brief A temporary vector for storing a point on the ellipsoid.
    Eigen::Vector3d m_p;
    /// \brief A temporary vector for storing a transposed point on the ellipsoid.
    Eigen::Matrix<double, 1, 3> m_pt;
    /// \brief A temporary vector for calculations.
    Eigen::Matrix<double, 1, 3> m_t1;
    /// \brief A temporary vector for calculations.
    Eigen::Matrix<double, 1, 3> m_t2;
    /// \brief A temporary vector for storing an evaluation of the ellipsoid equation.
    Eigen::Matrix<double, 1, 1> m_v;
};

}

#endif
