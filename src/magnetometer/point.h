#ifndef POINT_H
#define POINT_H

#include <vector>

/// \brief A point in 3D space.
struct point_t
{
    /// \brief The x coordinate.
    double x;
    /// \brief The y coordinate.
    double y;
    /// \brief The z coordinate.
    double z;
};

/// \brief A type definition for a vector of points.
typedef std::vector<point_t> points_t;

#endif