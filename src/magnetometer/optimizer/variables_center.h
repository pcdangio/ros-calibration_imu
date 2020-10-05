#ifndef VARIABLES_CENTER_H
#define VARIABLES_CENTER_H

#include <ifopt/variable_set.h>

namespace ifopt
{
    /// \brief A variable set for the ellipse center point.
    class variables_center
        : public VariableSet
    {
    public:
        // CONSTRUCTOR
        /// \brief Creates a new center variable set.
        /// \param x The initial x value.
        /// \param y The initial y value.
        /// \param z The initial z value.
        /// \param max The max +/- range of the center values.
        variables_center(double x, double y, double z, double max);

        // OVERRIDES
        void SetVariables(const Eigen::VectorXd& x) override;
        Eigen::VectorXd GetValues() const override;
        VecBound GetBounds() const override;

        // METHODS
        /// \brief Gets the current center vector.
        /// \param c The center vector to populate.
        void center_vector(Eigen::Vector3d& c) const;

    private:
        // VARIABLES
        /// \brief Stores the current center x value.
        double x;
        /// \brief Stores the current center y value.
        double y;
        /// \brief Stores the current center z value.
        double z;

        // BOUNDS
        /// \brief Stores the max +/- range of the center values.
        double max;
    };
}

#endif