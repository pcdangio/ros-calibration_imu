#ifndef VARIABLES_RADIUS_H
#define VARIABLES_RADIUS_H

#include <ifopt/variable_set.h>

namespace ifopt
{
    /// \brief A variable set for the ellipse radii.
    class variables_radius
        : public VariableSet
    {
    public:
        // CONSTRUCTOR
        /// \brief Creates a new radius variable set.
        variables_radius();

        // OVERRIDES
        void SetVariables(const Eigen::VectorXd& x) override;
        Eigen::VectorXd GetValues() const override;
        VecBound GetBounds() const override;

        // METHODS
        /// \brief Gets the current radius matrix.
        /// \param e The radius matrix to populate.
        void radius_matrix(Eigen::Matrix3d& e);

        // PROPERTIES
        /// \brief Sets the allowable range of radius values.
        /// \param min The minimum allowable value.
        /// \param max The maximum allowable value.
        void set_range(const Eigen::Vector3d& min, const Eigen::Vector3d& max);

    private:
        // VARIABLES
        /// \brief The current x radius.
        double a;
        /// \brief The current y radius.
        double b;
        /// \brief The current z radius.
        double c;

        // BOUNDS
        /// \brief Stores the minimum acceptable range of radius values.
        Eigen::Vector3d min;
        /// \brief Stores the maximum acceptable range of radius values.
        Eigen::Vector3d max;
    };
}

#endif
