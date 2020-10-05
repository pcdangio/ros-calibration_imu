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
        /// \param r The initial a radius.
        /// \param p The initial b radius.
        /// \param y The initial c radius.
        /// \param max The max allowable radius.
        variables_radius(double a, double b, double c, double max);

        // OVERRIDES
        void SetVariables(const Eigen::VectorXd& x) override;
        Eigen::VectorXd GetValues() const override;
        VecBound GetBounds() const override;

        // METHODS
        /// \brief Gets the current radius matrix.
        /// \param e The radius matrix to populate.
        void radius_matrix(Eigen::Matrix3d& e);

    private:
        // VARIABLES
        /// \brief The current x radius.
        double a;
        /// \brief The current y radius.
        double b;
        /// \brief The current z radius.
        double c;

        // BOUNDS
        /// \brief The max allowable radius.
        double max;
    };
}

#endif