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
        variables_center();

        // OVERRIDES
        void SetVariables(const Eigen::VectorXd& x) override;
        Eigen::VectorXd GetValues() const override;
        VecBound GetBounds() const override;

        // METHODS
        /// \brief Gets the current center vector.
        /// \param c The center vector to populate.
        void center_vector(Eigen::Vector3d& c) const;

        // PARAMETERS
        /// \brief Sets the max +/- range of the center values.
        /// \param value The value to set.
        void p_max(double value);

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