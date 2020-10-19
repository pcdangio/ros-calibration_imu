/// \file magnetometer/calibration/variables_rotation.h
/// \brief Defines the ifopt::variables_rotation class.
#ifndef VARIABLE_ROTATION_H
#define VARIABLE_ROTATION_H

#include <ifopt/variable_set.h>

namespace ifopt
{
    /// \brief A variable set for the ellipse rotation.
    class variables_rotation
        : public VariableSet
    {
    public:
        // CONSTRUCTOR
        /// \brief Creates a new rotation variable set.
        variables_rotation();

        // OVERRIDES
        void SetVariables(const Eigen::VectorXd& x) override;
        Eigen::VectorXd GetValues() const override;
        VecBound GetBounds() const override;

        // PARAMETERS
        /// \brief Sets the maximum allowable rotation.
        /// \param value The value to set.
        void set_max(double value);

    private:
        // VARIABLES
        /// \brief Stores the current roll value.
        double r;
        /// \brief Stores the current pitch value.
        double p;
        /// \brief Stores the current yaw value.
        double y;

        // BOUNDS
        /// \brief The maximum allowable rotation.
        double max;
    };
}

#endif
