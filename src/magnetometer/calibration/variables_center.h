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
        /// \brief Sets the allowable range of center values.
        /// \param min The minimum allowable value.
        /// \param max The maximum allowable value.
        void set_range(double min, double max);

    private:
        // VARIABLES
        /// \brief Stores the current center x value.
        double x;
        /// \brief Stores the current center y value.
        double y;
        /// \brief Stores the current center z value.
        double z;

        // BOUNDS
        /// \brief Stores the minimum acceptable range of center values.
        double min;
        /// \brief Stores the maximum acceptable range of center values.
        double max;
    };
}

#endif
