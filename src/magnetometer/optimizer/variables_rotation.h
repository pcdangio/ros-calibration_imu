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
        ~variables_rotation();

        // OVERRIDES
        void SetVariables(const Eigen::VectorXd& x) override;
        Eigen::VectorXd GetValues() const override;
        VecBound GetBounds() const override;

        // METHODS
        /// \brief Gets the current rotation matrix.
        /// \param r The rotation matrix to populate.
        void rotation_matrix(Eigen::Matrix3d& r) const;

        // PARAMETERS
        /// \brief Sets the maximum allowable rotation.
        /// \param value The value to set.
        void p_max(double value);

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

        // PREALLOCATIONS
        /// \brief The x rotation matrix.
        Eigen::Matrix<double, 3, 3>* rx;
        /// \brief The y rotation matrix.
        Eigen::Matrix<double, 3, 3>* ry;
        /// \brief The z rotation matrix.
        Eigen::Matrix<double, 3, 3>* rz;
        /// \brief A temporary matrix.
        Eigen::Matrix<double, 3, 3>* t;
    };
}

#endif