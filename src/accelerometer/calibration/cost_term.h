/// \file accelerometer/calibration/cost_term.h
/// \brief Defines the ifopt::cost_term class.
#ifndef COST_TERM_H
#define COST_TERM_H

#include <ifopt/cost_term.h>

#include "common/geometry/ellipsoid.h"

/// \brief Includes software components related to optimization with the IFOPT ROS package.
namespace ifopt
{
    /// \brief The objective function for the ellipse fit.
    class cost_term
        : public CostTerm
    {
    public:
        // CONSTRUCTORS
        /// \brief Instantiates a new cost_term instance.
        /// \param data_set The data set to fit an ellipsoid to.
        cost_term(const Eigen::Matrix<double, 3, 6>& data_set);
        ~cost_term();

        // OVERRIDES
        double GetCost() const override;
        void FillJacobianBlock(std::string variable_set, Jacobian& jacobian) const override;

        // PARAMETERS
        /// \brief Sets the gradient perturbation parameter.
        /// \param value The new value.
        void p_gradient_perturbation(double value);

    private:
        // COMPONENTS
        /// \brief The data set to fit an ellipsoid to.
        Eigen::Matrix<double, 3, 6> m_data_set;

        // PARAMETERS
        /// \brief Stores the perturbation for gradient calculation.
        double m_gradient_perturbation;

        // PREALLOCATIONS
        /// \brief A preallocated ellipse for calculation.
        common::ellipsoid* m_ellipsoid;
    };
}

#endif

