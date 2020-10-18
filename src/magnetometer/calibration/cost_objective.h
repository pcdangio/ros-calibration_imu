#ifndef COST_OBJECTIVE_H
#define COST_OBJECTIVE_H

#include <ifopt/cost_term.h>

#include "magnetometer/geometry/ellipsoid.h"

namespace ifopt
{
    /// \brief The objective function for the ellipse fit.
    class cost_objective
        : public CostTerm
    {
    public:
        // CONSTRUCTORS
        /// \brief Instantiates a new cost_objective instance.
        /// \param data_points The vector of data points to fit the ellipse to.
        cost_objective(std::shared_ptr<std::vector<Eigen::Vector3d>>& data_points);
        ~cost_objective();

        // OVERRIDES
        double GetCost() const override;
        void FillJacobianBlock(std::string variable_set, Jacobian& jacobian) const override;

        // PARAMETERS
        /// \brief Sets the gradient perturbation parameter.
        /// \param value The new value.
        void p_gradient_perturbation(double value);

    private:
        // COMPONENTS
        std::shared_ptr<std::vector<Eigen::Vector3d>> m_data_points;

        // PARAMETERS
        /// \brief Stores the perturbation for gradient calculation.
        double m_gradient_perturbation;

        // PREALLOCATIONS
        /// \brief A preallocated ellipse for calculation.
        magnetometer::ellipsoid* m_ellipsoid;
    };
}

#endif
