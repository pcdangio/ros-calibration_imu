#ifndef COST_OBJECTIVE_H
#define COST_OBJECTIVE_H

#include <ifopt/cost_term.h>

#include "magnetometer/data/data_interface.h"
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
        /// \param data_interface A pointer to the magnetometer data interface.
        cost_objective(std::shared_ptr<magnetometer::data_interface>& data_interface);
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
        std::shared_ptr<magnetometer::data_interface> m_data_interface;

        // PARAMETERS
        /// \brief Stores the perturbation for gradient calculation.
        double m_gradient_perturbation;

        // PREALLOCATIONS
        /// \brief A preallocated ellipse for calculation.
        magnetometer::ellipsoid* m_ellipsoid;
    };
}

#endif
