#ifndef COST_OBJECTIVE_H
#define COST_OBJECTIVE_H

#include <ifopt/cost_term.h>

#include "magnetometer/data/data_interface.h"

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
        cost_objective(std::shared_ptr<magnetometer::data_interface> data_interface);
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
        /// \brief The fitted ellipse center point.
        Eigen::Matrix<double, 3, 1>* c;
        /// \brief A point of the ellipse.
        Eigen::Matrix<double, 3, 1>* p;
        /// \brief A transposed point of the ellipse.
        Eigen::Matrix<double, 1, 3>* p_t;
        /// \brief The rotation of the fitted ellipse.
        Eigen::Matrix<double, 3, 3>* r;
        /// \brief The transposed rotation of the fitted ellipse.
        Eigen::Matrix<double, 3, 3>* r_t;
        /// \brief The fitted ellipse radius matrix.
        Eigen::Matrix<double, 3, 3>* e;
        /// \brief A temporary matrix.
        Eigen::Matrix<double, 1, 3>* t1;
        /// \brief A temporary matrix.
        Eigen::Matrix<double, 1, 3>* t2;
        /// \brief The ellipse equation value.
        Eigen::Matrix<double, 1, 1>* v;
    };
}

#endif
