#include "magnetometer/calibration/cost_objective.h"

#include "magnetometer/calibration/variables_center.h"
#include "magnetometer/calibration/variables_rotation.h"
#include "magnetometer/calibration/variables_radius.h"

using namespace ifopt;

// CONSTRUCTORS
cost_objective::cost_objective(std::shared_ptr<magnetometer::data_interface> data_interface)
    : CostTerm("objective")
{
    // Store data interface.
    cost_objective::m_data_interface = data_interface;

    // Initialize parameters.
    cost_objective::m_gradient_perturbation = 0.000001;

    // Set up pre-allocations.
    cost_objective::c = new Eigen::Matrix<double, 3, 1>;
    cost_objective::p = new Eigen::Matrix<double, 3, 1>;
    cost_objective::p_t = new Eigen::Matrix<double, 1, 3>;
    cost_objective::r = new Eigen::Matrix<double, 3, 3>;
    cost_objective::r_t = new Eigen::Matrix<double, 3, 3>;
    cost_objective::e = new Eigen::Matrix<double, 3, 3>;
    cost_objective::t1 = new Eigen::Matrix<double, 1, 3>;
    cost_objective::t2 = new Eigen::Matrix<double, 1, 3>;
    cost_objective::v = new Eigen::Matrix<double, 1, 1>;
}
cost_objective::~cost_objective()
{
    // Clean up pre-allocations.
    delete cost_objective::c;
    delete cost_objective::p;
    delete cost_objective::p_t;
    delete cost_objective::r;
    delete cost_objective::r_t;
    delete cost_objective::e;
    delete cost_objective::t1;
    delete cost_objective::t2;
    delete cost_objective::v;
}

// OVERRIDES
double cost_objective::GetCost() const
{
    // Get references to allocated matrices.
    auto& c = *cost_objective::c;
    auto& p = *cost_objective::p;
    auto& p_t = *cost_objective::p_t;
    auto& r = *cost_objective::r;
    auto& r_t = *cost_objective::r_t;
    auto& e = *cost_objective::e;
    auto& t1 = *cost_objective::t1;
    auto& t2 = *cost_objective::t2;
    auto& v = *cost_objective::v;

    // Get variable sets.
    auto v_center = std::dynamic_pointer_cast<variables_center>(cost_objective::GetVariables()->GetComponent("center"));
    auto v_rotation = std::dynamic_pointer_cast<variables_rotation>(cost_objective::GetVariables()->GetComponent("rotation"));
    auto v_radius = std::dynamic_pointer_cast<variables_radius>(cost_objective::GetVariables()->GetComponent("radius"));

    // Get centers.
    v_center->center_vector(c);
    // Create rotation and radius matrices.
    v_rotation->rotation_matrix(r);
    r_t.noalias() = r.transpose();
    v_radius->radius_matrix(e);
    
    // Iterate through points.
    double mse = 0.0;
    for(uint32_t i = 0; i < cost_objective::m_data_interface->n_points(); ++i)
    {
        // Create p vector.
        cost_objective::m_data_interface->get_point(i, p);
        p -= c;
        p_t.noalias() = p.transpose();

        // Evaluate point against ellipsoid equation.
        // p'*r*A*r'*p = 1
        t1.noalias() = p_t * r;
        t2.noalias() = t1 * e;
        t1.noalias() = t2 * r_t;
        v.noalias() = t1 * p;

        // Add to MSE.
        mse += std::pow(v(0,0) - 1.0, 2.0);
    }
    // Take average of MSE.
    mse /= static_cast<double>(cost_objective::m_data_interface->n_points());

    return mse;
}
void cost_objective::FillJacobianBlock(std::string variable_set, Jacobian& jacobian) const
{
    // Get operating point.
    auto variable = cost_objective::GetVariables()->GetComponent(variable_set);
    auto operating_point = variable->GetValues();
    // Perform center finite difference.
    for(uint32_t i = 0; i < operating_point.size(); ++i)
    {
        // Calculate cost at negative perturbation.
        auto minus = operating_point;
        minus(i) -= cost_objective::m_gradient_perturbation;
        variable->SetVariables(minus);
        double cost_minus = cost_objective::GetCost();

        // Calculate cost at positive perturbation.
        auto plus = operating_point;
        plus(i) += cost_objective::m_gradient_perturbation;
        variable->SetVariables(plus);
        double cost_plus = cost_objective::GetCost();

        // Restore original variable value.
        variable->SetVariables(operating_point);

        // Calculate derivative.
        jacobian.coeffRef(0,i) = (cost_plus - cost_minus) / (2.0 * cost_objective::m_gradient_perturbation);
    }
}

// PARAMETERS
void cost_objective::p_gradient_perturbation(double value)
{
    cost_objective::m_gradient_perturbation = value;
}
