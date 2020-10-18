#include "magnetometer/calibration/cost_objective.h"

#include "magnetometer/calibration/variables_center.h"
#include "magnetometer/calibration/variables_rotation.h"
#include "magnetometer/calibration/variables_radius.h"

using namespace ifopt;

// CONSTRUCTORS
cost_objective::cost_objective(std::shared_ptr<std::vector<Eigen::Vector3d> > &data_points)
    : CostTerm("objective")
{
    // Store data interface.
    cost_objective::m_data_points = data_points;

    // Initialize parameters.
    cost_objective::m_gradient_perturbation = 0.000001;

    // Set up preallocated ellipsoid.
    cost_objective::m_ellipsoid = new magnetometer::ellipsoid();
}
cost_objective::~cost_objective()
{
    // Clean up preallocated ellipsoid.
    delete cost_objective::m_ellipsoid;
}

// OVERRIDES
double cost_objective::GetCost() const
{
    // Get variable sets.
    auto v_center = std::dynamic_pointer_cast<variables_center>(cost_objective::GetVariables()->GetComponent("center"));
    auto v_rotation = std::dynamic_pointer_cast<variables_rotation>(cost_objective::GetVariables()->GetComponent("rotation"));
    auto v_radius = std::dynamic_pointer_cast<variables_radius>(cost_objective::GetVariables()->GetComponent("radius"));

    // Update ellipsoid.
    cost_objective::m_ellipsoid->set_center(v_center->GetValues());
    cost_objective::m_ellipsoid->set_radius(v_radius->GetValues());
    cost_objective::m_ellipsoid->set_rotation(v_rotation->GetValues());
    
    // Iterate through points.
    double mse = 0.0;
    for(auto point = cost_objective::m_data_points->cbegin(); point != cost_objective::m_data_points->cend(); ++point)
    {
        // Run ellipsoid residual to calculate MSE for point.
        mse += std::pow(cost_objective::m_ellipsoid->residual(*point), 2.0);
    }
    // Take average of MSE.
    mse /= static_cast<double>(cost_objective::m_data_points->size());

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
