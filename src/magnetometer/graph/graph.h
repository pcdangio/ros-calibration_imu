/// \file magnetometer/graph/graph.h
/// \brief Defines the magnetometer::graph class.
#ifndef GRAPH_H
#define GRAPH_H

#include "magnetometer/data/data_interface.h"
#include "magnetometer/calibration/calibrator.h"

#include <QObject>
#include <QtDataVisualization/Q3DScatter>
#include <QtDataVisualization/QScatter3DSeries>
#include <QWidget>

namespace magnetometer
{

/// \brief Provides 3D plotting functions for magnetometer calibration.
class graph
    : public QObject
{
    Q_OBJECT
public:
    // CONSTRUCTORS
    /// \brief Creates a new graph instance.
    /// \param data_interface A pointer to the application's data_interface instance.
    /// \param calibrator A pointer to the application's calibrator instance.
    graph(std::shared_ptr<magnetometer::data_interface>& data_interface,
          std::shared_ptr<magnetometer::calibrator>& calibrator);
    ~graph();

    // INITIALIZATION
    /// \brief Gets the widget for displaying the graph.
    /// \returns The widget to draw.
    QWidget* get_widget();

    // PLOT VISIBILITY
    /// \brief Sets the visibility for the uncalibrated data plot.
    /// \param visible The visibility state to set.
    void uncalibrated_visible(bool visible);
    /// \brief Sets the visibility for the ellipsoid fit plot.
    /// \param visible The visibility state to set.
    void fit_visible(bool visible);
    /// \brief Sets the visibility for the calibrated data plot.
    /// \param visible The visibility state to set.
    void calibrated_visible(bool visible);
    /// \brief Sets the visibility for the true field sphere plot.
    /// \param visible The visibility state to set.
    void truth_visible(bool visible);
    /// \brief Sets the visibility of the highlight marker for the newest uncalibrated data point.
    /// \param enabled The visibility state to set.
    void indicate_new_point(bool enabled);

public slots:
    // PLOT UPDATE METHODS
    /// \brief Updates plots containing uncalibrated data points.
    void update_uncalibrated_plot();
    /// \brief Updates plots containing calibrated data points.
    /// \param calibration_success TRUE if the calibration succeeded, otherwise FALSE.
    void update_calibration_plots(bool calibration_success);
    /// \brief Updates the true field strength plot.
    /// \param true_field_strength The magnitude of the true field strength.
    void update_truth_plot(double true_field_strength);

private:
    // PARAMETERS
    /// \brief Stores the scale to plot magnetic field data at (from Tesla).
    /// \details As configured, plots data at the uT scale.
    const double m_field_scale = 1000000;

    // COMPONENTS
    /// \brief The application's data_interface instance.
    std::shared_ptr<magnetometer::data_interface> m_data_interface;
    /// \brief The application's calibrator instance.
    std::shared_ptr<magnetometer::calibrator> m_calibrator;

    // PLOTS
    /// \brief The 3D scatter graph.
    QtDataVisualization::Q3DScatter* m_graph;
    /// \brief The uncalibrated data series.
    QtDataVisualization::QScatter3DSeries* m_series_uncalibrated;
    /// \brief The new uncalibrated data indicator series.
    QtDataVisualization::QScatter3DSeries* m_series_uncalibrated_new;
    /// \brief The fit ellipsoid series.
    QtDataVisualization::QScatter3DSeries* m_series_fit;
    /// \brief The calibrated data series.
    QtDataVisualization::QScatter3DSeries* m_series_calibrated;
    /// \brief The true magnetic field series.
    QtDataVisualization::QScatter3DSeries* m_series_truth;

    // FLAGS
    /// \brief Specifies if a new uncalibrated data point should be indicated or not.
    bool f_indicate_new_point;

    // PLOT METHODS
    /// \brief Updates the axis ranges of the 3D plot.
    void autoscale();
    /// \brief Calculates a range for a data series.
    /// \param series The series to calculate the range for.
    /// \param min Captures the minimum value of the range.
    /// \param max Captures the maximum value of the range.
    void update_range(QtDataVisualization::QScatter3DSeries* series, float& min, float& max);
};

}

#endif
