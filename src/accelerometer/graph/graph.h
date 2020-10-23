/// \file accelerometer/graph/graph.h
/// \brief Defines the accelerometer::graph class.
#ifndef ACCELEROMETER_GRAPH_H
#define ACCELEROMETER_GRAPH_H

#include <QObject>

#include <QtCharts/QChart>
#include <QtCharts/QBarSeries>
#include <QtCharts/QBoxPlotSeries>

#include <eigen3/Eigen/Dense>

namespace accelerometer {

/// \brief Provides graphing functionality.
class graph : public QObject
{
    Q_OBJECT
public:
    // CONSTRUCTORS
    /// \brief Creates a new graph instance.
    graph();
    ~graph();

    // PLOT GET
    /// \brief Gets the graph's internal chart instance.
    /// \returns The internal chart instance.
    QtCharts::QChart* get_chart();

    // PLOT VISIBILITY
    /// \brief Sets the visibility of the measurement plot.
    /// \param visibile The visibility to set.
    void set_measurement_visible(bool visible);
    /// \brief Sets the visibility of the fit plot.
    /// \param visibile The visibility to set.
    void set_fit_visible(bool visible);
    /// \brief Sets the visibility of the calibration plot.
    /// \param visibile The visibility to set.
    void set_calibration_visible(bool visible);

public slots:
    // PLOT UPDATE
    /// \brief Updates the plot with a new measurement.
    /// \param measurement The new measurement.
    void new_measurement(Eigen::Vector3d measurement);
    /// \brief Updates the plot with a new fit.
    /// \param fit The new fit.
    void new_fit(Eigen::Matrix3d fit);
    /// \brief Updates the plot with a new calibration.
    /// \param calibration The new calibration.
    void new_calibration(Eigen::Matrix3d calibration);

private:
    /// \brief The graph's internal chart instance.
    QtCharts::QChart* m_chart;

    /// \brief The measurement series.
    QtCharts::QBarSeries* m_series_measurement;
    /// \brief The fit series.
    QtCharts::QBoxPlotSeries* m_series_fit;
    /// \brief The calibration series.
    QtCharts::QBoxPlotSeries* m_series_calibration;
};

}

#endif
