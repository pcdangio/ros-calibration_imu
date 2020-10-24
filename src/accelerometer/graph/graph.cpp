#include "graph.h"

#include <QtCharts/QBarSet>
#include <QtCharts/QBoxSet>
#include <QtCharts/QValueAxis>
#include <QtCharts/QBarCategoryAxis>

using namespace accelerometer;

// CONSTRUCTORS
graph::graph()
{
    // Set up chart.
    graph::m_chart = new QtCharts::QChart();

    // Set up measurement series.
    graph::m_series_measurement = new QtCharts::QBarSeries();
    graph::m_series_measurement->setName("Measurement");
    graph::m_chart->addSeries(graph::m_series_measurement);

    // Set up measurement set.
    QtCharts::QBarSet* set_measurement = new QtCharts::QBarSet("Measurement");
    set_measurement->append(0.0);
    set_measurement->append(0.0);
    set_measurement->append(0.0);
    graph::m_series_measurement->append(set_measurement);

    // Set up fit series.
    graph::m_series_fit = new QtCharts::QBoxPlotSeries();
    graph::m_series_fit->setName("Fit");
    graph::m_chart->addSeries(graph::m_series_fit);

    // Set up calibration series.
    graph::m_series_calibration = new QtCharts::QBoxPlotSeries();
    graph::m_series_calibration->setName("Calibration");
    graph::m_chart->addSeries(graph::m_series_calibration);

    // Set up fit/calibration set.
    for(uint32_t i = 0; i < 3; ++i)
    {
        graph::m_series_fit->append(new QtCharts::QBoxSet());
        graph::m_series_calibration->append(new QtCharts::QBoxSet());
    }

    // Set up truth series.
    graph::m_series_truth = new QtCharts::QLineSeries();
    graph::m_series_truth->setName("True Gravity");
    graph::m_chart->addSeries(graph::m_series_truth);
    auto pen_truth = graph::m_series_truth->pen();
    pen_truth.setStyle(Qt::PenStyle::DotLine);
    graph::m_series_truth->setPen(pen_truth);


    // Set up X axis.
    QtCharts::QBarCategoryAxis* axis_x = new QtCharts::QBarCategoryAxis();
    axis_x->append("x");
    axis_x->append("y");
    axis_x->append("z");
    graph::m_chart->addAxis(axis_x, Qt::AlignBottom);
    graph::m_series_measurement->attachAxis(axis_x);
    graph::m_series_fit->attachAxis(axis_x);
    graph::m_series_calibration->attachAxis(axis_x);
    graph::m_series_calibration->attachAxis(axis_x);

    // Set up Y axis.
    QtCharts::QValueAxis* axis_y = new QtCharts::QValueAxis();
    axis_y->setRange(-15, 15);
    axis_y->setTitleText("m/s^2");
    graph::m_chart->addAxis(axis_y, Qt::AlignLeft);
    graph::m_series_measurement->attachAxis(axis_y);
    graph::m_series_fit->attachAxis(axis_y);
    graph::m_series_calibration->attachAxis(axis_y);
    graph::m_series_truth->attachAxis(axis_y);
}
graph::~graph()
{
    delete graph::m_chart;
}

// PLOT GET
QtCharts::QChart* graph::get_chart()
{
    return graph::m_chart;
}

// PLOT VISIBILITY
void graph::set_measurement_visible(bool visible)
{
    graph::m_series_measurement->setVisible(visible);
}
void graph::set_fit_visible(bool visible)
{
    graph::m_series_fit->setVisible(visible);
}
void graph::set_calibration_visible(bool visible)
{
    graph::m_series_calibration->setVisible(visible);
}
void graph::set_truth_visible(bool visible)
{
    graph::m_series_truth->setVisible(visible);
}

// PLOT UPDATE
void graph::new_measurement(Eigen::Vector3d measurement)
{
    auto barset = graph::m_series_measurement->barSets().first();

    barset->replace(0, measurement(0));
    barset->replace(1, measurement(1));
    barset->replace(2, measurement(2));
}
void graph::new_fit(Eigen::Matrix3d fit)
{
    auto boxset = graph::m_series_fit->boxSets();

    for(uint32_t j = 0; j < 3; ++j)
    {
        boxset[j]->setValue(0, fit(0,j));
        boxset[j]->setValue(1, fit(0,j));
        boxset[j]->setValue(2, fit(1,j));
        boxset[j]->setValue(3, fit(2,j));
        boxset[j]->setValue(4, fit(2,j));
    }
}
void graph::new_calibration(Eigen::Matrix3d calibration)
{
    auto boxset = graph::m_series_calibration->boxSets();

    for(uint32_t j = 0; j < 3; ++j)
    {
        boxset[j]->setValue(0, calibration(0,j));
        boxset[j]->setValue(1, calibration(0,j));
        boxset[j]->setValue(2, calibration(1,j));
        boxset[j]->setValue(3, calibration(2,j));
        boxset[j]->setValue(4, calibration(2,j));
    }
}
void graph::new_truth(double truth)
{
    QList<QPointF> points;
    points.append(QPointF(3, truth));
    points.append(QPointF(-0.1, truth));

    points.append(QPointF(-0.1, -truth));
    points.append(QPointF(3, -truth));
    graph::m_series_truth->replace(points);
}
