#ifndef GRAPH_H
#define GRAPH_H

#include "magnetometer/data/data_interface.h"
#include "magnetometer/calibration/calibrator.h"

#include "magnetometer/geometry/ellipsoid.h"

#include <QObject>
#include <QtDataVisualization/Q3DScatter>
#include <QtDataVisualization/QScatter3DSeries>
#include <QWidget>

namespace magnetometer
{

class graph
    : public QObject
{
    Q_OBJECT
public:
    graph(std::shared_ptr<magnetometer::data_interface>& data_interface,
          std::shared_ptr<magnetometer::calibrator>& calibrator);
    ~graph();

    QWidget* get_widget();

    void uncalibrated_visible(bool visible);
    void fit_visible(bool visible);
    void calibrated_visible(bool visible);
    void truth_visible(bool visible);

    void indicate_new_point(bool enabled);


public slots:
    void update_uncalibrated_plot();
    void update_calibration_plots();

private:
    std::shared_ptr<magnetometer::data_interface> m_data_interface;
    std::shared_ptr<magnetometer::calibrator> m_calibrator;

    QtDataVisualization::Q3DScatter* m_graph;
    QtDataVisualization::QScatter3DSeries* m_series_uncalibrated;
    QtDataVisualization::QScatter3DSeries* m_series_uncalibrated_new;
    QtDataVisualization::QScatter3DSeries* m_series_fit;
    QtDataVisualization::QScatter3DSeries* m_series_calibrated;
    QtDataVisualization::QScatter3DSeries* m_series_truth;

    bool f_indicate_new_point;

    void autoscale();
    void update_range(QtDataVisualization::QScatter3DSeries* series, float& min, float& max);
};

}

#endif
