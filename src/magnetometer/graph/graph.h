#ifndef GRAPH_H
#define GRAPH_H

#include "magnetometer/data/data_interface.h"

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
    graph(std::shared_ptr<magnetometer::data_interface>& data_interface);
    ~graph();

    QWidget* get_widget();

    void uncalibrated_visible(bool visible);
    void indicate_new_point(bool enabled);
    void calibrated_visible(bool visible);

public slots:
    void update_uncalibrated_plot();
    void update_calibrated_plot();

private:
    std::shared_ptr<magnetometer::data_interface> m_data_interface;

    QtDataVisualization::Q3DScatter* m_graph;
    QtDataVisualization::QScatter3DSeries* m_series_uncalibrated;
    QtDataVisualization::QScatter3DSeries* m_series_uncalibrated_new;
    QtDataVisualization::QScatter3DSeries* m_series_calibrated;

    bool f_indicate_new_point;
};

}

#endif
