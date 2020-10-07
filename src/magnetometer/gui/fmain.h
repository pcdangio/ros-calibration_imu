#ifndef FMAIN_H
#define FMAIN_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class fmain; }
QT_END_NAMESPACE

#include "magnetometer/data/data_interface.h"

#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>
#include <QtCharts/QScatterSeries>
#include <QTimer>

#include <ros/ros.h>

#include <map>

class fmain : public QMainWindow
{
    Q_OBJECT

public:
    fmain(QWidget *parent = nullptr);
    ~fmain();

private slots:


    void on_button_start_collection_clicked();

    void on_button_stop_collection_clicked();

    void on_combobox_charts_currentIndexChanged(int index);

    void on_button_clear_collection_clicked();

    void on_button_start_fit_clicked();

private:
    Ui::fmain *ui;

    ros::NodeHandle* m_node;
    QTimer m_ros_spinner;
    void ros_spin();

    std::shared_ptr<magnetometer::data_interface> m_data_interface;

    enum class chart_t
    {
        XY = 0,
        XZ = 1,
        YZ = 2,
    };

    void start_fit();
    void stop_fit();

    enum class state_t
    {
        IDLE = 0,
        COLLECTION = 1,
        FIT = 2,
        TRANSFORM = 3
    };
    state_t m_state;

    std::map<chart_t, QtCharts::QChart*> m_charts;
    std::map<chart_t, QtCharts::QValueAxis*> m_axes_x;
    std::map<chart_t, QtCharts::QValueAxis*> m_axes_y;
    std::map<chart_t, QtCharts::QScatterSeries*> m_series_collections;
    std::map<chart_t, QtCharts::QScatterSeries*> m_series_current_position;
    void initialize_charts();
    void clear_charts();
    void update_charts();
};
#endif // FMAIN_H
