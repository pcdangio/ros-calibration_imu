#ifndef FMAIN_H
#define FMAIN_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class fmain; }
QT_END_NAMESPACE

#include "magnetometer/data/data_interface.h"
#include "magnetometer/graph/graph.h"

#include <QTimer>

#include <ros/ros.h>

class fmain : public QMainWindow
{
    Q_OBJECT

public:
    fmain(QWidget *parent = nullptr);
    ~fmain();

private slots:


    void on_button_start_collection_clicked();

    void on_button_stop_collection_clicked();

    void on_button_clear_collection_clicked();

    void collection_updated();

private:
    Ui::fmain *ui;

    ros::NodeHandle* m_node;
    QTimer m_ros_spinner;
    void ros_spin();

    std::shared_ptr<magnetometer::data_interface> m_data_interface;
    std::shared_ptr<magnetometer::graph> m_graph;
};
#endif // FMAIN_H
