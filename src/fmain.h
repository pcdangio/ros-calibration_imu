#ifndef FMAIN_H
#define FMAIN_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class fmain; }
QT_END_NAMESPACE

#include "magnetometer.h"

#include <QTimer>

class fmain : public QMainWindow
{
    Q_OBJECT

public:
    fmain(QWidget *parent = nullptr);
    ~fmain();

private slots:


    void on_button_magnetometer_start_collection_clicked();

    void on_button_magnetometer_stop_collection_clicked();

    void on_combobox_magnetometer_charts_currentIndexChanged(int index);

private:
    Ui::fmain *ui;

    ros::NodeHandle* m_node;
    QTimer m_ros_spinner;
    void ros_spin();

    magnetometer* m_magnetometer;
};
#endif // FMAIN_H
