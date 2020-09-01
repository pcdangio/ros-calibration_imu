#ifndef FMAIN_H
#define FMAIN_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class fmain; }
QT_END_NAMESPACE

#include <QTimer>

#include <ros/ros.h>

#include <sensor_msgs_ext/magnetometer.h>

class fmain : public QMainWindow
{
    Q_OBJECT

public:
    fmain(QWidget *parent = nullptr);
    ~fmain();

private:
    Ui::fmain *ui;
    ros::NodeHandle* m_node;
    QTimer m_ros_spinner;
    void ros_spin();

    ros::Subscriber m_subscriber_magnetometer;
    void subscriber_magnetometer(const sensor_msgs_ext::magnetometerConstPtr& message);
};
#endif // FMAIN_H
