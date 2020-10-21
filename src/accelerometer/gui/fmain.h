#ifndef FMAIN_H
#define FMAIN_H

#include <QMainWindow>

namespace Ui {
class fmain;
}

#include <QTimer>

#include <ros/ros.h>

class fmain : public QMainWindow
{
    Q_OBJECT

public:
    explicit fmain(QWidget *parent = nullptr);
    ~fmain();

private:
    // UI
    Ui::fmain *ui;

    // ROS
    /// \brief Stores the node's handle.
    ros::NodeHandle* m_node;
    /// \brief A timer for spinning ROS.
    QTimer m_ros_spinner;
    /// \brief The worker method for spinning ROS.
    void ros_spin();
};

#endif // FMAIN_H
