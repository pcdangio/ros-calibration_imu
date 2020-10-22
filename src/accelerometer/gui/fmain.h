#ifndef FMAIN_H
#define FMAIN_H

#include <QMainWindow>

namespace Ui {
class fmain;
}

#include <accelerometer/data/data_interface.h>

#include <QTimer>

#include <ros/ros.h>

class fmain : public QMainWindow
{
    Q_OBJECT

public:
    explicit fmain(QWidget *parent = nullptr);
    ~fmain();

private slots:

    // DATA COLLECTION
    void on_button_start_collection_clicked();
    void on_button_stop_collection_clicked();
    void on_button_grab_bottom_clicked();
    void on_button_grab_top_clicked();
    void on_button_grab_left_clicked();
    void on_button_grab_right_clicked();
    void on_button_grab_front_clicked();
    void on_button_grab_rear_clicked();
    void on_button_clear_data_clicked();

private:
    // UI
    Ui::fmain *ui;

    // COMPONENTS
    /// \brief The application's data_interface.
    std::shared_ptr<accelerometer::data_interface> m_data_interface;

    // ROS
    /// \brief Stores the node's handle.
    std::shared_ptr<ros::NodeHandle> m_node;
    /// \brief A timer for spinning ROS.
    QTimer m_ros_spinner;
    /// \brief The worker method for spinning ROS.
    void ros_spin();
};

#endif // FMAIN_H
