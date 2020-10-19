/// \file magnetometer/gui/fmain.h
/// \brief Defines the Ui::fmain class.
#ifndef FMAIN_H
#define FMAIN_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class fmain; }
QT_END_NAMESPACE

#include "magnetometer/data/data_interface.h"
#include "magnetometer/calibration/calibrator.h"
#include "magnetometer/graph/graph.h"

#include <QTimer>

#include <ros/ros.h>

/// \brief The primary GUI for magnetometer calibration.
class fmain : public QMainWindow
{
    Q_OBJECT

public:
    // CONSTRUCTORS
    fmain(QWidget *parent = nullptr);
    ~fmain();

private slots:
    // SLOTS: DATA COLLECTION
    void on_button_start_collection_clicked();
    void on_button_stop_collection_clicked();
    void on_button_clear_collection_clicked();
    void on_button_save_collection_clicked();
    void on_button_load_collection_clicked();
    void on_checkbox_graph_uncalibrated_stateChanged(int state);
    void collection_updated();

    // SLOTS: CALIBRATION
    void on_button_calibrate_clicked();
    void on_button_save_calibration_json_clicked();
    void on_button_save_calibration_yaml_clicked();
    void on_checkbox_graph_fit_stateChanged(int state);
    void on_checkbox_graph_calibrated_stateChanged(int state);
    void on_checkbox_graph_truth_stateChanged(int state);
    void calibration_finished(bool success);

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

    // COMPONENTS
    /// \brief The application's data_interface instance.
    std::shared_ptr<magnetometer::data_interface> m_data_interface;
    /// \brief The application's calibrator instance.
    std::shared_ptr<magnetometer::calibrator> m_calibrator;
    /// \brief The application's graph instance.
    std::shared_ptr<magnetometer::graph> m_graph;
};

#endif
