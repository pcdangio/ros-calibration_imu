#include "fmain.h"
#include "ui_fmain.h"

#include <QMessageBox>

// CONSTRUCTORS
fmain::fmain(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::fmain)
{
    // Set up UI.
    fmain::ui->setupUi(this);
    fmain::ui->progress_bar_calibration->setVisible(false);

    // Set up node handle.
    fmain::m_node = std::make_shared<ros::NodeHandle>();

    // Set up the components.
    fmain::m_data_interface = std::make_shared<accelerometer::data_interface>(fmain::m_node);

    // Start ros spinner.
    connect(&(fmain::m_ros_spinner), &QTimer::timeout, this, &fmain::ros_spin);
    fmain::m_ros_spinner.start(10);
}

fmain::~fmain()
{
    // Clean up UI.
    delete ui;
}

// ROS
void fmain::ros_spin()
{
    // Handle callbacks.
    ros::spinOnce();

    // Quit if ROS shutting down.
    if(!ros::ok())
    {
        QApplication::quit();
    }
}

// DATA COLLECTION
void fmain::on_button_start_collection_clicked()
{
    // Start collection.
    fmain::m_data_interface->start_collection();

    // Update data collection buttons.
    fmain::ui->button_start_collection->setEnabled(false);
    fmain::ui->button_stop_collection->setEnabled(true);
    fmain::ui->button_grab_bottom->setEnabled(true);
    fmain::ui->button_grab_top->setEnabled(true);
    fmain::ui->button_grab_left->setEnabled(true);
    fmain::ui->button_grab_right->setEnabled(true);
    fmain::ui->button_grab_front->setEnabled(true);
    fmain::ui->button_grab_rear->setEnabled(true);
}
void fmain::on_button_stop_collection_clicked()
{
    // Stop collection.
    fmain::m_data_interface->stop_collection();

    // Update data collection buttons.
    fmain::ui->button_start_collection->setEnabled(true);
    fmain::ui->button_stop_collection->setEnabled(false);
    fmain::ui->button_grab_bottom->setEnabled(false);
    fmain::ui->button_grab_top->setEnabled(false);
    fmain::ui->button_grab_left->setEnabled(false);
    fmain::ui->button_grab_right->setEnabled(false);
    fmain::ui->button_grab_front->setEnabled(false);
    fmain::ui->button_grab_rear->setEnabled(false);
}
void fmain::on_button_grab_bottom_clicked()
{
    if(fmain::m_data_interface->grab(accelerometer::data_interface::orientation_t::BOTTOM_DOWN))
    {
        fmain::ui->label_grab_bottom->setStyleSheet("QLabel{color:green;}");
    }
}
void fmain::on_button_grab_top_clicked()
{
    if(fmain::m_data_interface->grab(accelerometer::data_interface::orientation_t::TOP_DOWN))
    {
        fmain::ui->label_grab_top->setStyleSheet("QLabel{color:green;}");
    }
}
void fmain::on_button_grab_left_clicked()
{
    if(fmain::m_data_interface->grab(accelerometer::data_interface::orientation_t::LEFT_DOWN))
    {
        fmain::ui->label_grab_left->setStyleSheet("QLabel{color:green;}");
    }
}
void fmain::on_button_grab_right_clicked()
{
    if(fmain::m_data_interface->grab(accelerometer::data_interface::orientation_t::RIGHT_DOWN))
    {
        fmain::ui->label_grab_right->setStyleSheet("QLabel{color:green;}");
    }
}
void fmain::on_button_grab_front_clicked()
{
    if(fmain::m_data_interface->grab(accelerometer::data_interface::orientation_t::FRONT_DOWN))
    {
        fmain::ui->label_grab_front->setStyleSheet("QLabel{color:green;}");
    }
}
void fmain::on_button_grab_rear_clicked()
{
    if(fmain::m_data_interface->grab(accelerometer::data_interface::orientation_t::REAR_DOWN))
    {
        fmain::ui->label_grab_rear->setStyleSheet("QLabel{color:green;}");
    }
}
void fmain::on_button_clear_data_clicked()
{
    // Clear the data collection.
    fmain::m_data_interface->clear_dataset();

    // Reset labels.
    fmain::ui->label_grab_bottom->setStyleSheet("");
    fmain::ui->label_grab_top->setStyleSheet("");
    fmain::ui->label_grab_left->setStyleSheet("");
    fmain::ui->label_grab_right->setStyleSheet("");
    fmain::ui->label_grab_front->setStyleSheet("");
    fmain::ui->label_grab_rear->setStyleSheet("");
}
