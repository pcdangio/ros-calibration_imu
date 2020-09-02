#include "fmain.h"
#include "ui_fmain.h"

fmain::fmain(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::fmain)
{
    ui->setupUi(this);

    // Set up node handle.
    fmain::m_node = new ros::NodeHandle("~");

    // Start ros spinner.
    connect(&(fmain::m_ros_spinner), &QTimer::timeout, this, &fmain::ros_spin);
    fmain::m_ros_spinner.start(10);

    // Set up magnetometer calibrator.
    fmain::m_magnetometer = new magnetometer();
}

fmain::~fmain()
{
    // Clean up magnetometer calibrator.
    delete fmain::m_magnetometer;

    // Clean up node.
    delete fmain::m_node;

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

void fmain::on_button_enable_magnetometer_clicked()
{
    fmain::m_magnetometer->enable();
}

void fmain::on_button_disable_magnetometer_clicked()
{
    fmain::m_magnetometer->disable();
}
