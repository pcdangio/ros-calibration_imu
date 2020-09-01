#include "fmain.h"
#include "ui_fmain.h"

fmain::fmain(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::fmain)
{
    ui->setupUi(this);

    // Set up node handle.
    fmain::m_node = new ros::NodeHandle("~");

    // Set up subscribers.
    fmain::m_subscriber_magnetometer = fmain::m_node->subscribe("imu/magnetometer", 100, &fmain::subscriber_magnetometer, this);

    // Start ros spinner.
    connect(&(fmain::m_ros_spinner), &QTimer::timeout, this, &fmain::ros_spin);
    fmain::m_ros_spinner.start(10);
}

fmain::~fmain()
{
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

// SUBSCRIBERS
void fmain::subscriber_magnetometer(const sensor_msgs_ext::magnetometerConstPtr &message)
{
    fmain::setWindowTitle(QString::number(message->x));
}
