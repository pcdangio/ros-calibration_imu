#include "accelerometer/gui/fmain.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    // Initialize ROS
    ros::init(argc, argv, "calibration_accelerometer");

    QApplication a(argc, argv);
    fmain w;
    w.show();
    return a.exec();
}
