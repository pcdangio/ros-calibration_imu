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
    connect(fmain::m_magnetometer, &magnetometer::collection_updated, this, &fmain::magnetometer_collection_updated);

    // Set up magnetometer plot combobox.
    fmain::ui->combobox_magnetometer_charts->addItems({"XY", "XZ", "YZ"});
    fmain::ui->combobox_magnetometer_charts->setCurrentIndex(0);
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


void fmain::on_button_magnetometer_start_collection_clicked()
{
    fmain::m_magnetometer->start_collection();
}

void fmain::on_button_magnetometer_stop_collection_clicked()
{
    fmain::m_magnetometer->stop_collection();
}

void fmain::on_combobox_magnetometer_charts_currentIndexChanged(int index)
{
    magnetometer::chart_t chart_type = static_cast<magnetometer::chart_t>(index);
    auto chart = fmain::m_magnetometer->get_chart(chart_type);
    if(chart)
    {
        fmain::ui->chart_magnetometer_calibrate->setChart(chart);
    }
}

void fmain::on_button_magnetometer_clear_collection_clicked()
{
    fmain::m_magnetometer->clear_collection();
}

void fmain::magnetometer_collection_updated(uint32_t n_collection_points)
{
    fmain::ui->lineedit_magnetometer_n_collection_points->setText(QString::number(n_collection_points));
}
