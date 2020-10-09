#include "magnetometer/gui/fmain.h"
#include "ui_fmain.h"

fmain::fmain(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::fmain)
{
    ui->setupUi(this);

    // Set up node handle.
    fmain::m_node = new ros::NodeHandle();

    // Create data interface instance.
    fmain::m_data_interface = std::make_shared<magnetometer::data_interface>();

    // Create graph instance.
    fmain::m_graph = std::make_shared<magnetometer::graph>(fmain::m_data_interface);

    // Connect slots.
    connect(fmain::m_data_interface.get(), &magnetometer::data_interface::data_updated, fmain::m_graph.get(), &magnetometer::graph::update_uncalibrated_plot);
    connect(fmain::m_data_interface.get(), &magnetometer::data_interface::data_updated, this, &fmain::collection_updated);

    // Set up graph widget.
    fmain::ui->layout_graph->addWidget(fmain::m_graph->get_widget());

    // Start ros spinner.
    connect(&(fmain::m_ros_spinner), &QTimer::timeout, this, &fmain::ros_spin);
    fmain::m_ros_spinner.start(10);
}

fmain::~fmain()
{
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


void fmain::on_button_start_collection_clicked()
{
    // Start the data interface subscriber.
    fmain::m_data_interface->start_subscriber();
}
void fmain::on_button_stop_collection_clicked()
{
    // Stop the data interface subscribers.
    fmain::m_data_interface->stop_subscriber();
}
void fmain::on_button_clear_collection_clicked()
{
    // Clear the data.
    fmain::m_data_interface->clear_data();
}

void fmain::collection_updated()
{
    // Update point counter.
    fmain::ui->lineedit_n_collection_points->setText(QString::number(fmain::m_data_interface->n_points()));
}
