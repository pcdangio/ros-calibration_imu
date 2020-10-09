#include "magnetometer/gui/fmain.h"
#include "ui_fmain.h"

#include <QFileDialog>

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

void fmain::on_button_save_collection_clicked()
{
    // Create save dialog.
    QFileDialog save_dialog(this);
    save_dialog.setAcceptMode(QFileDialog::AcceptMode::AcceptSave);
    save_dialog.setFileMode(QFileDialog::FileMode::AnyFile);
    save_dialog.setNameFilter("ROS Bag Files (*.bag)");
    save_dialog.setViewMode(QFileDialog::ViewMode::Detail);
    save_dialog.setWindowTitle("Save Data");

    // Run dialog.
    if(save_dialog.exec())
    {
        // Save data.
        std::string bag_file = save_dialog.selectedFiles().first().toStdString();
        fmain::m_data_interface->save_data(bag_file);
    }
}

void fmain::on_button_load_collection_clicked()
{
    // Create load dialog.
    QFileDialog load_dialog(this);
    load_dialog.setAcceptMode(QFileDialog::AcceptMode::AcceptOpen);
    load_dialog.setFileMode(QFileDialog::FileMode::ExistingFile);
    load_dialog.setNameFilter("ROS Bag Files (*.bag)");
    load_dialog.setViewMode(QFileDialog::ViewMode::Detail);
    load_dialog.setWindowTitle("Load Data");

    // Run dialog.
    if(load_dialog.exec())
    {
        // Save data.
        std::string bag_file = load_dialog.selectedFiles().first().toStdString();
        fmain::m_data_interface->load_data(bag_file);
    }
}
