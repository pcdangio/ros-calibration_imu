#include "magnetometer/gui/fmain.h"
#include "ui_fmain.h"

#include <QValidator>
#include <QFileDialog>

fmain::fmain(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::fmain)
{
    // Set up UI.
    ui->setupUi(this);
    ui->lineedit_field_strength->setValidator(new QIntValidator(0,500));

    // Set up node handle.
    fmain::m_node = new ros::NodeHandle();

    // Create data interface instance.
    fmain::m_data_interface = std::make_shared<magnetometer::data_interface>();

    // Initialize calibrator.
    fmain::m_calibrator = std::make_shared<magnetometer::calibrator>(fmain::m_data_interface);

    // Create graph instance.
    fmain::m_graph = std::make_shared<magnetometer::graph>(fmain::m_data_interface, fmain::m_calibrator);

    // Connect slots.
    connect(fmain::m_data_interface.get(), &magnetometer::data_interface::data_updated, this, &fmain::collection_updated);
    connect(fmain::m_calibrator.get(), &magnetometer::calibrator::calibration_completed, this, &fmain::calibration_finished);

    // Set up graph widget.
    fmain::ui->layout_main->addWidget(fmain::m_graph->get_widget());

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
    // Enable graph new point indicator.
    fmain::m_graph->indicate_new_point(true);

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
        // Disable new point plotting.
        fmain::m_graph->indicate_new_point(false);
        // Load data.
        std::string bag_file = load_dialog.selectedFiles().first().toStdString();
        fmain::m_data_interface->load_data(bag_file);
    }
}

void fmain::on_checkbox_graph_uncalibrated_stateChanged(int state)
{
    fmain::m_graph->uncalibrated_visible(state == Qt::CheckState::Checked);
}
void fmain::on_checkbox_graph_fit_stateChanged(int state)
{
    fmain::m_graph->fit_visible(state == Qt::CheckState::Checked);
}
void fmain::on_checkbox_graph_truth_stateChanged(int state)
{
    fmain::m_graph->truth_visible(state == Qt::CheckState::Checked);
}

void fmain::on_button_calibrate_clicked()
{
    // Get specified field strength (in uT).
    double field_strength = fmain::ui->lineedit_field_strength->text().toDouble();

    // Create initial guess.
    magnetometer::ellipsoid initial_guess;
    Eigen::Vector3d initial_radius;
    initial_radius.fill(field_strength);
    initial_guess.set_radius(initial_radius);

    // Start calibration routine.
    fmain::m_calibrator->start(initial_guess, field_strength);
}

void fmain::calibration_finished(bool success)
{
    ROS_ERROR_STREAM(success);
    magnetometer::ellipsoid ellipse;
    fmain::m_calibrator->get_fit(ellipse);
    Eigen::Vector3d center, radius, rotation;
    ellipse.get_center(center);
    ellipse.get_radius(radius);
    ellipse.get_rotation(rotation);

    ROS_ERROR_STREAM("center:" << std::endl << center);
    ROS_ERROR_STREAM("radius:" << std::endl << radius);
    ROS_ERROR_STREAM("rotation:" << std::endl << rotation);
}

void fmain::on_checkbox_graph_calibrated_stateChanged(int state)
{
    fmain::m_graph->calibrated_visible(state == Qt::CheckState::Checked);
}
