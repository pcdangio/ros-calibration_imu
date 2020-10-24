#include "fmain.h"
#include "ui_fmain.h"

#include <QMessageBox>

Q_DECLARE_METATYPE(Eigen::Matrix3d)

// CONSTRUCTORS
fmain::fmain(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::fmain)
{
    // Set up UI.
    ui->setupUi(this);
    fmain::ui->progress_bar_calibration->setVisible(false);

    // Set up node handle.
    fmain::m_node = std::make_shared<ros::NodeHandle>();

    // Set up the components.
    fmain::m_data_interface = std::make_shared<accelerometer::data_interface>(fmain::m_node);
    fmain::m_calibrator = std::make_shared<accelerometer::calibrator>();
    fmain::m_graph = std::make_shared<accelerometer::graph>();

    // Make component connections.
    qRegisterMetaType<Eigen::Matrix3d>();
    connect(fmain::m_data_interface.get(), &accelerometer::data_interface::new_measurement, fmain::m_graph.get(), &accelerometer::graph::new_measurement);
    connect(fmain::m_calibrator.get(), &accelerometer::calibrator::new_fit, fmain::m_graph.get(), &accelerometer::graph::new_fit);
    connect(fmain::m_calibrator.get(), &accelerometer::calibrator::new_calibration, fmain::m_graph.get(), &accelerometer::graph::new_calibration);
    connect(fmain::m_calibrator.get(), &accelerometer::calibrator::calibration_completed, this, &fmain::calibration_completed);

    // Set up chart.
    fmain::ui->chart->setChart(fmain::m_graph->get_chart());
    fmain::m_graph->set_fit_visible(false);
    fmain::m_graph->set_calibration_visible(false);

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

// SLOTS: DATA COLLECTION
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

// SLOTS: CALIBRATION
void fmain::on_button_calibrate_clicked()
{
    // Check if dataset is complete.
    if(!fmain::m_data_interface->dataset_complete())
    {
        QMessageBox message_box(QMessageBox::Icon::Warning, "Error", "Not all orientations have been grabbed.", QMessageBox::StandardButton::Ok);
        message_box.exec();
        return;
    }

    // Get the true field strength.
    bool field_parsed;
    double true_field_strength = fmain::ui->lineedit_true_gravity->text().toDouble(&field_parsed);
    if(!field_parsed || true_field_strength <= 0)
    {
        QMessageBox message_box(QMessageBox::Icon::Warning, "Error", "Invalid true gravity field.", QMessageBox::StandardButton::Ok);
        message_box.exec();
        return;
    }

    // Start the calibrator.
    fmain::m_calibrator->start(fmain::m_data_interface->get_dataset(), true_field_strength);

    // Show the progress bar.
    fmain::ui->progress_bar_calibration->setVisible(true);
}
void fmain::calibration_completed(bool success)
{
    // Display error if failed.
    if(!success)
    {
        QMessageBox message_box(QMessageBox::Icon::Warning, "Error", "Calibration failed.", QMessageBox::StandardButton::Ok);
        message_box.exec();
    }

    // Hide progress bar.
    fmain::ui->progress_bar_calibration->setVisible(false);

    // Print into textedit.
    fmain::ui->textedit_calibration->setPlainText(QString::fromStdString(fmain::m_calibrator->print_calibration()));

    // Enable fit/calibration plots.
    fmain::m_graph->set_fit_visible(true);
    fmain::m_graph->set_calibration_visible(true);
}
