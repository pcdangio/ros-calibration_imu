#include "magnetometer/gui/fmain.h"
#include "ui_fmain.h"

#include <QValidator>
#include <QFileDialog>
#include <QMessageBox>

fmain::fmain(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::fmain)
{
    // Set up UI.
    ui->setupUi(this);
    ui->progressbar_calibrate->setVisible(false);

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
    save_dialog.setDefaultSuffix("bag");
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
void fmain::on_checkbox_graph_calibrated_stateChanged(int state)
{
    fmain::m_graph->calibrated_visible(state == Qt::CheckState::Checked);
}
void fmain::on_checkbox_graph_truth_stateChanged(int state)
{
    fmain::m_graph->truth_visible(state == Qt::CheckState::Checked);
}

void fmain::on_button_calibrate_clicked()
{
    // Validate and retrieve field strength.
    bool valid_field_strength;
    double nanoteslas = fmain::ui->lineedit_field_strength->text().toDouble(&valid_field_strength);
    if(!valid_field_strength || nanoteslas < 10000)
    {
        // Display error.
        QMessageBox message_box(QMessageBox::Icon::Warning, "", "True field strength must be a valid number above 10,000nT.", QMessageBox::StandardButton::Ok);
        message_box.exec();

        // Bring focus back to line edit.
        fmain::ui->lineedit_field_strength->setFocus();
        fmain::ui->lineedit_field_strength->selectAll();

        return;
    }
    double field_strength = nanoteslas / 1000000000.0;

    // Update graph's true field plot.
    fmain::m_graph->update_truth_plot(field_strength);

    // Start calibration routine.
    fmain::m_calibrator->start(field_strength);

    // Show the progress bar.
    fmain::ui->progressbar_calibrate->setVisible(true);

    // Clear any existing calibrations.
    fmain::ui->textedit_calibration->clear();
}

void fmain::calibration_finished(bool success)
{
    // Hide the progress bar.
    fmain::ui->progressbar_calibrate->setVisible(false);

    // Display the calibration if succeeded.
    if(success)
    {
        // Display calibration.
        fmain::ui->textedit_calibration->setPlainText(QString::fromStdString(fmain::m_calibrator->print_calibration()));
    }
    else
    {
        // Display error.
        QMessageBox message_box(QMessageBox::Icon::Warning, "", "Calibration failed.", QMessageBox::StandardButton::Ok);
        message_box.exec();
    }
}

void fmain::on_button_save_calibration_json_clicked()
{
    // Create save dialog.
    QFileDialog save_dialog(this);
    save_dialog.setAcceptMode(QFileDialog::AcceptMode::AcceptSave);
    save_dialog.setFileMode(QFileDialog::FileMode::AnyFile);
    save_dialog.setNameFilter("JSON file (*.json)");
    save_dialog.setDefaultSuffix("json");
    save_dialog.setViewMode(QFileDialog::ViewMode::Detail);
    save_dialog.setWindowTitle("Save JSON Calibration");

    // Run dialog.
    if(save_dialog.exec())
    {
        // Save data.
        std::string json_file = save_dialog.selectedFiles().first().toStdString();
        if(!fmain::m_calibrator->save_calibration_json(json_file))
        {
            // Display error.
            QMessageBox message_box(QMessageBox::Icon::Warning, "", "Save JSON calibration failed, check ROS logs for reason.", QMessageBox::StandardButton::Ok);
            message_box.exec();
        }
    }
}

void fmain::on_button_save_calibration_yaml_clicked()
{
    // Create save dialog.
    QFileDialog save_dialog(this);
    save_dialog.setAcceptMode(QFileDialog::AcceptMode::AcceptSave);
    save_dialog.setFileMode(QFileDialog::FileMode::AnyFile);
    save_dialog.setNameFilter("YAML file (*.yaml)");
    save_dialog.setDefaultSuffix("yaml");
    save_dialog.setViewMode(QFileDialog::ViewMode::Detail);
    save_dialog.setWindowTitle("Save YAML Calibration");

    // Run dialog.
    if(save_dialog.exec())
    {
        // Save data.
        std::string yaml_file = save_dialog.selectedFiles().first().toStdString();
        if(!fmain::m_calibrator->save_calibration_yaml(yaml_file))
        {
            // Display error.
            QMessageBox message_box(QMessageBox::Icon::Warning, "", "Save YAML calibration failed, check ROS logs for reason.", QMessageBox::StandardButton::Ok);
            message_box.exec();
        }
    }
}
