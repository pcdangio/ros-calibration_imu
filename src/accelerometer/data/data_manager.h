/// \file accelerometer/data/data_manager.h
/// \brief Defines the accelerometer::data_manager class.
#ifndef ACCELEROMETER_DATA_MANAGER_H
#define ACCELEROMETER_DATA_MANAGER_H

#include <QObject>

#include <ros/ros.h>
#include <sensor_msgs_ext/acceleration.h>

#include <eigen3/Eigen/Dense>

#include <unordered_map>

namespace accelerometer {

/// \brief Facilitates the collection of accelerometer data for calibration.
class data_manager : public QObject
{
    Q_OBJECT
public:
    // CONSTRUCTORS
    /// \brief Creates a new data_manager instance.
    /// \param node The shared ROS node handle.
    data_manager(std::shared_ptr<ros::NodeHandle> node);
    ~data_manager();

    // ENUMERATIONS
    /// \brief Enumerates the possible orientations of the accelerometer.
    enum class orientation_t
    {
        BOTTOM_DOWN = 0,    ///< Bottom is aligned with gravity vector.
        TOP_DOWN = 1,       ///< Top is aligned with gravity vector.
        LEFT_DOWN = 2,      ///< Left is aligned with gravity vector.
        RIGHT_DOWN = 3,     ///< Right is aligned with gravity vector.
        FRONT_DOWN = 4,     ///< Front is aligned with gravity vector.
        BACK_DOWN = 5       ///< Back is aligned with gravity vector.
    };

    // COLLECTION
    /// \brief Starts collecting data from the accelerometer.
    void start_collection();
    /// \brief Stops collecting data from the accelerometer.
    void stop_collection();
    /// \brief Grabs a snapshot of the current average accelerometer value and stores it in the data matrix.
    /// \param orientation The orientation to grab the measurement for.
    /// \returns TRUE if the grab succeeded, otherwise FALSE.
    bool grab(orientation_t orientation);

    // DATASET
    /// \brief Indicates if all orientations have been grabbed.
    /// \returns TRUE if the data set is complete, otherwise FALSE.
    bool dataset_complete() const;
    /// \brief Gets a copy of the dataset to work with.
    /// \param data The matrix to copy the data set into.
    void get_dataset(Eigen::Matrix<double, 3, 6>& data) const;

signals:
    /// \brief Signals a new measurement is available from the ongoing collection.
    /// \param measurement The average value of the current measurement.
    /// \details This is primarily used for plotting.
    void new_measurement(Eigen::Vector3d measurement);

private:
    // ROS
    /// \brief The application's node handle.
    std::shared_ptr<ros::NodeHandle> m_node;

    // SUBSCRIBERS
    /// \brief The accelerometer data subscriber.
    ros::Subscriber m_subscriber;
    /// \brief The accelerometer subscriber callback.
    /// \param message The received acceleration message.
    void subscriber(const sensor_msgs_ext::accelerationConstPtr& message);

    // FLAGS
    /// \brief Indicates if the data_manager is currently collecting data.
    bool f_is_collecting;

    // BUFFER
    /// \brief A parameter controlling the size of the collection buffer.
    uint32_t p_buffer_size;
    /// \brief A FIFO buffer for storing the most recently collected data points.
    std::deque<Eigen::Vector3d> m_buffer;
    /// \brief Stores the current average of the collection buffer.
    Eigen::Vector3d m_buffer_average;

    // DATASET
    /// \brief Stores the averaged measurements for each of the six grabbed orientations.
    Eigen::Matrix<double, 3, 6> m_data;
};

}

#endif
