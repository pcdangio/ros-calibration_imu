#ifndef MAGNETOMETER___DATA_INTERFACE_H
#define MAGNETOMETER___DATA_INTERFACE_H

#include <QObject>
#include <QVector3D>
#include <QElapsedTimer>

#include <boost/thread.hpp>

#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs_ext/magnetometer.h>

#include <mutex>

namespace magnetometer
{

/// \brief Provides an interface for data access.
class data_interface
    : public QObject
{
    Q_OBJECT
public:
    // CONSTRUCTOR
    /// \brief Creates a new data interface instance.
    data_interface();
    ~data_interface();

    // DATA SUBSCRIBER
    /// \brief Starts the subscriber for magnetometer data.
    void start_subscriber();
    /// \brief Stops the data subscriber.
    void stop_subscriber();

    // DATA FILE IO
    /// \brief Saves the current data to a ROS bag file.
    /// \param bag_file The path to save the bag file to.
    /// \returns TRUE if the save was successful, otherwise FALSE.
    bool save_data(std::string& bag_file) const;
    /// \brief Loads data from a ROS bag file.
    /// \param bag_file the path to load the bag file from.
    /// \returns TRUE if the load was successful, otherwise FALSE.
    bool load_data(std::string& bag_file);

    // DATA MANAGEMENT
    /// \brief Clears all stored data.
    void clear_data();

    // DATA ACCESS
    /// \brief Gets the number of currently stored points.
    uint32_t n_points();
    /// \brief Gets a data point as an Eigen::Vector3d.
    /// \param index The index of the point to get.
    /// \param point The instance to capture the point in.
    bool get_point(uint32_t index, Eigen::Vector3d& point);
    /// \brief Gets a data point as a QVector3D.
    /// \param index The index of the point to get.
    /// \param point The instance to capture the point in.
    bool get_point(uint32_t index, QVector3D& point);

signals:
    /// \brief Indicates when the data has changed.
    void data_updated();

private:
    // DATA SUBSCRIBER
    /// \brief A subscriber for magnetometer data.
    ros::Subscriber m_subscriber;
    /// \brief The subscriber method for receiving magnetometer data.
    void subscriber(const sensor_msgs_ext::magnetometerConstPtr& message);
    /// \brief A flag indicating if the data subscriber is enabled.
    bool f_subscriber_enabled;

    // DATA RATE
    /// \brief The maximum rate at which magnetometer data is collected.
    double p_max_data_rate;
    /// \brief The timer for implementing the max data rate.
    QElapsedTimer m_data_timer;

    // DATA
    /// \brief Scale factor for converting from T to uT.
    const double m_scale_factor = 1000000.0;
    /// \brief Magnetometer x-axis data.
    std::vector<double> m_x;
    /// \brief Magnetometer y-axis data.
    std::vector<double> m_y;
    /// \brief Magnetometer z-axis data.
    std::vector<double> m_z;
};

}

#endif
