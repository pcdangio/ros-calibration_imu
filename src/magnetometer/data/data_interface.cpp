#include "magnetometer/data/data_interface.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace magnetometer;

// CONSTRUCTORS
data_interface::data_interface()
{
    // Initialize flags.
    data_interface::f_subscriber_enabled = false;

    // Load parameters.
    ros::NodeHandle private_handle("~");
    data_interface::p_max_data_rate = private_handle.param<double>("max_data_rate", 1000.0);
}
data_interface::~data_interface()
{
    // Stop the subscriber if it's running.
    data_interface::stop_subscriber();
}

// DATA SUBSCRIBER
void data_interface::start_subscriber()
{
    if(!data_interface::f_subscriber_enabled)
    {
        ros::NodeHandle node_handle;
        data_interface::m_subscriber = node_handle.subscribe("/imu/magnetometer", 100, &data_interface::subscriber, this);
        data_interface::m_data_timer.start();
        data_interface::f_subscriber_enabled = true;
    }
}
void data_interface::stop_subscriber()
{
    if(data_interface::f_subscriber_enabled)
    {
        data_interface::m_subscriber.shutdown();
        data_interface::m_data_timer.invalidate();
        data_interface::f_subscriber_enabled = false;
    }
}

// DATA FILE IO
bool data_interface::save_data(std::string& bag_file) const
{
    // Get remapped topic to write to.
    std::string topic = ros::names::resolve("/imu/magnetometer");

    // Write file.
    try
    {
        // Open the bag file for writing.
        rosbag::Bag bag(bag_file, rosbag::bagmode::Write);

        // Iterate over points.
        sensor_msgs_ext::magnetometer message;
        for(uint32_t i = 0; i < data_interface::m_x.size(); ++i)
        {
            // Populate message.
            message.x = data_interface::m_x.at(i);
            message.y = data_interface::m_y.at(i);
            message.z = data_interface::m_z.at(i);

            // Write message to bag.
            bag.write(topic, ros::Time::now(), message);
        }

        // Close bag.
        bag.close();

        return true;
    }
    catch(std::exception& e)
    {
        ROS_ERROR_STREAM("error writing data to bag file (" << e.what() << ")");
        return false;
    }
}
bool data_interface::load_data(std::string& bag_file)
{
    // Clear existing data.
    data_interface::clear_data();

    // Get remapped topic to read from.
    std::string topic = ros::names::resolve("/imu/magnetometer");

    // Read file.
    try
    {
        // Open the bag file for reading.
        rosbag::Bag bag(bag_file, rosbag::bagmode::Read);

        // Get view to magnetometer topic.
        rosbag::View view(bag, rosbag::TopicQuery(topic));

        // Iterate through view.
        for(auto instance = view.begin(); instance != view.end(); ++instance)
        {
            // Instantiate message.
            auto message = instance->instantiate<sensor_msgs_ext::magnetometer>();

            // Read message.
            if(message)
            {
                data_interface::m_x.push_back(message->x);
                data_interface::m_y.push_back(message->x);
                data_interface::m_z.push_back(message->x);
            }
        }

        return true;
    }
    catch(std::exception& e)
    {
        ROS_ERROR_STREAM("error reading from bag file (" << e.what() << ")");
        return false;
    }
}

// DATA MANAGER
void data_interface::clear_data()
{
    // Clear data vectors.
    data_interface::m_x.clear();
    data_interface::m_y.clear();
    data_interface::m_z.clear();
}

// DATA ACCESS
uint32_t data_interface::n_points()
{
    return data_interface::m_x.size();
}
bool data_interface::get_point(uint32_t index, Eigen::Vector3d& point)
{
    if(index < data_interface::m_x.size())
    {
        point(0) = data_interface::m_x.at(index);
        point(1) = data_interface::m_y.at(index);
        point(2) = data_interface::m_z.at(index);

        return true;
    }
    else
    {
        return false;
    }
}
bool data_interface::get_point(uint32_t index, QVector3D& point)
{
    if(index < data_interface::m_x.size())
    {
        point.setX(data_interface::m_x.at(index));
        point.setY(data_interface::m_y.at(index));
        point.setZ(data_interface::m_z.at(index));

        return true;
    }
    else
    {
        return false;
    }
}

// DATA SUBSCRIBER
void data_interface::subscriber(const sensor_msgs_ext::magnetometerConstPtr& message)
{
    // Enforce max data rate.
    if(data_interface::m_data_timer.elapsed() >= 1000.0/data_interface::p_max_data_rate)
    {
        // Reset data timer.
        data_interface::m_data_timer.restart();

        // Capture point.
        data_interface::m_x.push_back(message->x);
        data_interface::m_y.push_back(message->y);
        data_interface::m_z.push_back(message->z);

        // Raise signal.
        emit data_interface::point_received();
    }
}
