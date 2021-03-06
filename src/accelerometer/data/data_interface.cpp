#include "data_interface.h"

using namespace accelerometer;

// CONSTRUCTORS
data_interface::data_interface(std::shared_ptr<ros::NodeHandle> node)
{
    // Store nodehandle.
    data_interface::m_node = node;

    // Read parameters.
    ros::NodeHandle private_node("~");
    data_interface::p_buffer_size = private_node.param<int32_t>("sample_size", 100);

    // Initialize matrix instances.
    data_interface::m_buffer_average.setZero();
    data_interface::m_data.setZero();

    // Initialize flags.
    data_interface::f_is_collecting = false;
}
data_interface::~data_interface()
{
    // Stop collection if it's ongoing.
    stop_collection();
}

// COLLECTION
void data_interface::start_collection()
{
    if(!data_interface::f_is_collecting)
    {
        // Set flag.
        data_interface::f_is_collecting = true;

        // Clear the buffer.
        data_interface::m_buffer.clear();

        // Get remapped topic name.
        std::string topic_name = ros::names::resolve("imu/accelerometer");

        // Start subscriber.
        data_interface::m_subscriber = data_interface::m_node->subscribe(topic_name, 100, &data_interface::subscriber, this);
    }
}
void data_interface::stop_collection()
{
    if(data_interface::f_is_collecting)
    {
        data_interface::f_is_collecting = false;
        data_interface::m_subscriber.shutdown();
    }
}
bool data_interface::grab(orientation_t orientation)
{
    // Check that collection is running.
    if(!data_interface::f_is_collecting)
    {
        return false;
    }

    // Check that buffer is full.
    if(data_interface::m_buffer.size() < data_interface::p_buffer_size)
    {
        return false;
    }

    // Store the current average into the data matrix.
    data_interface::m_data.block(0, static_cast<uint32_t>(orientation), 3, 1) = data_interface::m_buffer_average;

    return true;
}

// DATASET
bool data_interface::dataset_complete() const
{
    // Check that each column of the data matrix is nonzero.
    for(uint32_t j = 0; j < data_interface::m_data.cols(); ++j)
    {
        if(data_interface::m_data.col(j).isZero())
        {
            return false;
        }
    }

    // If this point reached, all columns have been verified as nonzero.
    return true;
}
Eigen::Matrix<double, 3, 6> data_interface::get_dataset() const
{
    return data_interface::m_data;
}
void data_interface::clear_dataset()
{
    data_interface::m_data.setZero();
}

// SUBSCRIBER
void data_interface::subscriber(const sensor_msgs_ext::accelerometerConstPtr& message)
{
    // Check if collecting; subscriber may have been shut down but still going through queue.
    if(data_interface::f_is_collecting)
    {
        // Add point to FIFO buffer.
        if(data_interface::m_buffer.size() == data_interface::p_buffer_size)
        {
            data_interface::m_buffer.pop_front();
        }
        data_interface::m_buffer.push_back({message->x, message->y, message->z});

        // Check if buffer full.
        if(data_interface::m_buffer.size() == data_interface::p_buffer_size)
        {
            // Calculate average.
            data_interface::m_buffer_average.setZero();
            for(auto point = data_interface::m_buffer.cbegin(); point != data_interface::m_buffer.cend(); ++point)
            {
                data_interface::m_buffer_average += *point;
            }
            data_interface::m_buffer_average /= static_cast<double>(data_interface::p_buffer_size);

            // Signal new measurement.
            emit data_interface::new_measurement(data_interface::m_buffer_average);
        }
    }
}
