#include "data_manager.h"

using namespace accelerometer;

// CONSTRUCTORS
data_manager::data_manager(std::shared_ptr<ros::NodeHandle> node)
{
    // Store nodehandle.
    data_manager::m_node = node;

    // Read parameters.
    ros::NodeHandle private_node("~");
    data_manager::p_buffer_size = private_node.param<int32_t>("sample_size", 100);

    // Initialize matrix instances.
    data_manager::m_buffer_average.setZero();
    data_manager::m_data.setZero();

    // Initialize flags.
    data_manager::f_is_collecting = false;
}
data_manager::~data_manager()
{
    // Stop collection if it's ongoing.
    stop_collection();
}

// COLLECTION
void data_manager::start_collection()
{
    if(!data_manager::f_is_collecting)
    {
        // Clear the buffer.
        data_manager::m_buffer.clear();

        // Get remapped topic name.
        std::string topic_name = ros::names::resolve("imu/accelerometer");

        // Start subscriber.
        data_manager::m_subscriber = data_manager::m_node->subscribe(topic_name, 100, &data_manager::subscriber, this);
    }
}
void data_manager::stop_collection()
{
    if(data_manager::f_is_collecting)
    {
        data_manager::m_subscriber.shutdown();
    }
}
bool data_manager::grab(orientation_t orientation)
{
    // Check that collection is running.
    if(!data_manager::f_is_collecting)
    {
        return false;
    }

    // Check that buffer is full.
    if(data_manager::m_buffer.size() < data_manager::p_buffer_size)
    {
        return false;
    }

    // Store the current average into the data matrix.
    data_manager::m_data.block(0, static_cast<uint32_t>(orientation), 3, 1) = data_manager::m_buffer_average;

    return true;
}

// DATASET
bool data_manager::dataset_complete() const
{
    // Check that each column of the data matrix is nonzero.
    for(uint32_t j = 0; j < data_manager::m_data.cols(); ++j)
    {
        if(data_manager::m_data.col(j).isZero())
        {
            return false;
        }
    }

    // If this point reached, all columns have been verified as nonzero.
    return true;
}
void data_manager::get_dataset(Eigen::Matrix<double, 3, 6>& data) const
{
    data = data_manager::m_data;
}

// SUBSCRIBER
void data_manager::subscriber(const sensor_msgs_ext::accelerationConstPtr& message)
{
    // Check if collecting; subscriber may have been shut down but still going through queue.
    if(data_manager::f_is_collecting)
    {
        // Add point to FIFO buffer.
        if(data_manager::m_buffer.size() == data_manager::p_buffer_size)
        {
            data_manager::m_buffer.pop_front();
        }
        data_manager::m_buffer.push_back({message->x, message->y, message->z});

        // Check if buffer full.
        if(data_manager::m_buffer.size() == data_manager::p_buffer_size)
        {
            // Calculate average.
            data_manager::m_buffer_average.setZero();
            for(auto point = data_manager::m_buffer.cbegin(); point != data_manager::m_buffer.cend(); ++point)
            {
                data_manager::m_buffer_average += *point;
            }
            data_manager::m_buffer_average /= static_cast<double>(data_manager::p_buffer_size);

            // Signal new measurement.
            emit data_manager::new_measurement(data_manager::m_buffer_average);
        }
    }
}
