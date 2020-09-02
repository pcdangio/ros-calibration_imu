#include "magnetometer.h"

magnetometer::magnetometer()
{
    // Initialize flags.
    magnetometer::f_enabled = false;

    // Read parameters.
    ros::NodeHandle private_handle("~");
    // Force cell count to be even.
    uint32_t p_cell_count = std::floor(private_handle.param<double>("magnetometer_cell_count", 20) / 2.0) * 2.0;
    magnetometer::p_cell_pitch = 2*M_PI / static_cast<double>(p_cell_count);
    magnetometer::p_cell_population = private_handle.param<int32_t>("magnetometer_cell_population", 3);

    // Set up cell 3D vector.
    magnetometer::m_cells.resize(p_cell_count);
    for(uint32_t i = 0; i < p_cell_count; ++i)
    {
        magnetometer::m_cells[i].resize(p_cell_count);
        for(uint32_t j = 0; j < p_cell_count; ++j)
        {
            magnetometer::m_cells[i][j].resize(p_cell_count, 0);
        }
    }
}
magnetometer::~magnetometer()
{
    // Disable the calibrator.
    magnetometer::disable();
}

void magnetometer::enable()
{
    if(!magnetometer::f_enabled)
    {
        // Subscribe to magnetometer data stream.
        ros::NodeHandle public_handle;
        magnetometer::m_subscriber = public_handle.subscribe("/imu/magnetometer", 100, &magnetometer::subscriber, this);

        // Update flag.
        magnetometer::f_enabled = true;
    }
}
void magnetometer::disable()
{
    if(magnetometer::f_enabled)
    {
        // Close down subscriber.
        magnetometer::m_subscriber.shutdown();

        // Update flag.
        magnetometer::f_enabled = false;
    }
}

// SUBSCRIBERS
void magnetometer::subscriber(const sensor_msgs_ext::magnetometerConstPtr &message)
{
    // Convert message into new point instance.
    point_t* point = new point_t();
    point->x = message->x;
    point->y = message->y;
    point->z = message->z;

    // Add point to calibration.
    // NOTE: add_point will take ownership of the pointer.
    magnetometer::add_point(point);
}

void magnetometer::add_point(point_t *point)
{
    // Before discriminating against the point via cell, check if it causes a center update.
    if(point->x < magnetometer::m_x_center.min)

    // Try to add point to cell count.
    if(magnetometer::populate_cell(point))
    {
        // Point counted in cell.  Store in point deque.
        magnetometer::m_points.push_back(point);

        // Update center point if necessary.
        magnetometer::update_center(point);
    }
    else
    {
        // Cell is already full. Discard point.
        delete point;
    }
}
bool magnetometer::populate_cell(const point_t *point)
{

}
