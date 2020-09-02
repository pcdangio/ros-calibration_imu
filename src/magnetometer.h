#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include <ros/ros.h>

#include <sensor_msgs_ext/magnetometer.h>

#include <vector>
#include <deque>

class magnetometer
{
public:
    magnetometer();
    ~magnetometer();

    void enable();
    void disable();

private:
    ros::Subscriber m_subscriber;
    void subscriber(const sensor_msgs_ext::magnetometerConstPtr& message);

    bool f_enabled;

    struct point_t
    {
        double x;
        double y;
        double z;
    };
    std::deque<point_t*> m_points;
    void add_point(point_t* point);

    struct center_t
    {
        double min;
        double max;
        double center;
    };
    center_t m_x_center;
    center_t m_y_center;
    center_t m_z_center;
    void update_center(const point_t* point);

    double p_cell_pitch;
    uint8_t p_cell_population;
    std::vector<std::vector<std::vector<uint8_t>>> m_cells;
    bool populate_cell(const point_t* point);
};

#endif // MAGNETOMETER_H
