#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>
#include <QtCharts/QScatterSeries>
#include <QElapsedTimer>

#include <ros/ros.h>

#include <sensor_msgs_ext/magnetometer.h>

#include <map>
#include <deque>

class magnetometer
{
public:
    magnetometer();
    ~magnetometer();

    enum class chart_t
    {
        XY = 0,
        XZ = 1,
        YZ = 2,
    };
    QtCharts::QChart* get_chart(chart_t chart);

    void start_collection();
    void stop_collection();


private:
    double p_max_data_rate;

    ros::Subscriber m_subscriber;
    void subscriber(const sensor_msgs_ext::magnetometerConstPtr& message);

    enum class state_t
    {
        IDLE = 0,
        STEP_COLLECT = 1
    };
    state_t m_state;

    struct point_t
    {
        double x;
        double y;
        double z;
    };
    std::deque<point_t*> m_points;
    QElapsedTimer m_point_timer;

    std::map<chart_t, QtCharts::QChart*> m_charts;
    std::map<chart_t, QtCharts::QValueAxis*> m_axes_x;
    std::map<chart_t, QtCharts::QValueAxis*> m_axes_y;
    std::map<chart_t, QtCharts::QScatterSeries*> m_series_collections;
    std::map<chart_t, QtCharts::QScatterSeries*> m_series_current_position;
    void initialize_charts();
    void clear_charts();
    void update_charts();
};

#endif // MAGNETOMETER_H
