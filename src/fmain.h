#ifndef FMAIN_H
#define FMAIN_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class fmain; }
QT_END_NAMESPACE

#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>
#include <QtCharts/QScatterSeries>
#include <QTimer>
#include <QElapsedTimer>

#include <ros/ros.h>
#include <sensor_msgs_ext/magnetometer.h>
#include <qn_optimizer/qn_optimizer.h>

#include <map>
#include <deque>

class fmain : public QMainWindow
{
    Q_OBJECT

public:
    fmain(QWidget *parent = nullptr);
    ~fmain();

private slots:


    void on_button_start_collection_clicked();

    void on_button_stop_collection_clicked();

    void on_combobox_charts_currentIndexChanged(int index);

    void on_button_clear_collection_clicked();

    void on_button_start_fit_clicked();

private:
    Ui::fmain *ui;

    ros::NodeHandle* m_node;
    QTimer m_ros_spinner;
    void ros_spin();

    enum class chart_t
    {
        XY = 0,
        XZ = 1,
        YZ = 2,
    };

    void start_collection();
    void stop_collection();
    void clear_collection();

    void start_fit();
    void stop_fit();

    double p_max_data_rate;

    ros::Subscriber m_subscriber;
    void subscriber(const sensor_msgs_ext::magnetometerConstPtr& message);

    enum class state_t
    {
        IDLE = 0,
        COLLECTION = 1,
        FIT = 2,
        TRANSFORM = 3
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

    qn_optimizer* m_optimizer_fit;
    double objective_fit(const Eigen::VectorXd& variables);
    void gradient_fit(const Eigen::VectorXd& operating_point, Eigen::VectorXd& gradient);
};
#endif // FMAIN_H
