#ifndef __ODOM_GNSS_EKF_H
#define __ODOM_GNSS_EKF_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Dense>

#include "amsl_navigation_msgs/NodeEdgeMap.h"

class EKF
{
public:
    EKF(void);

    // state vector x {x, y, \theta, v}
    Eigen::Vector4d get_next_state(const Eigen::Vector4d&, const Eigen::Vector2d&, double);
    Eigen::Vector2d get_observation(const Eigen::Vector4d&);
    Eigen::Matrix4d get_f(double);// transition model
    Eigen::Matrix4d get_q(double);// transition noise
    Eigen::Matrix4d get_jacobian_f(const Eigen::Vector4d&, const Eigen::Vector2d&, double);

    double sigma_a;
    Eigen::Matrix<double, 2, 4> h;
    Eigen::Matrix2d r;
private:

};

class OdomGNSSEKF
{
public:
    OdomGNSSEKF(void);

    void process(void);
    void odom_callback(const nav_msgs::OdometryConstPtr&);
    void gnss_callback(const nav_msgs::OdometryConstPtr&, const sensor_msgs::NavSatFixConstPtr&);
    void map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr&);

    void predict(const Eigen::Vector4d&, const Eigen::Matrix4d&, const Eigen::Vector2d&, double, Eigen::Vector4d&, Eigen::Matrix4d&);
    void update(const Eigen::Vector4d&, const Eigen::Matrix4d&, const Eigen::Vector2d&, double, Eigen::Vector4d&, Eigen::Matrix4d&);

private:
    double INIT_X;
    double INIT_Y;
    double INIT_YAW;
    std::string ROBOT_FRAME;
    bool ENABLE_FLOAT;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;
    ros::Subscriber odom_sub;
    ros::Subscriber map_sub;
    ros::Publisher odom_pub;
    message_filters::Subscriber<nav_msgs::Odometry> gnss_odom_sub;
    message_filters::Subscriber<sensor_msgs::NavSatFix> fix_sub;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::NavSatFix> sync_policy;
    message_filters::Synchronizer<sync_policy> sync_subs;

    nav_msgs::Odometry wheel_odom;
    nav_msgs::Odometry gnss_odom;
    geometry_msgs::Point origin;
    EKF ekf;
    std::string map_frame;

    Eigen::Vector4d X;
    Eigen::Matrix4d P;
    bool map_subscribed;
    bool gnss_odom_updated;
};

#endif// __ODOM_GNSS_EKF_H
