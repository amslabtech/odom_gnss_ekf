#include "odom_gnss_ekf/odom_gnss_ekf.h"

EKF::EKF(void)
{
    sigma_a = 0.01;

    h << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0;

    r << 1e0, 0.0,
         0.0, 1e0;
}

Eigen::Vector4d EKF::get_next_state(const Eigen::Vector4d& x, const Eigen::Vector2d& u, double dt)
{
    Eigen::Matrix4d f;
    f << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 0.0;

    Eigen::Matrix<double, 4, 2> b;
    b << dt * cos(x(2)), 0.0,
         dt * sin(x(2)), 0.0,
         0.0, dt,
         1.0, 0.0;
    return f * x + b * u;
}

Eigen::Vector2d EKF::get_observation(const Eigen::Vector4d& x)
{
    Eigen::Vector2d z = h * x;
    return z;
}

Eigen::Matrix4d EKF::get_f(double dt)
{
    Eigen::Matrix4d f;
    f << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 0.0;
    return f;
}

Eigen::Matrix4d EKF::get_q(double dt)
{
    Eigen::Matrix<double, 4, 2> g;
    g <<  dt, 0.0,
          dt, 0.0,
         0.0,  dt,
         1.0, 0.0;
    Eigen::Matrix4d q = sigma_a * sigma_a * g * g.transpose();
    return q;
}

Eigen::Matrix4d EKF::get_jacobian_f(const Eigen::Vector4d& x, const Eigen::Vector2d& u, double dt)
{
    Eigen::Matrix4d jf;
    jf << 1.0, 0.0, -dt * u(0) * sin(x(2)), dt * cos(x(2)),
          0.0, 1.0,  dt * u(0) * cos(x(2)), dt * sin(x(2)),
          0.0, 0.0,                    1.0,            0.0,
          0.0, 0.0,                    0.0,            1.0;
    return jf;
}

OdomGNSSEKF::OdomGNSSEKF(void)
:local_nh("~"), gnss_odom_sub(nh, "/odom/gps", 10), fix_sub(nh, "/fix", 10)
, sync_subs(sync_policy(10), gnss_odom_sub, fix_sub)
{
    odom_sub = nh.subscribe("/odom/complement", 1, &OdomGNSSEKF::odom_callback, this);
    map_sub = nh.subscribe("/node_edge_map/map", 1, &OdomGNSSEKF::map_callback, this);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom/gnss_ekf", 1);

    sync_subs.registerCallback(boost::bind(&OdomGNSSEKF::gnss_callback, this, _1, _2));

    local_nh.param("INIT_X", INIT_X, {0.0});
    local_nh.param("INIT_Y", INIT_Y, {0.0});
    local_nh.param("INIT_YAW", INIT_YAW, {0.0});
    local_nh.param("ROBOT_FRAME", ROBOT_FRAME, {"base_link"});
    local_nh.param("ENABLE_FLOAT", ENABLE_FLOAT, {false});

    X << INIT_X, INIT_Y, INIT_YAW, 0;
    P << 1e1, 0.0, 0.0, 0.0,
         0.0, 1e1, 0.0, 0.0,
         0.0, 0.0, 1e0, 0.0,
         0.0, 0.0, 0.0, 1e0;

    map_subscribed = false;
    gnss_odom_updated = false;
    map_frame = "map";

    std::cout << "=== odom_gnss_ekf ===" << std::endl;
    std::cout << "INIT_X: " << INIT_X << std::endl;
    std::cout << "INIT_Y: " << INIT_Y << std::endl;
    std::cout << "INIT_YAW: " << INIT_YAW << std::endl;
    std::cout << "ROBOT_FRAME: " << ROBOT_FRAME << std::endl;
    std::cout << "ENABLE_FLOAT: " << ENABLE_FLOAT << std::endl;
}

void OdomGNSSEKF::process(void)
{
    ros::spin();
}

void OdomGNSSEKF::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    if(!map_subscribed){
        std::cout << "\033[Awaiting for map" << std::endl;
        return;
    }
    std::cout << "=== odom_callback ===" << std::endl;
    static bool first_callback = true;
    static Eigen::Vector3d first_odom = Eigen::Vector3d::Zero();
    static double first_yaw = 0;
    static double last_time = ros::Time::now().toSec();
    static Eigen::Vector3d last_odom = Eigen::Vector3d::Zero();
    static double last_yaw = 0;

    std::cout << "first_odom:\n" << first_odom << std::endl;
    std::cout << "first_yaw:\n" << first_yaw << std::endl;

    wheel_odom = *msg;
    double current_yaw = tf::getYaw(wheel_odom.pose.pose.orientation);
    current_yaw -= first_yaw;
    current_yaw = atan2(sin(current_yaw), cos(current_yaw));

    Eigen::Vector3d current_odom(wheel_odom.pose.pose.position.x, wheel_odom.pose.pose.position.y, 0);
    current_odom -= first_odom;
    Eigen::AngleAxis<double> first_yaw_rotation(-first_yaw, Eigen::Vector3d::UnitZ());
    current_odom = first_yaw_rotation * current_odom;

    if(!first_callback){
        double current_time = wheel_odom.header.stamp.toSec();
        double dt = current_time - last_time;
        double current_v = (current_odom - last_odom).norm() / dt;
        double current_omega = (current_yaw - last_yaw) / dt;
        Eigen::Vector2d u(current_v, current_omega);
        std::cout << "before prediction" << std::endl;
        std::cout << "X:\n" << X << std::endl;
        std::cout << "P:\n" << P << std::endl;
        std::cout << "u: " << u.transpose() << std::endl;
        std::cout << "dt: " << dt << std::endl;
        predict(X, P, u, dt, X, P);
        std::cout << "after prediction" << std::endl;
        std::cout << "X:\n" << X << std::endl;
        std::cout << "P:\n" << P << std::endl;

        if(gnss_odom_updated){
            Eigen::Vector2d z(gnss_odom.pose.pose.position.x, gnss_odom.pose.pose.position.y);
            std::cout << "update" << std::endl;
            std::cout << "z: " << z.transpose() << std::endl;
            std::cout << "R:\n" << ekf.r << std::endl;
            update(X, P, z, dt, X, P);
            std::cout << "after update" << std::endl;
            std::cout << "X:\n" << X << std::endl;
            std::cout << "P:\n" << P << std::endl;
        }else{
            std::cout << "\033[031mupdate process is skipped because gnss data is not updated\033[0m" << std::endl;
        }

        nav_msgs::Odometry ekf_odom;
        ekf_odom.header.frame_id = map_frame;
        ekf_odom.header.stamp = msg->header.stamp;
        ekf_odom.child_frame_id = ROBOT_FRAME;
        ekf_odom.pose.pose.position.x = X(0);
        ekf_odom.pose.pose.position.y = X(1);
        ekf_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(X(2));
        for(int i=0;i<3;i++){
            if(i != 2){
                ekf_odom.pose.covariance[0 + i * 6] = P(i, 0);
                ekf_odom.pose.covariance[1 + i * 6] = P(i, 1);
                ekf_odom.pose.covariance[5 + i * 6] = P(i, 2);
            }else{
                ekf_odom.pose.covariance[0 + 5 * 6] = P(i, 0);
                ekf_odom.pose.covariance[1 + 5 * 6] = P(i, 1);
                ekf_odom.pose.covariance[5 + 5 * 6] = P(i, 2);
            }
        }
        ekf_odom.twist.twist.linear.x = X(3);
        ekf_odom.twist.twist.angular.z = current_omega;
        odom_pub.publish(ekf_odom);

        last_time = current_time;
        last_odom = current_odom;
        last_yaw = current_yaw;
    }else{
        last_time = wheel_odom.header.stamp.toSec();
        first_odom = current_odom;
        last_odom = current_odom - first_odom;
        first_yaw = current_yaw;
        last_yaw = current_yaw - first_yaw;
        first_callback = false;
    }
    gnss_odom_updated = false;
    std::cout << std::endl;
}

void OdomGNSSEKF::gnss_callback(const nav_msgs::OdometryConstPtr& msg_odom, const sensor_msgs::NavSatFixConstPtr& msg_fix)
{
    if(map_subscribed){
        std::cout << "=== gnss_callback ===" << std::endl;
        if((msg_fix->status.status != sensor_msgs::NavSatStatus::STATUS_NO_FIX) || ENABLE_FLOAT){
            gnss_odom = *msg_odom;
            gnss_odom.pose.pose.position.x -= origin.x;
            gnss_odom.pose.pose.position.y -= origin.y;
            gnss_odom.pose.pose.position.z = 0;
            ekf.r << gnss_odom.pose.covariance[0], 0.0,
                     0.0, gnss_odom.pose.covariance[7];
            gnss_odom_updated = true;
            std::cout << "gnss data is updated" << std::endl;
        }else{
            std::cout << "\033[031m! float !\033[0m" << std::endl;
        }
    }
}

void OdomGNSSEKF::map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr& msg)
{
    origin = msg->origin_utm;
    map_frame = msg->header.frame_id;
    map_subscribed = true;
}

void OdomGNSSEKF::predict(const Eigen::Vector4d& x, const Eigen::Matrix4d& p, const Eigen::Vector2d& u, double dt, Eigen::Vector4d& output_x, Eigen::Matrix4d& output_p)
{
    Eigen::Vector4d _x = ekf.get_next_state(x, u, dt);
    Eigen::Matrix4d jf = ekf.get_jacobian_f(x, u, dt);
    Eigen::Matrix4d _p = jf * p * jf.transpose() + ekf.get_q(dt);
    output_x = _x;
    output_p = _p;
    output_x(2) = atan2(sin(output_x(2)), cos(output_x(2)));
}

void OdomGNSSEKF::update(const Eigen::Vector4d& x, const Eigen::Matrix4d& p, const Eigen::Vector2d& z, double dt, Eigen::Vector4d& output_x, Eigen::Matrix4d& output_p)
{
    Eigen::Vector2d z_ = ekf.get_observation(x);
    Eigen::Vector2d e = z - z_;
    Eigen::Matrix2d s = ekf.h * p * ekf.h.transpose() + ekf.r;
    Eigen::Matrix<double, 4, 2> k = p * ekf.h.transpose() * s.inverse();
    output_x = x + k * e;
    output_x(2) = atan2(sin(output_x(2)), cos(output_x(2)));
    output_p = (Eigen::Matrix4d::Identity() - k * ekf.h) * p;
}
