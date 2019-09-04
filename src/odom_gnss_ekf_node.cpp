#include "odom_gnss_ekf/odom_gnss_ekf.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "odom_gnss_ekf");
    OdomGNSSEKF odom_gnss_ekf;
    odom_gnss_ekf.process();
    return 0;
}
