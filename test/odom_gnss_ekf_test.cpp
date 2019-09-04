#include <gtest/gtest.h>

#include <ros/ros.h>

#include "odom_gnss_ekf/odom_gnss_ekf.h"

TEST(TestSuite, test0)
{

}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "odom_gnss_ekf_test");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Duration(3.0).sleep();

    int r_e_t = RUN_ALL_TESTS();

    spinner.stop();

    ros::shutdown();

    return r_e_t;
}
