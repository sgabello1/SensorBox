//
// Created by wang on 18-12-12.
//

#include <ros/ros.h>
#include <ros/time.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <cmath>


class ERR_MARK_SHOW
{
public:
    ERR_MARK_SHOW(ros::NodeHandle nh) : nh_(nh), husky_dist_(0)
    {
        tfBuffer_ = new tf2_ros::Buffer;
        tfListener_ = new tf2_ros::TransformListener(*tfBuffer_);

        err_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("err_calc", 2);
        odom_sub_ = nh_.subscribe("/odometry/filtered", 5, &ERR_MARK_SHOW::odomCb, this);

    }

    void listenTF()
    {
        try {
            huskyRTSStamped_ = tfBuffer_->lookupTransform("world", "husky", ros::Time(0), ros::Duration(1.0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }
        try {
            huskyStamped_ = tfBuffer_->lookupTransform("world", "base_link", ros::Time(0), ros::Duration(1.0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }
    }


    void calcPubErr()
    {
        std_msgs::Float32MultiArray arr;

        float base_link_x = huskyStamped_.transform.translation.x;
        float base_link_y = huskyStamped_.transform.translation.y;
        float husky_RTS_x = huskyRTSStamped_.transform.translation.x;
        float husky_RTS_y = huskyRTSStamped_.transform.translation.y;
        husky_amcl_err_ = sqrt((base_link_x-husky_RTS_x)*(base_link_x-husky_RTS_x) + (base_link_y-husky_RTS_y)*(base_link_y-husky_RTS_y));

        arr.data.clear();
        arr.data.push_back(husky_amcl_err_);
        arr.data.push_back(husky_vx_);
        arr.data.push_back(husky_vth_);
        arr.data.push_back(huskyStamped_.transform.translation.x);
        arr.data.push_back(huskyStamped_.transform.translation.y);
        arr.data.push_back(huskyStamped_.transform.rotation.x);
        arr.data.push_back(huskyStamped_.transform.rotation.y);
        arr.data.push_back(huskyStamped_.transform.rotation.z);
        arr.data.push_back(huskyStamped_.transform.rotation.w);
        arr.data.push_back(huskyRTSStamped_.transform.translation.x);
        arr.data.push_back(huskyRTSStamped_.transform.translation.y);
        err_pub_.publish(arr);
    }

    void odomCb(nav_msgs::Odometry msg)
    {
        husky_vx_ = msg.twist.twist.linear.x;
        husky_vth_ = msg.twist.twist.angular.z;
    }


private:
    ros::NodeHandle nh_;
    ros::Publisher err_pub_;
    ros::Subscriber odom_sub_;
    tf2_ros::Buffer* tfBuffer_;
    tf2_ros::TransformListener* tfListener_;

    // husky amcl location
    geometry_msgs::TransformStamped huskyStamped_;
    // rts location
    geometry_msgs::TransformStamped huskyRTSStamped_;

    std::vector<double> lastHuskyPosition_;

    double husky_dist_;

    float husky_amcl_err_;
    float husky_vx_;
    float husky_vth_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "husky_calc");
    ros::NodeHandle nh;
    ERR_MARK_SHOW err_mark_show(nh);
    ros::Rate rate(10);
    while (ros::ok())
    {
        err_mark_show.listenTF();
        err_mark_show.calcPubErr();
        ros::spinOnce();
        rate.sleep();
    }
}
