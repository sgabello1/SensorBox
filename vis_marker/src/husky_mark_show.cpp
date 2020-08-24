//
// Created by wang on 18-9-27.
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
        husky_pub_ = nh_.advertise<visualization_msgs::Marker>("husky_mark", 1);
        amcl_pub_ = nh_.advertise<visualization_msgs::Marker> ("amcl_mark", 1);
        husky_err_sub_ = nh_.subscribe("/husky_err", 1, &ERR_MARK_SHOW::errCb, this);

        MarkInit();
    }

    void errCb(std_msgs::Float32MultiArray msg)
    {
        huskyStamped_.transform.translation.x = msg.data[3];
        huskyStamped_.transform.translation.y = msg.data[4];
        huskyStamped_.transform.rotation.x = msg.data[5];
        huskyStamped_.transform.rotation.y = msg.data[6];
        huskyStamped_.transform.rotation.z = msg.data[7];
        huskyStamped_.transform.rotation.w = msg.data[8];
        huskyRTSStamped_.transform.translation.x = msg.data[9];
        huskyRTSStamped_.transform.translation.y = msg.data[10];
    }

    void MarkInit()
    {
        husky.ns  = "husky_body";
        husky.action = visualization_msgs::Marker::ADD;
        husky.type = visualization_msgs::Marker::CUBE;
        husky.scale.x = 0.6;
        husky.scale.y = 0.6;
        husky.scale.z = 0.6;
        husky.color.r = 0.97;
        husky.color.g = 0.8;
        husky.color.b = 0.0;
        husky.color.a = 1.0;


        huskyRTSTraj_.ns = "husky_traj";
        huskyRTSTraj_.action = visualization_msgs::Marker::ADD;
        huskyRTSTraj_.type = visualization_msgs::Marker::LINE_STRIP;
        huskyRTSTraj_.scale.x = 0.1;
        huskyRTSTraj_.scale.y = 0.1;
        huskyRTSTraj_.scale.z = 0.1;
        huskyRTSTraj_.color.g = 0.3;
        huskyRTSTraj_.color.a = 1.0;


        amclTraj_.ns = "amcl";
        amclTraj_.action = visualization_msgs::Marker::ADD;
        amclTraj_.type = visualization_msgs::Marker::LINE_STRIP;
        amclTraj_.scale.x = 0.05;
        amclTraj_.scale.y = 0.05;
        amclTraj_.scale.z = 0.05;
        amclTraj_.color.r = 0.16;
        amclTraj_.color.g = 0.4;
        amclTraj_.color.b = 0.47;
        amclTraj_.color.a = 1.0;
    }

    void pubHuskyMark()
    {
        pubHusky();
        pubHuskyTraj();
        pubAmclTraj();
    }

    void pubHusky()
    {
        husky.header.frame_id = "world";
        husky.header.stamp = ros::Time::now();
        husky.pose.position.x = huskyStamped_.transform.translation.x;
        husky.pose.position.y = huskyStamped_.transform.translation.y;
        husky.pose.position.z = huskyStamped_.transform.translation.z;
        husky.pose.orientation.x = huskyStamped_.transform.rotation.x;
        husky.pose.orientation.y = huskyStamped_.transform.rotation.y;
        husky.pose.orientation.z = huskyStamped_.transform.rotation.z;
        husky.pose.orientation.w = huskyStamped_.transform.rotation.w;
        husky_pub_.publish(husky);
    }


    void pubHuskyTraj()
    {
        huskyRTSTraj_.header.frame_id = "world";
        huskyRTSTraj_.header.stamp = ros::Time::now();
        geometry_msgs::Point p;
        p.x = huskyRTSStamped_.transform.translation.x;
        p.y = huskyRTSStamped_.transform.translation.y;
        huskyRTSTraj_.points.push_back(p);
        if (huskyRTSTraj_.points.size() > 300)
            huskyRTSTraj_.points.erase(huskyRTSTraj_.points.begin());
        husky_pub_.publish(huskyRTSTraj_);
    }


    void pubAmclTraj()
    {
        amclTraj_.header.frame_id = "world";
        amclTraj_.header.stamp = ros::Time::now();
        geometry_msgs::Point p;
        p.x = huskyStamped_.transform.translation.x;
        p.y = huskyStamped_.transform.translation.y;
        amclTraj_.points.push_back(p);
        if (amclTraj_.points.size() > 300)
            amclTraj_.points.erase(amclTraj_.points.begin());
        amcl_pub_.publish(amclTraj_);
    }



private:
    ros::NodeHandle nh_;
    ros::Publisher husky_pub_;
    // husky amcl publisher
    ros::Publisher amcl_pub_;
    ros::Publisher err_pub_;
    ros::Subscriber husky_err_sub_;

    // husky amcl location
    geometry_msgs::TransformStamped huskyStamped_;
    // rts location
    geometry_msgs::TransformStamped huskyRTSStamped_;
    // vo location

    visualization_msgs::Marker husky;
    visualization_msgs::Marker huskyRTSTraj_;
    visualization_msgs::Marker amclTraj_;

    double husky_dist_;
    float husky_amcl_err_;
    float husky_vx_;
    float husky_vth_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "husky_show");
    ros::NodeHandle nh;
    ERR_MARK_SHOW err_mark_show(nh);
    ros::Rate rate(10);
    while (ros::ok())
    {
        err_mark_show.pubHuskyMark();
        ros::spinOnce();
        rate.sleep();
    }
}
