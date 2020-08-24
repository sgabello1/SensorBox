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
    ERR_MARK_SHOW(ros::NodeHandle nh) : nh_(nh)
    {
        slamTec_pub_ = nh_.advertise<visualization_msgs::Marker>("slamtec_mark", 1);
        vo_pub_ = nh_.advertise<visualization_msgs::Marker>("vo_mark", 1);
        vo_err_sub_ = nh_.subscribe("/vo_err", 1, &ERR_MARK_SHOW::errCb, this);

        lastSlamtecPosition_.resize(2);
        MarkInit();
    }

    void errCb(std_msgs::Float32MultiArray msg)
    {
        voStamped_.transform.translation.x = msg.data[4];
        voStamped_.transform.translation.y = msg.data[5];
        voStamped_.transform.translation.z = msg.data[6];
        slamTecStamped_.transform.translation.x = msg.data[7];
        slamTecStamped_.transform.translation.y = msg.data[8];
        slamTecStamped_.transform.translation.z = msg.data[9];
    }

    void MarkInit()
    {
        slamtec.ns  = "slamtec_body";
        slamtec.action = visualization_msgs::Marker::ADD;
        slamtec.type = visualization_msgs::Marker::CYLINDER;
        slamtec.scale.x = 0.6;
        slamtec.scale.y = 0.6;
        slamtec.scale.z = 0.6;
        slamtec.color.r = 1.0;
        slamtec.color.g = 1.0;
        slamtec.color.b = 1.0;
        slamtec.color.a = 1.0;

        slamtecRTSTraj_.ns = "slamtec_traj";
        slamtecRTSTraj_.action = visualization_msgs::Marker::ADD;
        slamtecRTSTraj_.type = visualization_msgs::Marker::LINE_STRIP;
        slamtecRTSTraj_.scale.x = 0.1;
        slamtecRTSTraj_.scale.y = 0.1;
        slamtecRTSTraj_.scale.z = 0.1;
        slamtecRTSTraj_.color.g = 0.3;
        slamtecRTSTraj_.color.a = 1.0;


        voTraj_.ns = "vo";
        voTraj_.action = visualization_msgs::Marker::ADD;
        voTraj_.type = visualization_msgs::Marker::LINE_STRIP;
        voTraj_.scale.x = 0.05;
        voTraj_.scale.y = 0.05;
        voTraj_.scale.z = 0.05;
        voTraj_.color.b = 0.62;
        voTraj_.color.g = 0.28;
        voTraj_.color.r = 0.43;
        voTraj_.color.a = 1.0;

    }

    void pubVOMark()
    {
        pubSlamtec();
        pubSlamTecTraj();
        pubVoTraj();
    }

    void pubSlamtec()
    {
        slamtec.header.frame_id = "world";
        slamtec.header.stamp = ros::Time::now();
        // slamtec.pose.position.x = slamTecStamped_.transform.translation.x;
        // slamtec.pose.position.y = slamTecStamped_.transform.translation.y;
        // slamtec.pose.position.z = slamTecStamped_.transform.translation.z;
        slamtec.pose.position.x = voStamped_.transform.translation.x;
        slamtec.pose.position.y = voStamped_.transform.translation.y;
        slamtec.pose.position.z = voStamped_.transform.translation.z;
        slamtec.pose.orientation.w = 1.0;
        slamTec_pub_.publish(slamtec);
    }

    void pubSlamTecTraj()
    {
        slamtecRTSTraj_.header.frame_id = "world";
        slamtecRTSTraj_.header.stamp = ros::Time::now();
        geometry_msgs::Point p;
        p.x = slamTecStamped_.transform.translation.x;
        p.y = slamTecStamped_.transform.translation.y;
	p.z = slamTecStamped_.transform.translation.z;
        if (abs(p.x-lastSlamtecPosition_[0])>0.2 || abs(p.y-lastSlamtecPosition_[1])>0.2)
            return;
        lastSlamtecPosition_[0] = p.x;
        lastSlamtecPosition_[1] = p.y;
        slamtecRTSTraj_.points.push_back(p);
        if (slamtecRTSTraj_.points.size() > 900)
            slamtecRTSTraj_.points.erase(slamtecRTSTraj_.points.begin());
        slamTec_pub_.publish(slamtecRTSTraj_);
    }


    void pubVoTraj()
    {
        voTraj_.header.frame_id = "world";
        voTraj_.header.stamp = ros::Time::now();
        geometry_msgs::Point p;
        p.x = voStamped_.transform.translation.x;
        p.y = voStamped_.transform.translation.y;
        p.z = voStamped_.transform.translation.z;
        voTraj_.points.push_back(p);
        if (voTraj_.points.size() > 900)
            voTraj_.points.erase(voTraj_.points.begin());
        vo_pub_.publish(voTraj_);
    }




private:
    ros::NodeHandle nh_;
    ros::Publisher slamTec_pub_;
    ros::Publisher vo_pub_;
    ros::Subscriber vo_err_sub_;


    // rts location
    geometry_msgs::TransformStamped slamTecStamped_;
    // vo location
    geometry_msgs::TransformStamped voStamped_;

    std::vector<double> lastSlamtecPosition_;
    visualization_msgs::Marker slamtec;
    visualization_msgs::Marker slamtecRTSTraj_;
    visualization_msgs::Marker voTraj_;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vo_show");
    ros::NodeHandle nh;
    ERR_MARK_SHOW err_mark_show(nh);
    ros::Rate rate(10);
    while (ros::ok())
    {
        err_mark_show.pubVOMark();
        ros::spinOnce();
        rate.sleep();
    }
}
