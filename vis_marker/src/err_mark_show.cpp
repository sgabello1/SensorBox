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
    ERR_MARK_SHOW(ros::NodeHandle nh) : nh_(nh), husky_dist_(0), vo_max_err_(0.0), vo_min_err_(10000.0), vo_mean_err_(0.0)
    {
        tfBuffer_ = new tf2_ros::Buffer;
        tfListener_ = new tf2_ros::TransformListener(*tfBuffer_);
        slamTec_pub_ = nh_.advertise<visualization_msgs::Marker>("slamtec_mark", 1);
        husky_pub_ = nh_.advertise<visualization_msgs::Marker>("husky_mark", 1);
        vo_pub_ = nh_.advertise<visualization_msgs::Marker>("vo_mark", 1);
        amcl_pub_ = nh_.advertise<visualization_msgs::Marker> ("amcl_mark", 1);

        err_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("eval_show", 2);
        odom_sub_ = nh_.subscribe("/odometry/filtered", 5, &ERR_MARK_SHOW::odomCb, this);
        lastHuskyPosition_.resize(2);
        lastSlamtecPosition_.resize(2);
        MarkInit();
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

        huskyRTSTraj_.ns = "husky_traj";
        huskyRTSTraj_.action = visualization_msgs::Marker::ADD;
        huskyRTSTraj_.type = visualization_msgs::Marker::LINE_STRIP;
        huskyRTSTraj_.scale.x = 0.1;
        huskyRTSTraj_.scale.y = 0.1;
        huskyRTSTraj_.scale.z = 0.1;
        huskyRTSTraj_.color.g = 0.3;
        huskyRTSTraj_.color.a = 1.0;

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

    void listenTF()
    {
        huskyRTSStamped_ = tfBuffer_->lookupTransform("world", "husky", ros::Time(0), ros::Duration(1.0));
        slamTecStamped_ = tfBuffer_->lookupTransform("world", "slamtec", ros::Time(0), ros::Duration(1.0));
        huskyStamped_ = tfBuffer_->lookupTransform("world", "base_link", ros::Time(0), ros::Duration(1.0));
        voStamped_ = tfBuffer_->lookupTransform("world", "vo", ros::Time(0), ros::Duration(1.0));
        pubSlamtec();
        pubSlamTecTraj();
        pubHuskyTraj();
        pubVoTraj();
        pubAmclTraj();
        calcDist();
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
        slamtecRTSTraj_.points.push_back(p);
        if (slamtecRTSTraj_.points.size() > 900)
            slamtecRTSTraj_.points.erase(slamtecRTSTraj_.points.begin());
        slamTec_pub_.publish(slamtecRTSTraj_);
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

    void calcPubErr()
    {
        std_msgs::Float32MultiArray arr;

        float base_link_x = huskyStamped_.transform.translation.x;
        float base_link_y = huskyStamped_.transform.translation.y;
        float husky_RTS_x = huskyRTSStamped_.transform.translation.x;
        float husky_RTS_y = huskyRTSStamped_.transform.translation.y;
        husky_amcl_err_ = sqrt((base_link_x-husky_RTS_x)*(base_link_x-husky_RTS_x) + (base_link_y-husky_RTS_y)*(base_link_y-husky_RTS_y));

        float slamtec_x = slamTecStamped_.transform.translation.x;
        float slamtec_y = slamTecStamped_.transform.translation.y;
        float slamtec_z = slamTecStamped_.transform.translation.z;

        if(voStamped_.transform.translation.x==0 && voStamped_.transform.translation.y==0 && voStamped_.transform.translation.z==0)
            vo_err_ = -1;
        else
        {
            vo_err_ = sqrt(pow((slamtec_x-voStamped_.transform.translation.x), 2)+
                           pow((slamtec_y- voStamped_.transform.translation.y), 2)+
                           pow((slamtec_z - voStamped_.transform.translation.z), 2));
        }
        if (vo_err_ > vo_max_err_)
            vo_max_err_ = vo_err_;
        if (vo_err_ < vo_min_err_)
            vo_min_err_ = vo_err_;

        vo_mean_err_ = vo_mean_err_*vo_err_count_/(vo_err_count_+1) + vo_err_/(vo_err_count_ + 1);
        vo_err_count_++;


        arr.data.clear();
        arr.data.push_back(husky_amcl_err_);
        arr.data.push_back(husky_amcl_err_/husky_dist_);
        arr.data.push_back(husky_vx_);
        arr.data.push_back(husky_vth_);
        arr.data.push_back(vo_err_);
        arr.data.push_back(vo_max_err_);
        arr.data.push_back(vo_min_err_);
        arr.data.push_back(vo_mean_err_);
        err_pub_.publish(arr);
    }

    void odomCb(nav_msgs::Odometry msg)
    {
        husky_vx_ = msg.twist.twist.linear.x;
        husky_vth_ = msg.twist.twist.angular.z;
    }

    void calcDist()
    {
        // husky
        if(lastHuskyPosition_[0] ==0 || lastHuskyPosition_[1] ==0)
        {
            lastHuskyPosition_[0] = huskyRTSStamped_.transform.translation.x;
            lastHuskyPosition_[1] = huskyRTSStamped_.transform.translation.y;
        }
        husky_dist_ += sqrt(pow((huskyRTSStamped_.transform.translation.x - lastHuskyPosition_[0]), 2) + pow((huskyRTSStamped_.transform.translation.y - lastHuskyPosition_[1]), 2));
        lastHuskyPosition_[0] = huskyRTSStamped_.transform.translation.x;
        lastHuskyPosition_[1] = huskyRTSStamped_.transform.translation.y;
        lastSlamtecPosition_[0] = slamTecStamped_.transform.translation.x;
        lastSlamtecPosition_[1] = slamTecStamped_.transform.translation.y;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher slamTec_pub_;
    ros::Publisher husky_pub_;
    // husky amdl publisher
    ros::Publisher amcl_pub_;
    ros::Publisher vo_pub_;
    ros::Publisher err_pub_;
    ros::Subscriber odom_sub_;
    tf2_ros::Buffer* tfBuffer_;
    tf2_ros::TransformListener* tfListener_;

    // husky amcl location
    geometry_msgs::TransformStamped huskyStamped_;
    // rts location
    geometry_msgs::TransformStamped slamTecStamped_;
    geometry_msgs::TransformStamped huskyRTSStamped_;
    // vo location
    geometry_msgs::TransformStamped voStamped_;

    std::vector<double> lastSlamtecPosition_;
    std::vector<double> lastHuskyPosition_;
    visualization_msgs::Marker slamtec;
    visualization_msgs::Marker slamtecRTSTraj_;
    visualization_msgs::Marker huskyRTSTraj_;
    visualization_msgs::Marker voTraj_;
    visualization_msgs::Marker amclTraj_;

    double husky_dist_;

    float husky_amcl_err_;
    float vo_err_;
    float vo_max_err_;
    float vo_min_err_;
    float vo_mean_err_;
    long int vo_err_count_;
    float husky_vx_;
    float husky_vth_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "show");
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
