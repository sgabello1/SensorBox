//
// Created by wang on 18-10-10.
//

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <iostream>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "temp");
    ros::NodeHandle nh;
    std::ifstream in("/home/sirius/ws/husky_ws/slamtecPoints.txt");
    ros::Publisher fakepath_pub = nh.advertise<visualization_msgs::Marker>("slam", 1);

    visualization_msgs::Marker slamtecPlanTraj;
    slamtecPlanTraj.header.frame_id = "world";
    slamtecPlanTraj.ns = "slamtec_plan";
    slamtecPlanTraj.action = visualization_msgs::Marker::ADD;
    slamtecPlanTraj.type = visualization_msgs::Marker::LINE_STRIP;
    slamtecPlanTraj.scale.x = 0.05;
    slamtecPlanTraj.scale.y = 0.05;
    slamtecPlanTraj.scale.z = 0.05;
    slamtecPlanTraj.color.r = 1.0;
    slamtecPlanTraj.color.g = 1.0;
    slamtecPlanTraj.color.a = 1.0;

    geometry_msgs::TransformStamped slamtecPoints;
    float x, y;
    while (
        in >> x &&
        in >> y
    )
    {  
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        slamtecPlanTraj.points.push_back(p);
    }
    in.close();

    ros::Rate rate(10.0);
    while(ros::ok())
    {
        fakepath_pub.publish(slamtecPlanTraj);
        rate.sleep();
    }
}