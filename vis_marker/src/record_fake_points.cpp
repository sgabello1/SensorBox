//
// Created by wang on 18-10-10.
//

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <fstream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "record_fake_points");
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10.0);

    std::ofstream out;
    out.open("slamtecPoints.txt", std::ios_base::binary);

    std::cout << std::cin.get() << std::endl;
    while (ros::ok())
    {
        geometry_msgs::TransformStamped slamtecPoints;
        slamtecPoints = tfBuffer.lookupTransform("world", "slamtec", ros::Time(0), ros::Duration(1.0));
        out << slamtecPoints.transform.translation.x << "\n";
        out << slamtecPoints.transform.translation.y << "\n";
        // out << slamtecPoints.transform.translation.z << "\n";
        std::cout << "record" << std::cin.get() << std::endl;
    }
    out.close();
}