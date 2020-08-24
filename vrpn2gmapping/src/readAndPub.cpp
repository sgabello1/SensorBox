#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <fstream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "readAndPub");
    ros::NodeHandle nh;

    std::ifstream reader;
    reader.open("/home/husky/husky_demo_ws/src/husky_ws/pose/pose.txt");
    double t_x, t_y, t_z, o_roll, o_pitch, o_yaw;
    reader >> t_x;
    reader >> t_y;
    reader >> t_z;
    reader >> o_roll;
    reader >> o_pitch;
    reader >> o_yaw;
    // std::cout << t_x << "," << t_y << "," << t_z << "," << o_roll << ","<< o_pitch << "," << o_yaw << std::endl;



    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "map";
    transformStamped.transform.translation.x = t_x;
    transformStamped.transform.translation.y = t_y;
    transformStamped.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, o_yaw);
    transformStamped.transform.rotation.x = q[0];
    transformStamped.transform.rotation.y = q[1];
    transformStamped.transform.rotation.z = q[2];
    transformStamped.transform.rotation.w = q[3];


    br.sendTransform(transformStamped);
    ros::spin();
    
    
}
