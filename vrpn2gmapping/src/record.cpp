#include <ros/ros.h>
#include <tf2/convert.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <fstream>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "record");
    ros::NodeHandle nh;
    
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer);
    ros::Rate rate(10.0);
    geometry_msgs::TransformStamped transformStamped;

    std::ofstream write;
    write.open("pose.txt");

    int loop_counter = 0;
    double t_x, t_y, t_z, o_roll, o_pitch, o_yaw = 0;
    while (loop_counter<10)
    {
        try
        {
            transformStamped = tfBuffer.lookupTransform("world", "husky", ros::Time(0), ros::Duration(0.5));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        rate.sleep();

        t_x += transformStamped.transform.translation.x;
        t_y += transformStamped.transform.translation.y;
        
        double ox = transformStamped.transform.rotation.x;
        double oy = transformStamped.transform.rotation.y;
        double oz = transformStamped.transform.rotation.z;
        double ow = transformStamped.transform.rotation.w;
        tf::Quaternion q(ox, oy, oz, ow);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        o_yaw += yaw;

        loop_counter++;
    }

    t_x /= 10;
    t_y /= 10;
    t_z /= 10;
    o_yaw /= 10;
    write << t_x << "\n" << t_y << "\n" << 0 << "\n" << 0 << "\n" << 0 << "\n" << o_yaw << std::endl;

    
}