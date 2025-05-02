#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf2_ros/transform_broadcaster.h> // Menggunakan tf2_ros
#include <geometry_msgs/TransformStamped.h>  // Untuk pesan transform

// Publisher global untuk mempublikasikan pesan odometry
ros::Publisher odom_pub;

// Callback untuk pesan dari topik /odometry/filtered
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Publikasi ulang pesan yang diterima ke topik /odom
    odom_pub.publish(msg);
    ROS_INFO("Status: Published message");
    
    // Menyebarkan transform dari odometry sebagai bagian dari TF tree menggunakan tf2_ros
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.stamp = msg->header.stamp;
    transformStamped.header.frame_id = "odom";      // frame parent
    transformStamped.child_frame_id = "base_link";    // frame anak
    
    transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = msg->pose.pose.position.y;
    transformStamped.transform.translation.z = msg->pose.pose.position.z;
    
    transformStamped.transform.rotation = msg->pose.pose.orientation;
    
    br.sendTransform(transformStamped);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom");
    ros::NodeHandle nh;

    // Inisialisasi publisher untuk topik /odom dengan buffer queue size 10
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    
    // Inisialisasi subscriber untuk topik /odometry/filtered dengan queue size 10
    ros::Subscriber odom_sub = nh.subscribe("/odometry/filtered", 10, odomCallback);

    // Spin untuk menjalankan callback
    ros::spin();

    return 0;
}