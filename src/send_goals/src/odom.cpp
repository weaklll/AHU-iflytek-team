#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // 获取位置（position）
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;

    // 获取方向（orientation）
    double ox = msg->pose.pose.orientation.x;
    double oy = msg->pose.pose.orientation.y;
    double oz = msg->pose.pose.orientation.z;
    double ow = msg->pose.pose.orientation.w;

    // 打印获取到的信息
    ROS_INFO("Position: [x: %f, y: %f, z: %f]", x, y, z);
    ROS_INFO("Orientation: [x: %f, y: %f, z: %f, w: %f]", ox, oy, oz, ow);
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "odom_listener");
    ros::NodeHandle nh;

    // 订阅主题
    ros::Subscriber sub = nh.subscribe("/odom", 1000, odomCallback);

    // 循环等待回调
    ros::spin();

    return 0;
}
