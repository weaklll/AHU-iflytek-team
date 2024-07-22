#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

// 全局变量，用于存储最短距离及其对应的索引
static double min_dist = std::numeric_limits<double>::infinity();
static int min_dist_index = -1;
static int range_x=0;
// 回调函数，用于处理接收到的激光雷达数据
void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // min_dist = std::numeric_limits<double>::infinity(); // 重置最短距离
    // min_dist_index = -1; // 重置最短距离索引

    // // 遍历激光雷达数据，找到最短距离及其对应的索引
    // for (size_t i = 0; i < msg->ranges.size(); ++i) 
    // {
    //     if (msg->ranges[i] < min_dist&&(msg->ranges[i]!=0)) 
    //     {
    //         min_dist = msg->ranges[i];
    //         min_dist_index = i;
    //     }
    // }
    int the_x=7;
    range_x=round(the_x*909/360);
    min_dist=msg->ranges[455+range_x];
    // 打印最短距离及其对应的索引
    ROS_INFO("最短距离为：%f 米", min_dist);
    ROS_INFO("最短距离为：%d 米", range_x);
}

int main(int argc, char **argv)
{
    // 设置语言环境
    setlocale(LC_ALL, ""); 

    // 初始化ROS节点
    ros::init(argc, argv, "lidar_node");

    // 创建节点句柄
    ros::NodeHandle n;

    // 订阅/scan话题
    ros::Subscriber lidar_sub = n.subscribe("/scan", 10, LidarCallback);

    // 保持循环等待回调函数的调用
    ros::spin();

    return 0;
}
