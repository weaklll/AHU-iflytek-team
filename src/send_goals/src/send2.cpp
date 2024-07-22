#include <thread>
#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
using namespace std;
#define PI 3.14159

int target=0;

float x_mid=0.0;


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Goal 
{
    double x, y, orientation_z, orientation_w;
};

vector<Goal> readGoalsFromFile(const string& filename) 
{
    vector<Goal> goals;
    ifstream file(filename);
    if (file.is_open()) {
        Goal goal;
        while (file >> goal.x >> goal.y >> goal.orientation_z >> goal.orientation_w) {
            goals.push_back(goal);
        }
        file.close();
    }
    return goals;
}

void broadcast_terrorist_num(void)
{
    std::int32_t terrorist_num=0;
    while(!terrorist_num)
    {
        ros::param::get("/terrorist_num", terrorist_num);
                        
        if(terrorist_num==1)
        {
            system("play /home/iflytek/move/src/send_goals/voice/terrorist_one.wav");
            ROS_INFO("播报1个人");
            ros::param::set("/target", 1);
            break;
        }
        else if(terrorist_num==2)
        {
            system("play /home/iflytek/move/src/send_goals/voice/terrorists_two.wav");
            ROS_INFO("播报2个人");
            ros::param::set("/target", 2);
            break;
        }
        else if(terrorist_num==3)
        {
            system("play /home/iflytek/move/src/send_goals/voice/terrorists_three.wav");
            ROS_INFO("播报3个人");
            ros::param::set("/target", 3);
            break;
        }
    }
}

void sendGoals(const vector<Goal>& goals, MoveBaseClient& ac) 
{
    tf::TransformListener listener;
    for (size_t i = 0; i < goals.size(); i++) 
    {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = goals[i].x;
        goal.target_pose.pose.position.y = goals[i].y;
        goal.target_pose.pose.orientation.z = goals[i].orientation_z;
        goal.target_pose.pose.orientation.w = goals[i].orientation_w;

        ac.sendGoal(goal);
        ROS_INFO("Sending Goal %zu", i +1);

        ac.waitForResult();
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Goal %zu achieved success!", i + 1);
            if(i==5)
            {
                system("play /home/iflytek/move/src/send_goals/voice/Finish.wav");
            }
            if(i==0)
            {
              broadcast_terrorist_num();
            }
            
            if(i==1)
            {
                system("play /home/iflytek/move/src/send_goals/voice/first_aid_kit.wav");
            }

            if(i==2)
            {

                ros::param::set("/detect", true);
                while(1)
                {
                    ros::param::get("/x_mid", x_mid);
                    ROS_INFO("x_mid:%f---------",x_mid);
                    if(x_mid!=0)
                    {
                        break;
                    }
                }

                double x_tan = (x_mid - 320)*6.8/2800;
                if(250<x_mid<390)
                {
                    x_tan = (x_mid - 320)*6.93/2800 ;
                    std::cout<<"已经分类矫正"<<std::endl;
                }


                double x_rad = atan(x_tan) ;//弧度

                double x_the = x_rad*180/PI ;//弧度化成角度

                ros::param::set("/the_x",x_the);

                ros::param::set("/goal_laser",true);
                double dist=0;
                double dist_left=0;
                double dist_right=0;
                ros::param::get("/dist",dist);
                ros::param::get("/dist_left",dist_left);//中点偏左
                ros::param::get("/dist_right",dist_right);//中点偏右
                while(1)
                {
                    ros::param::get("/dist",dist);
                    ros::param::get("/dist_left",dist_left);//中点偏左
                    ros::param::get("/dist_right",dist_right);//中点偏右
                    if(5>dist&&dist>0&&dist_left&&dist_left>0&&dist_right&&dist_right>0)
                    {
                        break;
                    }
                }
                std::cout<<"该角度上的物品距离为:"<<dist<<std::endl;
                double x_true = (dist-0.25)*cos(x_rad);
                double y_true = (dist-0.25)*sin(x_rad);
                double x_true_l = dist_left*cos(x_rad+0.00747998);//0.00747998是雷达信息角度差（弧度）
                double y_true_l = dist_left*sin(x_rad+0.00747998);
                // double x_true_r = dist_right*cos(x_rad-0.00747998);
                // double y_true_r = -dist_right*sin(x_rad-0.00747998);
                double k = (y_true_l-y_true)/(x_true_l-x_true);//两直线斜率
                double yaw_k = atan(-1/k);//正对障碍物的角度
                double act_k=atan(k);
                double act_k_the = act_k*180/PI;
                double yaw_the = yaw_k*180/PI ;//弧度化成角度
                std::cout<<"目标点所正对的角度为:"<<yaw_the<<std::endl;
                std::cout<<"斜率角度为:"<<act_k_the<<std::endl;
                std::cout<<"目标点x在雷达坐标系的位置:"<<x_true<<std::endl;
                std::cout<<"目标点y在雷达坐标系的位置:"<<y_true<<std::endl;
                if(x_the>0 & k<0)
                {
                    x_true = x_true-(0.25*cos(yaw_k));
                    y_true = y_true-(0.25*sin(yaw_k));
                }
                if(x_the<0 & k>0)
                {
                    x_true = x_true-(0.25*cos(yaw_k));
                    y_true = y_true-(0.25*sin(yaw_k));
                }   
                if(x_the > 0 & k > 0)//排除不会正对的意外
                {   
                    if(act_k < x_rad)//该情况不常见
                    {
                        x_true = x_true+(0.25*cos(yaw_k));
                        y_true = y_true+(0.25*sin(yaw_k));
                        yaw_the = 180 + yaw_the ; 
                        yaw_k = yaw_the * PI/180 ;
                        std::cout<<"已排除不会正对的意外1"<<std::endl;
                    }
                    if(act_k > x_rad)
                    {
                        x_true = x_true-(0.25*cos(yaw_k));
                        y_true = y_true-(0.25*sin(yaw_k));
                            // yaw_the = 180 + yaw_the ; 
                            // yaw_k = yaw_the * PI/180 ;
                        std::cout<<"已排除不会正对的意外2"<<std::endl;
                    }
                }
                if(x_the < 0 & k < 0)
                {   
                    if(act_k>x_rad)//该情况不常见
                    {
                        x_true = x_true+(0.25*cos(yaw_k));
                        y_true = y_true+(0.25*sin(yaw_k));
                        yaw_the = -180 + yaw_the ; 
                        yaw_k = yaw_the * PI/180 ;
                        std::cout<<"已排除不会正对的意外3"<<std::endl;
                    }
                    if(act_k<x_rad)
                    {
                        x_true = x_true-(0.25*cos(yaw_k));
                        y_true = y_true-(0.25*sin(yaw_k));
                            // yaw_the = -180 + yaw_the ; 
                            // yaw_k = yaw_the * PI/180 ;
                        std::cout<<"已排除不会正对的意外4"<<std::endl;
                    }
                }

                x_rad = atan(y_true/x_true) ;
                x_the = x_rad*180/PI ;
                std::cout<<"新的目标所在角度为:"<<x_the<<std::endl;
                std::cout<<"斜率横坐标1为:"<<x_true_l<<std::endl;
                std::cout<<"斜率纵坐标1为:"<<y_true_l<<std::endl;
                // std::cout<<"斜率横坐标2为:"<<x_true_r<<std::endl;
                // std::cout<<"斜率纵坐标2为:"<<y_true_r<<std::endl;
                std::cout<<"目标点所正对的角度为:"<<yaw_the<<std::endl;
                std::cout<<"新的目标点x在雷达坐标系的位置:"<<x_true<<std::endl;
                std::cout<<"新的目标点y在雷达坐标系的位置:"<<y_true<<std::endl;

                double roll = 0.0, pitch = 0.0, yaw = yaw_k;                     // 定义欧拉角，可以理解为绕x、y、z轴转动
                geometry_msgs::Quaternion q;                                   // 定义四元数
                q = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw); // 将欧拉角转为四元数
                geometry_msgs::Quaternion q_true;                                   // 定义四元数
                q_true = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, x_rad); // 将欧拉角转为四元数
                geometry_msgs::PoseStamped laser_point;
                laser_point.header.frame_id = "laser_frame";
                laser_point.header.stamp = ros::Time();
                laser_point.pose.position.x = x_true;
                laser_point.pose.position.y = y_true;
                laser_point.pose.position.z = 0;
                laser_point.pose.orientation.z = q.z;
                laser_point.pose.orientation.w = q.w;
                laser_point.pose.orientation.x = q_true.x;
                laser_point.pose.orientation.y = q_true.y;

                // 将base_link坐标系转为map坐标系
                try
                {
                    // 基于map坐标系创建目标点
                    geometry_msgs::PoseStamped map_point;
                    map_point.header.frame_id = "map";
                    map_point.header.stamp = ros::Time();//此处是ros::Time()，而非ros::Time::now()，两者是有区别的
                    ROS_INFO("starting laser_frame—>map ");
                     // 实现map—>laser_frame，何时转：ros::Time(0)，多久内完成：ros::Duration(3.0)
                    listener.waitForTransform("map", "laser_frame", ros::Time(0), ros::Duration(3.0));
                    // "map"为目标坐标系，从laser_point转map_point
                    // listener.transformPoint("map", laser_point, map_point);
                    listener.transformPose("map",  ros::Time(0), laser_point, "laser_frame" ,map_point);
                    ROS_INFO("map [x]=%.2f,[y]=%.2f", map_point.pose.position.x, map_point.pose.position.y);
            
                    // 设置目标点信息
                    move_base_msgs::MoveBaseGoal goal;
                    goal.target_pose.header.frame_id = "map"; // map
                    goal.target_pose.header.stamp = ros::Time::now();
                    goal.target_pose.pose.position = map_point.pose.position;
                    goal.target_pose.pose.orientation = map_point.pose.orientation ;
                        
                
                    ROS_INFO("Sending goal");
                    ac.sendGoal(goal);
                    // Wait for the action to return
                    ac.waitForResult();
                    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                        ROS_INFO("You have reached the goal!");
                    else
                        ROS_INFO("The base failed for some reason");
                }
                catch (tf::TransformException &ex)
                {
                    ROS_ERROR("%s", ex.what());
                }
                int thing_class=0;
                ros::param::get("/target",thing_class);
                // switch(thing_class)
                // {
                //     case 1:
                //             system("play /home/iflytek/move/src/send_goals/voice/spontoon.wav");
                //         break;
                //     case 2:
                //             system("play /home/iflytek/move/src/send_goals/voice/bulletproof_vest.wav");
                //         break;
                //     case 3:
                //             system("play /home/iflytek/move/src/send_goals/voice/teargas.wav");
                //         break;
                //     default:
                //         break;
                // }
                if (thing_class==1)
                {
                    system("play /home/iflytek/move/src/send_goals/voice/spontoon.wav");
                }
                else if(thing_class==2)
                {
                    system("play /home/iflytek/move/src/send_goals/voice/bulletproof_vest.wav");
                }
                else if(thing_class==3)
                {
                    system("play /home/iflytek/move/src/send_goals/voice/teargas.wav");
                }
            }


        } 
        else 
        {
            ROS_WARN("Failed to achieve Goal %zu", i + 1);
        }
    }
}

void robotNavigation(int argc,char** argv,const string& goals_file, const string& launch_file) 
{
    ros::init(argc,argv,"send_goals");
    ros::NodeHandle nh;
    MoveBaseClient ac("move_base", true);

    // system(("gnome-terminal -- roslaunch " + launch_file).c_str());

    cout << "Have you confirmed the operation? (Y/N): ";
    string userInput;
    cin >> userInput;
    if (userInput != "Y" && userInput != "y") 
    {
        cout << "Operation not confirmed. Exiting." << endl;
        return;
    }

    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    vector<Goal> goals = readGoalsFromFile(goals_file);
    if (goals.empty()) {
        ROS_ERROR("Failed to read goals from file: %s", goals_file.c_str());
        return;
    }

    sendGoals(goals, ac);
}

int main(int argc,char*argv[])
 {
    // 指定目标点文件的绝对路径
    string goals_file = "/home/iflytek/move/src/send_goals/src/goals.txt";
    // 指定启动文件的绝对路径
    string launch_file = "/home/ucar_lzhw/src/ucar_nav/launch/ucar_navi.launch";
    
    // 创建导航线程，并传递目标点文件名和启动文件名
    thread navigationThread(robotNavigation, argc, argv,goals_file, launch_file);
    navigationThread.join();

    return 0;
}
