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
#define DEG2RAD(x) ((x)/180.0*PI)
#define RAD2DEG(x) ((x)/PI*180.0)

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int target=0;

float x_mid=0.0;

bool if_get=false;


float min_front_dis=10.0;
float min_left_dis=10.0;
float min_right_dis=10.0;


void go_front(MoveBaseClient& ac)
{
    min_front_dis=10.0;   //初始化一下
    ros::param::set("/begin_detect_front",true);
    ros::param::get("/min_front_dis",min_front_dis);
    while(1)
    {
        ros::param::get("/min_front_dis",min_front_dis);
        if(min_front_dis>0.0&&min_front_dis<2.0)
        {
            break;
        }
    }

    if(min_front_dis>1.2)
    {
        printf("min_front_dis=%f,前方距离充足,向前直行\n",min_front_dis);
        // 使用全局命名空间创建Publisher，不需要显式创建节点句柄
        ros::Publisher cmd_vel_pub = ros::NodeHandle().advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // 设置发布频率
        ros::Rate rate(10); // 10Hz

        // 创建Twist消息实例
        geometry_msgs::Twist twist;

        // 设置旋转速度 (沿x轴)
        twist.linear.x = 0.3;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0; // 旋转速度为1弧度/秒

        for(int count=20;count>0;count--) 
        {
            // 发布消息
            cmd_vel_pub.publish(twist);
            // 休眠直到下一个发布周期
            rate.sleep();
        }
        ros::param::set("/min_front_dis",10.0);
    }
    else if(min_front_dis<0.7)
    {
        printf("min_front_dis=%f,距离过小,绕到板子后面\n",min_front_dis);

        //发点
        //-1.852 0.006 0.714 0.700
        move_base_msgs::MoveBaseGoal goal3;

        goal3.target_pose.header.frame_id = "map";
        goal3.target_pose.header.stamp = ros::Time::now();
        goal3.target_pose.pose.position.x = -1.852;
        goal3.target_pose.pose.position.y = (0.006 + min_front_dis + 0.3);
        goal3.target_pose.pose.orientation.z = 0.714;
        goal3.target_pose.pose.orientation.w = 0.700;

        ac.sendGoal(goal3);
        ROS_INFO("Sending Goal left_up");
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Goal left_up achieved success");
            
            
        }
    }

}

void go_left()
{
    min_left_dis=10.0;   //初始化一下
    ros::param::set("/begin_detect_left",true);
    ros::param::get("/min_left_dis",min_left_dis);

    while(1)
    {
        ros::param::get("/min_left_dis",min_left_dis);
        if(min_left_dis>0.0&&min_left_dis<2.0)
        {
            break;
        }
    }
        
    if(min_left_dis>0.7)
    {
        ROS_INFO("go left");
        // 使用全局命名空间创建Publisher，不需要显式创建节点句柄
        ros::Publisher cmd_vel_pub = ros::NodeHandle().advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // 设置发布频率
        ros::Rate rate(10); // 10Hz

        // 创建Twist消息实例
        geometry_msgs::Twist twist;

        // 设置旋转速度 (沿y轴)
        twist.linear.x = 0.0;
        twist.linear.y = 0.3;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0; // 旋转速度为1弧度/秒

        for(int count=20;count>0;count--) 
        {
            // 发布消息
            cmd_vel_pub.publish(twist);
            // 休眠直到下一个发布周期
            rate.sleep();
        }
        ros::param::set("/min_left_dis",10.0);
    }
    else
    {
        printf("min_left_dis=%f,距离过小\n",min_left_dis);
    }

}

void go_right()
{
    min_right_dis=10.0;   //初始化一下
    ros::param::set("/begin_detect_right",true);
    ros::param::get("/min_right_dis",min_right_dis);
    while(1)
    {
        ros::param::get("/min_right_dis",min_right_dis);
        if(min_right_dis>0.0&&min_right_dis<2.0)
        {
            break;
        }
    }

    if(min_right_dis>0.8)
    {
        ROS_INFO("go right");
        // 使用全局命名空间创建Publisher，不需要显式创建节点句柄
        ros::Publisher cmd_vel_pub = ros::NodeHandle().advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // 设置发布频率
        ros::Rate rate(10); // 10Hz

        // 创建Twist消息实例
        geometry_msgs::Twist twist;

        // 设置旋转速度 (沿y轴)
        twist.linear.x = 0.0;
        twist.linear.y = -0.2;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0; // 旋转速度为1弧度/秒

        for(int count=15;count>0;count--) 
        {
            // 发布消息
            cmd_vel_pub.publish(twist);
            // 休眠直到下一个发布周期
            rate.sleep();
        }
        ros::param::set("/min_right_dis",10.0);
    }
    else
    {
        printf("min_right_dis=%f,距离过小\n",min_right_dis);
    }
    

}




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

void turn_left_45() //1 -> 18
{
    ROS_INFO("turning left");
    // 使用全局命名空间创建Publisher，不需要显式创建节点句柄
    ros::Publisher cmd_vel_pub = ros::NodeHandle().advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // 设置发布频率
    ros::Rate rate(10); // 10Hz

    // 创建Twist消息实例
    geometry_msgs::Twist twist;

    // 设置旋转速度 (绕z轴)
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 2.0; // 旋转速度为1弧度/秒

    ROS_INFO("Rotating the robot");

    for(int count=7;count>0;count--) 
    {
        // 发布消息
        cmd_vel_pub.publish(twist);
        // 休眠直到下一个发布周期
        rate.sleep();
    }
}

void turn_left_60() //1 -> 18
{
    ROS_INFO("turning left");
    // 使用全局命名空间创建Publisher，不需要显式创建节点句柄
    ros::Publisher cmd_vel_pub = ros::NodeHandle().advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // 设置发布频率
    ros::Rate rate(10); // 10Hz

    // 创建Twist消息实例
    geometry_msgs::Twist twist;

    // 设置旋转速度 (绕z轴)
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 2.0; // 旋转速度为1弧度/秒

    ROS_INFO("Rotating the robot");

    for(int count=9;count>0;count--) 
    {
        // 发布消息
        cmd_vel_pub.publish(twist);
        // 休眠直到下一个发布周期
        rate.sleep();
    }
}

void turn_left_90(void) //1 -> 20
{
    ROS_INFO("turning left 90");
    // 使用全局命名空间创建Publisher，不需要显式创建节点句柄
    ros::Publisher cmd_vel_pub = ros::NodeHandle().advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 设置发布频率
    ros::Rate rate(10); // 10Hz

    // 创建Twist消息实例
    geometry_msgs::Twist twist;

    // 设置旋转速度 (绕z轴)
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 2.2; // 旋转速度为1弧度/秒

    ROS_INFO("Rotating the robot");

    for(int count=11;count>0;count--) 
    {
        // 发布消息
        cmd_vel_pub.publish(twist);
        // 休眠直到下一个发布周期
        rate.sleep();
    }
}

void turn_right_90(void) 
{
    ROS_INFO("turning left 90");
    // 使用全局命名空间创建Publisher，不需要显式创建节点句柄
    ros::Publisher cmd_vel_pub = ros::NodeHandle().advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 设置发布频率
    ros::Rate rate(10); // 10Hz

    // 创建Twist消息实例
    geometry_msgs::Twist twist;

    // 设置旋转速度 (绕z轴)
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = -2.2; // 旋转速度为1弧度/秒

    ROS_INFO("Rotating the robot");

    for(int count=11;count>0;count--) 
    {
        // 发布消息
        cmd_vel_pub.publish(twist);
        // 休眠直到下一个发布周期
        rate.sleep();
    }
}


void turn_right_60(void) 
{
    // 使用全局命名空间创建Publisher，不需要显式创建节点句柄
    ROS_INFO("turning right");
    ros::Publisher cmd_vel_pub = ros::NodeHandle().advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 设置发布频率
    ros::Rate rate(10); // 10Hz

    // 创建Twist消息实例
    geometry_msgs::Twist twist;

    // 设置旋转速度 (绕z轴)
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = -2.0; // 旋转速度为-1弧度/秒

    ROS_INFO("Rotating the robot");

    for(int count=9;count>0;count--) 
    {
        // 发布消息
        cmd_vel_pub.publish(twist);
        // 休眠直到下一个发布周期
        rate.sleep();
    }
}

void turn_right_45(void) 
{
    // 使用全局命名空间创建Publisher，不需要显式创建节点句柄
    ROS_INFO("turning right");
    ros::Publisher cmd_vel_pub = ros::NodeHandle().advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 设置发布频率
    ros::Rate rate(10); // 10Hz

    // 创建Twist消息实例
    geometry_msgs::Twist twist;

    // 设置旋转速度 (绕z轴)
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = -2.0; // 旋转速度为-1弧度/秒

    ROS_INFO("Rotating the robot");

    for(int count=7;count>0;count--) 
    {
        // 发布消息
        cmd_vel_pub.publish(twist);
        // 休眠直到下一个发布周期
        rate.sleep();
    }
}

void stop(void) 
{
    // 使用全局命名空间创建Publisher，不需要显式创建节点句柄
    ROS_INFO("stop");
    ros::Publisher cmd_vel_pub = ros::NodeHandle().advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 设置发布频率
    ros::Rate rate(10); // 10Hz

    // 创建Twist消息实例
    geometry_msgs::Twist twist;

    // 设置旋转速度 (绕z轴)
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0; // 旋转速度为-1弧度/秒

    ROS_INFO("Rotating the robot");
    system("play /home/iflytek/move/src/send_goals/voice/Finish.wav");
    for(int count=18;count>0;count--) 
    {
        // 发布消息
        cmd_vel_pub.publish(twist);
        // 休眠直到下一个发布周期
        rate.sleep();
    }
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

void find(MoveBaseClient& ac)
{
    tf::TransformListener listener;

    for(int i=300;i>0;i--)
    {
        ros::param::get("/x_mid", x_mid);
        if(x_mid!=0)
        {
            ROS_INFO("x_mid:%f---------",x_mid);
            if_get=true;
            break;
        }
    }

    if(if_get)
    {
        double x_tan;
        x_tan = (320-x_mid)*6.86/2800;

        double x_rad = atan(x_tan) ;//弧度[-PI/2,PI/2]

        double x_the = x_rad*180/PI ;//弧度化成角度

        std::cout<<"目标所在角度为:"<<x_the<<std::endl;

        ros::param::set("/the_x",x_the);//传入雷达，采集对应位置数据

        ros::param::set("/goal_laser",true);//开启测距

        double dist=0;
        double dist_left=0;
        double dist_right=0;
        while(1)
        {
            ros::param::get("/dist",dist);//终点
            ros::param::get("/dist_left",dist_left);//中点偏左
            ros::param::get("/dist_right",dist_right);//中点偏右
            if(10>dist&&dist>0&&10>dist_left&&dist_left>0&&10>dist_right&&dist_right>0)
            {
                ros::param::set("/goal_laser",false);
                break;
            }
        }
        std::cout<<"该角度上的物品距离为:"<<dist<<std::endl;
    
        int error_laser=0;//雷达正前方偏移位数

        x_the=x_the+(error_laser*909/360);
        x_rad=x_the*PI/180;

        // std::cout<<"归一到雷达坐标系中的角度："<<x_the<<std::endl;

        double x_true_l = (dist_left)*cos(x_rad+0.00747998*6);//0.00747998是雷达信息角度差（弧度）=（360/840）*PI/180
        double y_true_l = (dist_left)*sin(x_rad+0.00747998*6);

        double x_true_r = (dist_right)*cos(x_rad-0.00747998*6);
        double y_true_r = (dist_right)*sin(x_rad-0.00747998*6);

        // double k1 = (y_true_l-y_true_r)/(x_true_l-x_true_r);//两直线斜率
        // double k2= (y_true_l-y_true)/(x_true_l-x_true);
        // double k3= (y_true_r-y_true)/(x_true_r-x_true);

        double k=(y_true_l-y_true_r)/(x_true_l-x_true_r);//对于雷达坐标系的板子的斜率

        double act_k=atan(k);//板子斜率与x轴正向成的角度（弧度）

        double yaw_k = atan(-1/k);//正对障碍物的角度------板子斜率的法线斜率与x轴正方向成的角度，弧度制
        double yaw_the = yaw_k*180/PI ;//弧度化成角度

        double x_true = 0.0;
        double y_true = 0.0;

        // if(act_k>-0.26&&act_k<0.26)
        if(act_k>-1&&act_k<1)
        {
            printf("act_k=%f,过小，return\n",act_k);
            return;
        }

        if(act_k<=0)
        {
            x_true = dist*cos(x_rad)-0.35*sin(-act_k);
            y_true = dist*sin(x_rad)-0.35*cos(-act_k);
        }
        else
        {
            x_true = dist*cos(x_rad)-0.35*sin(act_k);
            y_true = dist*sin(x_rad)+0.35*cos(act_k);
        }
        printf("**************************\n");
        printf("**************************\n");
        // printf("act_k=%f,k=%f\n",act_k,k);
        // printf("0.25*sin(PI-act_k)=%f,0.25*cos(PI-act_k)=%f\n",0.25*sin(-act_k),0.25*cos(-act_k));
        printf("act_k=%f\n",act_k);
        printf("(x,y)=(%f,%f)\n",dist*cos(x_rad),dist*sin(x_rad));
        printf("(x',y')=(%f , %f)\n",x_true,y_true);
        printf("deta_d=%f\n",sqrt((x_true-dist*cos(x_rad))*(x_true-dist*cos(x_rad))+(y_true-dist*sin(x_rad))*(y_true-dist*sin(x_rad))));
        printf("**************************\n");
        printf("**************************\n");

        // if((k>0.03)&&(x_the>0))
        // {
        //     error_laser=4;
        // }
        // else if((k<-0.03)&&(x_the<0))
        // {
        //     error_laser=-4;
        // }


        // x_the=x_the+(error_laser*909/360);
        // x_rad=x_the*PI/180;

        // if(k>0.11&&x_the>25)
        // {
        //     yaw_k=90-yaw_k;
        //     x_true=x_true+0.4;
        //     y_true=y_true+0.2;
        // }
        // else if(k<-0.11&&x_tan<-25)
        // {
        //     yaw_k=-90-yaw_k;
        //     x_true=x_true-0.2;
        //     y_true=y_true+0.2;
        // }
        std::cout<<"目标点法线所在的弧度为:"<<yaw_k<<std::endl;
        std::cout<<"目标点所在直线的角度为:"<<act_k<<std::endl;
        std::cout<<"目标点法线所在的角度为:"<<yaw_the<<std::endl;
        std::cout<<"目标点x在雷达坐标系的位置:"<<x_true<<std::endl;
        std::cout<<"目标点y在雷达坐标系的位置:"<<y_true<<std::endl;
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

        switch(thing_class)
        {
            case 1:
                    system("play /home/iflytek/move/src/send_goals/voice/spontoon.wav");
                break;
            case 2:
                    system("play /home/iflytek/move/src/send_goals/voice/bulletproof_vest.wav");
                break;
            case 3:
                    system("play /home/iflytek/move/src/send_goals/voice/teargas.wav");
                break;
            default:
                break;
        }

    }

    
}


bool getRobotPosition(tf::TransformListener& listener, double& x, double& y) 
{
    tf::StampedTransform transform;
    try {
        // 从/map到/base_link的变换关系
        listener.lookupTransform("map", "base_link", ros::Time(0), transform);
        
        // 获取x, y坐标
        x = transform.getOrigin().x();
        y = transform.getOrigin().y();
        
        return true; // 成功获取坐标
    } catch (tf::TransformException &ex) 
    {
        ROS_WARN("%s", ex.what());
        return false; // 获取坐标失败
    }
}

void sendGoals(const vector<Goal>& goals, MoveBaseClient& ac) 
{
    tf::TransformListener listener;
    for (size_t i = 0; i <goals.size(); i++) 
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

            switch (i)
            {
                case 0:
                    broadcast_terrorist_num();
                    break;
                case 2:
                    ros::param::set("/detect", true);
                    find(ac);
                    ros::param::set("/detect", false);

                    for(int turn_num=2;turn_num>0 && (!if_get);turn_num--)
                    {
                        turn_left_60();
                        ros::param::set("/detect", true);
                        find(ac);
                        ros::param::set("/detect", false);
                    }

                    break;
                case 3:
                    system("play /home/iflytek/move/src/send_goals/voice/first_aid_kit.wav");
                    turn_left_90();
                    if(!if_get)
                    {
                        ros::param::set("/detect", true);
                        find(ac);
                        ros::param::set("/detect", false);
                    }
                    
                    if(!if_get)
                    {
                        turn_left_90();
                        ros::param::set("/detect", true);
                        find(ac);
                        ros::param::set("/detect", false);
                    }
                    break;
                case 4:
                    if(!if_get)
                    {
                        ros::param::set("/detect", true);
                        find(ac);
                        ros::param::set("/detect", false);
                    }

                    // if(!if_get)
                    // {
                    //     go_left();
                    // }
                    // if(!if_get)
                    // // {
                    // //     ros::param::set("/detect", true);
                    // //     find(ac);
                    // //     ros::param::set("/detect", false);
                    // // }

                    
                    if(!if_get)
                    {
                        //发点
                        move_base_msgs::MoveBaseGoal goal2;

                        goal2.target_pose.header.frame_id = "map";
                        goal2.target_pose.header.stamp = ros::Time::now();
                        goal2.target_pose.pose.position.x = -2.196;
                        goal2.target_pose.pose.position.y = 1.059;
                        goal2.target_pose.pose.orientation.z = 0.252;
                        goal2.target_pose.pose.orientation.w = 0.968;

                        ac.sendGoal(goal2);
                        ROS_INFO("Sending Goal left_up");
                        ac.waitForResult();
                        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                        {
                            ROS_INFO("Goal left_up achieved success");
                            
                            ros::param::set("/detect", true);
                            find(ac);
                            ros::param::set("/detect", false);
                            if(!if_get)
                            {
                                turn_right_60();
                                ros::param::set("/detect", true);
                                find(ac);
                                ros::param::set("/detect", false);

                                if(!if_get)
                                {
                                    turn_right_60();
                                    ros::param::set("/detect", true);
                                    find(ac);
                                    ros::param::set("/detect", false);
                                }
                            }
                        }
                    }
                    break;
                case 5:
                    break;
                case 7:
                    ros::param::set("/if_follow_line", 1);
                    while (1) 
                    {
                        double x, y;
                        if (getRobotPosition(listener, x, y))
                         {
                            // ROS_INFO("Robot position: x = %f, y = %f", x, y);
                            // printf("此时的坐标(%f,%f)\n",x,y);
                            if((abs(x-1.94)<0.5)&&(abs(y+0.3)<0.12))
                            {
                                system("killall usb_cam_node");
                                printf("----------到达终点-------------");
                                stop();
                                break;
                            }
                        } 
                        else 
                        {
                            ROS_WARN("Failed to get robot position");
                        }
                        
                    }
                    break;

                    //[ INFO] [1717847695.968883410]: Setting goal: Frame:map, Position(1.941, -0.101, 0.000), Orientation(0.000, 0.000, 0.039, 0.999) = Angle: 0.079
                default:
                    break;
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
    cout << "启动导航"<<endl;
    
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
    //巡线初始化
    // 指定目标点文件的绝对路径
    string goals_file = "/home/iflytek/move/src/send_goals/src/goals/goals2.txt";
    // 指定启动文件的绝对路径
    string launch_file = "/home/ucar_lzhw/src/ucar_nav/launch/ucar_navi.launch";
    
    // 创建导航线程，并传递目标点文件名和启动文件名
    thread navigationThread(robotNavigation, argc, argv,goals_file, launch_file);
    navigationThread.join();

    return 0;
}
