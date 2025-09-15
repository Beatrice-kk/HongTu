// /**
//  * @file lidar_pose_radar_only.cpp
//  * @brief 只使用雷达定位（订阅 /Odometry），解析保存并打印位姿信息
//  */

// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Odometry.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <iostream>
// #include <iomanip>

// using namespace std;

// class lidar_pose
// {
// public:
//     lidar_pose(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_);

//     double pi;
//     struct attitude
//     {
//         double pitch;
//         double roll;
//         double yaw;
//     };

//     attitude radarAttitude;
//     geometry_msgs::PoseStamped radarPose;

//     bool radarOdomRec_flag;

//     ros::Rate *rate;

//     ros::NodeHandle nh;
//     ros::NodeHandle nh_private;

//     ros::Subscriber odom_sub;
//     ros::Subscriber move_base_feedback_sub;


//     void estimator_odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
//     void start();
// };

// lidar_pose::lidar_pose(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_)
//     :nh(nh_), nh_private(nh_private_), radarOdomRec_flag(false)
// {
//     pi = 3.1415926;
//     rate = new ros::Rate(30);

//     odom_sub = nh.subscribe<nav_msgs::Odometry>("/slam_odom", 2, &lidar_pose::estimator_odom_cb,this);
//     move_base_feedback_sub = nh.subscribe<nav_msgs::Odometry>("/move_base/feedback", 2, &lidar_pose::estimator_odom_cb,this);

//     radarAttitude.pitch = 0;
//     radarAttitude.roll = 0;
//     radarAttitude.yaw = 0;
// }

// void lidar_pose::estimator_odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
// {
//     radarPose.pose = msg->pose.pose;

//     tf2::Quaternion quat;
//     tf2::fromMsg(msg->pose.pose.orientation, quat);
//     double roll, pitch, yaw;
//     tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
//     radarAttitude.pitch = pitch * 180 / pi;
//     radarAttitude.roll = roll * 180 / pi;
//     radarAttitude.yaw = yaw * 180 / pi;

//     radarOdomRec_flag = true;
// }

// // void lidar_pose::start()
// // {
// //     while(ros::ok())
// //     {
// //         if(!radarOdomRec_flag){
// //             cout << "\033[K" << "\033[31m noooooooooooooooooo   lidarrrrrrrrrrrrr     dataaaaaaaaaaaaaaaaaaaaaaaaaa!!! \033[0m" << endl;
// //         } else {
// //             cout << "\033[K" << "\033[32m yesyesyes      lidar    data   receiveddddddddddddddddddd! \033[0m" << endl;
// //             cout << "\033[K" << "       雷达定位" << endl;
// //             cout << setiosflags(ios::fixed) << setprecision(7)
// //                 << "\033[K" << "x      " << radarPose.pose.position.x << endl;
// //             cout << setiosflags(ios::fixed) << setprecision(7)
// //                 << "\033[K" << "y      " << radarPose.pose.position.y << endl;
// //             cout << setiosflags(ios::fixed) << setprecision(7)
// //                 << "\033[K" << "z      " << radarPose.pose.position.z << endl;
// //             cout << setiosflags(ios::fixed) << setprecision(7)
// //                 << "\033[K" << "pitch  " << radarAttitude.pitch << endl;
// //             cout << setiosflags(ios::fixed) << setprecision(7)
// //                 << "\033[K" << "roll   " << radarAttitude.roll << endl;
// //             cout << setiosflags(ios::fixed) << setprecision(7)
// //                 << "\033[K" << "yaw    " << radarAttitude.yaw << endl;
// //             cout << "\033[7A" << endl;
// //         }
// //         ros::spinOnce();
// //         rate->sleep();
// //     }
// //     cout << "\033[7B" << endl;
// // }


// void lidar_pose::start()
// {
//     while (ros::ok())
//     {
//         cout << "\033[2J\033[H";

//         if (!radarOdomRec_flag)
//         {
//             cout << "\033[31mNooooooooooo LiDAR dataaaaaaaaaaaaaaaaaa!a\033[0m" << endl;
//         }
//         else
//         {
//             cout << "\033[32mLiDARRRRRRRRRRRRRRRRRRRRRRRRRR data receivedddddddddddddddddddddddddddd!\033[0m" << endl;
//             cout << "==================== odometry information ====================" << endl;

//             cout << setiosflags(ios::fixed) << setprecision(7);
//             cout << "x      : " << radarPose.pose.position.x << endl;
//             cout << "y      : " << radarPose.pose.position.y << endl;
//             cout << "z      : " << radarPose.pose.position.z << endl;
//             cout << "pitch  : " << radarAttitude.pitch << endl;
//             cout << "roll   : " << radarAttitude.roll << endl;
//             cout << "yaw    : " << radarAttitude.yaw << endl;

//             cout << "=====================================================" << endl;
//         }

//         ros::spinOnce();
//         rate->sleep();
//     }
// }


// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "lidar_pose_node");
//     ros::NodeHandle nh_("");
//     ros::NodeHandle nh_private_("~");
//     lidar_pose vision(nh_,nh_private_);
//     vision.start();

//     return 0;
// }




#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace std;

class lidar_pose
{
public:
    lidar_pose(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_);

    double pi;
    struct attitude
    {
        double pitch;
        double roll;
        double yaw;
    };

    attitude radarAttitude;
    geometry_msgs::PoseStamped radarPose;

    geometry_msgs::PoseStamped moveBasePose;

    bool radarOdomRec_flag;
    bool moveBaseFeedbackRec_flag;

    ros::Rate *rate;

    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    ros::Subscriber odom_sub;
    ros::Subscriber move_base_feedback_sub;

    void estimator_odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void move_base_feedback_cb(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg);
    void start();
};

lidar_pose::lidar_pose(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_)
    :nh(nh_), nh_private(nh_private_), radarOdomRec_flag(false), moveBaseFeedbackRec_flag(false)
{
    pi = 3.1415926;
    rate = new ros::Rate(30);

    odom_sub = nh.subscribe<nav_msgs::Odometry>("/slam_odom", 2, &lidar_pose::estimator_odom_cb, this);
    move_base_feedback_sub = nh.subscribe<move_base_msgs::MoveBaseActionFeedback>("/move_base/feedback", 2, &lidar_pose::move_base_feedback_cb, this);

    radarAttitude.pitch = 0;
    radarAttitude.roll = 0;
    radarAttitude.yaw = 0;
}

void lidar_pose::estimator_odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    radarPose.pose = msg->pose.pose;

    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    radarAttitude.pitch = pitch * 180 / pi;
    radarAttitude.roll = roll * 180 / pi;
    radarAttitude.yaw = yaw * 180 / pi;

    radarOdomRec_flag = true;
}

void lidar_pose::move_base_feedback_cb(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg)
{
    moveBasePose = msg->feedback.base_position;
    moveBaseFeedbackRec_flag = true;
}

void lidar_pose::start()
{
    while (ros::ok())
    {
        cout << "\033[2J\033[H"; // 清屏

        if (!radarOdomRec_flag)
        {
            cout << "\033[31mNo LiDAR odometry data!\033[0m" << endl;
        }
        else
        {
            cout << "\033[32mLiDAR odometry received!\033[0m" << endl;
            cout << "==================== LiDAR odometry ====================" << endl;
            cout << setiosflags(ios::fixed) << setprecision(7);
            cout << "x      : " << radarPose.pose.position.x << endl;
            cout << "y      : " << radarPose.pose.position.y << endl;
            cout << "z      : " << radarPose.pose.position.z << endl;
            cout << "pitch  : " << radarAttitude.pitch << endl;
            cout << "roll   : " << radarAttitude.roll << endl;
            cout << "yaw    : " << radarAttitude.yaw << endl;
            cout << "========================================================" << endl;
        }

        if (!moveBaseFeedbackRec_flag)
        {
            cout << "\033[31mNo move_base feedback data!\033[0m" << endl;
        }
        else
        {
            cout << "\033[32mmove_base feedback received!\033[0m" << endl;
            cout << "==================== move_base feedback ====================" << endl;
            cout << setiosflags(ios::fixed) << setprecision(7);
            cout << "x      : " << moveBasePose.pose.position.x << endl;
            cout << "y      : " << moveBasePose.pose.position.y << endl;
            cout << "z      : " << moveBasePose.pose.position.z << endl;

            tf2::Quaternion quat;
            tf2::fromMsg(moveBasePose.pose.orientation, quat);
            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            cout << "pitch  : " << pitch * 180.0 / pi << endl;
            cout << "roll   : " << roll * 180.0 / pi << endl;
            cout << "yaw    : " << yaw * 180.0 / pi << endl;
            cout << "========================================================" << endl;
        }

        if (radarOdomRec_flag && moveBaseFeedbackRec_flag)
        {
            // 位置欧式距离差
            double dx = radarPose.pose.position.x - moveBasePose.pose.position.x;
            double dy = radarPose.pose.position.y - moveBasePose.pose.position.y;
            double dz = radarPose.pose.position.z - moveBasePose.pose.position.z;
            double dist = sqrt(dx*dx + dy*dy + dz*dz);

            // 姿态 yaw 差（简单对比，单位度）
            tf2::Quaternion quat1, quat2;
            tf2::fromMsg(radarPose.pose.orientation, quat1);
            tf2::fromMsg(moveBasePose.pose.orientation, quat2);
            double roll1, pitch1, yaw1, roll2, pitch2, yaw2;
            tf2::Matrix3x3(quat1).getRPY(roll1, pitch1, yaw1);
            tf2::Matrix3x3(quat2).getRPY(roll2, pitch2, yaw2);
            double dyaw = (yaw1 - yaw2) * 180.0 / pi;

            cout << "\033[33m[Compare] Position diff: " << dist << " m; Yaw diff: " << dyaw << " deg\033[0m" << endl;
        }

        ros::spinOnce();
        rate->sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_pose_node");
    ros::NodeHandle nh_("");
    ros::NodeHandle nh_private_("~");
    lidar_pose vision(nh_, nh_private_);
    vision.start();

    return 0;
}