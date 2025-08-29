#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/filesystem.hpp>
#include <sstream>
#include <iomanip>

class RealTimePCDSaver {
public:
    RealTimePCDSaver(ros::NodeHandle& nh)
        : count_(0)
    {
        // 参数获取
        nh.param<std::string>("save_folder", save_folder_, "/home/nvidia/bag");
        nh.param<bool>("save_map", save_map_, false);

        // 创建保存目录
        boost::filesystem::create_directories(save_folder_ + "/pcd");

        // 订阅与同步器
        sub_odom_.subscribe(nh, "/odom_topic", 10);
        sub_pc2_.subscribe(nh, "/pc2_topic", 10);
        sync_.reset(new Sync(SyncPolicy(10), sub_odom_, sub_pc2_));
        sync_->registerCallback(boost::bind(&RealTimePCDSaver::callback, this, _1, _2));

        cloud_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        ROS_INFO("RealTimePCDSaver started.");
    }

    ~RealTimePCDSaver() {
        if (save_map_) {
            std::string map_file = save_folder_ + "/raw_map.pcd";
            pcl::io::savePCDFileBinary(map_file, *cloud_map_);
            ROS_INFO_STREAM("Saved raw map to: " << map_file << ", with " << cloud_map_->size() << " points.");
        }
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;

    void callback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::PointCloud2ConstPtr& pc2) {
        // 读取位姿
        Eigen::Vector3f pos(
            odom->pose.pose.position.x,
            odom->pose.pose.position.y,
            odom->pose.pose.position.z
        );
        Eigen::Quaternionf q(
            odom->pose.pose.orientation.w,
            odom->pose.pose.orientation.x,
            odom->pose.pose.orientation.y,
            odom->pose.pose.orientation.z
        );
        ROS_INFO("Processing frame %d: Position: [%f, %f, %f], Orientation: [%f, %f, %f, %f]",
                 count_, pos.x(), pos.y(), pos.z(), q.w(), q.x(), q.y(), q.z());

        // 点云转换
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*pc2, *pcl_cloud);

        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3,3>(0,0) = q.toRotationMatrix();
        transform.block<3,1>(0,3) = pos;
        pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, transform);

        // 设置 viewpoint
        pcl_cloud->sensor_origin_ = Eigen::Vector4f(pos.x(), pos.y(), pos.z(), 0.0);
        pcl_cloud->sensor_orientation_ = q;

        // 保存单帧 pcd
        std::ostringstream oss;
        oss << save_folder_ << "/pcd/" << std::setw(6) << std::setfill('0') << count_ << ".pcd";
        pcl::io::savePCDFileBinary(oss.str(), *pcl_cloud);
        ROS_INFO_STREAM("Saved " << oss.str());

        // 拼入地图（可选）
        if (save_map_) {
            cloud_map_->insert(cloud_map_->end(), pcl_cloud->begin(), pcl_cloud->end());
        }

        count_++;
    }

    std::string save_folder_;
    bool save_map_;
    int count_;

    message_filters::Subscriber<nav_msgs::Odometry> sub_odom_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pc2_;
    std::shared_ptr<Sync> sync_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "realtime_pcd_saver");
    ros::NodeHandle nh("~");

    RealTimePCDSaver saver(nh);
    ros::spin();
    return 0;
}
