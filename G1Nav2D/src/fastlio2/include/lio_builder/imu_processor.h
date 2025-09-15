#pragma once
#include <memory>
#include "commons.h"
#include <Eigen/Geometry>

namespace fastlio
{
   // 位姿结构体：保存 IMU 在某一时刻的状态信息
   struct Pose
   {
      Pose();
      Pose(double t, Eigen::Vector3d a, Eigen::Vector3d g, Eigen::Vector3d v, Eigen::Vector3d p, Eigen::Matrix3d r)
          : offset(t), acc(a), gyro(g), vel(v), pos(p), rot(r) {}
      double offset;        // 时间偏移量（相对于参考时刻）
      Eigen::Vector3d acc;  // 加速度测量
      Eigen::Vector3d gyro; // 陀螺仪测量
      Eigen::Matrix3d rot;  // 姿态旋转矩阵
      Eigen::Vector3d pos;  // 位置
      Eigen::Vector3d vel;  // 速度
   };

   // IMU 处理器：用于初始化、去畸变点云、协方差配置等
   class IMUProcessor
   {
   public:
      // 构造函数：传入一个 ESKF（扩展卡尔曼滤波器）对象
      IMUProcessor(std::shared_ptr<esekfom::esekf<state_ikfom, 12, input_ikfom>> kf);

      // 初始化 IMU 状态（用于系统刚启动时）
      void init(const MeasureGroup &meas);

      // 利用 IMU 轨迹对点云进行去畸变
      void undistortPointcloud(const MeasureGroup &meas, PointCloudXYZI::Ptr &out);

      // 主处理函数：输入测量数据，输出去畸变后的点云
      bool operator()(const MeasureGroup &meas, PointCloudXYZI::Ptr &out);

      // 获取 IMU 加速度和陀螺仪的平均值（用于初始化对齐）
      Eigen::Vector3d getMeanAcc() const { return mean_acc_; }
      Eigen::Vector3d getMeanGyro() const { return mean_gyro_; }

      // 判断是否完成初始化
      bool isInitialized() const { return init_flag_; }

      // 设置最大初始化次数（用于估计初始重力方向）
      void setMaxInitCount(int max_init_count) { max_init_count_ = max_init_count; }

      // 设置外参：IMU 到 LiDAR 的旋转和平移
      void setExtParams(Eigen::Matrix3d &rot_ext, Eigen::Vector3d &pos_ext);

      // 设置传感器噪声协方差（加速度、陀螺仪及其偏置）
      void setAccCov(Eigen::Vector3d acc_cov) { acc_cov_ = acc_cov; }
      void setGyroCov(Eigen::Vector3d gyro_cov) { gyro_cov_ = gyro_cov; }
      void setAccBiasCov(Eigen::Vector3d acc_bias_cov) { acc_bias_cov_ = acc_bias_cov; }
      void setGyroBiasCov(Eigen::Vector3d gyro_bias_cov) { gyro_bias_cov_ = gyro_bias_cov; }

      // 一次性设置所有协方差参数
      void setCov(Eigen::Vector3d gyro_cov, Eigen::Vector3d acc_cov, Eigen::Vector3d gyro_bias_cov, Eigen::Vector3d acc_bias_cov);
      void setCov(double gyro_cov, double acc_cov, double gyro_bias_cov, double acc_bias_cov);

      // 是否在初始化时将姿态对齐到重力方向
      void setAlignGravity(bool align_gravity) { align_gravity_ = align_gravity; }

      // 重置 IMU 处理器状态
      void reset();

   private:
      // 初始化过程
      int init_count_ = 0;                                               // 当前已累计的初始化次数
      int max_init_count_ = 20;                                          // 最大初始化次数
      Eigen::Matrix3d rot_ext_;                                          // 外参旋转（IMU->LiDAR）
      Eigen::Vector3d pos_ext_;                                          // 外参平移（IMU->LiDAR）
      std::shared_ptr<esekfom::esekf<state_ikfom, 12, input_ikfom>> kf_; // ESKF 滤波器对象

      IMU last_imu_;              // 上一帧 IMU 数据
      bool init_flag_ = false;    // 是否完成初始化
      bool align_gravity_ = true; // 是否进行重力方向对齐

      Eigen::Vector3d mean_acc_;  // 初始化时的平均加速度（用于估计重力）
      Eigen::Vector3d mean_gyro_; // 初始化时的平均角速度（用于判断是否静止）

      Eigen::Vector3d last_acc_;  // 上一次 IMU 的加速度
      Eigen::Vector3d last_gyro_; // 上一次 IMU 的角速度

      std::vector<Pose> imu_poses_; // 存储 IMU 积分得到的轨迹（用于点云去畸变）

      double last_lidar_time_end_; // 上一帧 LiDAR 扫描的结束时间

      // 协方差参数
      Eigen::Vector3d gyro_cov_;      // 陀螺仪噪声协方差
      Eigen::Vector3d acc_cov_;       // 加速度噪声协方差
      Eigen::Vector3d gyro_bias_cov_; // 陀螺仪偏置噪声协方差
      Eigen::Vector3d acc_bias_cov_;  // 加速度偏置噪声协方差

      Eigen::Matrix<double, 12, 12> Q_; // IMU 噪声协方差矩阵（总的过程噪声）
   };
} // namespace fastlio
