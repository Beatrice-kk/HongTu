<div align="center">
  <h1 align="center"> 「元启・鸿图HongTu」 </h1>
  <h3 align="center"> 上海元启智体 </h3>
</div>

## 介绍
> ***员工双休，教程会在工作日完善，可先自己尝试，若有疑问可加群联系。***
## 部署

### 克隆仓库
  ``` bash
  git clone https://github.com/yuanqizhiti/HongTu.git
  ```

### 2D导航
- 安装 [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)
    ```bash
    sudo apt install cmake
    ```

    ```bash
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    cd ./Livox-SDK2/
    mkdir build
    cd build
    cmake .. && make -j
    sudo make install
    ```

- 更改雷达ip及地图保存路径
  ``` bash
  # 修改本机与雷达ip
  cd HongTu/G1Nav2D/HongTu/src/livox_ros_driver2-master/config/
  gedit MID360_config.json
  
  # 修改地图保存路径，将该文件下最底部路径改为自己的电脑
  cd HongTu/G1Nav2D/src/fastlio2/src/
  gedit map_builder_node.cpp
  ```

- 编译程序
  ``` bash
  cd HongTu/G1Nav2D/
  catkin_make
  
  #遇到报错可先执行以下命令
  cd HongTu/G1Nav2D/livox_ros_driver2-master/
  ./build.sh ROS1
  cd HongTu/G1Nav2D/
  catkin_make
  ```

- 建图及保存
  ``` bash
  # 建图
  cd HongTu/G1Nav2D/
  source devel/setup.bash
  roslaunch fastlio mapping.launch
  
  # 打开新终端
  cd HongTu/G1Nav2D/
  source devel/setup.bash
  # 保存地图，自定义路径及地图名称
  rosrun map_server map_saver map:=/projected_map -f /home/nvidia/mymap
  ```

- 开启导航
  ``` bash
  # 启动导航，启动导航后需自行按照雷达位置重定位
  cd HongTu/G1Nav2D/
  source devel/setup.bash
  roslaunch fastlio navigation.launch
  ```

- 启动运控  
安装unitree_sdk2_python参考[宇树官方文档](https://github.com/unitreerobotics/unitree_sdk2_python.git)
  ``` bash
  # 打开新终端，网口可通过ifconfig命令查询自行更改
  cd HongTu/unitee_sdk2_python/example/g1/high_level/
  python3 g1_control.py 网口
  ```
在rviz中发布目标点即可自主导航

### 语音交互
> ***待补充。***


## 公司招聘
招聘岗位：  
- Slam导航算法工程师  
- 嵌入式工程师  
- 结构工程师

其余相关研发岗位均在招聘中，欢迎联系。  
  
公司地址：上海市浦东新区张江机器人谷  
投递邮箱：707556641@qq.com  

## 交流群及打赏
<table style="margin: 0 auto;">
  <tr>
    <!-- 第一张图：固定宽度200px，居中显示 -->
    <td style="padding: 0 10px; text-align: center;">
      <img src="wxzhifu.jpeg" alt="vx支付" width="300" style="height: auto;">
    </td>
    <!-- 第二张图：与第一张保持相同宽度 -->
    <td style="padding: 0 10px; text-align: center;">
      <img src="dayiqun.jpeg" alt="dayiqun" width="300" style="height: auto;">
    </td>
  </tr>
</table>
