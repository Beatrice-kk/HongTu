# 快速操作指南

## 1. 建图

```bash
roslaunch fastlio mapping.launch
```

---

## 2. 保存地图

- **自定义路径和名称，默认保存为：** `cwk_map_+时间`
- **命令行自定义参数示例：**

```bash
roslaunch map_server save_map.launch mapname:=/home/yourname/yourmap occ:=70 free:=20
```

---

## 3. 编辑地图

```bash
roslaunch ros_map_edit map_edit.launch
```
- Map Eraser Tool 修改地图，`Ctrl` + `+/-` 改变画笔大小，编辑后保存。

---

## 4. 修改导航地图

- 编辑 `HongTu/G1Nav2D/src/fastlio2/launch/gridmap_load.launch`
- 手动更改地图路径为最新地图。

---

## 5. 启动导航

```bash
roslaunch fastlio navigation.launch

or

roslaunch fastlio navigation.launch use_rviz:=false

查看后台日志（遥控器启动）： tail -f /home/unitree/HongTu/PythonProject/point_nav/logs/fastlio_*.log

后续想看可视化界面可以：rviz -d $(rospack find fastlio)/rviz/localize.rviz
```
- 启动后需根据实际雷达位置重定位。

---

## 6. 启动运控

```bash
cd HongTu/unitree_sdk2_python/example/g1/high_level/
python3 g1_control.py <网口>
```

---

## 7. RViz 与点选导航

- RViz 界面点击导航
- 或在 `~/HongTu/PythonProject/point_nav/` 目录下运行脚本

---