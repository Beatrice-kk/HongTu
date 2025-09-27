# 优化的自动重定位系统

## 概述

这是针对后台运行环境优化的自动重定位系统，解决了原版本在无终端、无显示屏环境下的问题。

## 主要改进

### 1. 增强的错误处理和日志记录
- **详细日志记录**: 所有操作都有时间戳的日志记录
- 日志文件自动按时间戳命名，便于追踪
- 支持不同日志级别（DEBUG, INFO, WARNING, ERROR）
- 同时输出到文件和控制台

### 2. 服务依赖检查
- **服务健康检查**: 启动前检查所有必要的ROS服务
- **超时保护**: 避免无限等待服务启动
- **重试机制**: 支持多次重试失败的服务调用

### 3. 优化的重定位策略
- **智能角度搜索**: 支持多种搜索策略
- **可配置参数**: 通过YAML文件灵活配置
- **多候选位置**: 支持多个可能的起始位置
- **状态验证**: 多次验证重定位结果

### 4. 后台运行支持
- **无终端运行**: 完全支持后台运行
- **进程管理**: 自动管理相关进程的启动和停止
- **状态监控**: 实时监控重定位状态
- **优雅退出**: 支持信号处理和资源清理

## 文件结构

```
scripts/
├── auto_relocalize_improved.py    # 优化的重定位脚本
├── start_navigation_background.sh # 后台启动脚本
└── README_reloc_improved.md       # 使用说明

config/
├── reloc_improved.yaml           # 优化配置文件
└── reloc.yaml                    # 原配置文件
```

## 使用方法

### 1. 后台启动导航系统

```bash
# 启动导航系统
./start_navigation_background.sh start

# 停止导航系统
./start_navigation_background.sh stop

# 重启导航系统
./start_navigation_background.sh restart

# 检查状态
./start_navigation_background.sh status
```

### 2. 直接使用重定位脚本

```bash
# 加载配置
rosparam load config/reloc_improved.yaml /one_key_reloc_node

# 运行重定位脚本
python3 auto_relocalize_improved.py
```

### 3. 在Python代码中集成

```python
# 在g1_client_now.py中的使用示例
def _start_fastlio_navigation_improved(self):
    """使用优化的后台启动方式"""
    try:
        # 使用后台启动脚本
        result = subprocess.run([
            "/home/unitree/HongTu/G1Nav2D/src/fastlio2/scripts/start_navigation_background.sh",
            "start"
        ], capture_output=True, text=True, timeout=120)
        
        if result.returncode == 0:
            print("? 导航系统启动成功")
            return True
        else:
            print(f"? 导航系统启动失败: {result.stderr}")
            return False
            
    except subprocess.TimeoutExpired:
        print("? 导航系统启动超时")
        return False
    except Exception as e:
        print(f"? 导航系统启动异常: {e}")
        return False
```

## 配置说明

### reloc_improved.yaml 配置项

```yaml
# 地图设置
map_pcd_path: "/path/to/map.pcd"

# 初始位置
initial_pose:
  x: 0.0
  y: 0.0

# 搜索参数
yaw_search:
  range_deg: 180      # 搜索范围
  step_deg: 20        # 搜索步长
  strategy: "centered" # 搜索策略

# 重定位参数
reloc_params:
  attempt_wait_time: 1.0      # 尝试间隔
  max_status_checks: 5        # 状态检查次数
  status_check_interval: 1.0  # 检查间隔
  service_timeout: 10.0       # 服务超时

# 日志设置
logging:
  level: "INFO"       # 日志级别
  verbose: true       # 详细日志
  retention_days: 7   # 日志保留天数

# 错误处理
error_handling:
  max_retries: 3           # 最大重试次数
  retry_interval: 2.0     # 重试间隔
  try_alternative_poses: false  # 是否尝试其他位置
```

## 日志文件

日志文件保存在: `/home/unitree/HongTu/PythonProject/point_nav/logs/`

- `auto_reloc_YYYYMMDD_HHMMSS.log`: 重定位脚本日志
- `navigation_startup_YYYYMMDD_HHMMSS.log`: 导航启动日志

## 故障排除

### 1. 重定位失败
- 检查地图文件路径是否正确
- 确认初始位置是否合理
- 查看日志文件了解详细错误信息

### 2. 服务启动失败
- 确认ROS环境已正确设置
- 检查roscore是否运行
- 验证所有依赖包已安装

### 3. 后台运行问题
- 检查脚本执行权限
- 确认日志目录可写
- 查看系统资源使用情况

## 性能优化建议

1. **调整搜索参数**: 根据环境特点调整搜索范围和步长
2. **优化初始位置**: 设置更准确的初始位置估计
3. **监控资源使用**: 定期检查CPU和内存使用情况
4. **日志管理**: 定期清理旧日志文件

## 与原版本对比

| 特性 | 原版本 | 优化版本 |
|------|--------|----------|
| 错误处理 | 基础 | 完善的多层错误处理 |
| 日志记录 | 简单输出 | 详细的时间戳日志 |
| 服务检查 | 无 | 完整的服务健康检查 |
| 后台运行 | 有限支持 | 完全支持 |
| 配置管理 | 固定 | 灵活的YAML配置 |
| 进程管理 | 手动 | 自动进程管理 |
| 状态监控 | 基础 | 实时状态监控 |

## 注意事项

1. **权限设置**: 确保脚本有执行权限
2. **环境变量**: 确保ROS环境变量正确设置
3. **资源监控**: 定期检查系统资源使用情况
4. **日志清理**: 定期清理旧日志文件避免磁盘空间不足
