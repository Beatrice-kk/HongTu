import time
import json

def TestMove(self, vx: float, vy: float, omega: float, duration: float = 3.0, interval: float = 0.1):
    """
    连续发送速度命令，保证机器人运动
    vx, vy, omega : 速度
    duration : 总运行时间 (秒)
    interval : 每次发送命令间隔
    """
    print(f"开始测试运动: vx={vx}, vy={vy}, omega={omega}, duration={duration}s")
    
    # 循环发送速度命令
    steps = int(duration / interval)
    for i in range(steps):
        p = {
            "velocity": [vx, vy, omega],
            "duration": interval
        }
        parameter = json.dumps(p)
        code, data = self._Call(7105, parameter)  # ROBOT_API_ID_LOCO_SET_VELOCITY
        print(f"[{i+1}/{steps}] SetVelocity 返回: code={code}, data={data}")
        time.sleep(interval)
    
    # 停止机器人
    stop_param = json.dumps({"velocity": [0, 0, 0], "duration": 0.1})
    self._Call(7105, stop_param)
    print("测试完成，机器人已停止")
