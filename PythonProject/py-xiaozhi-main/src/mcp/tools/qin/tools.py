import subprocess
import logging
from src.utils.logging_config import get_logger

# 初始化日志记录器
logger = get_logger(__name__)


async def kiss_function(args: dict) -> str:
    """
    触发亲一下动作（ID=7），拉起指定路径的g1_arm_action_example.py脚本
    对应动作：left kiss（飞吻）
    :param args: 工具调用参数（本工具无需参数，为空字典）
    :return: 执行结果文本（成功/失败信息）
    """
    try:
        # 目标脚本的绝对路径（与原路径一致）
        script_path = r"/home/zhuo/PycharmProjects/PythonProject/unitree_sdk2_python-master/example/g1/high_level/g1_arm_action_example.py"

        # 网络接口（保持与机器人配置一致）
        network_interface = "enp2s0"

        # 动作ID：7 对应“亲一下”
        action_id = "7"

        # 执行脚本并传入参数
        result = subprocess.run(
            ["python3", script_path, network_interface, action_id],
            capture_output=True,
            text=True,
            check=True,
            encoding="utf-8",
            errors="ignore"
        )

        # 打印脚本输出到控制台
        print(f"[{script_path} 输出] {result.stdout.strip()}")

        # 记录脚本输出到日志
        logger.info(f"ID=7动作（亲一下）执行输出: {result.stdout.strip()}")

        # 构造返回结果
        output = result.stdout.strip() or "ID=7动作（亲一下）执行成功（无输出）"
        return f"ID=7动作（亲一下）执行成功：{output}"

    except subprocess.CalledProcessError as e:
        error_msg = f"脚本执行失败（状态码：{e.returncode}）：{e.stderr.strip()}"
        logger.error(error_msg)
        return error_msg

    except FileNotFoundError:
        error_msg = f"未找到文件，请检查路径：{script_path}"
        logger.error(error_msg)
        return error_msg

    except Exception as e:
        error_msg = f"工具调用异常：{str(e)}"
        logger.error(error_msg)
        return error_msg
