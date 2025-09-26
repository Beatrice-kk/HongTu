import subprocess

# 运行 shell 脚本
script_path = "/home/unitree/HongTu/py_contrl.sh"

try:
    result = subprocess.run(["bash", script_path], check=True, capture_output=True, text=True)
    print("脚本输出：")
    print(result.stdout)
except subprocess.CalledProcessError as e:
    print("脚本执行失败：")
    print(e.stderr)