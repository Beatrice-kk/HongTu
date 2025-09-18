import subprocess

# 注意路径要用引号
script_path = "/home/unitree/HongTu/new_sh.sh"

result = subprocess.run(["/bin/bash", script_path], capture_output=True, text=True)

print("标准输出:\n", result.stdout)
print("标准错误:\n", result.stderr)
print("返回码:", result.returncode)
