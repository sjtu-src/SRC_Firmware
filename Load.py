import tkinter as tk
from tkinter import messagebox, filedialog
import subprocess
import os
import re

# 获取脚本的当前路径
script_dir = os.path.dirname(os.path.abspath(__file__))

# 构造相对路径
project_path = os.path.join(script_dir, "MDK-ARM", "SRC_Firmware.uvprojx")
cfg_file_path = os.path.join(script_dir, "Core", "Inc", "cfg.h")
# Fixed paths
PROJECT_PATH = project_path
CFG_FILE_PATH = cfg_file_path
UV4_PATH = r"D:\Keil_v5\UV4\UV4.exe"

# 可能的 ROBOT_VERSION 值
ROBOT_VERSIONS = ["LEBO_ROBOT", "NEUMANN_S01", "NEUMANN_S02"]

def update_cfg_file(cfg_file, value):
    try:
        with open(cfg_file, 'r', encoding='utf-8') as file:
            lines = file.readlines()

        with open(cfg_file, 'w', encoding='utf-8') as file:
            for line in lines:
                if "#define ROBOT_VERSION" in line:
                    line = re.sub(r'#define\s+ROBOT_VERSION\s+\w+', f'#define ROBOT_VERSION {value}', line)
                file.write(line)
        print(f"Updated {cfg_file} with ROBOT_VERSION = {value}")
    except Exception as e:
        print(f"Error updating {cfg_file}: {e}")
        messagebox.showerror("Error", f"Error updating {cfg_file}: {e}")
        return False
    return True

def build_and_flash():
    value = selected_version.get()

    if not os.path.exists(UV4_PATH):
        messagebox.showerror("Error", "Invalid Keil UV4 path")
        return

    if not update_cfg_file(CFG_FILE_PATH, value):
        return

    try:
        # 使用绝对路径
        project_path = os.path.abspath(PROJECT_PATH)
        subprocess.run([UV4_PATH, "-b", project_path], check=True)
        subprocess.run([UV4_PATH, "-f", project_path], check=True)
        messagebox.showinfo("Success", "Build and Flash completed successfully")
    except subprocess.CalledProcessError as e:
        messagebox.showerror("Error", f"Build or Flash failed: {e}")

# 创建主窗口
root = tk.Tk()
root.title("Burn Tool")

# 创建和放置控件
tk.Label(root, text="Robot Version:").grid(row=0, column=0, padx=10, pady=5)
selected_version = tk.StringVar(root)
selected_version.set(ROBOT_VERSIONS[0])  # 默认值
version_option = tk.OptionMenu(root, selected_version, *ROBOT_VERSIONS)
version_option.grid(row=0, column=1, padx=10, pady=5)

build_button = tk.Button(root, text="Build and Flash", command=build_and_flash)
build_button.grid(row=1, column=0, columnspan=2, pady=10)

root.mainloop()