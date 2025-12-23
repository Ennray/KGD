"""
model6 项目打包脚本
使用 PyInstaller 将整个项目打包成 exe 文件
"""

import os
import sys
import shutil

def clean_build():
    """清理之前的构建文件"""
    print("清理构建文件...")
    dirs_to_remove = ['build', 'dist', '__pycache__']
    for dir_name in dirs_to_remove:
        if os.path.exists(dir_name):
            shutil.rmtree(dir_name)
            print(f"已删除: {dir_name}")

    spec_files = [f for f in os.listdir('.') if f.endswith('.spec')]
    for spec_file in spec_files:
        os.remove(spec_file)
        print(f"已删除: {spec_file}")

def build_exe():
    """构建 exe 文件"""
    print("\n开始打包...")

    # PyInstaller 命令
    cmd = [
        'pyinstaller',
        '--name=model6',  # exe 文件名
        '--onefile',  # 打包成单个文件
        '--distpath=E:\\work\\model4\\model4789\\model6_suanfa',  # 指定输出目录
        '--hidden-import=numpy',
        '--hidden-import=socket',
        '--hidden-import=json',
        '--hidden-import=time',
        '--hidden-import=math',
        '--hidden-import=threading',
        '--hidden-import=warnings',
        '--hidden-import=asyncio',
        '--hidden-import=collections',
        '--hidden-import=websockets',
        '--hidden-import=websockets.server',
        '--hidden-import=websockets.client',
        '--hidden-import=websockets.legacy',
        '--hidden-import=websockets.legacy.server',
        '--hidden-import=websockets.legacy.client',
        '--hidden-import=websockets.exceptions',
        '--hidden-import=websockets.frames',
        '--hidden-import=websockets.protocol',
        '--hidden-import=GeodeticConverter_instant',
        '--hidden-import=predictor_long_term',
        '--hidden-import=maneuver_classifier',
        '--hidden-import=engine',
        '--hidden-import=engine_instant',
        '--collect-all=websockets',
        '--collect-all=numpy',
        'udp_server_enemy_analysis_Last.py'  # 主入口文件
    ]

    os.system(' '.join(cmd))
    print("\n打包完成！")
    print(f"生成的 exe 文件位于: F:\\work2\\model4789\\model6.exe")

if __name__ == '__main__':
    print("=" * 60)
    print("model6 项目打包工具")
    print("=" * 60)

    # 清理之前的构建
    clean_build()

    # 开始构建
    build_exe()

    print("\n打包完成！运行 F:\\work2\\model4789\\model6.exe 启动UDP服务")
    print("UDP端口: 5005")
    print("WebSocket端口: 7070")
