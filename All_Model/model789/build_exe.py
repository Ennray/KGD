"""
model789 项目打包脚本
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
        '--name=model789',  # exe 文件名
        '--onefile',  # 打包成单个文件
        '--distpath=E:\\work\\model4\\out',  # 指定输出目录
        '--add-data=unified_interface_README.md;.',  # 添加文档文件
        '--hidden-import=flask',
        '--hidden-import=flask.json',
        '--hidden-import=werkzeug',
        '--hidden-import=werkzeug.routing',
        '--hidden-import=werkzeug.serving',
        '--hidden-import=werkzeug.security',
        '--hidden-import=jinja2',
        '--hidden-import=click',
        '--hidden-import=itsdangerous',
        '--hidden-import=numpy',
        '--hidden-import=json',
        '--hidden-import=typing',
        '--hidden-import=GeodeticConverter',
        '--hidden-import=Impact',
        '--hidden-import=Interference',
        '--hidden-import=Dropbombs',
        '--hidden-import=Formation',
        '--hidden-import=Turnning',
        '--hidden-import=RetreatImpact',
        '--hidden-import=RetreatDropbombs',
        '--collect-all=flask',
        '--collect-all=werkzeug',
        '--collect-all=jinja2',
        'unified_interface.py'  # 主入口文件
    ]

    os.system(' '.join(cmd))
    print("\n打包完成！")
    print(f"生成的 exe 文件位于: F:\\work2\\model4789\\model789.exe")

if __name__ == '__main__':
    print("=" * 60)
    print("model789 项目打包工具")
    print("=" * 60)

    # 清理之前的构建
    clean_build()

    # 开始构建
    build_exe()

    print("\n打包完成！运行 F:\\work2\\model4789\\model789.exe 启动服务")
