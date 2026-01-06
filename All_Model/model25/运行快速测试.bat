@echo off
chcp 65001 >nul
echo ====================================
echo   目标检测快速测试
echo ====================================
echo.

REM 使用系统Python运行（不使用虚拟环境）
python test_quick.py

echo.
echo 测试完成！按任意键退出...
pause >nul
