@echo off
chcp 65001 >nul
echo ====================================
echo   目标检测完整测试 (pytest)
echo ====================================
echo.

REM 使用系统Python运行pytest
python -m pytest test_detection.py -v --tb=short -s

echo.
echo 测试完成！按任意键退出...
pause >nul
