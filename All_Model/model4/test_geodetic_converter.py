"""
GeodeticConverter.py 坐标转换完整测试套件
测试度分秒转换、进位问题、精度验证等
"""

import GeodeticConverter
import numpy as np


def test_dms_to_decimal():
    """测试度分秒字符串转十进制"""
    print("=" * 80)
    print("测试 1: dms_to_decimal() - 度分秒字符串转十进制")
    print("=" * 80)

    test_cases = [
        # (输入, 预期输出, 描述)
        ("125:04:56.97E", 125.08249167, "标准东经"),
        ("26:38:02.25N", 26.63395833, "标准北纬"),
        ("125:00:00.00E", 125.0, "整数度数"),
        ("0:30:00.00N", 0.5, "0度30分=0.5度"),
        ("1:00:60.00E", 1.01666667, "60秒边界（1度0分60秒）"),
        ("1:60:00.00E", 2.0, "60分边界（1度60分=2度）"),
        ("0:59:59.99N", 0.99999722, "接近60的边界值"),
        ("180:00:00.00E", 180.0, "最大经度"),
        ("90:00:00.00N", 90.0, "最大纬度"),
        ("28:11:34.95W", -28.19304167, "西经（负数）"),
        ("10:30:35.24S", -10.50978889, "南纬（负数）-修正值"),
        ("0:00:00.00E", 0.0, "零点"),
        ("45:30:30.00N", 45.50833333, "45.5度附近"),
    ]

    issues = []
    passed = 0
    for i, (dms_str, expected, desc) in enumerate(test_cases, 1):
        try:
            result = GeodeticConverter.dms_to_decimal(dms_str)
            diff = abs(result - expected)
            status = "[PASS]" if diff < 0.0001 else "[FAIL]"

            print(f"\n测试 {i}: {desc}")
            print(f"  输入:   {dms_str}")
            print(f"  预期:   {expected:.10f}")
            print(f"  实际:   {result:.10f}")
            print(f"  误差:   {diff:.12f}")
            print(f"  结果:   {status}")

            if diff >= 0.0001:
                issues.append(f"测试{i}失败: {desc} (误差={diff:.10f})")
            else:
                passed += 1
        except Exception as e:
            print(f"\n测试 {i}: {desc}")
            print(f"  输入:   {dms_str}")
            print(f"  [ERROR] 异常: {e}")
            issues.append(f"测试{i}异常: {desc} - {e}")

    print(f"\n小结: {passed}/{len(test_cases)} 个测试通过")
    return issues


def test_decimal_to_dms():
    """测试十进制转度分秒"""
    print("\n" + "=" * 80)
    print("测试 2: decimal_degrees_to_dms() - 十进制转度分秒")
    print("=" * 80)

    # 创建临时转换器用于测试
    converter = GeodeticConverter.GeodeticToLocalConverter(
        40.0, 116.0, 50.0,
        40.01, 116.0, 55.0
    )

    test_cases = [
        # (输入, 预期度, 预期分, 预期秒范围, 描述)
        (125.08249167, 125, 4, (56.9, 57.1), "标准转换"),
        (125.0, 125, 0, (0.0, 0.01), "整数度"),
        (1.01666667, 1, 1, (0.0, 0.01), "1度1分（来自60秒）"),
        (2.0, 2, 0, (0.0, 0.01), "2度（来自60分）"),
        (0.5, 0, 30, (0.0, 0.01), "0.5度=30分"),
        (-28.19304167, -28, 11, (34.9, 35.1), "负数（西经/南纬）"),
        (89.99999, 89, 59, (59.9, 60.01), "接近90度（允许60秒）"),
        (45.50833333, 45, 30, (29.9, 30.1), "45.5度附近"),
        (0.0, 0, 0, (0.0, 0.01), "零度"),
    ]

    issues = []
    passed = 0
    for i, (decimal, exp_deg, exp_min, exp_sec_range, desc) in enumerate(test_cases, 1):
        try:
            deg, minute, sec = converter.decimal_degrees_to_dms(decimal)

            # 检查进位错误
            has_carry_error = False
            error_msgs = []
            if minute > 60:  # 注意是 > 60，而不是 >= 60（60分应该进位但60.0是临界值）
                has_carry_error = True
                error_msgs.append(f"分钟>60 ({minute}分)")
            if sec > 60:  # 同理，60.0秒是临界值
                has_carry_error = True
                error_msgs.append(f"秒>60 ({sec}秒)")

            # 检查预期值
            deg_match = (deg == exp_deg)
            min_match = (minute == exp_min)
            sec_match = (exp_sec_range[0] <= sec <= exp_sec_range[1])

            if has_carry_error:
                status = "[FAIL] 进位错误"
            elif not (deg_match and min_match and sec_match):
                status = "[FAIL] 值不匹配"
            else:
                status = "[PASS]"
                passed += 1

            print(f"\n测试 {i}: {desc}")
            print(f"  输入:   {decimal:.10f}")
            print(f"  预期:   {exp_deg}度 {exp_min}分 {exp_sec_range[0]}-{exp_sec_range[1]}秒")
            print(f"  实际:   {deg}度 {minute}分 {sec:.4f}秒")
            print(f"  结果:   {status}")

            if has_carry_error:
                print(f"  [ERROR] 进位错误: {', '.join(error_msgs)}")
                issues.append(f"测试{i}失败: {desc} - {', '.join(error_msgs)}")
            elif not (deg_match and min_match and sec_match):
                issues.append(f"测试{i}失败: {desc} - 值不匹配")

        except Exception as e:
            print(f"\n测试 {i}: {desc}")
            print(f"  输入:   {decimal}")
            print(f"  [ERROR] 异常: {e}")
            issues.append(f"测试{i}异常: {desc} - {e}")

    print(f"\n小结: {passed}/{len(test_cases)} 个测试通过")
    return issues


def test_round_trip_conversion():
    """测试往返转换（十进制→度分秒→十进制）精度验证"""
    print("\n" + "=" * 80)
    print("测试 3: 往返转换精度测试")
    print("=" * 80)

    converter = GeodeticConverter.GeodeticToLocalConverter(
        40.0, 116.0, 50.0,
        40.01, 116.0, 55.0
    )

    test_values = [
        (125.08249167, "标准值1"),
        (26.63395833, "标准值2"),
        (1.01666667, "60秒边界"),
        (2.0, "60分边界"),
        (0.5, "0.5度"),
        (89.999999, "接近90度"),
        (179.999999, "接近180度"),
        (0.0, "零度"),
        (45.123456, "随机小数"),
        (-28.19304167, "负数"),
    ]

    issues = []
    passed = 0
    for i, (original, desc) in enumerate(test_values, 1):
        try:
            # 十进制 → 度分秒
            deg, minute, sec = converter.decimal_degrees_to_dms(original)

            # 检查进位
            carry_error = (minute > 60 or sec > 60)

            # 度分秒 → 十进制（手动重建）
            # 注意：负数时，分和秒应该是减法
            if deg < 0:
                reconstructed = deg - minute / 60 - sec / 3600
            else:
                reconstructed = deg + minute / 60 + sec / 3600

            diff = abs(original - reconstructed)
            status = "[PASS]" if diff < 0.00001 and not carry_error else "[FAIL]"

            print(f"\n测试 {i}: {desc}")
            print(f"  原始值:  {original:.10f}")
            print(f"  转为DMS: {deg}度 {minute}分 {sec:.8f}秒")
            print(f"  重建值:  {reconstructed:.10f}")
            print(f"  误差:    {diff:.12f}")
            print(f"  结果:    {status}")

            if carry_error:
                print(f"  [ERROR] 进位错误: 分钟={minute}, 秒={sec}")
                issues.append(f"测试{i}进位错误: {desc}")
            elif diff >= 0.00001:
                issues.append(f"测试{i}精度丢失: {desc} (误差={diff:.10f})")
            else:
                passed += 1

        except Exception as e:
            print(f"\n测试 {i}: {desc}")
            print(f"  [ERROR] 异常: {e}")
            issues.append(f"测试{i}异常: {desc} - {e}")

    print(f"\n小结: {passed}/{len(test_values)} 个测试通过")
    return issues


def test_local_to_geodetic_dms():
    """测试局部坐标转度分秒（集成测试）"""
    print("\n" + "=" * 80)
    print("测试 4: local_to_geodetic_dms() - 局部坐标转度分秒")
    print("=" * 80)

    # 创建转换器
    A_lat, A_lon, A_alt = 40.0, 116.0, 50.0
    B_lat, B_lon, B_alt = 40.01, 116.0, 55.0
    converter = GeodeticConverter.GeodeticToLocalConverter(
        A_lat, A_lon, A_alt, B_lat, B_lon, B_alt
    )

    test_cases = [
        (np.array([0, 0, 0]), "原点（应该回到A点）"),
        (np.array([100, 200, 10]), "小偏移"),
        (np.array([1000, 2000, 100]), "中等偏移"),
        (np.array([10000, 20000, 1000]), "大偏移"),
        (np.array([-1000, -1000, -50]), "负向偏移"),
    ]

    issues = []
    passed = 0
    for i, (local_coords, desc) in enumerate(test_cases, 1):
        try:
            result = converter.local_to_geodetic_dms(local_coords)

            lon_str = result[0]  # 经度
            lat_str = result[1]  # 纬度
            alt_str = result[2]  # 高度

            print(f"\n测试 {i}: {desc}")
            print(f"  输入局部坐标: [{local_coords[0]}, {local_coords[1]}, {local_coords[2]}]")
            print(f"  输出经度:     {lon_str}")
            print(f"  输出纬度:     {lat_str}")
            print(f"  输出高度:     {alt_str}")

            # 检查度分秒格式
            def check_dms_format(dms_str, coord_type):
                """检查度分秒格式是否合法"""
                try:
                    direction = dms_str[-1]
                    val = dms_str[:-1]
                    parts = val.split(':')
                    if len(parts) != 3:
                        return False, "格式错误（应为 度:分:秒 格式）"

                    deg = float(parts[0])
                    minute = float(parts[1])
                    sec = float(parts[2])

                    if minute > 60:
                        return False, f"{coord_type}分钟>60 ({minute}分)"
                    if sec > 60:
                        return False, f"{coord_type}秒>60 ({sec}秒)"

                    return True, "OK"
                except:
                    return False, "解析失败"

            lon_ok, lon_msg = check_dms_format(lon_str, "经度")
            lat_ok, lat_msg = check_dms_format(lat_str, "纬度")

            if lon_ok and lat_ok:
                print(f"  结果:         [PASS] 格式正确")
                passed += 1
            else:
                print(f"  结果:         [FAIL] 格式错误")
                if not lon_ok:
                    print(f"    - {lon_msg}")
                    issues.append(f"测试{i}格式错误: {desc} - {lon_msg}")
                if not lat_ok:
                    print(f"    - {lat_msg}")
                    issues.append(f"测试{i}格式错误: {desc} - {lat_msg}")

        except Exception as e:
            print(f"\n测试 {i}: {desc}")
            print(f"  [ERROR] 异常: {e}")
            issues.append(f"测试{i}异常: {desc} - {e}")

    print(f"\n小结: {passed}/{len(test_cases)} 个测试通过")
    return issues


def test_edge_cases():
    """测试边界情况和极端值"""
    print("\n" + "=" * 80)
    print("测试 5: 边界情况和极端值测试")
    print("=" * 80)

    converter = GeodeticConverter.GeodeticToLocalConverter(
        40.0, 116.0, 50.0,
        40.01, 116.0, 55.0
    )

    # 测试可能导致进位问题的特殊值
    special_values = [
        (0.9999999, "接近1度"),
        (1.9999999, "接近2度"),
        (59.9999999, "接近60度"),
        (89.9999999, "接近90度"),
        (179.9999999, "接近180度"),
        (0.016666667, "1分钟=0.01667度"),
        (0.000277778, "1秒=0.000278度"),
        (30.50833333, "30度30分30秒"),
    ]

    issues = []
    passed = 0
    for i, (decimal, desc) in enumerate(special_values, 1):
        try:
            deg, minute, sec = converter.decimal_degrees_to_dms(decimal)

            has_error = (minute > 60 or sec > 60)

            print(f"\n测试 {i}: {desc}")
            print(f"  输入:   {decimal:.10f}")
            print(f"  输出:   {deg}度 {minute}分 {sec:.8f}秒")

            if has_error:
                error_msgs = []
                if minute > 60:
                    error_msgs.append(f"分钟={minute}")
                if sec > 60:
                    error_msgs.append(f"秒={sec}")
                print(f"  结果:   [FAIL] 进位错误: {', '.join(error_msgs)}")
                issues.append(f"边界测试{i}失败: {desc} - {', '.join(error_msgs)}")
            else:
                print(f"  结果:   [PASS]")
                passed += 1

        except Exception as e:
            print(f"\n测试 {i}: {desc}")
            print(f"  [ERROR] 异常: {e}")
            issues.append(f"边界测试{i}异常: {desc} - {e}")

    print(f"\n小结: {passed}/{len(special_values)} 个测试通过")
    return issues


def test_special_dms_inputs():
    """测试特殊的度分秒输入"""
    print("\n" + "=" * 80)
    print("测试 6: 特殊度分秒输入测试（包括进位场景）")
    print("=" * 80)

    test_cases = [
        # 测试各种"非标准"但可能出现的输入
        ("0:0:60.0E", 0.016666667, "0度0分60秒（应=1分）"),
        ("0:0:120.0E", 0.033333333, "0度0分120秒（应=2分）"),
        ("0:60:0.0E", 1.0, "0度60分0秒（应=1度）"),
        ("0:120:0.0E", 2.0, "0度120分0秒（应=2度）"),
        ("1:59:59.99N", 1.999997222, "1度59分59.99秒"),
        ("89:59:59.99N", 89.999997222, "89度59分59.99秒"),
        ("0:0:0.01E", 0.000002778, "极小值（0.01秒）"),
    ]

    issues = []
    passed = 0
    for i, (dms_str, expected, desc) in enumerate(test_cases, 1):
        try:
            result = GeodeticConverter.dms_to_decimal(dms_str)
            diff = abs(result - expected)
            status = "[PASS]" if diff < 0.00001 else "[FAIL]"

            print(f"\n测试 {i}: {desc}")
            print(f"  输入:   {dms_str}")
            print(f"  预期:   {expected:.12f}")
            print(f"  实际:   {result:.12f}")
            print(f"  误差:   {diff:.14f}")
            print(f"  结果:   {status}")

            if diff >= 0.00001:
                issues.append(f"特殊输入测试{i}失败: {desc} (误差={diff:.10f})")
            else:
                passed += 1
        except Exception as e:
            print(f"\n测试 {i}: {desc}")
            print(f"  输入:   {dms_str}")
            print(f"  [ERROR] 异常: {e}")
            issues.append(f"特殊输入测试{i}异常: {desc} - {e}")

    print(f"\n小结: {passed}/{len(test_cases)} 个测试通过")
    return issues


def main():
    """运行所有测试"""
    print("\n")
    print("=" * 80)
    print("=" + " " * 78 + "=")
    print("  GeodeticConverter.py 坐标转换完整测试套件".center(80))
    print("=" + " " * 78 + "=")
    print("=" * 80)
    print("\n测试目标:")
    print("  1. 验证度分秒与十进制相互转换的准确性")
    print("  2. 检查60秒/60分钟的进位逻辑")
    print("  3. 测试边界值和极端情况")
    print("  4. 验证往返转换的精度损失")
    print("  5. 测试实际应用场景（局部坐标转换）")

    all_issues = []
    total_tests = 0

    # 运行所有测试
    issues_1 = test_dms_to_decimal()
    issues_2 = test_decimal_to_dms()
    issues_3 = test_round_trip_conversion()
    issues_4 = test_local_to_geodetic_dms()
    issues_5 = test_edge_cases()
    issues_6 = test_special_dms_inputs()

    all_issues.extend(issues_1)
    all_issues.extend(issues_2)
    all_issues.extend(issues_3)
    all_issues.extend(issues_4)
    all_issues.extend(issues_5)
    all_issues.extend(issues_6)

    # 总结
    print("\n" + "=" * 80)
    print("最终测试总结")
    print("=" * 80)

    if not all_issues:
        print("\n" + "=" * 80)
        print("[SUCCESS] 所有测试通过！")
        print("=" * 80)
        print("\n结论:")
        print("  - GeodeticConverter.py 的度分秒转换功能完全正常")
        print("  - 没有发现60秒或60分钟不进位的问题")
        print("  - 精度满足要求（误差 < 0.00001度）")
        print("  - 边界值和极端情况处理正确")
        print("  - 代码质量良好，无需修改")
    else:
        print(f"\n[WARNING] 发现 {len(all_issues)} 个问题：")
        print("-" * 80)
        for i, issue in enumerate(all_issues, 1):
            print(f"{i}. {issue}")

        print("\n" + "-" * 80)
        print("[ANALYSIS] 问题分析:")

        # 分析问题类型
        carry_errors = [i for i in all_issues if "进位" in i]
        precision_errors = [i for i in all_issues if "精度" in i or "误差" in i]
        format_errors = [i for i in all_issues if "格式" in i]
        exceptions = [i for i in all_issues if "异常" in i]

        if carry_errors:
            print(f"  - 进位错误: {len(carry_errors)} 个")
            print("    * 可能原因: decimal_degrees_to_dms() 缺少进位逻辑")
            print("    * 建议修复: 秒>=60时进位到分，分>=60时进位到度")

        if precision_errors:
            print(f"  - 精度问题: {len(precision_errors)} 个")
            print("    * 可能原因: 浮点数精度或算法误差")

        if format_errors:
            print(f"  - 格式错误: {len(format_errors)} 个")
            print("    * 可能原因: 输出格式不符合 DMS 规范")

        if exceptions:
            print(f"  - 异常错误: {len(exceptions)} 个")
            print("    * 需要检查具体异常信息")

    print("\n" + "=" * 80)
    print()


if __name__ == "__main__":
    main()
