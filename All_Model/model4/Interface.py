import json
import traceback
from flask import Flask, request, jsonify
from werkzeug.exceptions import BadRequest
import numpy as np
import TurnOccupy as TO


app = Flask(__name__)

# 关闭键排序 & 允许中文直出
app.json.sort_keys = False
app.json.ensure_ascii = False


def to_jsonable(obj):
    """将 NumPy 等类型转换为可 JSON 序列化的类型"""
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, (np.floating,)):
        return float(obj)
    if isinstance(obj, (np.ndarray,)):
        return obj.tolist()
    if isinstance(obj, (set, tuple)):
        return list(obj)
    if isinstance(obj, dict):
        # Python 3.7+ 字典本身按插入顺序保序
        return {k: to_jsonable(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [to_jsonable(v) for v in obj]
    return obj


def dms_to_decimal(dms_str: str) -> float:
    """
    将度分秒字符串转换为十进制度数
    格式: "125:04:56.97E" 或 "26:38:02.25N"
    规则: E/N 为正，W/S 为负
    """
    direction = dms_str[-1]
    value_str = dms_str[:-1]
    parts = value_str.split(':')
    degrees = float(parts[0])
    minutes = float(parts[1])
    seconds = float(parts[2])
    decimal = degrees + minutes / 60 + seconds / 3600
    return -decimal if direction in ('W', 'S') else decimal


def to_lon_lat_alt(uav_triplet):
    """
    统一输入格式为 [lon, lat, alt]（字符串）
    兼容输入: [lat, lon, alt] 或 [lon, lat, alt]
    """
    if len(uav_triplet) != 3:
        return ["", "", ""]

    a, b, c = uav_triplet

    # 通过末尾字母判断是否为纬度经度
    if isinstance(a, str) and a.endswith(('N','S')) and isinstance(b, str) and b.endswith(('E','W')):
        # [lat, lon, alt] -> [lon, lat, alt]
        return [b, a, str(c)]

    return [str(a), str(b), str(c)]


def process_input_data(raw_config: dict) -> dict:
    """
    处理输入数据，将其转换为 TurnOccupy.main() 所需的格式
    """
    # 读取基础字段
    basepoint = to_lon_lat_alt(raw_config.get('basepoint', ["", "", ""]))
    enemy_approx = to_lon_lat_alt(raw_config.get('enemy_approx', ["", "", ""]))
    near_detect = raw_config.get('min_detect', [900, 600])
    detect_distance = raw_config.get('detect_distance', 30000)
    maximum_speed = raw_config.get('maximum_speed', 500)
    minimum_speed = raw_config.get('minimum_speed', 200)
    speed = raw_config.get('speed', 240)
    acceleration = raw_config.get('acceleration', 80)
    radii = raw_config.get('radii', [2500, 2350, 2550, 2500, 2400, 2400, 2480, 2380])
    enemy_number = raw_config.get('enemy_number', 350)

    enemy_lonrange = raw_config.get('enemy_lonrange', [])
    enemy_latrange = raw_config.get('enemy_latrange', [])

    # 统一范围从小到大
    if len(enemy_lonrange) == 2:
        if dms_to_decimal(enemy_lonrange[0]) > dms_to_decimal(enemy_lonrange[1]):
            enemy_lonrange = [enemy_lonrange[1], enemy_lonrange[0]]
    if len(enemy_latrange) == 2:
        if dms_to_decimal(enemy_latrange[0]) > dms_to_decimal(enemy_latrange[1]):
            enemy_latrange = [enemy_latrange[1], enemy_latrange[0]]

    # 处理第一批无人机
    first_uavs_raw = raw_config.get('first_uavs', [])
    first_uavs = [to_lon_lat_alt(u) for u in first_uavs_raw]
    first_num = raw_config.get('first_num', len(first_uavs))

    # 处理第二批无人机（支持多纵队）
    second_groups = []

    # 方式1：使用 second_uav_groups 字段
    if isinstance(raw_config.get('second_uav_groups'), list):
        for grp in raw_config['second_uav_groups']:
            if isinstance(grp, list):
                second_groups.append([to_lon_lat_alt(u) for u in grp])

    # 方式2：使用 second_uavs_1, second_uavs_2 等字段（兼容旧格式）
    if not second_groups:
        import re
        pattern = re.compile(r'^second_uavs_(\d+)$')
        temp = []
        for k, v in raw_config.items():
            m = pattern.match(k)
            if m and isinstance(v, list):
                idx = int(m.group(1))
                temp.append((idx, [to_lon_lat_alt(u) for u in v]))
        if temp:
            temp.sort(key=lambda x: x[0])
            for _, grp in temp:
                second_groups.append(grp)

    # 如果还是没有，尝试使用 second_uavs 字段
    if not second_groups and 'second_uavs' in raw_config:
        second_uavs_raw = raw_config['second_uavs']
        if isinstance(second_uavs_raw, list):
            second_groups.append([to_lon_lat_alt(u) for u in second_uavs_raw])

    # 计算纵队参数
    K = len(second_groups)
    second_groups_counts = [len(g) for g in second_groups]
    second_num = sum(second_groups_counts)
    second_uavs = [u for grp in second_groups for u in grp]

    column = raw_config.get('column', K)
    num_of_column = raw_config.get('num_of_columns', max(second_groups_counts) if second_groups_counts else 0)
    num_per_column = second_groups_counts[:]

    # 构建配置字典
    config = {
        'basepoint': basepoint,
        'near_detect_distance': near_detect,
        'detect_distance': detect_distance,
        'maximum_speed': maximum_speed,
        'minimum_speed': minimum_speed,
        'speed': speed,
        'acceleration': acceleration,
        'column': column,
        'num_of_column': num_of_column,
        'num_per_column': num_per_column,
        'enemy_approx': enemy_approx,
        'radii': radii,
        'enemy_number': enemy_number,
        'enemy_lonrange': enemy_lonrange,
        'enemy_latrange': enemy_latrange,
        'first_num': first_num,
        'first_uavs': first_uavs,
        'second_num': second_num,
        'second_uavs': second_uavs,
        'second_groups': second_groups,
        'second_groups_counts': second_groups_counts,
    }

    # 添加动态的 second_num_i 和 second_uavs_i
    for gi, grp in enumerate(second_groups, start=1):
        config[f'second_num_{gi}'] = len(grp)
        config[f'second_uavs_{gi}'] = grp

    return config


@app.route('/', methods=['GET'])
def index():

    return jsonify({
        "service": "Model4 - 转弯占位算法接口",
        "version": "2.0",
        "endpoints": {
            "/": "GET - 查看接口说明",
            "/execute_main": "POST - 执行转弯占位算法（推荐）",
            "/health": "GET - 健康检查"
        },
        "usage": {
            "method": "POST",
            "url": "http://localhost:8000/execute_main",
            "content_type": "application/json",
            "body_format": {
                "config": {
                    "basepoint": ["经度E/W", "纬度N/S", "高度"],
                    "enemy_approx": ["经度E/W", "纬度N/S", "高度"],
                    "first_uavs": [["经度", "纬度", "高度"], "..."],
                    "second_uav_groups": [[["经度", "纬度", "高度"], "..."], "..."],
                    "其他参数": "见文档"
                }
            }
        },
        "example": {
            "basepoint": ["125:04:56.97E", "26:38:02.25N", "5000"],
            "enemy_approx": ["126:08:45.00E", "26:34:30.98N", "5000"]
        }
    }), 200


@app.route('/health', methods=['GET'])
def health():
    return jsonify({"status": "ok", "service": "model4"}), 200


@app.route('/execute_main', methods=['POST'])
def execute_main():
    """
    执行转弯占位算法的主接口
    支持两种输入格式：
    1. {"config": {...}}
    2. 直接 {...}
    """
    try:
        # 获取请求数据
        payload = request.get_json(silent=True) or {}
        if not isinstance(payload, dict):
            raise BadRequest("JSON body 必须是对象（dict）。")

        # 兼容两种写法：带 config 包一层，或者直接就是配置
        raw_config = payload.get("config") if "config" in payload else payload
        if raw_config is not None and not isinstance(raw_config, dict):
            raise BadRequest("config 必须是对象（dict）。")

        # 如果没有提供任何配置，使用默认数据
        if not raw_config:
            from data.dataset import dataset
            config = dataset()
            print("[INFO] 使用默认配置数据")
        else:
            # 处理输入数据
            print("[INFO] 处理输入数据...")
            config = process_input_data(raw_config)
            print("[INFO] 数据处理完成")

        # 调用算法主函数
        print("[INFO] 执行 TurnOccupy 算法...")
        result = TO.main(config)
        print("[INFO] 算法执行完成")

        # 转换为可 JSON 序列化的格式
        result_jsonable = to_jsonable(result)

        return jsonify({
            "status": "ok",
            "algorithm": "TurnOccupy",
            "data": result_jsonable
        }), 200

    except BadRequest as e:
        return jsonify({
            "status": "error",
            "message": str(e)
        }), 400
    except Exception as e:
        return jsonify({
            "status": "error",
            "message": f"服务器内部错误: {str(e)}",
            "traceback": traceback.format_exc()
        }), 500


if __name__ == '__main__':
    print("=" * 80)
    print("Model4 - 转弯占位算法接口")
    print("端口: 8000")
    print("访问 http://localhost:8000/ 查看接口说明")
    print("执行接口: POST http://localhost:8000/execute_main")
    print("=" * 80)
    app.run(host="0.0.0.0", port=8000, debug=True)
