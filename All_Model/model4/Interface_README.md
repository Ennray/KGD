# Model4 - 转弯占位算法接口文档

## 接口概述

Model4 提供了无人机集群转弯占位算法的 HTTP API 接口。该算法用于计算多波次无人机在迎头、转弯、追击等复杂机动过程中的最优路径和占位策略。

**服务地址：** `http://localhost:8000`
**主接口：** `POST /execute_main`
**方法：** POST
**Content-Type：** application/json

---

## 快速开始

### 1. 启动服务

```bash
cd E:\work\model4\model4789\model4
python Interface.py
```

服务将在 `http://localhost:8000` 启动。

### 2. 查看接口说明

访问 `http://localhost:8000/` 可查看所有可用接口。

### 3. 健康检查

```
GET http://localhost:8000/health
```

---

## 完整配置参数说明

### 配置结构

```json
{
  "config": {
    "basepoint": ["经度", "纬度", "高度"],
    "enemy_approx": ["经度", "纬度", "高度"],
    "enemy_lonrange": ["最小经度", "最大经度"],
    "enemy_latrange": ["最小纬度", "最大纬度"],
    "first_uavs": [["经度", "纬度", "高度"], ...],
    "second_uav_groups": [[["经度", "纬度", "高度"], ...], ...],
    "min_detect": [横向距离, 纵向距离],
    "detect_distance": 探测距离,
    "maximum_speed": 最大速度,
    "minimum_speed": 最小速度,
    "speed": 敌机速度,
    "acceleration": 加速度,
    "radii": [半径1, 半径2, ...],
    "enemy_number": 敌机数量,
    "column": 纵队数量
  }
}
```

### 参数详解

#### 1. 基础位置参数

| 参数 | 类型 | 必填 | 说明 | 示例 |
|------|------|------|------|------|
| `basepoint` | Array[3] | 是 | 基准点坐标 [经度, 纬度, 高度(米)] | `["125:04:56.97E", "26:38:02.25N", "5000"]` |
| `enemy_approx` | Array[3] | 是 | 敌机中心近似位置 [经度, 纬度, 高度(米)] | `["126:08:45.00E", "26:34:30.98N", "5000"]` |
| `enemy_lonrange` | Array[2] | 否 | 敌群经度范围 [最小, 最大] | `["126:06:19.94E", "126:11:10.06E"]` |
| `enemy_latrange` | Array[2] | 否 | 敌群纬度范围 [最小, 最大] | `["26:32:21.25N", "26:36:40.71N"]` |

**坐标格式说明：**
- 经度格式：`度:分:秒.小数E/W`（E 为东经，W 为西经）
- 纬度格式：`度:分:秒.小数N/S`（N 为北纬，S 为南纬）
- 高度单位：米

#### 2. 无人机位置参数

| 参数 | 类型 | 必填 | 说明 | 示例 |
|------|------|------|------|------|
| `first_uavs` | Array[Array[3]] | 是 | 第1波次无人机位置列表 | `[["125:4:52.52E", "26:36:48.33N", "5856.38"], ...]` |
| `first_num` | Integer | 否 | 第1波次无人机数量（默认为 first_uavs 长度） | `53` |
| `second_uav_groups` | Array[Array[Array[3]]] | 是 | 第2波次无人机分组列表（每组为一个纵队） | 见下方示例 |
| `column` | Integer | 否 | 纵队数量（默认为 second_uav_groups 长度） | `2` |

**second_uav_groups 格式说明：**
```json
"second_uav_groups": [
  [  // 第1纵队
    ["125:4:39.41E", "26:38:10.29N", "5065.61"],
    ["125:4:21.38E", "26:38:11.22N", "5069.82"]
  ],
  [  // 第2纵队
    ["125:4:2.31E", "26:37:55.94N", "5073.21"],
    ["125:3:44.28E", "26:37:56.86N", "5077.50"]
  ]
]
```

**兼容旧格式（可选）：**
```json
"second_uavs_1": [["经度", "纬度", "高度"], ...],
"second_uavs_2": [["经度", "纬度", "高度"], ...]
```

#### 3. 探测参数

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `min_detect` | Array[2] | 否 | `[900, 600]` | 近距离探测范围 [横向(米), 纵向(米)] |
| `detect_distance` | Number | 否 | `30000` | 最远探测距离（米） |

#### 4. 速度和加速度参数

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `maximum_speed` | Number | 否 | `500` | 无人机最大速度（米/秒） |
| `minimum_speed` | Number | 否 | `200` | 无人机最小速度（米/秒） |
| `speed` | Number | 否 | `240` | 敌机速度（米/秒） |
| `acceleration` | Number | 否 | `80` | 加速度（米/秒²） |

#### 5. 敌群参数

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `radii` | Array[8] | 否 | `[2500, 2350, 2550, ...]` | 敌群不规则边界半径（8个方向，单位：米） |
| `enemy_number` | Number | 否 | `350` | 敌机数量 |

---

## 请求示例

### 示例 1：最简请求（推荐）

只提供必需参数，其他使用默认值：

```json
POST http://localhost:8000/execute_main
Content-Type: application/json

{
  "basepoint": ["125:04:56.97E", "26:38:02.25N", "5000"],
  "enemy_approx": ["126:08:45.00E", "26:34:30.98N", "5000"],
  "enemy_lonrange": ["126:06:19.94E", "126:11:10.06E"],
  "enemy_latrange": ["26:32:21.25N", "26:36:40.71N"],
  "first_uavs": [
    ["125:4:52.52E", "26:36:48.33N", "5856.38"],
    ["125:4:52.25E", "26:36:48.33N", "4956.42"],
    ["125:4:51.98E", "26:36:48.34N", "4056.45"]
  ],
  "second_uav_groups": [
    [
      ["125:4:39.41E", "26:38:10.29N", "5065.61"],
      ["125:4:21.38E", "26:38:11.22N", "5069.82"]
    ],
    [
      ["125:4:2.31E", "26:37:55.94N", "5073.21"],
      ["125:3:44.28E", "26:37:56.86N", "5077.50"]
    ]
  ]
}
```

### 示例 2：完整配置

包含所有可配置参数：

```json
POST http://localhost:8000/execute_main
Content-Type: application/json

{
  "config": {
    "basepoint": ["125:04:56.97E", "26:38:02.25N", "5000"],
    "enemy_approx": ["126:08:45.00E", "26:34:30.98N", "5000"],
    "enemy_lonrange": ["126:06:19.94E", "126:11:10.06E"],
    "enemy_latrange": ["26:32:21.25N", "26:36:40.71N"],
    "min_detect": [900, 600],
    "detect_distance": 30000,
    "maximum_speed": 500,
    "minimum_speed": 200,
    "speed": 240,
    "acceleration": 80,
    "radii": [2500, 2350, 2550, 2500, 2400, 2400, 2480, 2380],
    "enemy_number": 350,
    "first_num": 3,
    "first_uavs": [
      ["125:4:52.52E", "26:36:48.33N", "5856.38"],
      ["125:4:52.25E", "26:36:48.33N", "4956.42"],
      ["125:4:51.98E", "26:36:48.34N", "4056.45"]
    ],
    "column": 2,
    "second_uav_groups": [
      [
        ["125:4:39.41E", "26:38:10.29N", "5065.61"],
        ["125:4:21.38E", "26:38:11.22N", "5069.82"],
        ["125:4:3.34E", "26:38:12.14N", "5074.08"]
      ],
      [
        ["125:4:2.31E", "26:37:55.94N", "5073.21"],
        ["125:3:44.28E", "26:37:56.86N", "5077.50"]
      ]
    ]
  }
}
```

### 示例 3：使用默认数据测试

不传入任何参数，使用内置的默认数据集：

```json
POST http://localhost:8000/execute_main
Content-Type: application/json

{}
```

---

## 响应格式

### 成功响应

```json
{
  "status": "ok",
  "algorithm": "TurnOccupy",
  "data": {
    "uavs_speed": {
      "current": 200,
      "acceleration": 80,
      "max_speed": 500,
      "turn_speed": 100
    },
    "enemy_speed": 240,
    "first_init_point": [
      ["125:4:52.52E", "26:36:48.33N", "5856.38"],
      ...
    ],
    "second_init_point": [
      ["125:4:39.41E", "26:38:10.29N", "5065.61"],
      ...
    ],
    "last_init_point": [
      ["125:4:20.34E", "26:37:55.01N", "5068.96"]
    ],
    "enemy_init_point": ["126:08:45.00E", "26:34:30.98N", "5000"],

    "last_uav_turn_point": [
      ["126:02:45.67E", "26:35:12.34N", "5500.12"]
    ],
    "last_uav_after_turn_point": [
      ["126:04:23.45E", "26:36:45.67N", "5600.23"]
    ],
    "last_uav_chase_point": [
      ["126:05:12.34E", "26:37:23.45N", "5650.34"]
    ],

    "last_uav_meet_time": [120.5],
    "last_uav_turn_time": [15.3],
    "last_uav_chase_time": [45.7],

    "first_uav_turn_point": [...],
    "first_uav_after_turn_point": [...],
    "first_uav_chase_point": [...],
    "first_uav_meet_time": [...],
    "first_uav_turn_time": 18.2,
    "first_uav_chase_time": [...],

    "second_uav_turn_point": [...],
    "second_uav_after_turn_point": [...],
    "second_uav_chase_point": [...],
    "second_uav_meet_time": [...],
    "second_uav_turn_time": 18.2,
    "second_uav_chase_time": [...],

    "relative_position": [
      [时间戳, "状态描述", "第1波状态", "第2波状态", "前置转弯状态", "说明", [位置数据]],
      ...
    ]
  }
}
```

### 响应字段说明

#### 速度信息
- `uavs_speed.current`: 无人机当前速度（米/秒）
- `uavs_speed.acceleration`: 加速度（米/秒²）
- `uavs_speed.max_speed`: 最大速度（米/秒）
- `uavs_speed.turn_speed`: 转弯速度（米/秒）
- `enemy_speed`: 敌机速度（米/秒）

#### 初始点位
- `first_init_point`: 第1波次无人机初始位置列表
- `second_init_point`: 第2波次无人机初始位置列表
- `last_init_point`: 每个纵队最后一架无人机的初始位置列表
- `enemy_init_point`: 敌机群初始中心位置

#### 关键点位（每个纵队最后一架）
- `last_uav_turn_point`: 开始转弯的点位（与敌群相遇点）
- `last_uav_after_turn_point`: 转弯后的点位（完成180°转弯）
- `last_uav_chase_point`: 追击到达的点位（占位点）

#### 关键时间（每个纵队最后一架）
- `last_uav_meet_time`: 从开始到与敌群相遇的时间（秒）
- `last_uav_turn_time`: 转弯所需时间（秒）
- `last_uav_chase_time`: 追击到占位点所需时间（秒）

#### 第1波次关键信息
- `first_uav_turn_point`: 各无人机开始转弯的点位列表
- `first_uav_after_turn_point`: 各无人机转弯后的点位列表
- `first_uav_chase_point`: 各无人机追击到达的占位点列表
- `first_uav_meet_time`: 各无人机从开始到相遇的时间列表（秒）
- `first_uav_turn_time`: 转弯时间（秒）
- `first_uav_chase_time`: 各无人机追击时间列表（秒）

#### 第2波次关键信息
- `second_uav_turn_point`: 各无人机开始转弯的点位列表
- `second_uav_after_turn_point`: 各无人机转弯后的点位列表
- `second_uav_chase_point`: 各无人机追击到达的占位点列表
- `second_uav_meet_time`: 各无人机从开始到相遇的时间列表（秒）
- `second_uav_turn_time`: 转弯时间（秒）
- `second_uav_chase_time`: 各无人机追击时间列表（秒）

#### 相对位置信息
- `relative_position`: 转弯过程中各波次的状态和位置信息数组
  - 格式：`[时间, 消息, 第1波状态, 第2波状态, 前置转弯状态, 描述文本, 位置数据]`
  - 状态包括：匀速前进、加速前进、减速前进、转弯、追击、追击结束

### 错误响应

```json
{
  "status": "error",
  "message": "错误描述信息",
  "traceback": "详细错误堆栈（仅在 debug 模式下返回）"
}
```

**常见错误：**
- `400 Bad Request`: 请求格式错误或缺少必需参数
- `500 Internal Server Error`: 服务器内部错误（算法执行失败）

---

## Postman 测试步骤

### 1. 创建新请求
- 点击 Postman 中的 "New" → "HTTP Request"
- 方法选择：`POST`
- URL 输入：`http://localhost:8000/execute_main`

### 2. 设置 Headers
添加以下 Header：
```
Content-Type: application/json
```

### 3. 设置 Body
- 选择 "Body" 标签
- 选择 "raw" 选项
- 格式选择 "JSON"
- 粘贴上面的示例 1（最简请求）

### 4. 发送请求
点击 "Send" 按钮发送请求。

### 5. 查看结果
在下方的 "Response" 区域查看返回结果。

---

## 使用注意事项

### 1. 坐标格式
- **必须严格遵守** DMS（度分秒）格式
- 经度：`度:分:秒.小数E/W`
- 纬度：`度:分:秒.小数N/S`
- 示例：`"125:04:56.97E"`, `"26:38:02.25N"`

### 2. 数组长度要求
- `radii`: 必须包含 8 个元素（表示 8 个方向的半径）
- `min_detect`: 必须包含 2 个元素（横向和纵向距离）
- 所有坐标点：必须是 3 元素数组 `[经度, 纬度, 高度]`

### 3. 输入格式兼容性
接口支持三种输入格式：
1. **直接格式**（推荐）：JSON 根对象直接包含所有参数
2. **config 包装格式**：`{"config": {...}}`
3. **空对象**：使用默认数据集进行测试

### 4. 纵队配置
- 推荐使用 `second_uav_groups` 字段（二维数组）
- 也支持旧格式 `second_uavs_1`, `second_uavs_2` 等
- `column` 参数应与实际纵队数量一致

### 5. 性能考虑
- 无人机数量较多时（>100架），计算时间可能较长（几秒到几十秒）
- 建议在测试时先使用较少的无人机数量

---

## 常见问题

### Q1: 如何快速测试接口是否正常？
**A:** 发送空 JSON 对象 `{}` 到 `/execute_main`，将使用默认数据集进行测试。

### Q2: 坐标格式错误怎么办？
**A:** 确保坐标格式为 `"度:分:秒.小数方向"`，例如 `"125:04:56.97E"`。注意方向字母（E/W/N/S）必须在最后。

### Q3: 如何理解 second_uav_groups？
**A:** 这是一个三维数组：
```
[
  纵队1 [ [无人机1坐标], [无人机2坐标], ... ],
  纵队2 [ [无人机1坐标], [无人机2坐标], ... ],
  ...
]
```

### Q4: 响应时间过长怎么办？
**A:**
1. 减少无人机数量进行测试
2. 检查服务器性能
3. 查看日志中是否有警告信息

### Q5: 如何调试算法输出？
**A:**
1. 查看 `relative_position` 字段了解各波次状态变化
2. 对比各个关键点位（turn_point, after_turn_point, chase_point）
3. 检查时间参数是否合理（meet_time, turn_time, chase_time）

---

## 技术支持

如有问题，请查看：
1. 服务启动日志输出
2. Postman 响应中的 `traceback` 字段（仅 debug 模式）
3. 确认输入数据格式是否符合要求

---

## 更新日志

**v2.0** (当前版本)
- ✅ 支持 POST 请求直接传入数据
- ✅ 集成数据处理逻辑，无需手动运行 import_afsim_data.py
- ✅ 支持多种输入格式
- ✅ 添加详细的错误信息和堆栈追踪
- ✅ 兼容旧数据格式

**v1.0**
- 仅支持 GET 请求
- 需要预先处理数据
- 配置通过文件读取
