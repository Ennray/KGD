根据代码分析，以下是各个接口的配置参数详细总结：

## 1. Formation.py (阵型生成)
**配置结构：**
```python
config = {
    "algorithm_type": "formation",
    "name": "敌机阵型1-单层25x14",
    "center_dms": ["127:12:02.17E", "26:37:06.00N", "3000.0"],
    "total_planes": 350,
    "layers": 1,
    "cols_per_layer": 14,  
    "lateral_spacing_m": 60, 
    "longitudinal_spacing_m": 90,  
    "layer_height_delta":100
}
```
## 2. Turning.py (转弯算法)
**配置结构：**
```python
config = {
    "algorithm_type": "turnning",
    "enemy_center": ["128:19:36.77E", "29:28:00.11N", "5000.0"],
    "uav_center": ["121:09:10.25E", "29:40:36.34N", "5000.0"],
    "uav_position": [
        ["120:38:16.97E", "29:46:10.11N", "5000"],
        ["120:38:13.80E", "29:46:10.14N", "5000"],
        ["120:38:10.64E", "29:46:10.17N", "5000"]
    ],
    "enemy_speed": 240,
    "uav_speed": 300, 
    "uav_direction": 45,
    "radius": 20
}
```
## 3. Impact.py (撞击算法)
**配置结构：**
```python
config = {
    "algorithm_type": "impact",
    "enemy_course_deg": 270,
    "uav_course_deg": 270,
    "uav_speed": 240,
    "enemy_speed": 240,
    "uav_num": 2,
    "wing_half": 5,
    "rear_spacing_m": 90,
    "a_max": 100,
    "v_max": 500,
    "impact_type": "oblique",
    "lateral": "right",
    "enemy_dms": [
        [
          "122:26:18.60E",
          "26:42:22.32N",
          "5000.00"
        ],
        [
          "122:26:18.60E",
          "26:42:33.39N",
          "5000.00"
        ]
    ],
    "uav_dms": [
        [
          "122:29:34.05E",
          "26:42:31.20N",
          "5000.00"
        ],
        [
          "122:29:34.04E",
          "26:42:45.39N",
          "5000.00"
        ]
    ]
}
```

## 4. Interference.py (干扰算法)
**配置结构：**
```python
config = {
    "algorithm_type": "interference",
    "enemy_speed": 240,
    "uav_start_speed": 240,
    "uav_max_speed": 500,
    "time_to_interference": 10,
    "acc_max": 100,
    "bearing": 270,
    "high_distance": 50,
    "max_c_points":130,
    "coverage_neighbors":4,
    "custom_radius":500,
    "interference_mode":"limit",
    "enemy_dms": [
        [
            "122:25:51.36E",
            "26:42:14.69N",
            "5000.00"
        ],
        [
            "122:25:51.36E",
            "26:42:28.20N",
            "5000.00"
        ],
        [
            "122:25:51.36E",
            "26:42:41.70N",
            "5000.00"
        ],
        [
            "122:25:51.36E",
            "26:42:55.21N",
            "5000.00"
        ],
        [
            "122:25:51.35E",
            "26:43:08.71N",
            "5000.00"
        ]
    ],
       "uav_center": [
        "122:30:39.92E",
        "26:46:16.35N",
        "5000"
    ]
}

```

## 5. Dropbombs.py (投弹算法)
**配置结构：**
```python
config = {
    "algorithm_type": "dropbombs",
    "enemy_speed": 240,
    "uav_speed": 240,
    "spacing": 10,
    "time_to_first_drop": 1,
    "bearing": 45,
    "delta_distance": 1,
    "delta_time": 1,
    "time_buffer": 2,
    "enemy_position": [
        [
            "122:52:06.45E",
            "26:42:27.98N",
            "5000.00"
        ],
        [
            "122:52:06.44E",
            "26:42:44.21N",
            "5000.00"
        ]
    ],
    "enemy_center": [
        "122:54:40.08E",
        "26:45:02.21N",
        "5000"
    ],
    "uav_center": [
        "123:04:58.48E",
        "26:44:20.29N",
        "5000"
    ]
    
}
```
## 6. Retreatimpact.py (撞击后撤退算法)
**配置结构：**
```python
config = {
    "algorithm_type": "retreatimpact",
    "uav_speed": 120,
    "uav_direction": 90,
    "formation": "formation_2_5x10x7_front_low", 
    "uav_position": [ 
        ["120:38:16.97E", "29:46:10.11N", "5000"]
    ]
}
```

## 7. Retreatdropbombs.py (投弹后撤退算法)
**配置结构：**
```python
config = {
    "algorithm_type": "retreatdropbombs",
    "uav_speed": 120,
    "uav_direction": 90,
    "formation": "formation_2_5x10x7_front_low", 
    "uav_position": [ 
        ["120:38:16.97E", "29:46:10.11N", "5000"]
    ]
}
```





