'''0723五架无人机 各自一个视场 参考敌机位置 各敌群全部位置 设置每个敌群五架'''
'''命名：无人机xxx-01；敌机xxx-01-0101（xxx-无人机编号-批号-序号）
    由于没有无人机具体id，因此编程时先用了01/02代替，后续有具体id之后再修改'''
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import json
import warnings
from collections import Counter
warnings.filterwarnings("ignore", category=UserWarning)
from matplotlib import font_manager
font_path = 'C:/Windows/Fonts/simhei.ttf'  # Windows黑体路径
font_manager.fontManager.addfont(font_path)
plt.rcParams['font.family'] = font_manager.FontProperties(fname=font_path).get_name()
########0723五架无人机 各自一个视场 参考敌机位置 各敌群全部位置 设置每个敌群五架########
########0724按照速度表单完成3.2.2##################################################
# 从文件中读取数据
def load_data(filename):
    with open(filename, 'r', encoding='utf-8') as f:
        data = json.load(f)
    return data


# 3.2.1敌群编队阵型判定
# 侦察无人机位置
def drone_location(data):
    # 随机生成xxx-01的初始位置
    #np.random.seed(42)  # 固定随机种子以便复现
    #drone01_pos = np.random.rand(3) * 20  # 在0-20范围内随机生成3D坐标
    drone01_pos = [4,1000,15]#后续修改位置
    #print(f'drone01_pos:{drone01_pos}')
    # 创建无人机位置字典
    drones = {
        data["id"]: drone01_pos
    }
    # 根据间距计算其他无人机位置
    for item in data["result"]:
        target_id = item["targetId"]
        gap = np.array([item["x_gap"], item["y_gap"], item["z_gap"]])
        drones[target_id] = (drones[data["id"]] + gap).tolist()#基准无人机的位置+间距
        #print(f"gap:{gap}")
   # print(f"drones:{drones}") #计算得到的位置存入drones字典，键为 target_id
    return drones


# 参考敌机位置Reference enemy aircraft
def Reference_enemy_aircraft(data):
    drones = drone_location(data)
    enemy_pos = {}
    i = 1
    for item in data["single_location"]:
        enemy_id = item["Reference_enemy_aircraft"]
        enemy_addpos= np.array([item["x_gap"],item["y_gap"],item["z_gap"]])
        drone_key = f"xxx-{i:02d}" #:02d确保两位数
        #enemy_pos[enemy_id] = (drones[i].values() + enemy_addpos).tolist()
        drone_pos = np.array(drones[drone_key])  # 假设 drones[i] 是类似 [4, 1000, 15] 的列表
        enemy_pos[enemy_id] = (drone_pos + enemy_addpos).tolist()  # 正确调用 tolist()
        i += 1
    #print(f"enemy_pos:{enemy_pos}")
    return enemy_pos


# 敌群内各个敌机位置
def enemy_group_position(data):
    reference_enemy_pos = Reference_enemy_aircraft(data)
    enemy_group_pos = {}
    group_spacing = data["Group_spacing"][0]
    reference_id = group_spacing["reference_id"]
    enemy_id_distance = group_spacing["enemy_id_distance"]
    for enemy_id, distance in enemy_id_distance.items():
        # 提取敌群前缀（例如：xxx-01-0102 -> xxx-01）
        group_prefix = '-'.join(enemy_id.split('-')[:2])
        # 构建参考敌机ID（例如：xxx-01 -> xxx-01-0101）
        reference_id = f"{group_prefix}-0101" #xxx-01第几架参考敌机，-0101第几批第几个
        # 获取参考敌机位置
        ref_x, ref_y, ref_z = reference_enemy_pos[reference_id]
        # 计算敌机位置
        enemy_x = ref_x + distance[0]
        enemy_y = ref_y + distance[1]
        enemy_z = ref_z + distance[2]
        enemy_group_pos[enemy_id] = [enemy_x, enemy_y, enemy_z]
    return enemy_group_pos


# 3D可视化无人机位置,参考敌机位置,各个敌群 , '^', 's', 'D', 'P'
def drone_visualization(data, drones, enemy_pos,enemy_group_pos):
    # 可视化设置
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    #print("Drones data:", drones)
    #print("Enemy data:", enemy_pos)

    # 颜色和标记样式
    drone_color = 'red'
    drone_marker = 'o'
    enemy_color = 'blue'
    enemy_marker = 'P'
    enemy_group_color = 'green'
    enemy_group_marker = '^'

    # 绘制每个无人机
    for drone_id, pos in drones.items():
        ax.scatter(pos[0], pos[1], pos[2], 
                 c=drone_color, marker=drone_marker, s=50,
                 label=f'{drone_id} (x={pos[0]:.1f}, y={pos[1]:.1f}, z={pos[2]:.1f})',
                 depthshade=False)

        # 添加3D标签
        if drone_id == 'xxx-01':
            ax.text(pos[0], pos[1], pos[2], f'{drone_id}', size=6,
               zorder=1, color='k', bbox=dict(facecolor='white', alpha=0.5))

    # 绘制连接线（从xxx-01到其他无人机）
    for item in data.get("result", []):
            start = [float(coord) for coord in drones[data["id"]]][:3]
            end = [float(coord) for coord in drones[item["targetId"]]][:3]
            ax.plot([start[0], end[0]], [start[1], end[1]], [start[2], end[2]],
                    color='gray', linestyle='--', alpha=0.5, linewidth=1)

    # 绘制每个参考敌机
    for enemy_id, e_pos in enemy_pos.items():       
        ax.scatter(e_pos[0], e_pos[1], e_pos[2],
                 c=enemy_color, marker=enemy_marker, s=50,
                 label=f'{enemy_id} (x={e_pos[0]:.1f}, y={e_pos[1]:.1f}, z={e_pos[2]:.1f})',
                 depthshade=False)
        if enemy_id == 'xxx-01-0101': #仅标记一个点
            ax.text(e_pos[0], e_pos[1], e_pos[2], f'{enemy_id}', size=6,
               zorder=1, color='k', bbox=dict(facecolor='white', alpha=0.5))

    #绘制无人机和参考敌机的连接线
    for i, (drone_id,pos) in enumerate(drones.items(),1):
        enemy_id = f'xxx-{i:02d}-0101'
        start = [float(coord) for coord in pos]
        end = [float(coord) for coord in enemy_pos[enemy_id]]
        ax.plot([start[0], end[0]], [start[1], end[1]], [start[2], end[2]],
                    color='red', linestyle='-.', alpha=0.5, linewidth=1)

    #绘制每个敌群全部敌机
    for enemy_group_id,eg_pos in enemy_group_pos.items():
        ax.scatter(eg_pos[0], eg_pos[1], eg_pos[2],c=enemy_group_color,marker=enemy_marker,
                   s = 50,label = f'{enemy_group_id}(x={eg_pos[0]:.1f}, y={eg_pos[1]:.1f},z={eg_pos[2]:.1f})',
                   depthshade=False)
        ax.text(eg_pos[0],eg_pos[1],eg_pos[2],f'{enemy_group_id}',size=6,
                zorder=1, color='k', bbox=dict(facecolor='white', alpha=0.5))

    # 设置坐标轴
    ax.set_xlabel('X Axis (km)', fontsize=12)
    ax.set_ylabel('Y Axis (km)', fontsize=12)
    ax.set_zlabel('Z Axis (km)', fontsize=12)
    ax.set_title('三维空间分布', fontsize=15, pad=20)

    # 自动计算坐标轴范围
    all_pos = []
    for pos in list(drones.values()) + list(enemy_pos.values()):
        pos = [float(coord) for coord in pos][:3]
        if len(pos) < 3:
            pos += [0] * (3 - len(pos))
        all_pos.append(pos)
    #print(f"all_pos:{all_pos}")
    all_pos = np.array(all_pos)
    margin = 0.1  # 10% 扩展范围
    x_range = max(all_pos[:, 0]) - min(all_pos[:, 0])
    y_range = max(all_pos[:, 1]) - min(all_pos[:, 1])
    z_range = max(all_pos[:, 2]) - min(all_pos[:, 2])
    
    ax.set_xlim([min(all_pos[:, 0]) - margin*x_range, max(all_pos[:, 0]) + margin*x_range])
    ax.set_ylim([min(all_pos[:, 1]) - margin*y_range, max(all_pos[:, 1]) + margin*y_range])
    ax.set_zlim([min(all_pos[:, 2]) - margin*z_range, max(all_pos[:, 2]) + margin*z_range])
   # 添加图例/调整视角/显示网格
   # ax.legend(loc='upper right', fontsize=5)
   # ax.view_init(elev=25, azim=45)
   # ax.grid(True, linestyle='--', alpha=0.5)
   # plt.tight_layout()
    
   # 保存图像
    plt.savefig('无人机三维分布图.png', dpi=300)
    plt.show()


# 3.2.2敌群编队整体运动速度判定
def enemy_group_direction(enemy_data):
    """
    根据80%敌机运行方向确定群体方向
    返回:
        群体运行方向满足80%敌机方向一致，速度去除前后10%取均值
    """
    # 提取所有敌机的方向角度
    headings = [enemy['heading'] for enemy in enemy_data]
    pitches = [enemy['pitch'] for enemy in enemy_data]
    rolls = [enemy['roll'] for enemy in enemy_data]

    # 方向分组（考虑测量误差，±2度视为同一方向）
    rounded_headings = [round(h/2)*2 for h in headings]  # 多少度为分组间隔的离散化处理
    rounded_pitches = [round(p/2)*2 for p in pitches]
    rounded_rolls = [round(r/2)*2 for r in rolls]

    # 统计各方向出现的频率
    heading_counts = Counter(rounded_headings)
    pitches_counts = Counter(rounded_pitches)
    rolls_counts = Counter(rounded_rolls)
    # print("航向角格式：", heading_counts)
    # print("俯仰角格式：", pitches_counts)
    # print("滚转角格式：", rolls_counts)

    # 按频率降序排序
    sorted_headings = sorted(heading_counts.items(), key=lambda x: x[1], reverse=True)
    sorted_pitches = sorted(pitches_counts.items(), key=lambda x: x[1], reverse=True)   
    sorted_rolls = sorted(rolls_counts.items(), key=lambda x:x[1], reverse= True)
    
    # 计算80%阈值
    threshold = 0.9 * len(enemy_data)
    h_count,p_count,r_count= 0,0,0
    headings_directions,pitches_directions,rolls_directions = [],[],[]

    # 累加直到达到80%
    for h_direction, count in sorted_headings:
        h_count += count
        headings_directions.append(h_direction)
        if h_count >= threshold:
            break
    # print("航向角：",headings_directions)
    for p_direction, count in sorted_pitches:
        p_count += count
        pitches_directions.append(p_direction)
        if p_count >= threshold:
         break
    # print("俯仰角：",pitches_directions)
    for r_direction, count in sorted_rolls:
        r_count += count
        rolls_directions.append(r_direction)
        if r_count >= threshold:
         break
    # print("滚转角：",rolls_directions)

    #针对speed计算
    speed = [enemy['speed'] for enemy in enemy_data]
    sorted_speed = sorted(speed)
    n = len(sorted_speed)
    cut_part = 0.1
    lower_cut = int(n*cut_part)
    upper_cut = n-lower_cut
    speed_part= sorted_speed[lower_cut:upper_cut]
    # print("headings,pitches,rolls,speed:", headings,pitches,rolls,speed)

    # 返回速度信息，众数方向（或加权平均）
    return (np.mean(headings_directions), 
    np.mean(pitches_directions),
    np.mean(rolls_directions),
    round(np.mean(speed_part),2)) # 或多方向时取平均


# 主程序
def run_port(config: dict | None = None) -> dict:
    data = load_data("间距.txt")
    speed_data = load_data("速度.txt")
    enemy_speed = enemy_group_direction(speed_data)
    print("敌群整体运动速度判定结果(平均航向角，平均俯仰角，平均滚转角，平均速度)：", enemy_speed)

    # 输出各无人机坐标

    drones = drone_location(data)

    #输出敌群参考敌机的坐标
    enemy = Reference_enemy_aircraft(data)

    #输出敌群所有坐标位置
    enemy_group_pos = enemy_group_position(data)
    # print(f"侦察无人机位置:{drones}")
    # print(f"参考敌机位置:{enemy}")
    # print(f"敌群位置:{enemy_group_pos}")

    #可视化无人机、参考敌机、敌群位置 .ljust(8) :8.2f
    drone_visualization(data,drones,enemy,enemy_group_pos)
    #输出json数据
    out = {
        # ==================敌群速度信息===================
        "enemy_overall_speed": enemy_speed,

        #==================无人机/敌群位置信息===================
        "uav_position":  drones,
        "ref_enemy_position": enemy,
        "enemy_group_position": enemy_group_pos,
    }
    print("out", out)
    return out
    #无人机位置
    # print("Coordinates of unmanned aircraft and enemy groups：")
    # print("-"*40)
    # for drone_id, pos in drones.items():
    #     print(f"{drone_id}: X={pos[0]}m, Y={pos[1]}m, Z={pos[2]}m")
    # print("-"*40)


def main(config: dict | None = None):
    """
    与旧代码风格兼容的入口：POST 进来的 JSON 会作为 config 覆盖默认 dataset。
    """
    return run_port(config or {})


if __name__ == "__main__":
    result = run_port()  # 可传 config 覆盖默认数据
    # 你若还想显示可视化，可在这里读取 result 后，调用原有 plot_* 方法
    # e.g. plot_positions(...使用 result 里的字段组织参数...)
    # main()