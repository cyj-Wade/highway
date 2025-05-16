import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation

# 计算安全车距的函数
# speed: 当前车辆的速度
# reaction_time: 驾驶员的反应时间，默认为1.1秒
# friction_coefficient: 路面摩擦系数，默认为0.8
# gravity: 重力加速度，默认为9.8 m/s²
def safe_distance(speed, reaction_time=1.1, friction_coefficient=0.8, gravity=9.8):
    # 将速度从 km/h 转换为 m/s
    speed_ms = speed / 3.6
    # 计算反应距离，即驾驶员在反应时间内车辆行驶的距离
    reaction_distance = speed_ms * reaction_time
    # 计算制动距离，即车辆从当前速度减速到停止所行驶的距离
    braking_distance = (speed_ms ** 2) / (2 * friction_coefficient * gravity)
    # 返回安全车距，即反应距离和制动距离之和
    return reaction_distance + braking_distance

# 加减速控制函数
# self_speed: 当前车辆的速度
# front_distance: 当前车辆与前车的距离
# is_aggressive: 当前车辆是否为激进型车辆
def acceleration_control(self_speed, front_distance, is_aggressive):
    # 最小安全车距
    min_distance = 5
    # 计算当前速度下的安全车距
    safe_dist = max(safe_distance(self_speed), min_distance)
    # 如果前车距离小于安全车距，当前车辆减速
    if front_distance < safe_dist:
        return -1
    # 如果当前车辆速度加5后不超过60 km/h，且加5后的速度对应的安全车距小于前车距离
    elif (self_speed + 5) <= 60 and safe_distance(self_speed + 5) <= front_distance:
        if is_aggressive:
            # 激进型车辆有70%的概率加速
            if np.random.rand() < 0.7:
                return 1
        else:
            # 非激进型车辆有10%的概率加速
            if np.random.rand() < 0.1:
                return 1
    # 其他情况保持当前速度
    return 0

# 封装变道判断逻辑的函数
# self: 当前车辆对象
# target_lane: 目标车道
# vehicles: 所有车辆的列表
# min_distance: 最小安全车距，默认为5米
# adjust_safe_dist: 是否调整安全车距，默认为False
def can_change_lane(self, target_lane, vehicles, min_distance=5, adjust_safe_dist=False):
    # 获取目标车道上的所有车辆
    target_lane_vehicles = [v for v in vehicles if v.lane == target_lane]
    # 假设可以变道
    can_change = True
    for v in target_lane_vehicles:
        if adjust_safe_dist:
            # 如果需要调整安全车距，取当前速度安全车距的0.9倍和最小安全车距的最大值
            safe_dist = max(safe_distance(self.speed) * 0.9, min_distance)
        else:
            # 否则取当前速度安全车距和最小安全车距的最大值
            safe_dist = max(safe_distance(self.speed), min_distance)
        # 如果目标车道上有车辆与当前车辆的距离小于安全车距，则不能变道
        if abs(v.position - self.position) < safe_dist:
            can_change = False
            break
    return can_change

# 车辆类，用于表示公路上的车辆
class Vehicle:
    def __init__(self, lane, position, speed, is_aggressive):
        # 车辆当前所在车道
        self.lane = lane
        # 车辆当前的位置
        self.position = position
        # 车辆当前的速度
        self.speed = speed
        # 车辆是否为激进型车辆
        self.is_aggressive = is_aggressive
        # 变道后倒计时，防止连续变道
        self.post_change_countdown = 0
        # 车辆是否停止
        self.stopped = False
        # 车辆是否减速
        self.slow_down = False
        # 车辆是否已经变道
        self.has_changed_lane = False
        # 车辆行驶的总距离
        self.distance_traveled = 0
        # 变道开始的信息（位置、车道、时间）
        self.lane_change_start = None
        # 变道结束的信息（位置、车道、时间）
        self.lane_change_end = None
        # 车辆的行驶轨迹
        self.trace = []
        # 车辆的颜色，激进型为蓝色，非激进型为绿色
        self.color = 'blue' if is_aggressive else 'green'
        # 标记车辆是否从0车道变到1车道
        self.changed_from_0_to_1 = False
        # 标记车辆是否从7车道变到6车道
        self.changed_from_7_to_6 = False


    def update(self, vehicles, num_lanes, lane_lengths, merge_start_150, merge_start_200, current_time):
        # 初始化前车对象
        front_vehicle = None
        for v in vehicles:
            # 找到当前车道上在当前车辆前方且距离最近的车辆
            if v.lane == self.lane and v.position > self.position:
                if front_vehicle is None or v.position < front_vehicle.position:
                    front_vehicle = v

        if front_vehicle is None:
            if self.position < lane_lengths[0]:
                # 如果前方没有车辆且当前位置在第一段车道内，前车距离为第一段车道长度减去当前位置
                front_distance = lane_lengths[0] - self.position
            else:
                # 否则前车距离为所有车道总长度减去当前位置
                front_distance = sum(lane_lengths) - self.position
        else:
            # 计算与前车的距离
            front_distance = front_vehicle.position - self.position

        if not self.stopped:
            # 计算当前速度下的安全车距
            safe_dist = max(safe_distance(self.speed), 5)
            if front_distance >= safe_dist:
                # 如果前车距离大于等于安全车距，加速但不超过60 km/h
                self.speed = min(60, self.speed + 5)
            elif self.slow_down:
                # 如果需要减速，减速但不低于5 km/h
                self.speed = max(5, self.speed - 2)
            elif self.lane in [2, 3, 4, 5]:
                # 如果车辆在中间车道且速度小于60 km/h，加速到60 km/h
                if self.speed < 60:
                    self.speed = min(60, self.speed + 5)
            else:
                # 其他情况根据加减速控制函数调整速度
                acc = acceleration_control(self.speed, front_distance, self.is_aggressive)
                if acc == 1:
                    self.speed = min(60, self.speed + 5)
                elif acc == -1:
                    self.speed = max(0, self.speed - 5)

        if not self.stopped:
            # 计算当前时间步内车辆行驶的距离
            distance_this_step = self.speed * 1 / 3.6
            # 累加车辆行驶的总距离
            self.distance_traveled += distance_this_step

        # 记录车辆原来所在的车道
        original_lane = self.lane
        if self.post_change_countdown == 0 and self.distance_traveled >= 25:
            if (self.lane in [0, 7] and self.position < merge_start_150) or (
                    self.lane in [1, 6] and self.position < merge_start_200):
                # 目标车道列表
                target_lanes = [2, 3, 4, 5]
                if self.lane < 1:
                    # 如果车辆在最左侧车道，可能的目标车道为1
                    possible_targets = [ 1]
                elif self.lane < 2 and self.lane > 0:
                    # 如果车辆在车道1，可能的目标车道为2
                    possible_targets = [ 2]
                elif self.lane > 6:
                    # 如果车辆在最右侧车道，可能的目标车道为6
                    possible_targets = [ 6]
                else:
                    # 其他情况可能的目标车道为5
                    possible_targets = [ 5]

                for target_lane in possible_targets:
                    # 判断是否可以变道到目标车道
                    if can_change_lane(self, target_lane, vehicles):
                        if self.lane_change_start is None:
                            # 记录变道开始的信息
                            self.lane_change_start = (
                                self.position - self.speed * 0.1 / 3.6, self.lane, current_time)
                        # 改变车辆所在车道
                        self.lane = target_lane
                        # 设置变道后倒计时
                        self.post_change_countdown = 0.1 / 1
                        # 取消减速状态
                        self.slow_down = False
                        # 标记车辆已经变道
                        self.has_changed_lane = True
                        # 车辆颜色变为紫色
                        self.color = 'purple'
                        if original_lane == 0 and target_lane == 1:
                            # 标记车辆从0车道变到1车道
                            self.changed_from_0_to_1 = True
                        break

                if (self.lane in [0, 7] and merge_start_150 - self.position < 20) or (
                        self.lane in [1, 6] and merge_start_200 - self.position < 20):
                    for target_lane in possible_targets:
                        # 紧急情况下调整安全车距判断是否可以变道
                        if can_change_lane(self, target_lane, vehicles, adjust_safe_dist=True):
                            if self.lane_change_start is None:
                                self.lane_change_start = (
                                    self.position - self.speed * 0.1 / 3.6, self.lane, current_time)
                            self.lane = target_lane
                            self.post_change_countdown = 0.1 / 1
                            self.slow_down = False
                            self.has_changed_lane = True
                            self.color = 'purple'
                            if original_lane == 0 and target_lane == 1:
                                self.changed_from_0_to_1 = True
                            if original_lane == 7 and target_lane == 6:
                                self.changed_from_7_to_6 = True
                            break

                if not self.has_changed_lane and ((self.lane in [0, 7] and merge_start_150 - self.position < 30) or (
                        self.lane in [1, 6] and merge_start_200 - self.position < 30)):
                    # 如果在接近合并区域还未变道，开始减速
                    self.slow_down = True

            elif (self.position >= merge_start_150 and self.lane in [0, 7]) or (
                    self.position >= merge_start_200 and self.lane in [1, 6]):
                # 如果车辆到达合并区域且未进入目标车道，停止
                self.speed = 0
                self.stopped = True
                target_lanes = [2, 3, 4, 5]
                if self.lane not in target_lanes:
                    if self.lane < 1:
                        possible_targets = [1]
                    elif self.lane > 0 and self.lane < 2:
                        possible_targets = [2]
                    elif self.lane > 6:
                        possible_targets = [6]
                    else:
                        possible_targets = [5]

                    for target_lane in possible_targets:
                        # 尝试变道到目标车道
                        if can_change_lane(self, target_lane, vehicles) and np.random.rand() < 0.3:
                            if self.lane_change_start is None:
                                self.lane_change_start = (
                                    self.position - self.speed * 0.1 / 3.6, self.lane, current_time)
                            self.lane = target_lane
                            self.post_change_countdown = 0.1 / 1
                            self.stopped = False
                            self.has_changed_lane = True
                            self.color = 'purple'
                            if original_lane == 0 and target_lane == 1:
                                self.changed_from_0_to_1 = True
                            if original_lane == 7 and target_lane == 6:
                                self.changed_from_7_to_6 = True
                            break

            elif self.lane in [2, 3, 4, 5]:
                # 如果车辆在中间车道，取消减速状态
                self.slow_down = False

            else:
                if self.lane < 3 and self.lane >1:
                    # 如果车辆不在最右侧车道，有50%的概率尝试向右变道
                    if can_change_lane(self, self.lane + 1, vehicles) and np.random.rand() < 0.5:
                        if self.lane_change_start is None:
                            self.lane_change_start = (
                                self.position - self.speed * 0.1 / 3.6, self.lane, current_time)
                        self.lane += 1
                        self.post_change_countdown = 0.1 / 1
                        self.has_changed_lane = True
                        self.color = 'pink'
                if self.lane > 4 and self.lane < 6:
                    # 如果车辆不在最左侧车道，有50%的概率尝试向左变道
                    if can_change_lane(self, self.lane - 1, vehicles) and np.random.rand() < 0.5:
                        if self.lane_change_start is None:
                            self.lane_change_start = (
                                self.position - self.speed * 0.1 / 3.6, self.lane, current_time)
                        self.lane -= 1
                        self.post_change_countdown = 0.1 / 1
                        self.has_changed_lane = True
                        self.color = 'pink'

            # 新增逻辑：从0车道变到1车道后，尝试变道到2车道
            if self.changed_from_0_to_1 and self.lane == 1 and self.position < merge_start_200:
                # 增加计时器变量
                if not hasattr(self, 'wait_timer_0_to_1'):
                    self.wait_timer_0_to_1 = 0.1  # 设置初始等待时间为0.1秒

                if self.wait_timer_0_to_1 > 0:
                    self.wait_timer_0_to_1 -= time_step  # 倒计时
                else:
                    if can_change_lane(self, 2, vehicles):
                        if self.lane_change_start is None:
                            self.lane_change_start = (
                                self.position - self.speed * 0.1 / 3.6, self.lane, current_time)
                        self.lane = 2
                        self.post_change_countdown = 0.1 / 1
                        self.has_changed_lane = True
                        self.color = 'purple'  # 保持与其他变道颜色一致
                        # 变道到2车道后，重置标记和计时器
                        self.changed_from_0_to_1 = False
                        delattr(self, 'wait_timer_0_to_1')  # 移除计时器属性

            # 新增逻辑：从7车道变到6车道后，尝试变道到6车道
            if self.changed_from_7_to_6 and self.lane == 5 and self.position < merge_start_200:
                            # 增加计时器变量
                if not hasattr(self, 'wait_timer_0_to_1'):
                    self.wait_timer_7_to_6 = 0.1  # 设置初始等待时间为0.1秒

                if self.wait_timer_0_to_1 > 0:
                    self.wait_timer_0_to_1 -= time_step  # 倒计时
                else:
                    if can_change_lane(self, 5, vehicles):
                        if self.lane_change_start is None:
                             self.lane_change_start = (
                                 self.position - self.speed * 0.1 / 3.6, self.lane, current_time)
                        self.lane = 5
                        self.post_change_countdown = 0.1 / 1
                        self.has_changed_lane = True
                        self.color = 'purple'  # 保持与其他变道颜色一致
                        # 变道到2车道后，重置标记和计时器
                        self.changed_from_7_to_6 = False
                        delattr(self, 'wait_timer_0_to_1')  # 移除计时器属性

        if self.post_change_countdown > 0:
            # 变道后倒计时减1
            self.post_change_countdown -= 1
            if self.post_change_countdown == 0 and self.lane_change_start is not None and self.lane_change_end is None:
                # 变道倒计时结束，记录变道结束信息
                self.lane_change_end = (self.position + self.speed * 0.1 / 3.6, self.lane, current_time)
                # 恢复车辆原来的颜色
                self.color = 'blue' if self.is_aggressive else 'green'

        if not self.stopped:
            if front_vehicle is not None:
                # 计算当前速度下的安全车距
                safe_dist = max(safe_distance(self.speed), 5)
                # 计算新的位置
                new_position = self.position + self.speed * 1 / 3.6
                # 确保车辆不会与前车碰撞
                self.position = min(new_position, front_vehicle.position - safe_dist)
            else:
                # 如果前方没有车辆，正常行驶
                self.position += self.speed * 1 / 3.6

        # 记录车辆的行驶轨迹
        self.trace.append((self.position, self.lane))
        if len(self.trace) > 20:
            # 只保留最近20个轨迹点
            self.trace.pop(0)

# 模拟参数
# 车道数量
num_lanes = 8
# 各段车道的长度
lane_lengths = [150, 100, 150]
# 模拟的总时间
simulation_time = 200
# 时间步长
time_step = 0.1
# 第一段合并区域开始的位置
merge_start_150 = 150
# 第二段合并区域开始的位置
merge_start_200 = 200

# 初始化车辆列表
vehicles = []
for i in range(num_lanes):
    # 每个车道初始有一辆车
    vehicles.append(Vehicle(i, 0, 20, np.random.rand() < 0.5))

# 初始化图形
fig, ax = plt.subplots()
# 设置图形的x轴范围
ax.set_xlim(0, sum(lane_lengths))
# 设置图形的y轴范围
ax.set_ylim(-0.5, num_lanes - 0.5)
# 设置x轴标签
ax.set_xlabel('Position')
# 设置y轴标签
ax.set_ylabel('Lane')
# 设置图形标题
ax.set_title('Highway Exit Simulation')

# 优化车道线绘制逻辑
def draw_lane_lines(ax):
    # 设置y轴的刻度
    ax.set_yticks(np.arange(-0.5, num_lanes, 1))

    # 绘制水平车道线
    for y in np.arange(-0.5, num_lanes, 1):
        if y in [-0.5, 7.5]:
            # 最外侧车道线绘制到第一段合并区域开始的位置
            ax.plot([0, merge_start_150], [y, y], 'k-', linewidth=1)
        elif y in [0.5, 6.5]:
            # 次外侧车道线绘制到第二段合并区域开始的位置
            ax.plot([0, merge_start_200], [y, y], 'k-', linewidth=1)
        else:
            # 其他车道线绘制到所有车道总长度
            ax.plot([0, sum(lane_lengths)], [y, y], 'k-', linewidth=1)

    # 绘制合并区域连接线
    ax.plot([merge_start_150, merge_start_150 + 100], [-0.5, 1.5], 'k-', linewidth=1)
    ax.plot([merge_start_150, merge_start_150 + 100], [7.5, 5.5], 'k-', linewidth=1)
    ax.plot([merge_start_200, merge_start_200 + 50], [0.5, 1.5], 'k-', linewidth=1)
    ax.plot([merge_start_200, merge_start_200 + 50], [6.5, 5.5], 'k-', linewidth=1)

# 调用优化后的车道线绘制函数
draw_lane_lines(ax)

# 新增变量
# 记录变道的车辆数量
changed_lane_count = 0
# 记录通过模拟区域的车辆数量
passed_vehicles_count = 0
# 记录车辆的矩形图形对象
vehicle_rectangles = []
# 记录车辆的轨迹线图形对象
traces = []

# 主循环
# 开启交互模式
plt.ion()
# 显示图形
plt.show()

for frame in range(int(simulation_time / time_step)):
    # 清空当前帧的矩形和轨迹线
    for rect in vehicle_rectangles:
        rect.remove()
    vehicle_rectangles.clear()

    for line in ax.lines[12:]:  # 保留车道线，从第12个元素开始删除
        line.remove()
    traces.clear()

    # 更新车辆状态
    new_vehicles = []
    if np.random.rand() < 0.5:  # 降低新车生成概率
        # 只从最外侧车道生成新车
        new_lane = np.random.choice([0,1,2,3,4,5,6,7])
        # 创建新的车辆对象
        new_vehicle = Vehicle(new_lane, 0, 20, np.random.rand() < 0.5)
        # 将新车辆添加到车辆列表中
        vehicles.append(new_vehicle)

    changed_lane_count = 0
    for vehicle in vehicles:
        # 更新车辆的状态
        vehicle.update(vehicles, num_lanes, lane_lengths, merge_start_150, merge_start_200, frame * time_step)
        if vehicle.has_changed_lane:
            # 统计变道的车辆数量
            changed_lane_count += 1
        if vehicle.position < sum(lane_lengths):
            # 如果车辆还在模拟区域内，保留该车辆
            new_vehicles.append(vehicle)
        else:
            # 如果车辆已经通过模拟区域，统计通过的车辆数量
            passed_vehicles_count += 1

    # 更新车辆列表
    vehicles = new_vehicles

    # 绘制车辆
    for vehicle in vehicles:
        rect = Rectangle(
            (vehicle.position, vehicle.lane - 0.3),
            5,  # 长度
            0.6,  # 宽度
            edgecolor='black',
            facecolor=vehicle.color,
            alpha=0.8
        )
        # 将车辆矩形添加到图形中
        ax.add_patch(rect)
        # 记录车辆矩形图形对象
        vehicle_rectangles.append(rect)

    # 绘制轨迹
    for vehicle in vehicles:
        if len(vehicle.trace) > 1:
            # 解包车辆的轨迹点
            x, y = zip(*vehicle.trace)
            # 绘制车辆的轨迹线
            trace, = ax.plot(x, y, color='orange', linewidth=1)
            # 记录车辆的轨迹线图形对象
            traces.append(trace)

    # 检测碰撞
    crash_count = 0
    for i, vehicle1 in enumerate(vehicles):
        for j, vehicle2 in enumerate(vehicles):
            if i < j:
                if (vehicle1.lane == vehicle2.lane and
                        abs(vehicle1.position - vehicle2.position) < 4):
                    # 如果两车在同一车道且距离小于4米，标记为碰撞
                    vehicle1.color = 'red'
                    vehicle2.color = 'red'
                    # 统计碰撞的车辆数量
                    crash_count += 1

    # 更新标题显示统计信息
    ax.set_title(f'Highway Exit Simulation - Time: {frame * time_step:.1f}s | '
                 f'Vehicles: {len(vehicles)} | Changed Lanes: {changed_lane_count} | '
                 f'Passed: {passed_vehicles_count} | Crashes: {crash_count}')

    # 刷新画布
    plt.draw()
    # 暂停一段时间，控制动画速度
    plt.pause(0.1)

# 关闭交互模式
plt.ioff()
# 显示最终图形
plt.show()