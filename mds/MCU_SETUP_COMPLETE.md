# MCU 0x91 消息接收完整指南

## 系统状态 ✓

您的MCU通讯系统已完全设置好！以下是已完成的工作：

### 已验证的硬件配置
- ✓ **MCU连接**: `/dev/ttyUSB0` (Silicon Labs CP210x USB转串口)
- ✓ **波特率**: 115200 bps
- ✓ **数据格式**: 8N1 (8数据位, 无校验, 1停止位)
- ✓ **MCU消息格式**: 0x91 SOF + 87字节数据 + CRC8 + 0xFE EOF = 89字节

### 已编译的组件
- ✓ MCU communicator 可执行文件: `/home/sentry_train_test/AstarTraining/DecisionNode/devel/lib/decision_node/mcu_communicator`
- ✓ ROS Launch文件已配置
- ✓ CRC8校验查表已实现（与MCU端完全同步）

### 新增的辅助工具
1. **start_mcu_receiver.sh** - 完整的启动脚本（推荐使用）
2. **test_mcu_receive.sh** - 快速测试脚本
3. **diagnose_serial.sh** - 串口诊断工具
4. **MCU_RECEPTION_GUIDE.md** - 详细的参考指南

---

## 快速开始 (3种方式)

### ✨ 方式1: 使用完整启动脚本（推荐）

```bash
cd /home/sentry_train_test/AstarTraining
./start_mcu_receiver.sh /dev/ttyUSB0 115200
```

**优点**: 
- 自动检查所有依赖和权限
- 自动启动roscore
- 自动编译（如果需要）
- 显示详细的诊断信息

### 方式2: 使用ROS launch命令

```bash
# 进入决策节点工作空间
cd /home/sentry_train_test/AstarTraining/DecisionNode
source devel/setup.bash

# 启动MCU通讯节点
roslaunch decision_node mcu_communicator.launch \
    serial_port:=/dev/ttyUSB0 \
    baudrate:=115200 \
    output:=screen
```

### 方式3: 使用完整系统launch

```bash
# 启动MCU通讯 + 决策节点
cd /home/sentry_train_test/AstarTraining/DecisionNode
source devel/setup.bash
roslaunch decision_node mcu_communicator_with_decision.launch
```

---

## 验证接收 ✓

### 1. 确认节点启动成功

启动脚本后，您应该看到：

```
[INFO] MCU Serial port opened successfully: /dev/ttyUSB0 @ 115200 baud
[INFO] MCU Communicator node started
```

### 2. 监听接收到的数据

在另一个终端中执行以下命令来查看接收到的MCU消息：

```bash
# 监听机器人血量信息
rostopic echo /robot/self_hp

# 监听敌方位置信息
rostopic echo /enemy/hero_position

# 监听比赛进度信息
rostopic echo /referee/game_progress

# 监听所有接收到的topics
rostopic list | grep -E "referee|robot|enemy|mcu"
```

### 3. 检查CRC8校验

MCU communicator会自动验证所有接收到的帧。如果您看到：

```
[WARN] CRC8 verification failed
```

这意味着数据损坏或MCU的CRC8计算不正确。

---

## 完整的ROS Topics列表

MCU communicator会发布以下ROS topics：

### 比赛信息 (/referee/*)
- `/referee/game_progress` (UInt8) - 比赛阶段 (0=未开始, 1=准备, 2=进行中, etc)
- `/referee/remain_hp` (UInt16) - 己方血量
- `/referee/bullet_remain` (UInt16) - 剩余弹量
- `/referee/occupy_status` (UInt8) - 占领状态

### 敌方信息 (/enemy/*)
- `/enemy/hero_position` (Point) - 敌方英雄位置 (x, y, z)
- `/enemy/engineer_position` (Point) - 敌方工程位置
- `/enemy/standard_3_position` (Point) - 敌方步兵3位置
- `/enemy/standard_4_position` (Point) - 敌方步兵4位置
- `/enemy/sentry_position` (Point) - 敌方哨兵位置

### 机器人信息 (/robot/*)
- `/robot/robot_id` (UInt8) - 机器人ID
- `/robot/robot_color` (UInt8) - 机器人颜色 (0=红, 1=蓝)
- `/robot/self_hp` (UInt16) - 自身血量
- `/robot/self_max_hp` (UInt16) - 最大血量

### 陀螺仪数据 (/mcu/*)
- `/mcu/yaw_angle` (Float32) - 云台yaw角 (弧度)
- `/mcu/chassis_imu` (Float32) - 底盘IMU角 (弧度)

### 雷达信息 (/radar/*)
- `/radar/suggested_target` (UInt8) - 建议目标
- `/radar/radar_flags` (UInt16) - 目标选择标志

### 红蓝方血量信息
- `/referee/red_1_hp`, `/referee/red_3_hp`, `/referee/red_7_hp` - 红方各单位血量
- `/referee/blue_1_hp`, `/referee/blue_3_hp`, `/referee/blue_7_hp` - 蓝方各单位血量
- `/referee/red_dead`, `/referee/blue_dead` - 红蓝方死亡位信息

---

## MCUDataFrame详细结构

| 字节偏移 | 大小 | 字段 | 数据类型 | 说明 |
|---------|------|------|---------|------|
| 0 | 1 | sof | uint8_t | 帧头 = 0x91 |
| 1-4 | 4 | yaw_angle | float | 云台yaw弧度 |
| 5-8 | 4 | chassis_imu | float | 底盘IMU弧度 |
| 9 | 1 | motion_mode | uint8_t | 运动模式 |
| 10-13 | 4 | operator_x | float | 操作手x坐标 |
| 14-17 | 4 | operator_y | float | 操作手y坐标 |
| 18 | 1 | robot_id | uint8_t | 机器人ID |
| 19 | 1 | robot_color | uint8_t | 颜色 (0=红, 1=蓝) |
| 20 | 1 | game_progress | uint8_t | 比赛阶段 |
| 21-26 | 6 | 红方血量 | uint16_t x3 | red_1_hp, red_3_hp, red_7_hp |
| 27-32 | 6 | 蓝方血量 | uint16_t x3 | blue_1_hp, blue_3_hp, blue_7_hp |
| 33-34 | 2 | red_dead | uint16_t | 红方死亡位 |
| 35-36 | 2 | blue_dead | uint16_t | 蓝方死亡位 |
| 37-38 | 2 | self_hp | uint16_t | 自身血量 |
| 39-40 | 2 | self_max_hp | uint16_t | 最大血量 |
| 41-42 | 2 | bullet_remain | uint16_t | 剩余弹量 |
| 43 | 1 | occupy_status | uint8_t | 占领状态 |
| 44-75 | 32 | 敌方位置 | float x8 | 英雄/工程/步兵3/步兵4/哨兵各2个坐标 |
| 76 | 1 | suggested_target | uint8_t | 建议目标 |
| 77-78 | 2 | radar_flags | uint16_t | 目标选择标志 |
| 79-87 | 9 | (reserved) | - | 保留字节 |
| 87 | 1 | crc8 | uint8_t | CRC8校验值 |
| 88 | 1 | eof | uint8_t | 帧尾 = 0xFE |

**总长度: 89字节**

---

## CRC8校验算法

MCU communicator使用查表法计算CRC8，初始值为0xFF，多项式为0x31：

```cpp
static constexpr uint8_t CRC8_TABLE[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, ...
};

uint8_t crc = 0xFF;
for (int i = 0; i < 87; i++) {  // 计算除去SOF、CRC8和EOF的87字节
    crc = CRC8_TABLE[crc ^ data[i]];
}
// crc现在是正确的CRC8值
```

---

## 故障排除

### 问题: "Serial port not found"
**解决**: 检查USB连接，或运行诊断脚本找到正确的端口
```bash
./diagnose_serial.sh
```

### 问题: "Permission denied"
**解决**: 提升串口权限
```bash
sudo chmod 666 /dev/ttyUSB0
```

### 问题: "CRC8 verification failed"
**解决**: 
1. 检查MCU端CRC8计算是否与上位机一致
2. 验证波特率和帧格式完全相同
3. 检查数据线连接质量

### 问题: "No frames received"
**解决**:
1. 确认MCU正在发送数据
2. 检查MCU的串口配置
3. 使用示波器或串口监听器验证信号

### 问题: "ROS topics not appearing"
**解决**: 
1. 确认节点正在运行: `rosnode list`
2. 检查节点状态: `rosnode info /mcu_communicator`
3. 查看节点输出: `roslaunch ... output:=screen`

---

## 代码位置参考

- **MCU communicator源代码**: [src/decision_node/src/mcu_communicator.cpp](DecisionNode/src/decision_node/src/mcu_communicator.cpp)
- **数据结构定义**: [include/decision_node/mcu_comm.hpp](DecisionNode/src/decision_node/include/decision_node/mcu_comm.hpp)
- **Launch文件**: [src/decision_node/launch/mcu_communicator.launch](DecisionNode/src/decision_node/launch/mcu_communicator.launch)
- **CRC8校验表**: [src/decision_node/src/mcu_communicator.cpp](DecisionNode/src/decision_node/src/mcu_communicator.cpp) 第15-31行

---

## 下一步

现在您可以：

1. ✓ 启动MCU接收
2. 在您的应用中订阅MCU topics
3. 根据接收到的数据做出决策
4. 发送命令控制MCU设备

祝使用愉快！如有问题，查看MCU_RECEPTION_GUIDE.md了解更多细节。
