# MCU 0x91 Message Reception Guide

## 您的硬件配置 (Your Hardware Setup)

- **MCU连接端口**: `/dev/ttyUSB0` ✓ (已检测到 Silicon Labs CP210x)
- **波特率**: 115200 bps
- **数据位**: 8 bits
- **校验位**: None (无)
- **停止位**: 1 bit
- **格式**: 8N1 ✓

## 预期的MCUDataFrame结构

```
0x91 (SOF) + 87字节数据 + CRC8 + 0xFE (EOF) = 89字节总长
```

### 字段详解:

| Byte Offset | Size | 字段名 | 说明 |
|---|---|---|---|
| 0 | 1 | sof | 帧头 (0x91) |
| 1-4 | 4 | yaw_angle | 云台yaw弧度 (float) |
| 5-8 | 4 | chassis_imu | 底盘IMU弧度 (float) |
| 9 | 1 | motion_mode | 运动模式 |
| 10-13 | 4 | operator_x | 操作手x坐标 (float) |
| 14-17 | 4 | operator_y | 操作手y坐标 (float) |
| 18 | 1 | robot_id | 机器人ID |
| 19 | 1 | robot_color | 颜色 (0=红, 1=蓝) |
| 20 | 1 | game_progress | 比赛阶段 |
| 21-22 | 2 | red_1_hp | 红英雄血量 |
| 23-24 | 2 | red_3_hp | 红步兵3血量 |
| 25-26 | 2 | red_7_hp | 红哨兵血量 |
| 27-28 | 2 | blue_1_hp | 蓝英雄血量 |
| 29-30 | 2 | blue_3_hp | 蓝步兵3血量 |
| 31-32 | 2 | blue_7_hp | 蓝哨兵血量 |
| 33-34 | 2 | red_dead | 红方死亡位 |
| 35-36 | 2 | blue_dead | 蓝方死亡位 |
| 37-38 | 2 | self_hp | 自身血量 |
| 39-40 | 2 | self_max_hp | 最大血量 |
| 41-42 | 2 | bullet_remain | 剩余弹量 |
| 43 | 1 | occupy_status | 占领状态 |
| 44-47 | 4 | enemy_hero_x | 敌方英雄X坐标 (float) |
| 48-51 | 4 | enemy_hero_y | 敌方英雄Y坐标 (float) |
| ... | ... | ... | (更多敌方单位位置数据) |
| 87 | 1 | crc8 | CRC8校验 |
| 88 | 1 | eof | 帧尾 (0xFE) |

## 快速启动 (Quick Start)

### 方法1: 使用提供的启动脚本

```bash
cd /home/sentry_train_test/AstarTraining
./test_mcu_receive.sh /dev/ttyUSB0 115200
```

### 方法2: 直接使用ROS launch命令

```bash
cd /home/sentry_train_test/AstarTraining/DecisionNode
roslaunch decision_node mcu_communicator.launch \
    serial_port:=/dev/ttyUSB0 \
    baudrate:=115200 \
    output:=screen
```

### 方法3: 使用自定义launch文件启动完整系统

```bash
roslaunch decision_node mcu_communicator_with_decision.launch
```

## 接收验证 (Verify Reception)

Once the MCU communicator is running, you should see:

1. **成功消息** (Success Messages):
   ```
   [INFO] MCU Serial port opened successfully: /dev/ttyUSB0 @ 115200 baud
   [DEBUG] Valid frame received: robot_id=X, game_progress=Y, crc8=0x##
   ```

2. **监听ROS Topics** (Monitor ROS Topics):
   ```bash
   # 在另一个终端监听接收到的数据
   rostopic echo /referee/remain_hp
   rostopic echo /enemy/hero_position
   rostopic echo /robot/self_hp
   ```

3. **完整的Topics列表** (Full Topics List):
   - `/referee/game_progress` - 比赛阶段
   - `/referee/remain_hp` - 己方血量
   - `/referee/bullet_remain` - 剩余弹量
   - `/robot/self_hp` - 自身血量
   - `/robot/robot_id` - 机器人ID
   - `/robot/robot_color` - 机器人颜色
   - `/enemy/hero_position` - 敌方英雄位置
   - `/enemy/engineer_position` - 敌方工程位置
   - `/enemy/standard_3_position` - 敌方步兵3位置
   - `/enemy/sentry_position` - 敌方哨兵位置
   - `/mcu/yaw_angle` - 云台yaw角
   - `/mcu/chassis_imu` - 底盘IMU角

## CRC8校验 (CRC8 Verification)

The MCU communicator automatically verifies CRC8 checksums using a lookup table. If you see CRC8 errors:

1. 检查MCU端的CRC8计算是否正确
2. 确认波特率和帧格式完全一致
3. 尝试重新连接MCU

## 故障排除 (Troubleshooting)

| 问题 | 原因 | 解决方案 |
|---|---|---|
| Serial port not found | 设备未连接 | 检查USB连接，运行`lsusb` |
| Permission denied | 没有读写权限 | `sudo chmod 666 /dev/ttyUSB0` |
| CRC8 verification failed | 数据损坏 | 检查MCU编码，尝试重新连接 |
| No frames received | MCU未发送数据 | 检查MCU代码，验证波特率 |

## 硬件连接确认 (Hardware Confirmation)

从dmesg日志可以看出：
```
[  780.989021] usb 1-7: cp210x converter now attached to ttyUSB0
```

您的MCU通过Silicon Labs CP210x芯片连接到`/dev/ttyUSB0`。这是一个常见的USB转串口芯片。
