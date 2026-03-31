# 网络配置详细步骤（Livox Mid-360 + Ubuntu）

## 第一步：确认网卡名称

```bash
ip link show
```

输出类似：
```
1: lo: <LOOPBACK,...>
2: enp3s0: <BROADCAST,...>   ← 这就是有线网卡，名字因机器而异
3: wlo1: <BROADCAST,...>     ← 这是无线网卡，不要动它
```

记下你的有线网卡名，后面统一用 `<网卡名>` 代替。

---

## 第二步：确认雷达 IP

Mid-360 出厂默认 IP 通常是 **192.168.1.1xx**，具体看设备底部标签，或者：

```bash
# 先随便设一个同网段 IP，然后扫描
sudo ip addr add 192.168.1.50/24 dev <网卡名>
sudo ip link set <网卡名> up
sudo apt install nmap
nmap -sn 192.168.1.0/24
```

扫描结果里除了你自己（.50）之外的设备就是雷达。

---

## 第三步：设置静态 IP（推荐用 nmcli，重启后仍然生效）

```bash
# 1. 查看当前网络连接名称
nmcli connection show
```

输出类似：
```
NAME                UUID                                  TYPE      DEVICE
有线连接 1          xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx  ethernet  enp3s0
```

```bash
# 2. 修改这个连接为静态 IP
nmcli connection modify "有线连接 1" \
  ipv4.method manual \
  ipv4.addresses 192.168.1.50/24 \
  ipv4.gateway "" \
  ipv4.dns ""

# 3. 重新激活
nmcli connection down "有线连接 1"
nmcli connection up   "有线连接 1"
```

> ⚠️ 连接名称用你自己 `nmcli connection show` 看到的，注意有中文的要加引号。

验证是否设置成功：
```bash
ip addr show <网卡名>
```

应该能看到 `inet 192.168.1.50/24`。

---

## 第四步：测试与雷达的连通性

```bash
ping -c 4 192.168.1.1xx   # 填你雷达的实际 IP
```

**✅ 正常情况：**
```
64 bytes from 192.168.1.1xx: icmp_seq=1 ttl=64 time=0.4 ms
```

**❌ 如果 ping 不通，先检查：**
```bash
# 确认网卡 UP 了
ip link show <网卡名>   # 应该有 UP 字样

# 确认 IP 在同一网段
ip addr show <网卡名>   # 应该看到 192.168.1.50/24

# 确认路由表有这条路
ip route show           # 应该有 192.168.1.0/24 dev <网卡名>
```

---

## 第五步：开放防火墙（很多人卡在这里）

Livox SDK 使用 UDP 广播通信，ufw 默认会拦截：

```bash
# 查看防火墙状态
sudo ufw status

# 如果是 active，执行以下放行规则
sudo ufw allow in on <网卡名> from 192.168.1.0/24
sudo ufw allow out on <网卡名> to 192.168.1.0/24
sudo ufw reload
```

或者临时完全关掉防火墙测试：
```bash
sudo ufw disable
```
能收到数据后再按需开启。

---

## 第六步：启动 ROS 驱动前确认配置文件

livox_ros_driver2 里有一个 JSON 配置文件，**里面的 IP 必须和实际一致**：

```bash
# 找到配置文件
find ~/ros2_ws -name "MID360_config.json"
```

打开文件，关键字段：

```json
{
  "lidar_summary_info": {
    "lidar_type": 8
  },
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": {
      "cmd_data_ip": "192.168.1.50",    ← 改成你电脑的 IP
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.1.50",    ← 同上
      "push_msg_port": 56201,
      "point_data_ip": "192.168.1.50",  ← 同上
      "point_data_port": 56301,
      "imu_data_ip": "192.168.1.50",    ← 同上
      "imu_data_port": 56401,
      "log_data_ip": "192.168.1.50",    ← 同上
      "log_data_port": 56501
    }
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.1xx",            ← 改成雷达的 IP
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "extrinsic_parameter": { ... }
    }
  ]
}
```

---

## 第七步：启动驱动

```bash
source /opt/ros/<你的ros版本>/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**✅ 正常输出应包含：**
```
[livox_ros_driver2]: Livox SDK2 init success.
[livox_ros_driver2]: Device 192.168.1.1xx connected.
```

---

## 第八步：验证数据（另开一个终端）

```bash
source ~/ros2_ws/install/setup.bash

# 查看 topic 是否有数据
ros2 topic list
ros2 topic hz /livox/lidar    # 应该约 10Hz
```

或者用 RViz2 可视化：
```bash
rviz2
# Add → PointCloud2 → Topic 选 /livox/lidar
# Fixed Frame 改为 livox_frame
```

---

## 排查速查表

| 问题 | 命令 | 预期结果 |
|---|---|---|
| 网卡没 IP | `ip addr show <网卡名>` | 看到 `192.168.1.50/24` |
| ping 不通 | `ping 192.168.1.1xx` | 有回包 |
| 端口被拦截 | `sudo ufw status` | inactive 或已放行 |
| 驱动找不到设备 | 检查 JSON 文件里的 IP | 两处 IP 都要对 |
| Topic 无数据 | `ros2 topic hz /livox/lidar` | 约 10Hz |

遇到哪一步卡住了，把报错贴给我，我帮你继续排查。