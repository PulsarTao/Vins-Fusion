#!/usr/bin/env python3
"""Allan 方差标定 IMU 噪声参数 —— VINS 需要的四个量 + g_norm。

为什么必须做这件事:
    VINS 论文要求 IMU 噪声【逐设备】标定。直接抄 EuRoC 数据集里 ADIS16448
    的参数(acc_n 0.1 / gyr_n 0.01 / acc_w 0.001 / gyr_w 0.0001)是错的 ——
    本机实测与之相差两个数量级。噪声设过大 -> IMU 因子权重过低 ->
    加速度计零偏几乎不受约束 -> 零偏被推到荒谬值(实测到 -1.53 m/s²) ->
    恒定的错误零偏积分出去，表现为位置单方向持续漂移。

原理:
    对静置数据，用不同的平均时长 tau 计算 Allan 偏差 sigma(tau)。
        白噪声(随机游走)   -> 曲线斜率 -1/2，外推到 tau=1s 读出 -> acc_n / gyr_n
        零偏随机游走       -> 曲线斜率 +1/2，在长 tau 段读出   -> acc_w / gyr_w
    这与 imu_utils / allan_variance_ros 的做法一致。

同时检查重力模长及其漂移 —— 它决定 g_norm 能否填固定值:
    本机实测静置 |a| 明显偏离标准重力(约 10.02 vs 9.79)，
    这个差值是加速度计的标度/零偏误差。若 g_norm 按标准值填，
    差值就成了恒定残差全压给 Ba，而 VINS 不建模标度因子、只有零偏一个自由度。

用法:
    python3 scripts/calib_imu_allan.py [采集秒数]     # 默认 600 秒
    采集期间设备必须【完全静止】，且已启动 seeker 驱动。
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 600
IMU_TOPIC = '/imu_data_raw'

A, G, T = [], [], []


class Collector(Node):
    def __init__(self):
        super().__init__('imu_allan')
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=500)
        self.create_subscription(Imu, IMU_TOPIC, self.cb, qos)

    def cb(self, m):
        a, g = m.linear_acceleration, m.angular_velocity
        A.append([a.x, a.y, a.z])
        G.append([g.x, g.y, g.z])
        T.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)


def allan_deviation(x, fs):
    """标准 Allan 偏差: 按块长 m 平均后取相邻块之差的均方根。"""
    n = len(x)
    taus, devs = [], []
    m = 1
    while m < n // 9:                       # 每个 tau 至少要有 9 个块才有统计意义
        k = n // m
        clusters = x[:k * m].reshape(k, m).mean(axis=1)
        d = np.diff(clusters)
        devs.append(np.sqrt(0.5 * np.mean(d ** 2)))
        taus.append(m / fs)
        m = int(np.ceil(m * 1.3))
    return np.array(taus), np.array(devs)


def readout(taus, devs):
    """从曲线上读白噪声 N 和随机游走 K。"""
    # 白噪声段: sigma = N/sqrt(tau)，故 N = sigma*sqrt(tau)，取短 tau 段的中位数
    lo = (taus >= 0.05) & (taus <= 1.0)
    N = np.median(devs[lo] * np.sqrt(taus[lo])) if lo.any() else float('nan')
    # 随机游走段: sigma = K*sqrt(tau/3)，取最长 tau 段
    hi = taus >= taus.max() * 0.4
    K = np.median(devs[hi] / np.sqrt(taus[hi] / 3.0)) / np.sqrt(3.0) if hi.any() else float('nan')
    return N, K


def main():
    rclpy.init()
    node = Collector()
    print(f'▸ 采集 {DUR:.0f} 秒静置数据，请勿触碰设备...', flush=True)
    end = time.time() + DUR
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.01)
    node.destroy_node()
    rclpy.shutdown()

    if len(A) < 1000:
        sys.exit(f'✗ 只收到 {len(A)} 条 IMU 数据，请确认驱动已启动')

    acc, gyr, ts = np.array(A), np.array(G), np.array(T)
    fs = (len(ts) - 1) / (ts[-1] - ts[0])
    print(f'采集 {len(acc)} 样本, 实际频率 {fs:.1f} Hz, 时长 {ts[-1]-ts[0]:.0f}s\n')

    print(f"{'轴':>8} {'白噪声 N':>14} {'随机游走 K':>14}")
    res = {}
    for label, data, keys in (('accel', acc, ('acc_n', 'acc_w')),
                              ('gyro', gyr, ('gyr_n', 'gyr_w'))):
        ns, ks = [], []
        for i, ax in enumerate('xyz'):
            t, d = allan_deviation(data[:, i], fs)
            N, K = readout(t, d)
            ns.append(N); ks.append(K)
            print(f'{label}.{ax:>2} {N:14.3e} {K:14.3e}')
        res[keys[0]], res[keys[1]] = max(ns), max(ks)

    # 安全系数: 实际工况的振动/温漂大于静置估计。
    # 白噪声 x5 是常规做法；随机游走 x10 —— 短时 Allan 会低估长期温漂。
    print('\n=== 写入 VINS 配置（已含安全系数：白噪声 x5，随机游走 x10）===')
    print(f"acc_n: {res['acc_n']*5:.6f}        # 官方 EuRoC 值 0.1")
    print(f"gyr_n: {res['gyr_n']*5:.6f}        # 官方 EuRoC 值 0.01")
    print(f"acc_w: {res['acc_w']*10:.6f}        # 官方 EuRoC 值 0.001")
    print(f"gyr_w: {res['gyr_w']*10:.6f}        # 官方 EuRoC 值 0.0001")

    # ---- 重力模长及其稳定性 ----
    seg = max(1, len(acc) // 10)
    mags = [np.linalg.norm(acc[i*seg:(i+1)*seg].mean(0)) for i in range(10)]
    mean_mag = float(np.linalg.norm(acc.mean(0)))
    print('\n=== 重力模长稳定性（整段切成 10 份）===')
    for i, m in enumerate(mags):
        print(f'  第{i+1:>2}段  {m:.5f}')
    print(f'  均值 {np.mean(mags):.5f}  极差 {max(mags)-min(mags):.5f}  '
          f'标准差 {np.std(mags):.5f}')
    print(f'\ng_norm: {mean_mag:.4f}')
    print(f'当地真实重力约 9.79~9.81 -> 加速度计标度/零偏误差约 '
          f'{100*(mean_mag-9.80)/9.80:+.2f}%')
    if max(mags) - min(mags) > 0.02:
        print('⚠ 采集期内重力模长就有明显漂移，固定 g_norm 覆盖不了，'
              '需要靠 acc_w 给零偏留足自由度')
    else:
        print('✓ 重力模长稳定，可以填固定 g_norm')


if __name__ == '__main__':
    main()
