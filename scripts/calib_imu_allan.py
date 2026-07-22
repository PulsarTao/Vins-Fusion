#!/usr/bin/env python3
"""IMU 噪声标定 —— 用 allan_variance_ros 的算法与拟合脚本，走 ROS2 数据。

为什么必须做这件事:
    VINS 论文要求 IMU 噪声【逐设备】标定。直接抄 EuRoC 数据集里 ADIS16448 的
    参数(acc_n 0.1 / gyr_n 0.01 / acc_w 0.001 / gyr_w 0.0001)是错的 ——
    本机实测与之相差约两个数量级。

为什么不直接用 allan_variance_ros 的可执行文件:
    该仓库是 ROS1(catkin)，C++ 节点只做「读 bag -> 算 Allan 偏差 -> 写 CSV」，
    真正的曲线拟合在它的 scripts/analysis.py 里。这里用 ROS2 采数据，
    严格复现它 AllanVarianceComputor.cpp 的分箱与偏差公式，产出同样格式的 CSV，
    再交给它自己的 analysis.py 拟合 —— 拟合环节用的是上游经过验证的实现，
    不是自己手写的读数。

算法(与 AllanVarianceComputor.cpp 逐行对应):
    对每个 period = 1..10000，取 tau = period * 0.1 秒(0.1s ~ 1000s)，
    把数据按 tau 分箱求均值，然后
        avar = sum((m[k+1]-m[k])^2) / (2*(N-1))
        adev = sqrt(avar)
    输出 CSV(空格分隔): tau accX accY accZ gyroX gyroY gyroZ

采集时长:
    官方建议至少 3 小时 —— 长 tau 段(零偏不稳定、随机游走)需要足够多的分箱
    才有统计意义。少于 1 小时时长 tau 段基本不可信。

用法:
    # 1) 先录静置数据(设备必须完全静止，期间勿触碰)
    ros2 bag record -o static_imu /imu_data_raw
    # 2) 算 Allan 偏差并拟合
    python3 scripts/calib_imu_allan.py <bag路径>
"""
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ANALYSIS = os.path.join(HERE, '..', '.calib_ws', 'src', 'allan_variance_ros',
                        'scripts', 'analysis.py')


def read_bag(bag_path, topic='/imu_data_raw'):
    """从 ROS2 bag 读 IMU。用 rosbag2_py 直读，避免起节点回放。"""
    from rclpy.serialization import deserialize_message
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from sensor_msgs.msg import Imu

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_path, storage_id='sqlite3'),
                ConverterOptions('', ''))
    t, acc, gyr = [], [], []
    while reader.has_next():
        tp, data, _ = reader.read_next()
        if tp != topic:
            continue
        m = deserialize_message(data, Imu)
        t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        acc.append([m.linear_acceleration.x, m.linear_acceleration.y,
                    m.linear_acceleration.z])
        gyr.append([m.angular_velocity.x, m.angular_velocity.y,
                    m.angular_velocity.z])
    return np.array(t), np.array(acc), np.array(gyr)


def allan_csv(t, acc, gyr, out_csv):
    """严格复现 AllanVarianceComputor.cpp 的分箱与偏差计算。

    与上游的两点差异，都是实现细节不影响结果:
      * 用累积和做分箱，避免逐 period 重复扫描整段数据(3 小时数据下
        朴素实现要 1e10 次操作，累积和把每个 period 降到 O(分箱数))
      * tau 用 period/10.0 而不是 period*0.1 —— 后者在浮点下
        100*0.1 = 10.000000000000002，而 analysis.py 用 `period == 10`
        精确比较来定白噪声/随机游走的分界点，会匹配不上
    """
    fs = (len(t) - 1) / (t[-1] - t[0])
    data = np.hstack([acc, gyr]).astype(np.float64)   # 6 列，顺序与工具一致
    n = len(data)
    csum = np.vstack([np.zeros(6), np.cumsum(data, axis=0)])   # 前缀和

    rows = []
    for period in range(1, 10001):                    # tau 0.1s ~ 1000s
        tau = period / 10.0
        b = int(tau * fs)
        if b < 1:
            continue
        nbin = n // b
        if nbin < 3:                                  # 分箱太少，统计无意义
            break
        idx = np.arange(nbin + 1) * b
        m = (csum[idx[1:]] - csum[idx[:-1]]) / b      # 每个分箱的均值
        d = np.diff(m, axis=0)
        avar = (d ** 2).sum(axis=0) / (2 * (nbin - 1))
        rows.append([tau] + list(np.sqrt(avar)))

    with open(out_csv, 'w') as f:
        for r in rows:
            f.write(' '.join(f'{v:.10e}' for v in r) + '\n')
    return fs, len(rows)


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    bag = sys.argv[1]
    if not os.path.exists(bag):
        sys.exit(f'✗ 找不到 bag: {bag}')

    print(f'▸ 读取 {bag} ...')
    t, acc, gyr = read_bag(bag)
    if len(t) < 10000:
        sys.exit(f'✗ 只有 {len(t)} 条 IMU 数据，太少')
    dur = t[-1] - t[0]
    print(f'  {len(t)} 条, 时长 {dur/60:.1f} 分钟, 频率 {(len(t)-1)/dur:.1f} Hz')
    if dur < 3600:
        print(f'  ⚠ 时长不足 1 小时。allan_variance_ros 建议至少 3 小时 —— '
              f'长 tau 段(零偏随机游走)的估计会不可靠')

    out_csv = os.path.join(os.path.dirname(bag) or '.', 'allan_deviation.csv')
    fs, nrow = allan_csv(t, acc, gyr, out_csv)
    print(f'✓ Allan 偏差已写入 {out_csv} ({nrow} 个 tau 点)')

    # ---- 重力模长及其稳定性: 决定 g_norm 能否填固定值 ----
    seg = max(1, len(acc) // 10)
    mags = [np.linalg.norm(acc[i * seg:(i + 1) * seg].mean(0)) for i in range(10)]
    mean_mag = float(np.linalg.norm(acc.mean(0)))
    print('\n=== 重力模长稳定性（整段切成 10 份）===')
    for i, m in enumerate(mags):
        print(f'  第{i+1:>2}段  {m:.5f}')
    print(f'  均值 {np.mean(mags):.5f}  极差 {max(mags)-min(mags):.5f}')
    print(f'\ng_norm 建议值: {mean_mag:.4f}   '
          f'(当地真实重力约 9.79~9.81 -> 标度/零偏误差 {100*(mean_mag-9.80)/9.80:+.2f}%)')

    # ---- 交给上游的 analysis.py 做拟合 ----
    if not os.path.exists(ANALYSIS):
        print(f'\n⚠ 找不到 {ANALYSIS}')
        print('  先执行: git clone https://github.com/ori-drs/allan_variance_ros '
              '.calib_ws/src/allan_variance_ros')
        return
    print('\n▸ 调用 allan_variance_ros/scripts/analysis.py 做拟合 ...')
    r = subprocess.run([sys.executable, ANALYSIS, '--data', out_csv,
                        '--skip', '1'],
                       capture_output=True, text=True)
    print(r.stdout or r.stderr)


if __name__ == '__main__':
    main()
