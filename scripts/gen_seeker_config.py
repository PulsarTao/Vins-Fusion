#!/usr/bin/env python3
"""seeker1 四目鱼眼 → VINS-Fusion 配置生成器（鱼眼去畸变成矫正双目）。

============================================================================
为什么不能直接把鱼眼标定喂给 VINS
============================================================================
出厂标定是 omni(MEI) 模型，镜面参数 xi = 3.22。VINS 的 CataCamera 反投影
(liftProjective, CataCamera.cc:624) 里有这么一句:

    P << mx_u, my_u, 1.0 - xi*(rho2_d+1.0) / (xi + sqrt(1.0 + (1.0-xi*xi)*rho2_d));
                                                          ~~~~~~~~~~~~~~~~~~~~~~~
    xi=3.22 -> (1-xi^2) = -9.37 -> 只要 rho2_d > 0.107 就是负数开方 -> NaN

标准 MEI 模型的 xi 定义域是 [0,1]（xi=0 针孔, xi=1 抛物面）。Kalibr 用的是
扩展形式，允许 xi>1，两者不兼容。实测直接用 MEI 配置，VINS 位姿全是 nan。

注意：正向投影 spaceToPlane 没有这个问题
    z = P.z + xi*||P||，xi>1 时恒有 z >= (xi-1)*||P|| > 0
所以「预计算一张查找表，把鱼眼重映射成针孔」在数学上完全安全。

============================================================================
厂家其实已经把这条路铺好了
============================================================================
seeker 自带两个标定脚本:
    1get_kalibr_info.py           读原始鱼眼标定 (omni, 4 个物理相机)
    3_get_undistort_kalibr_info.py 算矫正后标定 (pinhole, 8 个虚拟相机)

脚本 3 的做法(读源码 generatestereoinfo 得知): 把相邻两个物理鱼眼配成一对
**矫正双目**——不是把一个鱼眼切两半:
    front = 物理 cam0(left)  + cam1(right)
    right = 物理 cam1(right) + cam2(bright)
    back  = 物理 cam2(bright)+ cam3(bleft)
    left  = 物理 cam3(bleft) + cam0(left)
每个虚拟针孔图只来自一个物理鱼眼，且**光心不变、只做旋转**（脚本里那个 T
的平移是 0），所以重映射不需要深度，纯粹是方向重采样。

本脚本复现脚本 3 的矫正数学，额外产出 C++ 节点需要的旋转矩阵，并把结果与
厂家自己的输出(config/seeker/kalibr_undistorted.yaml)逐项比对做交叉验证。

============================================================================
产出
============================================================================
    config/seeker/seeker_remap.yaml    给 seeker_split_node 的重映射参数
    config/seeker/cam0_pinhole.yaml    VINS 相机内参(PINHOLE, 零畸变)
    config/seeker/cam1_pinhole.yaml
    config/seeker/seeker_stereo_imu.yaml  VINS 主配置

用法:
    python3 gen_seeker_config.py                  # front 对(默认), 用缓存标定
    python3 gen_seeker_config.py --pair back      # 换成后视双目
    python3 gen_seeker_config.py --from-device    # 重新从相机读标定
"""
import argparse
import os
import subprocess
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
KALIBR_SCRIPT = os.path.join(HERE, '..', 'ros2_ws_src', 'seeker1', 'script',
                             '1get_kalibr_info.py')
RAW_CACHE = os.path.join(HERE, '..', 'config', 'seeker', 'kalibr_raw_fisheye.yaml')
VENDOR_REF = os.path.join(HERE, '..', 'config', 'seeker', 'kalibr_undistorted.yaml')

# 物理相机顺序固定，来自驱动 seeker_ros2.cpp 的 image_topics 数组，
# /all/compressed 解码后就是按这个顺序垂直堆叠的
PHYS_NAMES = ['left', 'right', 'bright', 'bleft']

# 与厂家脚本 3 的 generatestereoinfo 调用一一对应。
# 值是 (左物理相机索引, 右物理相机索引, 厂家输出里的虚拟相机编号)
PAIRS = {
    'front': (0, 1, ('cam0', 'cam1')),
    'right': (1, 2, ('cam2', 'cam3')),
    'back':  (2, 3, ('cam4', 'cam5')),
    'left':  (3, 0, ('cam6', 'cam7')),
}

# 虚拟针孔相机规格，与厂家脚本 3 写死的值保持一致，改了就对不上厂家标定了
VIRT_W, VIRT_H = 640, 480
VIRT_FX = VIRT_FY = 320.0
VIRT_CX, VIRT_CY = VIRT_W / 2.0, VIRT_H / 2.0


def inv_T(T):
    """4x4 位姿求逆。"""
    R, t = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def read_calibration(from_device):
    """拿原始鱼眼标定：默认读缓存，--from-device 时跑官方脚本重新读设备。"""
    if not from_device and os.path.exists(RAW_CACHE):
        print(f'▸ 使用缓存标定 {os.path.relpath(RAW_CACHE)}（--from-device 可强制重读）')
        with open(RAW_CACHE) as f:
            return yaml.safe_load(f)

    if not os.path.exists(KALIBR_SCRIPT):
        sys.exit(f'✗ 找不到标定脚本: {KALIBR_SCRIPT}')
    print('▸ 从相机读取标定（相机需已连接，且未被驱动占用）...')
    r = subprocess.run([sys.executable, KALIBR_SCRIPT], capture_output=True,
                       text=True, timeout=120, cwd=os.path.dirname(KALIBR_SCRIPT))
    for line in r.stdout.splitlines():
        line = line.strip()
        if line.startswith('{') and 'cam0' in line:
            calib = eval(line)                      # 脚本打印的是 python dict 字面量
            with open(RAW_CACHE, 'w') as f:         # 顺手更新缓存
                yaml.safe_dump(calib, f, default_flow_style=None)
            print(f'✓ 标定已缓存到 {os.path.relpath(RAW_CACHE)}')
            return calib
    print(r.stdout[-500:], file=sys.stderr)
    print(r.stderr[-500:], file=sys.stderr)
    sys.exit('✗ 未能从脚本输出中解析标定，请确认相机已连接且驱动已停止')


def rectify_pair(calib, li, ri):
    """复现厂家脚本 3 的 generatestereoinfo：把两个物理鱼眼配成矫正双目。

    返回 (T_virtL_imu, T_virtR_imu, baseline)。
    两个虚拟相机共用同一朝向，X 轴沿基线，构成标准的行对齐双目。
    """
    T_l_imu = np.array(calib[f'cam{li}']['T_cam_imu'], dtype=np.float64)
    T_r_imu = np.array(calib[f'cam{ri}']['T_cam_imu'], dtype=np.float64)

    T_r_l = T_r_imu @ inv_T(T_l_imu)          # 左目 -> 右目
    baseline_vec = inv_T(T_r_l)[:3, 3]        # 右目光心在左目坐标系里的位置

    # 新坐标系: X 沿基线；Y = Zref × X（Zref 取左目原坐标系的 Z）；Z = X × Y
    x = baseline_vec / np.linalg.norm(baseline_vec)
    y = np.cross(np.array([0.0, 0.0, 1.0]), x)
    y /= np.linalg.norm(y)
    z = np.cross(x, y)
    z /= np.linalg.norm(z)

    R_l_rect = np.column_stack([x, y, z])     # 矫正系 -> 左目物理系
    T = np.eye(4)
    T[:3, :3] = R_l_rect

    T_vl_imu = inv_T(T) @ T_l_imu             # 纯旋转，光心不动
    baseline = float(np.linalg.norm(T_r_l[:3, 3]))

    # 右虚拟目 = 左虚拟目沿 X 平移一个基线，朝向完全相同
    shift = np.eye(4)
    shift[0, 3] = -baseline
    T_vr_imu = shift @ T_vl_imu
    return T_vl_imu, T_vr_imu, baseline


def mei_project(rays, intr, dist):
    """MEI(omni) 正向投影，与 VINS CataCamera::spaceToPlane 逐行对应。

    rays: (N,3) 方向向量（不必归一化）。返回 (N,2) 像素坐标（原始分辨率）。
    xi>1 时 z = Pz + xi*||P|| 恒为正，所以这里不会出现除零/开负根。
    """
    xi, fx, fy, cx, cy = intr
    k1, k2, p1, p2 = dist[:4]

    norm = np.linalg.norm(rays, axis=1)
    z = rays[:, 2] + xi * norm
    xu = rays[:, 0] / z
    yu = rays[:, 1] / z

    # radtan 畸变，与 CataCamera::distortion 一致
    x2, y2, xy = xu * xu, yu * yu, xu * yu
    r2 = x2 + y2
    rad = k1 * r2 + k2 * r2 * r2
    xd = xu + xu * rad + 2.0 * p1 * xy + p2 * (r2 + 2.0 * x2)
    yd = yu + yu * rad + 2.0 * p2 * xy + p1 * (r2 + 2.0 * y2)

    return np.column_stack([fx * xd + cx, fy * yd + cy])


def check_against_vendor(pair, T_vl, T_vr):
    """与厂家 3_get_undistort_kalibr_info.py 的输出做交叉验证。

    我们是自己重新实现的矫正数学，必须证明和厂家算的是同一个东西，
    否则外参错了 VINS 会以一个错误的 body_T_cam 收敛到错误的尺度/姿态。
    """
    if not os.path.exists(VENDOR_REF):
        print('  (跳过交叉验证：没有厂家参考文件)')
        return
    with open(VENDOR_REF) as f:
        ref = yaml.safe_load(f.read().replace('%YAML:1.0', ''))

    _, _, (kl, kr) = PAIRS[pair]
    worst = 0.0
    for key, ours in ((kl, T_vl), (kr, T_vr)):
        theirs = np.array(ref[key]['T_cam_imu'], dtype=np.float64)
        worst = max(worst, float(np.abs(theirs - ours).max()))
    if worst < 1e-9:
        print(f'✓ 外参与厂家 kalibr_undistorted.yaml 完全一致（最大差 {worst:.2e}）')
    else:
        print(f'⚠ 外参与厂家输出有差异，最大 {worst:.3e} —— 请检查矫正数学')


CAM_TPL = """%YAML:1.0
---
# 由 gen_seeker_config.py 生成，请勿手改
#
# 这是【虚拟】针孔相机 —— 物理上是鱼眼，由 seeker_split_node 用预计算的
# remap 表去畸变成针孔后才发出来。所以这里畸变系数全 0 是对的，不是漏填。
#
# 为什么不用鱼眼原生的 MEI 模型：出厂标定 xi={xi:.2f} > 1，VINS 的
# CataCamera 反投影会开负数根得到 NaN。详见 gen_seeker_config.py 头部说明。
model_type: PINHOLE
camera_name: cam{idx}
image_width: {w}
image_height: {h}
distortion_parameters:
   k1: 0.0
   k2: 0.0
   p1: 0.0
   p2: 0.0
projection_parameters:
   fx: {fx:.10f}
   fy: {fy:.10f}
   cx: {cx:.10f}
   cy: {cy:.10f}
"""


def mat_block(name, T):
    rows = ['          ' + ', '.join(f'{T[i][j]:.10f}' for j in range(4)) + ','
            for i in range(4)]
    body = '\n'.join(rows)[:-1]
    return (f"{name}: !!opencv-matrix\n   rows: 4\n   cols: 4\n   dt: d\n"
            f"   data: [\n{body} ]\n")


def write_remap_yaml(path, pair, li, ri, calib, T_vl, T_vr, scale):
    """写给 C++ 节点的重映射参数（OpenCV FileStorage 格式，cv::FileStorage 直接读）。

    节点拿到这些就能自己 initUndistortRectifyMap 等价的事：
    对每个虚拟像素反算方向 -> 用 R_phys_virt 转到物理鱼眼系 -> MEI 正投影
    """
    lines = ['%YAML:1.0', '---',
             '# 由 gen_seeker_config.py 生成，供 seeker_split_node 构建重映射表',
             f'# 双目对: {pair}  = 物理鱼眼 {li}({PHYS_NAMES[li]}) + {ri}({PHYS_NAMES[ri]})',
             '#',
             '# src_index    该虚拟相机取自 /all/compressed 的第几路（垂直堆叠的切片号）',
             '# mei_*        物理鱼眼的 omni 标定（原始 1088x1280 分辨率下的值）',
             '# R_phys_virt  虚拟针孔系 -> 物理鱼眼系 的旋转（列主序 3x3）',
             '#              光心相同，所以只需旋转，与深度无关',
             '# decode_scale JPEG 解码时的降采样比例；节点会据此缩放 mei 内参',
             '',
             f'virt_width: {VIRT_W}',
             f'virt_height: {VIRT_H}',
             f'virt_fx: {VIRT_FX}', f'virt_fy: {VIRT_FY}',
             f'virt_cx: {VIRT_CX}', f'virt_cy: {VIRT_CY}',
             f'decode_scale: {scale}',
             '']

    for slot, (phys_idx, T_virt) in enumerate(((li, T_vl), (ri, T_vr))):
        c = calib[f'cam{phys_idx}']
        R_phys_imu = np.array(c['T_cam_imu'], dtype=np.float64)[:3, :3]
        R_virt_imu = T_virt[:3, :3]
        # 物理系 <- 虚拟系:  R = R_phys_imu * R_virt_imu^T
        R = R_phys_imu @ R_virt_imu.T
        xi, fx, fy, cx, cy = c['intrinsics']
        k1, k2, p1, p2 = c['distortion_coeffs'][:4]
        w, h = c['resolution']

        # 注意缩进: OpenCV 的 FileStorage YAML 解析器要求 data 的续行缩进
        # 严格深于 data: 这个键本身，同级会报 "Incorrect indentation"
        rows = ['             ' + ', '.join(f'{R[i][j]:.12f}' for j in range(3)) + ','
                for i in range(3)]
        lines += [
            f'cam{slot}:',
            f'   src_index: {phys_idx}',
            f'   src_width: {w}',
            f'   src_height: {h}',
            f'   mei_xi: {xi:.12f}',
            f'   mei_fx: {fx:.12f}', f'   mei_fy: {fy:.12f}',
            f'   mei_cx: {cx:.12f}', f'   mei_cy: {cy:.12f}',
            f'   mei_k1: {k1:.12f}', f'   mei_k2: {k2:.12f}',
            f'   mei_p1: {p1:.12f}', f'   mei_p2: {p2:.12f}',
            '   R_phys_virt: !!opencv-matrix',
            '      rows: 3', '      cols: 3', '      dt: d',
            '      data: [', '\n'.join(rows)[:-1] + ' ]',
            '']
    with open(path, 'w') as f:
        f.write('\n'.join(lines))


def report_coverage(calib, phys_idx, T_virt, label):
    """检查虚拟视场是否完全落在鱼眼的有效成像圆内。

    虚拟相机 90°x74° 视场旋转到物理鱼眼系后，若有方向落到图像外，
    那部分就是黑边——VINS 在黑边上提不到特征，不致命但要心里有数。
    """
    c = calib[f'cam{phys_idx}']
    R = np.array(c['T_cam_imu'], dtype=np.float64)[:3, :3] @ T_virt[:3, :3].T
    w, h = c['resolution']

    u, v = np.meshgrid(np.arange(VIRT_W), np.arange(VIRT_H))
    rays = np.column_stack([((u.ravel() - VIRT_CX) / VIRT_FX),
                            ((v.ravel() - VIRT_CY) / VIRT_FY),
                            np.ones(u.size)])
    rays = rays @ R.T
    px = mei_project(rays, c['intrinsics'], c['distortion_coeffs'])

    inside = ((px[:, 0] >= 0) & (px[:, 0] < w) &
              (px[:, 1] >= 0) & (px[:, 1] < h))
    frac = inside.mean()
    # 虚拟光轴相对物理光轴偏了多少度
    axis = R @ np.array([0.0, 0.0, 1.0])
    ang = np.degrees(np.arccos(np.clip(axis[2], -1, 1)))
    flag = '✓' if frac > 0.999 else ('⚠' if frac > 0.9 else '✗')
    print(f'  {flag} {label}: 光轴偏转 {ang:5.1f}°, 有效像素 {frac*100:.2f}%')
    return frac


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pair', choices=list(PAIRS), default='front',
                    help='用哪一对矫正双目（默认 front = 物理鱼眼 left+right）')
    ap.add_argument('--scale', type=float, default=1.0,
                    help='JPEG 解码降采样比例。虚拟图 640x480 的角分辨率与鱼眼'
                         '全分辨率相当，降采样会丢细节，默认不降(1.0)')
    ap.add_argument('--from-device', action='store_true',
                    help='重新从相机读标定而不是用缓存')
    ap.add_argument('--out', default=os.path.join(HERE, '..', 'config', 'seeker'))
    a = ap.parse_args()

    calib = read_calibration(a.from_device)
    li, ri, _ = PAIRS[a.pair]
    os.makedirs(a.out, exist_ok=True)

    print(f'\n▸ 矫正双目 "{a.pair}" = 物理鱼眼 {li}({PHYS_NAMES[li]}) + {ri}({PHYS_NAMES[ri]})')
    T_vl, T_vr, baseline = rectify_pair(calib, li, ri)
    check_against_vendor(a.pair, T_vl, T_vr)

    print('▸ 视场覆盖检查:')
    report_coverage(calib, li, T_vl, f'cam0 <- 鱼眼{li}')
    report_coverage(calib, ri, T_vr, f'cam1 <- 鱼眼{ri}')

    # ---- 虚拟相机内参（零畸变针孔）----
    for idx in (0, 1):
        path = os.path.join(a.out, f'cam{idx}_pinhole.yaml')
        with open(path, 'w') as f:
            f.write(CAM_TPL.format(idx=idx, w=VIRT_W, h=VIRT_H,
                                   fx=VIRT_FX, fy=VIRT_FY,
                                   cx=VIRT_CX, cy=VIRT_CY,
                                   xi=calib[f'cam{li}']['intrinsics'][0]))
        print(f'✓ {os.path.relpath(path)}')

    # ---- 给 C++ 节点的重映射参数 ----
    remap_path = os.path.join(a.out, 'seeker_remap.yaml')
    write_remap_yaml(remap_path, a.pair, li, ri, calib, T_vl, T_vr, a.scale)
    print(f'✓ {os.path.relpath(remap_path)}')

    # ---- VINS 主配置。外参用虚拟相机的 T_imu_cam ----
    hfov = 2 * np.degrees(np.arctan(VIRT_W / 2 / VIRT_FX))
    vfov = 2 * np.degrees(np.arctan(VIRT_H / 2 / VIRT_FY))
    main_yaml = f"""%YAML:1.0

# =============================================================================
#  seeker1 四目鱼眼环视相机 · 矫正双目 + IMU · VINS-Fusion (ROS2 Humble)
#
#  本文件由 scripts/gen_seeker_config.py 生成，标定读自设备，不是估计值。
#
#  数据链路:
#    seeker 驱动 --/all/compressed(JPEG,四路堆叠)--> seeker_split_node
#      节点内: 解码 -> 切片 -> 鱼眼去畸变(预计算 remap) -> 发 /cam0 /cam1
#    为什么要中间这一层:
#      驱动的 /fisheye/*/image_raw 是 bgr8 1088x1280，单帧 4.18MB，
#      DDS 传这种大消息实测只有 2.2~8.0Hz；走压缩流本地拆分可跑满 20Hz。
#
#  相机: {a.pair} 矫正双目 = 物理鱼眼 {li}({PHYS_NAMES[li]}) + {ri}({PHYS_NAMES[ri]})
#  基线: {baseline*100:.2f} cm
#  模型: PINHOLE 零畸变（去畸变后的虚拟相机）
#        不能用 MEI —— 出厂 xi={calib[f'cam{li}']['intrinsics'][0]:.2f}>1，
#        VINS 的 CataCamera 反投影会开负数根输出 NaN
#  视场: {hfov:.1f}° x {vfov:.1f}°  ({VIRT_W}x{VIRT_H}, f={VIRT_FX:.0f})
# =============================================================================

imu: 1
num_of_cam: 2

imu_topic: "/imu_data_raw"
image0_topic: "/cam0/image_raw"
image1_topic: "/cam1/image_raw"
output_path: "/root/output/"

cam0_calib: "cam0_pinhole.yaml"
cam1_calib: "cam1_pinhole.yaml"
image_width: {VIRT_W}
image_height: {VIRT_H}

use_gpu         : 0
use_gpu_acc_flow: 0
use_gpu_ceres   : 0

# -----------------------------------------------------------------------------
# 外参: body(IMU) <- 虚拟相机
#   由厂家鱼眼标定的 T_cam_imu 经矫正旋转推出，已与厂家
#   3_get_undistort_kalibr_info.py 的输出逐项比对一致，所以设 0 完全信任。
# -----------------------------------------------------------------------------
estimate_extrinsic: 0

{mat_block('body_T_cam0', inv_T(T_vl))}
{mat_block('body_T_cam1', inv_T(T_vr))}

multiple_thread: 1

max_cnt: 150
min_dist: 25
freq: 10
F_threshold: 1.0
show_track: 1
flow_back: 1

max_solver_time: 0.04
max_num_iterations: 8

# -----------------------------------------------------------------------------
# keyframe_parallax —— 这个值必须远小于上游默认的 10.0，否则相机静止时必然发散
#
# 机理(读 estimator.cpp:1467 + feature_manager.cpp:117 得到):
#   视差 < 阈值 -> addFeatureCheckParallax 返回 false -> MARGIN_SECOND_NEW
#   -> 新帧的 IMU 采样被【并入前一帧的预积分】，而不是产生新关键帧
#   -> 相机一直不动就一直并，预积分区间 sum_dt 无界增长
#   -> 未建模的加速度计零偏(本机实测 0.083 m/s²) 乘以 sum_dt²
#   -> 位置按二次曲线爆炸，同时 jacobian>1e8 触发 "numerical unstable in preintegration"
#
# 本机实测(相机静置桌面，其余参数完全相同):
#     10.0 (上游默认)   35 秒漂移 244 米    numerical unstable 11 次
#      1.0              35 秒漂移  56 米    8 次
#      0.5              35 秒漂移 0.4 毫米  0 次
#      0.3              45 秒漂移 14.5 毫米 0 次   <- 采用
#   0.5 与 1.0 之间是悬崖：静止时的视差噪声就在 0.5~1.0 像素，
#   阈值必须压到噪声之下，才能保证静止时仍然产生关键帧。取 0.3 是为了留余量。
#
# 代价几乎为零：真实运动时(手持 0.3m/s、1 米处特征)每帧视差本就有约 10 像素，
# 阈值取 10 还是 0.3 都会走 MARGIN_OLD，行为一致。这个阈值只在「慢速/静止」
# 时起作用，而那正是它必须起作用的场景。
# 副作用：关键帧变多，solver 从 3ms 升到 12ms —— 10Hz 下仍只占 12% 时间预算。
#
# 更彻底的做法是改 VINS 源码，在预积分区间超过阈值(如 0.5s)时强制产生关键帧，
# 那样就不依赖噪声水平了。这里选择只改配置，避免改动上游代码。
# -----------------------------------------------------------------------------
keyframe_parallax: 0.3

# -----------------------------------------------------------------------------
# IMU 噪声 —— 厂家未提供，用经验值(宁大勿小，设太小机动时容易发散)。
# 要精确可用 imu_utils / allan_variance_ros 跑 2 小时静置数据标定。
# -----------------------------------------------------------------------------
acc_n: 0.1
gyr_n: 0.01
acc_w: 0.001
gyr_w: 0.0001
g_norm: 9.805

# -----------------------------------------------------------------------------
# estimate_td 必须关掉，开着会死锁 —— 这是本机实测踩过的最严重的坑
#
# td 是相机与 IMU 的时间偏移，它只有在【有运动】时才可观测：时间偏移造成的
# 特征位移正比于运动速度，静止时该项恒为 0，td 完全不可观。于是优化器可以
# 随意推动 td 去吸收别的误差。实测相机静置桌面时 td 的演化：
#     0.000 -> 0.463 -> 0.926 -> 1.387 -> 140.885 -> 558.826 秒
# 一旦 td 变大，VINS 内部 curTime = 图像时间 + td 就会远远领先 IMU 流，
# IMUAvailable() 永远返回 false，估计器卡死在 "wait for imu"（实测刷了 4.7 万次），
# 同时位置在死锁前已被打飞到千米量级。
#
# 关掉是安全的：厂家标定给出 timeshift_cam_imu = 0.0，且驱动的图像与 IMU
# 时间戳来自设备同一时钟(实测两者 dt 都干净、无重复无倒退)。
# 若将来确认存在固定偏移，直接把测得的值写进下面的 td，仍然保持 estimate_td: 0。
# -----------------------------------------------------------------------------
estimate_td: 0
td: {calib[f'cam{li}']['timeshift_cam_imu']}

load_previous_pose_graph: 0
pose_graph_save_path: "/root/output/pose_graph/"
save_image: 0
"""
    path = os.path.join(a.out, 'seeker_stereo_imu.yaml')
    with open(path, 'w') as f:
        f.write(main_yaml)
    print(f'✓ {os.path.relpath(path)}')
    print(f'\n  基线 {baseline*100:.2f} cm，虚拟图 {VIRT_W}x{VIRT_H}，'
          f'视场 {hfov:.0f}°x{vfov:.0f}°')


if __name__ == '__main__':
    main()
