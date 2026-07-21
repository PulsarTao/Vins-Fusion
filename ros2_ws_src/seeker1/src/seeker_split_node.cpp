// =============================================================================
//  seeker_split_node —— seeker1 四目鱼眼 → VINS 矫正双目输入
//
//  职责三件事：解压 / 切片 / 鱼眼去畸变，一步不能少，理由如下。
//
//  ── 为什么要解压这一层（实测数据，不是想当然）────────────────────────────
//    驱动同时发两种数据：
//      /fisheye/{left,right,bright,bleft}/image_raw
//          bgr8 1088x1280，单帧 4.18MB。DDS 传这种大消息严重掉速，
//          实测四路合计只有 2.2 ~ 8.0 Hz，且互相抢带宽帧率极不均匀。
//      /all/compressed
//          设备直出的 MJPEG，四路垂直堆叠成一张，约 346KB，实测稳定 19.9 Hz。
//    也就是说相机硬件本来就能出 20Hz，瓶颈在「DDS 传未压缩大图」。
//    先用 Python 写过一版中转，最高只有 9 Hz —— 瓶颈是 rclpy 本身：
//    纯接收(不解码不发布)上限 10.4 Hz，发布一张 350KB 图要 20.5ms。
//    这是 Python ROS2 的固有开销，调参解决不了，所以用 C++。
//
//  ── 为什么要去畸变（不去 VINS 直接输出 NaN）──────────────────────────────
//    出厂标定是 omni(MEI) 模型，镜面参数 xi = 3.22。
//    VINS 的 CataCamera 反投影 (CataCamera.cc:624) 里：
//        ... / (xi + sqrt(1.0 + (1.0 - xi*xi) * rho2_d))
//    xi=3.22 时 (1-xi²) = -9.37，只要 rho2_d > 0.107 就是负数开方 → NaN。
//    标准 MEI 的 xi 定义域是 [0,1]，Kalibr 用的是允许 xi>1 的扩展形式，
//    两者不兼容 —— 实测直接喂 MEI 配置，VINS 位姿全是 nan。
//
//    但【正向】投影没这个问题：z = Pz + xi*||P||，xi>1 时恒为正。
//    所以预计算一张「虚拟针孔像素 → 鱼眼像素」的查找表是安全的，
//    只用到正向投影。表由 gen_seeker_config.py 算好参数，这里构建。
//
//  ── 虚拟相机是什么 ──────────────────────────────────────────────────────
//    厂家脚本 3_get_undistort_kalibr_info.py 的做法：把相邻两个物理鱼眼
//    配成一对【矫正双目】(front/right/back/left 四对)。每个虚拟针孔图只来自
//    一个物理鱼眼，且光心不变、只做旋转 —— 所以重映射与深度无关，纯方向重采样。
//    这也意味着 VINS 拿到的是行对齐的标准双目，双目匹配走的是最可靠的路径。
//
//  ── 数据格式（读自驱动 seeker_ros2.cpp:186 onImage）───────────────────────
//    /all/compressed 解码后四路【垂直堆叠】，每路高 = rows/4，顺序固定：
//        0 = left    1 = right    2 = bright    3 = bleft
//    与 1get_kalibr_info.py 读出的 cam0~cam3 标定一一对应。
//
//  ── 可选的曝光对齐（默认关闭，理由见下）──────────────────────────────────
//    四个鱼眼各自跑独立自动曝光，物理朝向又差 90°，看到的光照不同，AE 会收敛到
//    不同的值。实测某时刻两目：cam0 均值 136.9(9.9% 饱和)，cam1 均值 111.5
//    (3.8% 饱和)，均值差 25 灰阶。这理论上会破坏 LK 的灰度不变假设。
//    驱动没开放曝光控制(seeker.hpp 只有 init/open/流控/标定/重启)，只能在这里补。
//
//    但实测结论是【收益不确定，所以默认关闭】。两次对照实验(各 40~50 对同步帧、
//    完整复现 VINS 的 LK + 反向校验)结果并不一致：
//                        第一次(强逆光)          第二次(光照较均匀)
//                        双目匹配                帧间跟踪 / 双目匹配
//        原始              53.8%                  99.9% / 59.3%
//        CLAHE 双边        64.0%                  99.9% / 59.9%
//        直方图匹配        67.2%                  99.9% / 59.5%
//        两者叠加          67.8%                  99.9% / 56.3%
//    也就是说只在两目曝光差很大时才有明显收益，光照均匀时反而可能略微变差。
//    第一次实验我只测了双目匹配没测帧间跟踪，差点据此做出错误结论 ——
//    VINS 两个指标都要：帧间跟踪决定特征能否攒够 4 次观测进入优化，
//    双目匹配决定有没有深度约束。
//
//    需要时用参数打开: -p hist_match:=true -p clahe:=true
//
//  ── 输出 ────────────────────────────────────────────────────────────────
//    /cam0/image_raw, /cam1/image_raw  —— mono8 640x480，VINS 直接可用
//    对应 config/seeker/cam{0,1}_pinhole.yaml（PINHOLE，零畸变）
// =============================================================================
#include <cmath>
#include <cstring>
#include <memory>
#include <string>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace
{
// MEI(omni) 正向投影，与 VINS 的 CataCamera::spaceToPlane + distortion 逐行对应。
// 只在启动时建表用，不在每帧热路径上。
struct MeiCamera
{
  double xi, fx, fy, cx, cy, k1, k2, p1, p2;

  cv::Point2d project(const cv::Vec3d & P) const
  {
    const double n = std::sqrt(P[0] * P[0] + P[1] * P[1] + P[2] * P[2]);
    const double z = P[2] + xi * n;          // xi>1 时恒 > 0，不会除零
    const double xu = P[0] / z, yu = P[1] / z;

    const double x2 = xu * xu, y2 = yu * yu, xy = xu * yu;
    const double r2 = x2 + y2;
    const double rad = k1 * r2 + k2 * r2 * r2;
    const double xd = xu + xu * rad + 2.0 * p1 * xy + p2 * (r2 + 2.0 * x2);
    const double yd = yu + yu * rad + 2.0 * p2 * xy + p1 * (r2 + 2.0 * y2);

    return {fx * xd + cx, fy * yd + cy};
  }
};

// 直方图匹配：把 src 的灰度分布映射成 ref 的分布，用来消除两目之间的曝光差。
// 做法是比对两者的累积分布(CDF)，为每个灰阶找到 ref 里 CDF 最接近的灰阶，
// 生成一张 256 项的查找表，再用 cv::LUT 一次性套用。
// 复杂度：两次直方图 O(N) + 建表 O(256) + LUT O(N)，640x480 下约 0.3ms。
void matchHistogram(const cv::Mat & src, const cv::Mat & ref, cv::Mat & dst)
{
  int hs[256] = {0}, hr[256] = {0};
  for (int r = 0; r < src.rows; ++r) {
    const uint8_t * p = src.ptr<uint8_t>(r);
    for (int c = 0; c < src.cols; ++c) ++hs[p[c]];
  }
  for (int r = 0; r < ref.rows; ++r) {
    const uint8_t * p = ref.ptr<uint8_t>(r);
    for (int c = 0; c < ref.cols; ++c) ++hr[p[c]];
  }

  double cs[256], cr[256], as = 0, ar = 0;
  const double ns = static_cast<double>(src.total());
  const double nr = static_cast<double>(ref.total());
  for (int i = 0; i < 256; ++i) {
    as += hs[i]; cs[i] = as / ns;
    ar += hr[i]; cr[i] = ar / nr;
  }

  cv::Mat lut(1, 256, CV_8UC1);
  uint8_t * L = lut.ptr<uint8_t>();
  int j = 0;
  for (int i = 0; i < 256; ++i) {          // 两条 CDF 都单调，一次同向扫描即可
    while (j < 255 && cr[j] < cs[i]) ++j;
    L[i] = static_cast<uint8_t>(j);
  }
  cv::LUT(src, lut, dst);
}
}  // namespace

class SeekerSplitNode : public rclcpp::Node
{
public:
  SeekerSplitNode() : Node("seeker_split")
  {
    // 默认路径指向容器内的挂载点；本机跑时用参数覆盖
    const auto remap_file = declare_parameter<std::string>(
        "remap_file", "/ros2_ws/vins_config/seeker/seeker_remap.yaml");
    input_topic_ = declare_parameter<std::string>("input_topic", "/all/compressed");
    // 曝光对齐默认关闭 —— 实测收益随光照条件而变，均匀光照下甚至略有负作用。
    // 两目曝光差很大(强逆光)时再打开。详见文件头的对照实验数据。
    hist_match_ = declare_parameter<bool>("hist_match", false);
    clahe_ = declare_parameter<bool>("clahe", false);
    const double clip = declare_parameter<double>("clahe_clip", 3.0);
    clahe_impl_ = cv::createCLAHE(clip, cv::Size(8, 8));

    if (!buildMaps(remap_file)) {
      throw std::runtime_error("重映射表构建失败: " + remap_file);
    }

    // 传感器数据用 BEST_EFFORT：丢帧也不阻塞，且与 VINS 订阅端 QoS 匹配
    auto qos = rclcpp::SensorDataQoS().keep_last(4);
    pub0_ = create_publisher<sensor_msgs::msg::Image>("/cam0/image_raw", qos);
    pub1_ = create_publisher<sensor_msgs::msg::Image>("/cam1/image_raw", qos);
    sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        input_topic_, qos,
        std::bind(&SeekerSplitNode::onCompressed, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "seeker_split: %s -> 鱼眼%d/%d 去畸变 -> /cam0 /cam1 (mono8 %dx%d)"
                " 曝光对齐: 直方图匹配=%s CLAHE=%s",
                input_topic_.c_str(), src_index_[0], src_index_[1],
                virt_w_, virt_h_, hist_match_ ? "开" : "关", clahe_ ? "开" : "关");
  }

private:
  static constexpr int kNumCams = 4;

  // ---------------------------------------------------------------------------
  // 建表：对每个虚拟像素反算方向 → 旋转到物理鱼眼系 → MEI 正投影得源像素。
  // 只在启动时跑一次，之后每帧只是一次 cv::remap 查表。
  // ---------------------------------------------------------------------------
  bool buildMaps(const std::string & path)
  {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      RCLCPP_ERROR(get_logger(), "打不开重映射配置: %s", path.c_str());
      RCLCPP_ERROR(get_logger(), "请先运行 scripts/gen_seeker_config.py 生成");
      return false;
    }

    virt_w_ = (int)fs["virt_width"];
    virt_h_ = (int)fs["virt_height"];
    const double vfx = (double)fs["virt_fx"], vfy = (double)fs["virt_fy"];
    const double vcx = (double)fs["virt_cx"], vcy = (double)fs["virt_cy"];
    // JPEG 解码若降采样，源图像素坐标要同比缩放。xi 和畸变系数无量纲，不缩放。
    const double scale = (double)fs["decode_scale"];

    if (std::abs(scale - 0.5) < 1e-9) {
      decode_flag_ = cv::IMREAD_REDUCED_GRAYSCALE_2;
    } else if (std::abs(scale - 0.25) < 1e-9) {
      decode_flag_ = cv::IMREAD_REDUCED_GRAYSCALE_4;
    } else {
      decode_flag_ = cv::IMREAD_GRAYSCALE;      // scale=1.0，默认
    }

    for (int slot = 0; slot < 2; ++slot) {
      cv::FileNode n = fs["cam" + std::to_string(slot)];
      if (n.empty()) {
        RCLCPP_ERROR(get_logger(), "配置里缺 cam%d", slot);
        return false;
      }
      src_index_[slot] = (int)n["src_index"];
      src_h_[slot] = (int)((double)n["src_height"] * scale);

      MeiCamera mei{(double)n["mei_xi"],
                    (double)n["mei_fx"] * scale, (double)n["mei_fy"] * scale,
                    (double)n["mei_cx"] * scale, (double)n["mei_cy"] * scale,
                    (double)n["mei_k1"], (double)n["mei_k2"],
                    (double)n["mei_p1"], (double)n["mei_p2"]};
      cv::Mat R;
      n["R_phys_virt"] >> R;                     // 虚拟系 -> 物理鱼眼系

      cv::Mat mx(virt_h_, virt_w_, CV_32FC1), my(virt_h_, virt_w_, CV_32FC1);
      const int sw = (int)((double)n["src_width"] * scale);
      const int sh = src_h_[slot];
      size_t outside = 0;

      for (int v = 0; v < virt_h_; ++v) {
        for (int u = 0; u < virt_w_; ++u) {
          // 虚拟针孔反投影（零畸变，所以就是这一行，不会有 NaN）
          const cv::Vec3d d{(u - vcx) / vfx, (v - vcy) / vfy, 1.0};
          const cv::Vec3d p{R.at<double>(0, 0) * d[0] + R.at<double>(0, 1) * d[1] +
                                R.at<double>(0, 2) * d[2],
                            R.at<double>(1, 0) * d[0] + R.at<double>(1, 1) * d[1] +
                                R.at<double>(1, 2) * d[2],
                            R.at<double>(2, 0) * d[0] + R.at<double>(2, 1) * d[1] +
                                R.at<double>(2, 2) * d[2]};
          const cv::Point2d s = mei.project(p);

          if (s.x < 0 || s.x >= sw || s.y < 0 || s.y >= sh) {
            // 落到成像圆外：置 -1，remap 会填黑边。VINS 在黑边上提不到特征，
            // 不致命。gen_seeker_config.py 的覆盖检查应当报 100%，若这里
            // 计数不为 0 说明标定或选的双目对有问题。
            mx.at<float>(v, u) = -1.0f;
            my.at<float>(v, u) = -1.0f;
            ++outside;
          } else {
            mx.at<float>(v, u) = (float)s.x;
            my.at<float>(v, u) = (float)s.y;
          }
        }
      }
      // 转定点格式：cv::remap 用 CV_16SC2 比 CV_32FC1 快一截，且省一半内存带宽
      cv::convertMaps(mx, my, map1_[slot], map2_[slot], CV_16SC2);

      if (outside) {
        RCLCPP_WARN(get_logger(), "cam%d 有 %zu/%d 个像素落在鱼眼成像圆外",
                    slot, outside, virt_w_ * virt_h_);
      }
      RCLCPP_INFO(get_logger(), "cam%d 重映射表就绪: 鱼眼%d (%dx%d) -> 针孔 %dx%d",
                  slot, src_index_[slot], sw, sh, virt_w_, virt_h_);
    }
    return true;
  }

  void onCompressed(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    // cv::imdecode 需要非 const 输入，这里包一层 Mat 头，不拷贝数据
    cv::Mat raw(1, static_cast<int>(msg->data.size()), CV_8UC1,
                const_cast<uint8_t *>(msg->data.data()));
    cv::Mat frame = cv::imdecode(raw, decode_flag_);
    if (frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "JPEG 解码失败");
      return;
    }

    const int h = frame.rows / kNumCams;
    if (h != src_h_[0] || h != src_h_[1]) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "分片高度 %d 与标定的 %d 不符，重映射表作废",
                            h, src_h_[0]);
      return;
    }

    // 先把两路都去畸变出来 —— 曝光对齐需要同时看到两幅图，没法逐路处理
    cv::Mat img[2];
    for (int s = 0; s < 2; ++s) {
      cv::Mat roi = frame(cv::Rect(0, src_index_[s] * h, frame.cols, h));
      cv::remap(roi, img[s], map1_[s], map2_[s], cv::INTER_LINEAR,
                cv::BORDER_CONSTANT, cv::Scalar(0));
    }

    // 曝光对齐（见文件头说明）。顺序不能反：先用原始分布做直方图匹配，
    // 再各自 CLAHE；反过来 CLAHE 已经改了分布，匹配就失去意义了。
    if (hist_match_) {
      cv::Mat m;
      matchHistogram(img[1], img[0], m);       // 让 cam1 去适配 cam0
      img[1] = m;
    }
    if (clahe_) {
      clahe_impl_->apply(img[0], img[0]);
      clahe_impl_->apply(img[1], img[1]);
    }

    // 两路共用同一时间戳：它们本来就来自同一张 JPEG，是硬件同步的同一时刻。
    // VINS 的双目匹配要求左右目时间戳一致（容差 3ms），这里天然满足。
    publishImage(pub0_, img[0], msg->header.stamp, "cam0");
    publishImage(pub1_, img[1], msg->header.stamp, "cam1");

    if (++count_ % 200 == 1) {
      RCLCPP_INFO(get_logger(), "已转发 %zu 帧", count_);
    }
  }

  void publishImage(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & pub,
                    const cv::Mat & out,
                    const builtin_interfaces::msg::Time & stamp,
                    const std::string & frame_id)
  {
    auto img = std::make_unique<sensor_msgs::msg::Image>();
    img->header.stamp = stamp;
    img->header.frame_id = frame_id;
    img->height = static_cast<uint32_t>(out.rows);
    img->width = static_cast<uint32_t>(out.cols);
    img->encoding = "mono8";
    img->is_bigendian = 0;
    img->step = static_cast<uint32_t>(out.cols);
    // remap / LUT / CLAHE 的输出都是新分配的连续 Mat，一次 memcpy 即可
    img->data.resize(static_cast<size_t>(out.rows) * out.cols);
    std::memcpy(img->data.data(), out.data, img->data.size());

    pub->publish(std::move(img));     // 移动语义，避免再拷一次
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub0_, pub1_;
  cv::Mat map1_[2], map2_[2];
  int src_index_[2]{0, 1};
  int src_h_[2]{0, 0};
  int virt_w_{0}, virt_h_{0};
  int decode_flag_{cv::IMREAD_GRAYSCALE};
  bool hist_match_{true}, clahe_{true};
  cv::Ptr<cv::CLAHE> clahe_impl_;
  std::string input_topic_;
  size_t count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SeekerSplitNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("seeker_split"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
