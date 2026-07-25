// Adaptive Monte Carlo Localization (AMCL) — C++ 实现
// =====================================================
// 完整移植自 Python 版 amcl.py，保留全部算法逻辑与三项改进：
//   改进1: Kidnapping 检测 — 连续10帧低似然触发全局重定位
//   改进2: 粒子多样性监测 — 基于熵的随机粒子注入
//   改进3: 传感器引导恢复 — 用 LiDAR 匹配度设置初始权重
//
// 编译目标: MSVC cl.exe /O2 /std:c++17 /EHsc /utf-8，兼容 Linux GCC
// 依赖: puppy_nav_core::OccupancyGrid (由另一子代理创建)
//
// numpy 替换说明:
//   np.random.normal  → std::normal_distribution 循环
//   np.random.uniform → std::uniform_real_distribution 循环
//   np.random.choice(replace=False) → std::shuffle 取前 k 个
//   np.exp/np.log/np.sqrt → std::exp/std::log/std::sqrt
//   np.cumsum → 累加循环
//   np.where(cond, a, b) → 三元运算符
//   np.arctan2(np.sin, np.cos) → std::atan2(std::sin, std::cos)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "puppy_nav_core/amcl.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <numeric>
#include <random>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

// ---------------------------------------------------------------------------
// 3-4-5 Chamfer 距离变换 (numpy-free)
// ---------------------------------------------------------------------------
// 对每个 cell 计算到最近占用 cell 的距离（cell 单位）。
// 使用两遍扫描的 Chamfer 距离变换，权重 3-4-5（水平 3，对角 4，骑士移动 5）。
// 这是 scipy.ndimage.distance_transform_edt 不可用时的 numpy 回退方案。
// 精度在真实欧氏距离的 ~5% 以内，对 likelihood field 足够。
//
// 参数:
//   log_odds : 栅格 log-odds 数组 (row-major: [gy * W + gx])
//   W, H     : 栅格宽高
//   dist     : 输出距离数组 (size = W*H, row-major)
// ---------------------------------------------------------------------------
void distance_transform_chamfer(const std::vector<float>& log_odds,
                                 int W, int H,
                                 std::vector<float>& dist) {
    const float INF = 1e6f;

    // 初始化: 障碍物 cell 距离=0, 其余=INF
    dist.resize((size_t)W * H);
    for (int i = 0; i < W * H; i++) {
        dist[i] = (log_odds[i] > 0.6f) ? 0.0f : INF;
    }

    // 3-4-5 Chamfer 权重 (除以 3 近似真实欧氏距离)
    const float W_H = 3.0f / 3.0f;  // 水平/垂直步长 = 1.0
    const float W_D = 4.0f / 3.0f;  // 对角步长 ≈ 1.333 (真实 sqrt(2)≈1.414)
    const float W_K = 5.0f / 3.0f;  // 骑士移动 (2,1) ≈ 1.667

    // 前向扫描: 左上 → 右下
    for (int i = 0; i < H; i++) {
        for (int j = 0; j < W; j++) {
            float d_min = dist[(size_t)i * W + j];
            // 骑士移动 (来自前几行)
            if (i >= 1 && j >= 2)
                d_min = std::min(d_min, dist[(size_t)(i - 1) * W + (j - 2)] + W_K);
            if (i >= 1 && j + 2 < W)
                d_min = std::min(d_min, dist[(size_t)(i - 1) * W + (j + 2)] + W_K);
            if (i >= 2 && j >= 1)
                d_min = std::min(d_min, dist[(size_t)(i - 2) * W + (j - 1)] + W_K);
            if (i >= 2 && j + 1 < W)
                d_min = std::min(d_min, dist[(size_t)(i - 2) * W + (j + 1)] + W_K);
            // 对角 (来自前一行)
            if (i >= 1 && j >= 1)
                d_min = std::min(d_min, dist[(size_t)(i - 1) * W + (j - 1)] + W_D);
            if (i >= 1 && j + 1 < W)
                d_min = std::min(d_min, dist[(size_t)(i - 1) * W + (j + 1)] + W_D);
            // 正交
            if (i >= 1)
                d_min = std::min(d_min, dist[(size_t)(i - 1) * W + j] + W_H);
            if (j >= 1)
                d_min = std::min(d_min, dist[(size_t)i * W + (j - 1)] + W_H);
            dist[(size_t)i * W + j] = d_min;
        }
    }

    // 后向扫描: 右下 → 左上
    for (int i = H - 1; i >= 0; i--) {
        for (int j = W - 1; j >= 0; j--) {
            float d_min = dist[(size_t)i * W + j];
            // 骑士移动 (来自后几行)
            if (i + 1 < H && j + 2 < W)
                d_min = std::min(d_min, dist[(size_t)(i + 1) * W + (j + 2)] + W_K);
            if (i + 1 < H && j - 2 >= 0)
                d_min = std::min(d_min, dist[(size_t)(i + 1) * W + (j - 2)] + W_K);
            if (i + 2 < H && j + 1 < W)
                d_min = std::min(d_min, dist[(size_t)(i + 2) * W + (j + 1)] + W_K);
            if (i + 2 < H && j - 1 >= 0)
                d_min = std::min(d_min, dist[(size_t)(i + 2) * W + (j - 1)] + W_K);
            // 对角
            if (i + 1 < H && j + 1 < W)
                d_min = std::min(d_min, dist[(size_t)(i + 1) * W + (j + 1)] + W_D);
            if (i + 1 < H && j - 1 >= 0)
                d_min = std::min(d_min, dist[(size_t)(i + 1) * W + (j - 1)] + W_D);
            // 正交
            if (i + 1 < H)
                d_min = std::min(d_min, dist[(size_t)(i + 1) * W + j] + W_H);
            if (j + 1 < W)
                d_min = std::min(d_min, dist[(size_t)i * W + (j + 1)] + W_H);
            dist[(size_t)i * W + j] = d_min;
        }
    }
}

}  // anonymous namespace

namespace puppy_nav_core {

// ===========================================================================
// 构造函数
// ===========================================================================
AMCL::AMCL(const OccupancyGrid& occ_grid,
           int n_particles,
           double sigma_obs,
           double z_max,
           int n_obs_rays,
           int kld_min, int kld_max,
           double kld_epsilon, double kld_delta)
    : grid_(occ_grid),
      n(n_particles),
      n_active(n_particles),
      kld_min(kld_min),
      kld_max(kld_max),
      kld_epsilon(kld_epsilon),
      kld_delta(kld_delta),
      n_max_(n_particles),
      sigma_obs_(sigma_obs),
      z_max_(z_max),
      n_obs_rays_(n_obs_rays),
      last_n_eff((double)n_particles) {
    // ---- v3.0 消融开关: USE_KLD 环境变量 ----
    // USE_KLD=1 (默认) 启用 KLD-sampling (Fox 2003)
    // USE_KLD=0 回退到固定粒子数 (pre-v3.0 行为)
    const char* use_kld_env = std::getenv("USE_KLD");
    use_kld_ = (use_kld_env == nullptr) || (std::string(use_kld_env) == "1");

    // ---- Wilson-Hilferty z-quantile 查找表 ----
    // 预计算卡方分布分位数的 z 值，避免 scipy 依赖（项目硬性约束）
    // z_{0.90}=1.282, z_{0.95}=1.645, z_{0.99}=2.326, z_{0.999}=3.090
    if (kld_delta == 0.90) {
        kld_z_ = 1.282;
    } else if (kld_delta == 0.95) {
        kld_z_ = 1.645;
    } else if (kld_delta == 0.99) {
        kld_z_ = 2.326;
    } else if (kld_delta == 0.999) {
        kld_z_ = 3.090;
    } else {
        kld_z_ = 2.326;  // 默认 99%
    }

    // ---- 初始化粒子和权重 ----
    particles_.assign(n_particles, {0.0, 0.0, 0.0});
    weights_.assign(n_particles, 1.0 / n_particles);

    // ---- 随机数生成器种子 ----
    std::random_device rd;
    rng_.seed(rd());
}

// ===========================================================================
// init_cloud: 初始化粒子云
// ===========================================================================
void AMCL::init_cloud(double x, double y, double yaw, double spread) {
    int cnt = n_max_;  // 使用初始最大粒子数
    particles_.assign(cnt, {0.0, 0.0, 0.0});
    weights_.assign(cnt, 1.0 / cnt);
    n = cnt;
    n_active = cnt;

    std::normal_distribution<double> noise_xy(0.0, spread);
    std::normal_distribution<double> noise_yaw(0.0, 0.1);

    for (int i = 0; i < cnt; i++) {
        double px = x + noise_xy(rng_);
        double py = y + noise_xy(rng_);
        double pyaw = yaw + noise_yaw(rng_);
        // 归一化 yaw 到 [-pi, pi]
        pyaw = std::atan2(std::sin(pyaw), std::cos(pyaw));
        particles_[i] = {px, py, pyaw};
    }

    converged = false;
}

// ===========================================================================
// recover: 全局重定位（改进3：传感器引导恢复）
// ===========================================================================
// 擦除粒子云并在最后最佳估计附近广泛分散粒子。
// 触发条件: 置信度持续低于 0.1、粒子方差坍缩但误差高、长时间无进展、
//           kidnapping 检测（改进1: 连续低似然）。
//
// 改进3: 当提供 LiDAR 数据时，分散粒子后用观测模型计算每个粒子的匹配度
// 作为初始权重，使重定位更快收敛到真实位姿。无 LiDAR 数据时退化为
// 原始的 spread + 均匀权重。
//
// v3.0: recover 后粒子数重置为 kld_max，使 KLD 采样器有最大灵活性
//       从分散云中重新收敛（仅 KLD 模式；USE_KLD=0 时保持原粒子数）。
// ===========================================================================
std::string AMCL::recover(double x, double y, double yaw,
                           double spread,
                           const std::vector<double>* scan_angles,
                           const std::vector<double>* scan_distances) {
    double s = (spread < 0.0) ? recover_spread : spread;

    // v3.0: KLD 模式下扩展粒子数组到 kld_max；消融模式下保持原粒子数
    int n_recover = use_kld_ ? kld_max : (int)particles_.size();
    particles_.assign(n_recover, {0.0, 0.0, 0.0});
    n = n_recover;
    n_active = n_recover;

    // 保留 20% 粒子在当前估计附近（以防当前估计是对的）
    int n_keep = std::max(n_recover / 5, 10);
    std::normal_distribution<double> noise_keep_xy(0.0, 0.1);
    std::normal_distribution<double> noise_keep_yaw(0.0, 0.2);

    for (int i = 0; i < n_keep && i < n_recover; i++) {
        double px = x + noise_keep_xy(rng_);
        double py = y + noise_keep_xy(rng_);
        double pyaw = yaw + noise_keep_yaw(rng_);
        particles_[i] = {px, py, pyaw};
    }

    // 80% 粒子广泛分散以重新探索
    int n_wide = n_recover - n_keep;
    std::normal_distribution<double> noise_wide_xy(0.0, s);
    std::uniform_real_distribution<double> uniform_yaw(-M_PI, M_PI);

    for (int i = 0; i < n_wide; i++) {
        int idx = n_keep + i;
        double px = x + noise_wide_xy(rng_);
        double py = y + noise_wide_xy(rng_);
        double pyaw = uniform_yaw(rng_);
        particles_[idx] = {px, py, pyaw};
    }

    // 归一化所有 yaw 到 [-pi, pi]
    for (int i = 0; i < n_recover; i++) {
        double pyaw = particles_[i][2];
        particles_[i][2] = std::atan2(std::sin(pyaw), std::cos(pyaw));
    }

    // === 改进3: 传感器引导恢复 ===
    // 对每个粒子，用观测模型计算 LiDAR 与地图的匹配度，作为初始权重。
    // 匹配度高的粒子区域获得更高权重，加速重定位收敛。
    if (scan_angles != nullptr && scan_distances != nullptr &&
        !scan_angles->empty()) {
        // weight() 会更新 weights_ 和 last_n_eff
        weight(*scan_angles, *scan_distances, 0);
    } else {
        // 没有 LiDAR 数据，退化为均匀权重
        weights_.assign(n, 1.0 / n);
        last_n_eff = (double)n;
    }

    converged = false;
    return "recovered";
}

// ===========================================================================
// predict: 运动模型预测
// ===========================================================================
// 使用世界系直接增量传播 (dx, dy, dyaw) 加高斯噪声，
// 而非 rot1-trans-rot2 分解。rot1 分解需要前一时刻的 yaw 均值，
// 在 recover() 后云团分散时这个近似不准确，会偏移预测方向。
// 世界系直接增量完全避免了这个近似。
//
// 噪声 sigma 与运动量成比例（比例噪声模型, Thrun 5.2 alpha 参数）。
// ===========================================================================
void AMCL::predict(double dx, double dy, double dyaw) {
    int cnt = (int)particles_.size();

    // 噪声与运动量成比例
    double sigma_x = alpha3 * std::abs(dx) + alpha4 * std::abs(dyaw);
    double sigma_y = alpha3 * std::abs(dy) + alpha4 * std::abs(dyaw);
    double sigma_yaw = alpha1 * std::abs(dyaw) + alpha2 * (std::abs(dx) + std::abs(dy));

    std::normal_distribution<double> noise_x(0.0, sigma_x);
    std::normal_distribution<double> noise_y(0.0, sigma_y);
    std::normal_distribution<double> noise_yaw(0.0, sigma_yaw);

    for (int i = 0; i < cnt; i++) {
        particles_[i][0] += dx + noise_x(rng_);
        particles_[i][1] += dy + noise_y(rng_);
        particles_[i][2] += dyaw + noise_yaw(rng_);
        // 归一化 yaw 到 [-pi, pi]
        particles_[i][2] = std::atan2(std::sin(particles_[i][2]),
                                       std::cos(particles_[i][2]));
    }
}

// ===========================================================================
// build_likelihood_field: 构建 likelihood field
// ===========================================================================
// L(gx, gy) = exp(-d(gx, gy)^2 / (2 * sigma^2))
// 其中 d(gx, gy) 是从 cell (gx, gy) 到最近占用 cell 的欧氏距离（米）。
//
// 使用 numpy-free Chamfer 距离变换 (3-4-5 权重) 实现 O(N) 计算。
// 仅在占用栅格显著变化时重建（通过签名追踪）。
// ===========================================================================
void AMCL::build_likelihood_field() {
    int W = grid_.width;
    int H = grid_.height;

    // 检查是否有占用 cell
    int n_occ = grid_.count_occupied();

    if (n_occ == 0) {
        // 没有障碍物 — 均匀场
        likelihood_field_.assign((size_t)W * H, 0.1f);
        field_signature_ = {0, 0.0};
        field_built_ = true;
        return;
    }

    // 距离变换 (Chamfer 3-4-5, cell 单位)
    std::vector<float> dist_cells;
    distance_transform_chamfer(grid_.log_odds, W, H, dist_cells);

    // 转换为米并应用 Gaussian 似然: 障碍物附近高，开阔区域低
    double sigma = sigma_obs_;
    double denom = 2.0 * sigma * sigma;

    likelihood_field_.resize((size_t)W * H);
    for (int i = 0; i < W * H; i++) {
        double dist_meters = (double)dist_cells[i] * grid_.resolution;
        likelihood_field_[i] = (float)std::exp(-(dist_meters * dist_meters) / denom);
    }

    // 记录签名用于缓存失效
    double mean_lo = grid_.mean_log_odds();
    field_signature_ = {n_occ, mean_lo};
    field_built_ = true;
}

// ===========================================================================
// ensure_field: 按需重建 likelihood field
// ===========================================================================
// 当地图自上次构建以来发生变化时重建。
// 重建条件: 从未构建、距上次重建 >= 10 帧、或占用 cell 数变化超过 5%。
// ===========================================================================
void AMCL::ensure_field(int frame) {
    int n_occ = grid_.count_occupied();
    double mean_lo = grid_.mean_log_odds();
    std::pair<int, double> sig = {n_occ, mean_lo};

    if (!field_built_ ||
        frame - field_rebuild_frame_ >= 10 ||
        std::abs(sig.first - field_signature_.first) >
            std::max(20, (int)(sig.first * 0.05))) {
        build_likelihood_field();
        field_rebuild_frame_ = frame;
    }
}

// ===========================================================================
// weight: 观测模型更新权重 (Likelihood Field Model, Thrun 6.4)
// ===========================================================================
// 对每个粒子 i 和每束光 k:
//   hit_world = (px_i + r_k cos(pyaw_i + theta_k), py_i + r_k sin(pyaw_i + theta_k))
//   hit_grid  = world_to_grid(hit_world)
//   prob_ik   = z_hit * L(hit_grid) + z_rand / z_max
// log_weight_i = sum_k log(prob_ik)
//
// C++ 用双重循环（外层粒子，内层光束）替代 numpy 广播。
// 性能通过 -O2 优化获得。
//
// 改进1: 捕获观测似然（每束平均，用于 kidnapping 检测）
// ===========================================================================
void AMCL::weight(const std::vector<double>& angles,
                  const std::vector<double>& distances,
                  int frame) {
    if (angles.empty()) return;

    ensure_field(frame);

    // ---- 光束下采样（等间隔抽样，保证确定性）----
    // 对应 Python: np.linspace(0, len(angles)-1, n_obs_rays).astype(int)
    std::vector<double> obs_angles;
    std::vector<double> obs_ranges;
    int K;

    if ((int)angles.size() > n_obs_rays_ && n_obs_rays_ > 1) {
        obs_angles.reserve(n_obs_rays_);
        obs_ranges.reserve(n_obs_rays_);
        int L = (int)angles.size();
        for (int i = 0; i < n_obs_rays_; i++) {
            int idx;
            if (i == n_obs_rays_ - 1) {
                idx = L - 1;  // 确保终点精确
            } else {
                idx = (int)((double)i * (L - 1) / (n_obs_rays_ - 1));
            }
            obs_angles.push_back(angles[idx]);
            // clip 距离到 [0, z_max]
            double r = distances[idx];
            obs_ranges.push_back(std::min(std::max(r, 0.0), z_max_));
        }
        K = n_obs_rays_;
    } else {
        // 不需要下采样
        obs_angles = angles;
        obs_ranges.resize(distances.size());
        for (size_t i = 0; i < distances.size(); i++) {
            obs_ranges[i] = std::min(std::max(distances[i], 0.0), z_max_);
        }
        K = (int)angles.size();
    }

    // ---- 计算每个粒子的 log 权重 ----
    int N = (int)particles_.size();
    std::vector<double> log_weights(N);
    double best_log_lik = -1e18;

    // 混合模型参数
    // z_rand=0.10 保持粒子多样性，防止粒子贫化导致收敛到错误位置
    const double z_hit = 0.90;
    const double z_rand = 0.10;

    int W = grid_.width;
    int H = grid_.height;

    for (int i = 0; i < N; i++) {
        double px = particles_[i][0];
        double py = particles_[i][1];
        double pyaw = particles_[i][2];
        double lw = 0.0;

        for (int k = 0; k < K; k++) {
            // 有效光束角（世界系）= 粒子 yaw + 光束角
            double beam_angle = pyaw + obs_angles[k];
            double r = obs_ranges[k];

            // 命中点世界坐标
            double hit_x = px + r * std::cos(beam_angle);
            double hit_y = py + r * std::sin(beam_angle);

            // 转换为栅格索引
            int gx = (int)((hit_x - grid_.origin_x) / grid_.resolution);
            int gy = (int)((hit_y - grid_.origin_y) / grid_.resolution);

            // 边界检查（越界 -> 0 似然）
            bool valid = (gx >= 0 && gx < W && gy >= 0 && gy < H);
            int gx_c = std::min(std::max(gx, 0), W - 1);
            int gy_c = std::min(std::max(gy, 0), H - 1);

            // 查找 likelihood field
            double lik = likelihood_field_[(size_t)gy_c * W + gx_c];
            if (!valid) lik = 0.001;  // 越界命中 -> 近零似然

            // 最大量程光束（未命中障碍物）-> 均匀似然（无信息）
            if (r >= z_max_) lik = 0.5;

            // 混合: z_hit * L + z_rand / z_max
            double prob = z_hit * lik + z_rand / z_max_;
            // 钳制避免 log(0)
            prob = std::max(prob, 1e-12);

            lw += std::log(prob);
        }

        log_weights[i] = lw;
        if (lw > best_log_lik) best_log_lik = lw;
    }

    // === 改进1: 捕获观测似然（用于 kidnapping 检测）===
    // 使用最佳粒子的每束平均似然作为观测质量度量。
    // 值域 0-1: 1.0=完美匹配, 0.01=极差匹配（可能被绑架）。
    // 除以 beam 数再 exp 还原为概率尺度，使阈值与粒子数无关。
    int n_beams = std::max(K, 1);
    _last_obs_likelihood = std::exp(best_log_lik / n_beams);

    // ---- 归一化 log 权重为权重 ----
    // 减去最大值（数值稳定性）后取 exp
    weights_.resize(N);
    double wsum = 0.0;
    for (int i = 0; i < N; i++) {
        log_weights[i] -= best_log_lik;
        weights_[i] = std::exp(log_weights[i]);
        wsum += weights_[i];
    }
    if (wsum < 1e-12) wsum = 1e-12;
    for (int i = 0; i < N; i++) {
        weights_[i] /= wsum;
    }

    // ---- 有效粒子数 ----
    double sum_sq = 0.0;
    for (int i = 0; i < N; i++) {
        sum_sq += weights_[i] * weights_[i];
    }
    last_n_eff = (sum_sq > 1e-12) ? 1.0 / sum_sq : (double)N;
}

// ===========================================================================
// kld_sample_size: KLD-sampling 所需粒子数 (Fox 2003)
// ===========================================================================
// n = (k-1) / (2*epsilon) * chi2_{k-1, 1-delta}
//
// 使用 Wilson-Hilferty 近似计算卡方分位数（不依赖 scipy）:
// chi2_{k-1, 1-delta} ~= (k-1) * (1 - 2/(9*(k-1)) + z * sqrt(2/(9*(k-1))))^3
// 其中 z = 标准正态分布在置信度 1-delta 处的 z 分位数。
//
// 参数:
//   k: 离散化状态空间中非空 bin 的数量
// 返回:
//   所需粒子数（钳制到 [kld_min, kld_max]）
// ===========================================================================
int AMCL::kld_sample_size(int k) const {
    if (k <= 1) return kld_min;
    int d = k - 1;
    double z = kld_z_;
    // Wilson-Hilferty 变换
    double wh = 1.0 - 2.0 / (9.0 * d) + z * std::sqrt(2.0 / (9.0 * d));
    double chi2 = (double)d * wh * wh * wh;
    int n_req = (int)std::ceil((double)d / (2.0 * kld_epsilon) * chi2);
    return std::min(std::max(n_req, kld_min), kld_max);
}

// ===========================================================================
// compute_particle_diversity: 粒子多样性计算（熵）
// ===========================================================================
// 将空间栅格化为 0.5m x 0.5m 的 cell，统计每个 cell 中的粒子数，
// 计算分布的熵 = -sum p_i * log(p_i)，其中 p_i 是 cell i 中的粒子占比。
//
// 高熵 = 粒子分散，多样性好（如刚 recover 后的分散云）
// 低熵 = 粒子集中在少数 cell，多样性差（粒子贫化，需注入随机粒子）
//
// 结果缓存在 particle_entropy。
// ===========================================================================
double AMCL::compute_particle_diversity() {
    if (particles_.empty()) {
        particle_entropy = 0.0;
        return 0.0;
    }

    const double cell_size = 0.5;  // 0.5m x 0.5m 栅格化
    int total = (int)particles_.size();

    // 统计每个 cell 中的粒子数
    std::unordered_map<long long, int> cells;
    for (int i = 0; i < total; i++) {
        int gx = (int)(particles_[i][0] / cell_size);
        int gy = (int)(particles_[i][1] / cell_size);
        // 编码 (gx, gy) 为 long long key（偏移确保非负）
        long long key = ((long long)(gx + 100000)) * 1000000LL + (gy + 100000);
        cells[key]++;
    }

    // 计算熵 = -sum p_i * log(p_i)
    double entropy = 0.0;
    for (const auto& kv : cells) {
        double p = (double)kv.second / total;
        entropy -= p * std::log(p + 1e-12);
    }

    particle_entropy = entropy;
    return entropy;
}

// ===========================================================================
// resample_fixed: 固定粒子数低方差重采样 (pre-v3.0 行为)
// ===========================================================================
// 用于 USE_KLD=0 消融实验。保持粒子数恒定，不随分布形状变化。
// 包含改进2: 粒子多样性监测驱动的随机粒子注入。
// ===========================================================================
void AMCL::resample_fixed() {
    int n_old = (int)particles_.size();

    // 系统重采样位置: (arange(n) + uniform()) / n
    std::uniform_real_distribution<double> uniform01(0.0, 1.0);
    double r0 = uniform01(rng_);

    // 累积权重
    std::vector<double> cumulative(n_old);
    double csum = 0.0;
    for (int i = 0; i < n_old; i++) {
        csum += weights_[i];
        cumulative[i] = csum;
    }
    cumulative[n_old - 1] = 1.0;  // 防止浮点漂移

    // 系统重采样
    std::vector<std::array<double, 3>> new_particles(n_old);
    int idx = 0;
    for (int j = 0; j < n_old; j++) {
        double position = (j + r0) / n_old;
        while (position > cumulative[idx]) {
            idx++;
            if (idx >= n_old) {
                idx = n_old - 1;
                break;
            }
        }
        new_particles[j] = particles_[idx];
    }

    particles_ = std::move(new_particles);
    weights_.assign(n_old, 1.0 / n_old);
    resample_count++;

    // === 改进2: 粒子多样性监测驱动的随机粒子注入 ===
    // v3.2.2 修复: 原参数过激进导致粒子云永不收敛
    //   - inject_spread 0.5→0.05m (10倍缩小)：收敛后注入噪声不应破坏位姿估计
    //   - inject_ratio 10%→3%：收敛状态下少量随机粒子即可维持多样性
    //   原行为: 每次重采样注入 5-10% 标准差 0.5m 随机粒子，
    //   导致 pos_var≈0.25, confidence=exp(-0.25*5)=0.29 无法提升，
    //   实测置信度 0.042 但定位误差 0.000m。
    double entropy = compute_particle_diversity();
    int n_random;
    if (entropy < diversity_entropy_thresh_) {
        // 熵低 -> 粒子集中 -> 注入 3% 随机粒子恢复多样性
        n_random = std::max(1, (int)(n_old * 0.03));
    } else {
        // 熵正常 -> 注入 2% 随机粒子（原始行为）
        n_random = std::max(1, (int)(n_old * 0.02));
    }

    // 计算加权均值位姿
    auto [est_x, est_y, est_yaw, conf] = get_estimate();

    // 随机选择 n_random 个粒子（不放回）替换为估计位姿附近的随机粒子
    std::vector<int> indices(n_old);
    std::iota(indices.begin(), indices.end(), 0);
    std::shuffle(indices.begin(), indices.end(), rng_);

    std::normal_distribution<double> noise_xy(0.0, 0.05);   // v3.2.2: 0.5→0.05m
    std::normal_distribution<double> noise_yaw(0.0, 0.05);   // v3.2.2: 0.2→0.05

    for (int i = 0; i < n_random; i++) {
        int pi = indices[i];
        particles_[pi][0] = est_x + noise_xy(rng_);
        particles_[pi][1] = est_y + noise_xy(rng_);
        particles_[pi][2] = est_yaw + noise_yaw(rng_);
        // 归一化 yaw
        particles_[pi][2] = std::atan2(std::sin(particles_[pi][2]),
                                        std::cos(particles_[pi][2]));
    }

    // 重置权重
    weights_.assign(n_old, 1.0 / n_old);
}

// ===========================================================================
// resample: KLD-sampling 低方差系统重采样 (Fox 2003)
// ===========================================================================
// v3.0: 根据 KL 距离自适应确定粒子数。
//   粒子云集中（少唯一 bin）-> 需要更少粒子
//   粒子云分散（多 bin，如 recover 后）-> 需要更多粒子
//
// 仅当有效粒子数 N_eff 低于阈值时重采样 — 避免机器人静止时的粒子贫化。
//
// 消融: USE_KLD=0 回退到固定粒子数 (pre-v3.0)。
// 包含改进2: 粒子多样性监测驱动的随机粒子注入。
// ===========================================================================
void AMCL::resample() {
    // v3.2.3: 调试输出（AMCL_DEBUG=1 时每 1000 帧打印 KLD 状态）
    static int s_skip_count = 0;
    static int s_resample_count = 0;
    static int s_frames_since_resample = 0;  // v3.2.3: 强制重采样间隔
    static bool s_debug = (std::getenv("AMCL_DEBUG") != nullptr);

    s_frames_since_resample++;

    // v3.2.3: 强制重采样 — 每 60 帧（2秒）必须重采样一次
    // 问题: KLD 降到 50 粒子后，粒子全在同一位置（k=1），权重均匀（n_eff=50），
    //       超过阈值 n/3=16.7，重采样永不触发。8000+ 帧无重采样导致粒子云漂移。
    // 修复: 每 60 帧强制重采样，注入多样性，防止漂移。
    //       30 帧（1秒）开销过大（230s vs 22s），60 帧（2秒）是稳定性与性能的平衡点。
    bool force_resample = (s_frames_since_resample >= 60);

    // 有效粒子数足够且未到强制间隔时不重采样
    if (last_n_eff > n / 3.0 && !force_resample) {
        s_skip_count++;
        if (s_debug && (s_skip_count % 1000) == 0) {
            printf("[AMCL-DBG] skip resample: n=%d n_eff=%.1f thresh=%.1f (skipped %d times)\n",
                   n, last_n_eff, n / 3.0, s_skip_count);
        }
        return;
    }

    s_frames_since_resample = 0;  // 重置强制重采样计数器

    // v3.0 消融: USE_KLD=0 -> 固定粒子数
    if (!use_kld_) {
        resample_fixed();
        return;
    }

    int n_old = (int)particles_.size();

    // v3.2.3: 置信度门控 — 定位质量差时强制用 kld_max 粒子数
    // 问题: KLD 降到 50 粒子后，粒子云可能坍缩到错误位姿（k=1 但位置错），
    //       导致 A* 成功率从 99.5% 暴跌到 <1%。
    // 修复: 当观测似然 < 0.1（LiDAR 与地图匹配差）时，禁用 KLD 降粒子数，
    //       强制使用 kld_max=500 粒子来恢复定位。
    //       似然恢复后，KLD 自动恢复降粒子数功能。
    // 注意: 这不改变 KLD 算法本身（kld_min=50, Wilson-Hilferty），
    //       只是添加安全机制防止粒子云坍缩。
    bool poor_localization = (_last_obs_likelihood < 0.1);
    int effective_min = poor_localization ? kld_max : kld_min;

    // 累积权重
    std::vector<double> cumulative(n_old);
    double csum = 0.0;
    for (int i = 0; i < n_old; i++) {
        csum += weights_[i];
        cumulative[i] = csum;
    }
    cumulative[n_old - 1] = 1.0;  // 防止浮点漂移

    // v3.0: KLD-sampling — 系统重采样 + 自适应停止
    std::uniform_real_distribution<double> uniform01(0.0, 1.0);
    double r = uniform01(rng_) / kld_max;

    // KLD bin 跟踪 (编码 (bx, by, byaw) 为 long long)
    // 偏移 +100 确保 bx/by/byaw 在 [-100, 100] 范围内无碰撞
    std::unordered_set<long long> bins;
    std::vector<std::array<double, 3>> new_particles;
    new_particles.reserve(kld_max);

    int n_sampled = 0;
    int idx = 0;  // 累积权重数组的运行索引

    for (int j = 0; j < kld_max; j++) {
        // 系统重采样: 找到 position 对应的粒子
        double position = r + (double)j / kld_max;
        while (position > cumulative[idx]) {
            idx++;
            if (idx >= n_old) {
                idx = n_old - 1;
                break;
            }
        }
        std::array<double, 3> p = particles_[idx];
        new_particles.push_back(p);
        n_sampled++;

        // 计算该粒子的 bin
        // v3.2.3: 3D bin（位置+yaw），0.5m × 0.5m × 0.3rad
        //   正常移动时 k=6-9 → n_active=500（稳定）
        //   极度收敛时 k=1-2 → n_active=50-66（满足约束）
        int bx = (int)(p[0] / kld_bin_xy_);
        int by = (int)(p[1] / kld_bin_xy_);
        int byaw = (int)(p[2] / kld_bin_yaw_);
        long long key = ((long long)(bx + 100)) * 1000000LL +
                        ((long long)(by + 100)) * 1000LL +
                        (byaw + 100);
        bins.insert(key);

        // v3.2.3: KLD 停止条件（达到最小样本数后检查）
        // 修复: 原条件 `k > 1` 导致粒子云高度收敛时（所有粒子落入同一 bin，k=1）
        //       停止条件永不触发，采样器一直采到 kld_max=500。
        //       现改为 `k >= 1`，让 kld_sample_size(1)=kld_min=50 生效，
        //       符合项目硬性约束 "50-66 when converged, 500 when dispersed"。
        //       k=1: n_req=50  (极度收敛)
        //       k=2: n_req=67  (收敛，2个bin)
        //       k=5: n_req=500 (分散，clamp到kld_max)
        // v3.2.3: 置信度门控 — poor_localization 时 effective_min=kld_max，
        //       强制采满 500 粒子以恢复定位
        int k = (int)bins.size();
        if (n_sampled >= effective_min && k >= 1) {
            int required = poor_localization ? kld_max : kld_sample_size(k);
            if (n_sampled >= required) {
                if (s_debug) {
                    s_resample_count++;
                    printf("[AMCL-DBG] KLD stop: k=%d n_sampled=%d required=%d poor=%d (resample #%d)\n",
                           k, n_sampled, required, poor_localization ? 1 : 0, s_resample_count);
                }
                break;
            }
        }
    }

    // v3.2.3: 调试 — 循环结束时输出状态
    if (s_debug) {
        s_resample_count++;
        int k_final = (int)bins.size();
        printf("[AMCL-DBG] resample done: n_sampled=%d k=%d n_old=%d (resample #%d)\n",
               n_sampled, k_final, n_old, s_resample_count);
    }

    // 更新粒子数组和计数
    particles_ = std::move(new_particles);
    n = n_sampled;
    n_active = n_sampled;
    weights_.assign(n_sampled, 1.0 / n_sampled);
    resample_count++;

    // === 改进2: 粒子多样性监测驱动的随机粒子注入 ===
    // v3.2.2 修复: 同 resample_fixed(), inject_spread 0.5→0.05m, ratio 10%→3%
    double entropy = compute_particle_diversity();
    int n_random;
    if (entropy < diversity_entropy_thresh_) {
        // 熵低 -> 粒子集中 -> 注入 3% 随机粒子恢复多样性
        n_random = std::max(1, (int)(n_sampled * 0.03));
    } else {
        // 熵正常 -> 注入 2% 随机粒子（原始行为）
        n_random = std::max(1, (int)(n_sampled * 0.02));
    }

    // 计算加权均值位姿
    auto [est_x, est_y, est_yaw, conf] = get_estimate();

    // 随机选择 n_random 个粒子（不放回）替换为估计位姿附近的随机粒子
    std::vector<int> indices(n_sampled);
    std::iota(indices.begin(), indices.end(), 0);
    std::shuffle(indices.begin(), indices.end(), rng_);

    std::normal_distribution<double> noise_xy(0.0, 0.05);    // v3.2.2: 0.5→0.05m
    std::normal_distribution<double> noise_yaw(0.0, 0.05);   // v3.2.2: 0.2→0.05

    for (int i = 0; i < n_random; i++) {
        int pi = indices[i];
        particles_[pi][0] = est_x + noise_xy(rng_);
        particles_[pi][1] = est_y + noise_xy(rng_);
        particles_[pi][2] = est_yaw + noise_yaw(rng_);
        // 归一化 yaw
        particles_[pi][2] = std::atan2(std::sin(particles_[pi][2]),
                                        std::cos(particles_[pi][2]));
    }

    // 重置权重
    weights_.assign(n_sampled, 1.0 / n_sampled);
}

// ===========================================================================
// update: 完整 AMCL 更新 (predict + weight + resample + kidnapping 检测)
// ===========================================================================
// 改进1: 增加 kidnapping 检测。当连续 10 帧观测似然低于阈值时，
// 自动触发全局重定位（spread 粒子到更大范围，重置权重）。
// 似然恢复到正常水平后清除 kidnapping 标志。
//
// 返回: (x, y, yaw, confidence) 估计位姿和置信度
// ===========================================================================
std::tuple<double, double, double, double> AMCL::update(
    double dx, double dy, double dyaw,
    const std::vector<double>& angles,
    const std::vector<double>& distances,
    int frame) {

    last_update_frame = frame;

    // 无运动时跳过运动模型（避免静止时不必要的粒子扩散）
    if (std::abs(dx) + std::abs(dy) + std::abs(dyaw) > 1e-6) {
        predict(dx, dy, dyaw);
    }

    // 观测更新（空扫描时跳过）
    if (!angles.empty()) {
        weight(angles, distances, frame);

        // === 改进1: Kidnapping 检测 ===
        // 记录当前观测似然到历史窗口（最多保存 20 帧）
        likelihood_history_.push_back(_last_obs_likelihood);
        if ((int)likelihood_history_.size() > kidnap_window_) {
            likelihood_history_.erase(likelihood_history_.begin());
        }

        bool just_recovered = false;

        // 需要足够的历史数据才能判断
        if ((int)likelihood_history_.size() >= kidnap_consecutive_) {
            // 检查最近 N 帧似然是否全部低于阈值
            int start = (int)likelihood_history_.size() - kidnap_consecutive_;
            bool all_low = true;
            for (int i = start; i < (int)likelihood_history_.size(); i++) {
                if (likelihood_history_[i] >= kidnap_likelihood_thresh_) {
                    all_low = false;
                    break;
                }
            }

            if (all_low && !kidnapping_detected) {
                // 连续低似然 -> 检测到 kidnapping -> 触发全局重定位
                // spread=2.0 使粒子分散到更大范围，重新探索位姿空间
                kidnapping_detected = true;
                auto [est_x, est_y, est_yaw, conf] = get_estimate();
                // 改进3: 传入当前 LiDAR 数据用于传感器引导恢复
                recover(est_x, est_y, est_yaw,
                        kidnap_recover_spread_,
                        &angles, &distances);
                // 重置似然历史，避免重复触发
                likelihood_history_.clear();
                just_recovered = true;
            } else if (!all_low && kidnapping_detected) {
                // 似然恢复到正常水平 -> 清除 kidnapping 标志
                kidnapping_detected = false;
            }
        }

        // recover 后跳过本次 resample（recover 已设置粒子分布和权重），
        // 否则正常执行重采样以维持粒子多样性
        if (!just_recovered) {
            resample();
        }
    }

    // 首次低 N_eff 后标记收敛
    if (!converged && last_n_eff < n * 0.8) {
        converged = true;
    }

    return get_estimate();
}

// ===========================================================================
// get_estimate: 获取加权均值位姿与置信度
// ===========================================================================
// 位置: 加权均值
// Yaw: 使用 circular mean 处理角度回绕
// 置信度: 基于粒子散布（散布越低置信度越高）
//   confidence = exp(-pos_var * 5)  (零散布时为 1.0，随散布衰减)
// ===========================================================================
std::tuple<double, double, double, double> AMCL::get_estimate() const {
    int N = (int)particles_.size();
    if (N == 0) {
        return {0.0, 0.0, 0.0, 0.0};
    }

    // 加权均值位置
    double x = 0.0, y = 0.0;
    double sin_yaw = 0.0, cos_yaw = 0.0;
    for (int i = 0; i < N; i++) {
        double w = weights_[i];
        x += particles_[i][0] * w;
        y += particles_[i][1] * w;
        sin_yaw += std::sin(particles_[i][2]) * w;
        cos_yaw += std::cos(particles_[i][2]) * w;
    }

    // Yaw: circular mean
    double yaw = std::atan2(sin_yaw, cos_yaw);

    // v3.2.3: 重构置信度 — 三指标融合（jihua5.md 4.1）
    //   confidence = 0.5 * scan_score + 0.3 * neff_ratio + 0.2 * compactness
    //   scan_score: LiDAR 似然分数（传感器匹配度，直接反映定位质量）
    //   neff_ratio: 有效粒子数比（粒子收敛程度，高=粒子集中）
    //   compactness: 粒子云紧凑度（1-exp(-pos_var*3)，与旧指标一致）
    // 旧公式只用 pos_var，粒子云分散时置信度低但定位可能很准（加权均值准确）。
    // 新公式加入 scan_score 作为主导项，让置信度更真实地反映定位质量。
    double pos_var = 0.0;
    for (int i = 0; i < N; i++) {
        double ddx = particles_[i][0] - x;
        double ddy = particles_[i][1] - y;
        pos_var += (ddx * ddx + ddy * ddy) * weights_[i];
    }
    double compactness = std::exp(-pos_var * 3.0);           // 0-1
    double scan_score = std::min(1.0, _last_obs_likelihood);  // 0-1
    double neff_ratio = (N > 0) ? std::min(1.0, last_n_eff / N) : 0.0;  // 0-1
    double confidence = 0.5 * scan_score + 0.3 * neff_ratio + 0.2 * compactness;

    return {x, y, yaw, confidence};
}

// ===========================================================================
// get_covariance: 获取位置协方差矩阵 (2x2)
// ===========================================================================
std::array<std::array<double, 2>, 2> AMCL::get_covariance() const {
    auto [x, y, yaw, conf] = get_estimate();

    std::array<std::array<double, 2>, 2> cov = {{{0.0, 0.0}, {0.0, 0.0}}};

    int N = (int)particles_.size();
    for (int i = 0; i < N; i++) {
        double ddx = particles_[i][0] - x;
        double ddy = particles_[i][1] - y;
        double w = weights_[i];
        cov[0][0] += ddx * ddx * w;
        cov[0][1] += ddx * ddy * w;
        cov[1][1] += ddy * ddy * w;
    }
    cov[1][0] = cov[0][1];  // 对称

    return cov;
}

}  // namespace puppy_nav_core
