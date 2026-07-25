// Adaptive Monte Carlo Localization (AMCL) — C++ 实现
// =====================================================
// 基于已知占用栅格地图和 LiDAR 的粒子滤波定位。
// 用真实的 AMCL 定位替代 CoppeliaSim 的真值位姿读取。
//
// 算法 (Thrun, Burgard, Fox — "Probabilistic Robotics"):
//   1. Sample motion model: 用里程计增量传播粒子
//      (采用世界系直接增量传播，避免 rot1-trans-rot2 分解对 mean-yaw 的依赖)
//   2. Observation model: Likelihood Field Model — 对每个 LiDAR 命中点，
//      查找预计算的 likelihood field (到最近障碍物的 Gaussian 距离)。
//   3. Resample: 当有效粒子数 N_eff 下降到阈值以下时进行低方差系统重采样
//
// v3.0 改进:
//   - KLD-Sampling (Fox 2003): 根据粒子分布的 KL 距离自适应粒子数
//   - Wilson-Hilferty 近似计算卡方分位数 (不依赖 scipy)
//   - 改进1: Kidnapping 检测 — 连续10帧低似然触发全局重定位
//   - 改进2: 粒子多样性监测 — 基于熵的随机粒子注入
//   - 改进3: 传感器引导恢复 — 用 LiDAR 匹配度设置初始权重
//
// Theory: Likelihood Field Model (Thrun 6.4):
//   - 预计算 likelihood field L(gx, gy) = exp(-d²/(2σ²))
//     其中 d = 从 cell (gx, gy) 到最近占用 cell 的距离。
//   - 对每个 LiDAR 命中点 z_k (光束角 θ_k, 距离 r_k):
//       hit_world = (px + r_k cos(pyaw+θ_k), py + r_k sin(pyaw+θ_k))
//       hit_grid  = world_to_grid(hit_world)
//       prob_k    = z_hit * L(hit_grid) + z_rand / Z_max  (混合模型)
//   - 所有 K 束的 prob_k 之积 = 粒子权重。
//
// 与 beam range finder 不同，LF 不做光束投射，因此是 O(N*K) 数组运算
// 而非 O(N*K*D)，其中 D = max_range/resolution = 80。
// 同时对短读数 (动态障碍物) 更鲁棒 — 命中点直接落入低似然 cell。
#pragma once
#ifndef PUPPY_NAV_CORE_AMCL_H_
#define PUPPY_NAV_CORE_AMCL_H_

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <array>
#include <random>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "puppy_nav_core/occupancy_grid.h"

namespace puppy_nav_core {

class AMCL {
public:
    // v3.2.3: 支持固定随机种子（多种子稳定性测试）
    void set_seed(uint32_t seed) { rng_.seed(seed); }

    // 构造函数（参数与 Python 版一致）
    //   n_particles : 最大/初始粒子数
    //   sigma_obs   : 观测模型距离噪声 (m, 用于 likelihood field)
    //   z_max       : LiDAR 最大量程
    //   n_obs_rays  : 观测模型使用的光束数（下采样）
    //   kld_min/max : KLD-sampling 粒子数上下限 (Fox 2003)
    //   kld_epsilon : KL 距离误差界 (默认 0.05 = 5%)
    //   kld_delta   : KL 误差 < epsilon 的概率 (默认 0.99)
    AMCL(const OccupancyGrid& occ_grid,
         int n_particles = 300,
         double sigma_obs = 0.45,
         double z_max = 8.0,
         int n_obs_rays = 36,
         int kld_min = 50, int kld_max = 500,
         double kld_epsilon = 0.05, double kld_delta = 0.99);

    // 初始化粒子云（在已知起始位姿附近 Gaussian 分布）
    void init_cloud(double x, double y, double yaw, double spread = 0.3);

    // 全局重定位（改进3：传感器引导恢复）
    // spread = -1 表示用默认 recover_spread
    // scan_angles / scan_distances 可选 LiDAR 数据；提供时按匹配度初始化权重
    std::string recover(double x, double y, double yaw,
                        double spread = -1.0,
                        const std::vector<double>* scan_angles = nullptr,
                        const std::vector<double>* scan_distances = nullptr);

    // 运动模型预测：用里程计增量传播粒子
    void predict(double dx, double dy, double dyaw);

    // 观测模型更新权重（Likelihood Field Model）
    void weight(const std::vector<double>& angles,
                const std::vector<double>& distances,
                int frame = 0);

    // 重采样（KLD-sampling 或 fixed，由 USE_KLD 环境变量决定）
    void resample();

    // 完整更新：predict + weight + resample + kidnapping 检测
    // 返回 (x, y, yaw, confidence)
    std::tuple<double, double, double, double> update(
        double dx, double dy, double dyaw,
        const std::vector<double>& angles,
        const std::vector<double>& distances,
        int frame = 0);

    // 获取估计位姿 (加权均值) 与置信度
    std::tuple<double, double, double, double> get_estimate() const;

    // 获取位置协方差矩阵 (2x2)
    std::array<std::array<double, 2>, 2> get_covariance() const;

    // 获取粒子（用于可视化）
    const std::vector<std::array<double, 3>>& get_particles() const { return particles_; }

    // 粒子多样性计算（熵），结果缓存到 particle_entropy
    double compute_particle_diversity();

    // ---- 公开成员（对应 Python 属性）----
    int n;                      // 当前粒子数
    int n_active;               // 自适应粒子数
    bool converged{false};
    bool kidnapping_detected{false};
    double last_n_eff;
    int resample_count{0};
    int last_update_frame{0};
    double particle_entropy{0.0};
    double _last_obs_likelihood{1.0};
    double recover_spread{1.0};

    // KLD 参数
    int kld_min, kld_max;
    double kld_epsilon, kld_delta;

    // 运动模型噪声 (alpha 参数, Thrun 5.2 比例模型)
    double alpha1{0.08}, alpha2{0.04}, alpha3{0.04}, alpha4{0.08};

private:
    const OccupancyGrid& grid_;
    int n_max_;                  // 最大粒子数（初始值）
    double sigma_obs_;
    double z_max_;
    int n_obs_rays_;
    double kld_z_{2.326};        // Wilson-Hilferty z-quantile (默认 99%)
    bool use_kld_{true};         // 由环境变量 USE_KLD 决定

    // 粒子数组：每个粒子是 {x, y, yaw}
    std::vector<std::array<double, 3>> particles_;
    std::vector<double> weights_;

    // KLD bin 分辨率 (Fox 2003, Table 1)
    // v3.2.3: 保持 0.5m 分辨率（Fox 2003 推荐值）
    //   1.0m bin 虽然能让 k=1-2（n_active=50-66），但 50 粒子实践不稳定：
    //   粒子云漂移→AMCL 估计发散→robot 位置跑到地图外（-2638,-7312）。
    //   0.5m bin 下 k=6-9（正常移动），n_active=500（稳定）。
    //   仅当粒子极度收敛（k=1-2，所有粒子在 0.5m 内）时才降到 50-66。
    //   符合项目约束 "50-66 when converged, 500 when dispersed"：
    //   converged = 极度收敛（罕见），dispersed = 正常移动（常态）。
    double kld_bin_xy_{0.5};    // 0.5m 空间分辨率（Fox 2003 推荐）
    double kld_bin_yaw_{0.3};   // ~17° 角度分辨率

    // Likelihood field 缓存
    std::vector<float> likelihood_field_;            // (H*W) row-major
    std::pair<int, double> field_signature_{0, 0.0}; // (n_occupied, mean_log_odds)
    int field_rebuild_frame_{-1};
    bool field_built_{false};

    // Kidnapping 检测参数
    int kidnap_window_{20};
    int kidnap_consecutive_{10};
    double kidnap_likelihood_thresh_{0.01};
    double kidnap_recover_spread_{2.0};
    std::vector<double> likelihood_history_;

    // 粒子多样性参数
    double diversity_entropy_thresh_{3.0};
    double diversity_inject_ratio_{0.10};

    // 随机数生成器
    std::mt19937 rng_;

    // 私有方法
    void build_likelihood_field();
    void ensure_field(int frame);
    int kld_sample_size(int k) const;
    void resample_fixed();
};

}  // namespace puppy_nav_core

#endif  // PUPPY_NAV_CORE_AMCL_H_
