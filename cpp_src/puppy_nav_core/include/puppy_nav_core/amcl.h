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

    // v3.2.11: 聚类估计 — 取最大簇加权均值，避免粒子云分裂时均值落墙内
    std::tuple<double, double, double, double> get_cluster_estimate() const;

    // v3.2.13: MHT 多假设跟踪估计 — 累积似然选择最优簇+软切换+限幅平滑
    // 替代 get_cluster_estimate() 的"取当前帧最大簇"，用多帧累积似然更鲁棒
    std::tuple<double, double, double, double> get_mht_estimate() const;

    // 获取位置协方差矩阵 (2x2)
    std::array<std::array<double, 2>, 2> get_covariance() const;

    // ---- R28 诊断接口（只读，不改变滤波器状态）----
    // debug_estimator_info: 同时导出两个估计器的原始值，用于定位系统性偏移来源
    //   返回值        = 簇数量
    //   out_top_share = 最大簇权重占比 [0,1]（<0.8 说明云分裂/被截断）
    //   out_mean_x/y  = 全云加权均值（统计学正确的后验均值）
    //   out_top_x/y   = 最大簇加权均值（未限幅，get_cluster_estimate 的原始值）
    int debug_estimator_info(double& out_top_share,
                             double& out_mean_x, double& out_mean_y,
                             double& out_top_x, double& out_top_y) const;

    // score_pose: 对任意位姿计算观测似然（扫描匹配打分）
    //   用途: 在真值周围做网格搜索，判断似然场的最优点是否落在真值上。
    //   若最优点在真值 → 观测模型无偏，问题在估计提取；否则是扫描/地图不一致。
    double score_pose(double x, double y, double yaw,
                      const std::vector<double>& angles,
                      const std::vector<double>& distances) const {
        return compute_scan_likelihood(x, y, yaw, angles, distances);
    }

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

    // v3.2.11: 聚类滞回状态 — 防止簇切换时估计位姿跳变导致碰撞
    //   原问题: 粒子云在两个位置间振荡时，最大簇每帧切换，估计位姿跳变 2m+
    //   修复: 记录当前簇中心，仅当新簇连续 N 帧权重更高时才切换
    //   mutable: const 方法 get_cluster_estimate() 需要修改这些缓存状态
    mutable double cluster_prev_x_ = 0;
    mutable double cluster_prev_y_ = 0;
    mutable bool cluster_initialized_ = false;
    mutable int cluster_switch_counter_ = 0;
    // P1-2.1: 上一帧置信度，用于自适应限幅（低置信度时减小MAX_STEP）
    mutable double last_confidence_ = 1.0;
    // P1-2.1: 尖峰统计（供 report 输出）
    mutable int spike_count_ = 0;       // 限幅触发次数（位姿跳变>MAX_STEP被裁剪）
    mutable int freeze_count_ = 0;      // 冻结次数（置信度极低，位姿不更新）

    // v3.2.12: 粒子云分裂检测+主动恢复
    //   原问题: 门道穿越时粒子云分裂成两个簇(各~50%权重)，最大簇每帧切换，
    //          导致估计位姿漂移(max_err 1.6m)和 A* 起点落墙(2.7% 失败)
    //   修复: 检测第二簇权重>30%且持续5帧时，把非最大簇粒子重新分配到
    //          最大簇附近(spread=0.5)，加速分裂收敛，不调用 disruptive 的 recover()
    //   v3.2.12a 失败教训: 阈值0.20/3帧太敏感，500粒子在门道穿越时常态分裂，
    //          几乎每帧触发恢复，weight()重计算导致粒子云过度收敛，avg_err 1.2m
    //   v3.2.12b 修复: 阈值0.30/5帧+冷却50帧+不重计算权重，给粒子云自然收敛时间
    int split_consecutive_frames_ = 0;  // 连续检测到分裂的帧数
    int split_cooldown_ = 0;            // 冷却计数器（触发恢复后50帧不检测）
    static const int SPLIT_TRIGGER_FRAMES = 5;      // 连续5帧分裂才触发恢复
    static const int SPLIT_COOLDOWN_FRAMES = 50;    // 触发后50帧冷却（~1.7秒）
    static constexpr double SPLIT_WEIGHT_THRESH = 0.30;  // 第二簇权重>30%判为分裂
    static constexpr double SPLIT_RECOVER_SPREAD = 0.5;  // 分裂恢复 spread（放宽）

    // v3.2.13: MHT 多假设跟踪 (Multi-Hypothesis Tracking)
    //   理论: Reid 1979, 多假设滤波在定位中的应用
    //   核心创新(相比v3.2.11):
    //     1. 多帧累积似然 — 不只看当前帧簇权重，累积多帧观测似然
    //     2. 软切换 — 新假设需连续N帧累积似然>旧假设×1.5才切换
    //     3. 假设管理 — 剪枝低权重+合并相近+年龄门控
    //   优势场景: 门道穿越时粒子云分裂，MHT用累积似然准确判别真实簇
    struct MHTHypothesis {
        double center_x, center_y, yaw;    // 簇中心位姿
        double cumulative_score;           // 累积观测似然（衰减+当前帧）
        int age;                           // 假设年龄（连续匹配帧数）
        int id;                            // 假设唯一ID
        double last_scan_score;            // 最近一帧的scan匹配度
        std::vector<int> particle_indices; // 当前帧粒子索引
    };
    std::vector<MHTHypothesis> mht_hypotheses_;
    int mht_next_id_ = 0;
    int mht_selected_id_ = -1;          // 当前选中假设ID
    int mht_switch_counter_ = 0;        // 软切换计数器
    // 限幅平滑状态（独立于cluster_prev_x_，避免互相干扰）
    mutable double mht_prev_x_ = 0;
    mutable double mht_prev_y_ = 0;
    mutable bool mht_initialized_ = false;

    // MHT 参数
    // v3.2.13b 混合策略: MHT 只在分裂时介入，正常情况回退到 v3.2.11
    //   v3.2.13a 失败: DECAY=0.50 切换太快，2次碰撞
    //   v3.2.13(原) 失败: DECAY=0.90 旧假设惯性大，avg_err 0.920m
    //   修复: 混合策略 — 第二簇权重>20%时激活MHT，否则回退get_cluster_estimate()
    static const int MHT_MIN_AGE = 3;              // 最小年龄才能被选为最优
    static constexpr double MHT_DECAY = 0.70;       // 累积似然衰减因子（中间值）
    static constexpr double MHT_PRUNE_SCORE = 0.05; // 剪枝阈值
    static constexpr double MHT_MERGE_DIST = 0.3;   // 合并阈值(m)
    static constexpr double MHT_ASSOC_DIST = 0.5;   // 数据关联阈值(m)
    static constexpr double MHT_SWITCH_RATIO = 1.5; // 软切换：新假设需比旧假设高1.5倍
    static const int MHT_SWITCH_FRAMES = 3;        // 软切换：需连续3帧满足条件
    static constexpr double MHT_SPLIT_THRESH = 0.20; // 第二簇权重>20%激活MHT

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

    // v3.2.12: 聚类私有方法（供 update() 和 get_cluster_estimate() 复用）
    // 簇结构: indices(粒子索引), center(加权中心), weight_sum(簇总权重)
    struct Cluster {
        std::vector<int> indices;
        std::pair<double, double> center;
        double weight_sum = 0.0;
    };
    std::vector<AMCL::Cluster> cluster_particles() const;

    // v3.2.12: 分裂检测+轻量级恢复
    //   检测第二簇权重>SPLIT_WEIGHT_THRESH 且持续 SPLIT_TRIGGER_FRAMES 帧
    //   触发: 把非最大簇粒子重新分配到最大簇附近(spread=SPLIT_RECOVER_SPREAD)
    //   返回 true 表示触发了恢复
    bool detect_and_recover_split(const std::vector<double>& angles,
                                  const std::vector<double>& distances);

    // v3.2.13: MHT 私有方法
    // 计算给定位置和yaw的LiDAR匹配度（用likelihood field，复用weight()的逻辑）
    double compute_scan_likelihood(double x, double y, double yaw,
                                    const std::vector<double>& angles,
                                    const std::vector<double>& distances) const;

    // MHT 更新：数据关联+累积似然+假设管理（剪枝/合并）
    // 在 update() 中 resample 后调用
    void mht_update(const std::vector<double>& angles,
                    const std::vector<double>& distances);
};

}  // namespace puppy_nav_core

#endif  // PUPPY_NAV_CORE_AMCL_H_
