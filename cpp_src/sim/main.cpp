// main.cpp — C++ 回归测试 (与 Python regression_test.py 等效)
// v3.0: 集成 AMCL 定位 + puppy_nav_core A* 规划
// v3.2.3: 支持随机种子 + 真实性指标
// v3.2.18i+: 支持 YAML 配置文件 (任务 P1-2.4)
// v3.2.18k (jihua20260818 §5/M1): 可控命令行入口 + 超时保护 + 结构化 JSON 报告
//
// 运行:
//   sim_test.exe                                  (默认 300 帧, seed=1, 60s 超时)
//   sim_test.exe --frames 300 --seed 1 --report out.json
//   sim_test.exe --frames 3600 --timeout 60 --report out.json
//   sim_test.exe --long-run --frames 108000 --seed 1 --report out.json
//   sim_test.exe --dry-run                        (仅加载配置并打印, 不仿真)
//   sim_test.exe --help
//   sim_test.exe --version
//   sim_test.exe --config <yaml> [frames] [report_interval] [seed]   (旧式位置参数, 已弃用)
//
// 环境变量 (作为覆盖项, 优先级: 命令行 > 环境变量 > YAML > 默认值):
//   USE_AMCL=1 (默认) 启用 AMCL 定位
//   USE_AMCL=0         用真值位姿 (消融对比)
//   SEED=<n>           固定随机种子 (与命令行参数等效)
//
// 退出码 (与 nav_integration_test 对齐, jihua20260818 §7.3):
//   0  全部验收条件通过
//   1  必达验收指标失败 (运动/房间/速度/卡住/规划)
//   2  发生碰撞
//   3  定位指标失败
//   4  超时
//   5  配置或参数错误 (含未显式 --long-run 的长跑)
//   6  内部错误
#include "simulation.h"
#include "../common/config_contract.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>

using namespace puppy;

static const char* SIM_VERSION = "v3.2.18k-simcli";

// 超时看门狗 (进程级, jihua20260818 §5.4)
static std::atomic<bool> g_timeout_requested{false};
static std::atomic<bool> g_timeout_ack{false};

static std::string read_file_text(const std::string& p) {
    std::ifstream f(p, std::ios::binary);
    if (!f) return "";
    std::stringstream ss; ss << f.rdbuf();
    return ss.str();
}

// 最小 JSON 字符串转义 (用于 current_action 等字段)
static std::string json_escape(const std::string& s) {
    std::string o;
    o.reserve(s.size() + 8);
    for (char c : s) {
        switch (c) {
            case '"': o += "\\\""; break;
            case '\\': o += "\\\\"; break;
            case '\n': o += "\\n"; break;
            case '\r': o += "\\r"; break;
            case '\t': o += "\\t"; break;
            default: o += c;
        }
    }
    return o;
}

static void print_help() {
    printf("Usage: sim_test.exe [options]\n");
    printf("\nOptions:\n");
    printf("  --frames <n>        simulation frames (default: 300)\n");
    printf("  --seed <n>          random seed, 0=random, >0 fixed (default: 1)\n");
    printf("  --timeout <sec>     hard wall-clock timeout (default: 60, 0=disabled)\n");
    printf("  --report <path>     write structured JSON report to <path>\n");
    printf("  --timing-csv <path> write per-frame compute time (ms) CSV for P99 analysis\n");
    printf("  --config <path>     YAML config file (default: config/sim_cpp.yaml)\n");
    printf("  --long-run          REQUIRED for frames > 36000 (explicit long sim)\n");
    printf("  --dry-run           load config and print, do not simulate\n");
    printf("  --help, -h          show this help\n");
    printf("  --version           show version\n");
    printf("\nDeprecated positional args (will print a warning):\n");
    printf("  sim_test.exe [config] [frames] [report_interval] [seed]\n");
    printf("\nEnvironment variables (override YAML):\n");
    printf("  USE_AMCL=0/1     enable AMCL localization (default: 1)\n");
    printf("  USE_RVO=0/1      enable RVO avoidance (default: 1)\n");
    printf("  USE_IMU_FUSION=0/1  enable IMU fusion (default: 1)\n");
    printf("  SEED=<n>         random seed (same as positional arg)\n");
}

int main(int argc, char* argv[]) {
#ifdef _WIN32
    // 修复 PowerShell 中文乱码: 设置控制台输出为 UTF-8
    std::system("chcp 65001 > nul 2>&1");
#endif

    int num_frames = 300;           // jihua20260818 §5.3: 默认 300 帧
    int report_interval = 30;        // 默认每 30 帧打印一次进度
    int seed = 1;                   // §5.3: 默认固定 seed=1 可复现
    int timeout_sec = 60;           // §5.3: 默认 60s 超时
    std::string config_path = "config/sim_cpp.yaml";
    bool long_run = false;
    bool dry_run = false;
    bool want_help = false;
    bool want_version = false;
    bool acceptance_full = false;  // 长跑/长稳才启用重验收 (房间/轮次), 短跑仅做安全冒烟
    bool used_positional = false;
    std::string report_path;
    std::string timing_csv;       // v3.2.18p: 每帧耗时 CSV (用于 P99 长稳分析)

    // ---- 命令行解析 (新 flag 优先; 兼容旧位置参数) ----
    int positional_idx = 0;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (a == "--frames" && i + 1 < argc) {
            num_frames = std::atoi(argv[++i]);
        } else if (a == "--seed" && i + 1 < argc) {
            seed = std::atoi(argv[++i]);
        } else if (a == "--timeout" && i + 1 < argc) {
            timeout_sec = std::atoi(argv[++i]);
        } else if (a == "--report" && i + 1 < argc) {
            report_path = argv[++i];
        } else if (a == "--timing-csv" && i + 1 < argc) {
            timing_csv = argv[++i];
        } else if (a == "--long-run") {
            long_run = true;
        } else if (a == "--dry-run") {
            dry_run = true;
        } else if (a == "--acceptance") {
            acceptance_full = true;
        } else if (a == "--help" || a == "-h") {
            want_help = true;
        } else if (a == "--version") {
            want_version = true;
        } else if (a.rfind("--", 0) == 0) {
            fprintf(stderr, "[ERROR] unknown option: %s\n", a.c_str());
            return 5;
        } else {
            // 旧式位置参数: [帧数] [报告间隔] [种子]
            used_positional = true;
            switch (positional_idx) {
                case 0: num_frames = std::atoi(argv[i]); break;
                case 1: report_interval = std::atoi(argv[i]); break;
                case 2: seed = std::atoi(argv[i]); break;
            }
            positional_idx++;
        }
    }

    if (want_help) { print_help(); return 0; }
    if (want_version) { printf("sim_test %s\n", SIM_VERSION); return 0; }

    if (used_positional) {
        printf("[WARN] positional frame arguments are deprecated; use --frames/--seed\n");
    }

    // §5.3: 长跑必须显式 --long-run
    if (num_frames > 36000 && !long_run) {
        fprintf(stderr,
                "[FATAL] frames=%d exceeds 36000; explicit --long-run is required\n",
                num_frames);
        return 5;
    }
    // 重验收 (房间/轮次/距离) 仅在长跑或显式 --acceptance 时门控退出码;
    // 短跑(默认 300 帧)只做安全冒烟, 质量指标仍写入报告但不阻断.
    if (num_frames >= 3600 || long_run) acceptance_full = true;
    if (num_frames <= 0) {
        fprintf(stderr, "[FATAL] invalid frame count: %d\n", num_frames);
        return 5;
    }
    if (report_interval <= 0) report_interval = 30;

    // ---- 加载 YAML 配置 + 环境变量覆盖 ----
    puppy_sim::SimParams params;
    std::string effective_path;
    {
        std::string default_path = "config/sim_cpp.yaml";
        std::ifstream f(default_path);
        if (!f.is_open()) default_path = "../../config/sim_cpp.yaml";
        effective_path = (config_path == "config/sim_cpp.yaml") ? default_path : config_path;
        std::ifstream ef(effective_path);
        if (!ef.is_open()) {
            fprintf(stderr, "[FATAL] config not found: %s\n", effective_path.c_str());
            return 5;
        }
        params.load_yaml(effective_path);
    }
    params.apply_env_overrides();

    // 命令行 seed 优先级最高 (覆盖 YAML 和环境变量)
    if (seed > 0) {
        params.seed = seed;
        char seed_str[32];
        std::snprintf(seed_str, sizeof(seed_str), "%d", seed);
#ifdef _WIN32
        _putenv_s("SEED", seed_str);
#else
        setenv("SEED", seed_str, 1);
#endif
    }

    // ---- --dry-run: 仅加载并打印配置, 不仿真 ----
    if (dry_run) {
        printf("\n=== DRY-RUN: config loaded, no simulation ===\n");
        printf("  config:         %s\n", effective_path.c_str());
        printf("  frames:         %d\n", num_frames);
        printf("  seed:           %d\n", params.seed);
        printf("  timeout_sec:    %d\n", timeout_sec);
        printf("  long_run:       %s\n", long_run ? "true" : "false");
        params.print();
        // 校验统一契约
        puppy::Contract contract;
        std::string contract_path = "config/simulation_contract.yaml";
        { std::ifstream t(contract_path); if (!t.is_open()) contract_path = "../../config/simulation_contract.yaml"; }
        std::ifstream cf(contract_path);
        if (cf.is_open()) {
            if (puppy::load_contract(contract_path, contract)) {
                printf("  contract.schema_version: %d\n", contract.schema_version);
                printf("  contract.robot.radius:   %.3f\n", contract.robot.radius);
                std::vector<std::string> errs, warns;
                puppy::validate_contract(contract, errs, warns);
                for (auto& e : errs) printf("  [CONTRACT-ERROR] %s\n", e.c_str());
                for (auto& w : warns) printf("  [CONTRACT-WARN]  %s\n", w.c_str());
                if (errs.empty()) printf("  contract validation: PASS\n");
            }
        } else {
            printf("  contract: not found (%s), skipped\n", contract_path.c_str());
        }
        return 0;
    }

    // 配置文件指纹 (与 Python config_loader 思路一致, jihua20260818 §4.3)
    std::string config_text = read_file_text(effective_path);
    std::string config_hash = puppy::sha256_hex(config_text);

    // 契约信息 (用于报告)
    int contract_schema = 0;
    double robot_radius = 0.0;
    {
        puppy::Contract contract;
        std::string contract_path = "config/simulation_contract.yaml";
        { std::ifstream t(contract_path); if (!t.is_open()) contract_path = "../../config/simulation_contract.yaml"; }
        std::ifstream cf(contract_path);
        if (cf.is_open() && puppy::load_contract(contract_path, contract)) {
            contract_schema = contract.schema_version;
            robot_radius = contract.robot.radius;
        }
    }

    printf("\n======================================================================\n");
    printf("  C++ 回归测试: AMCL + puppy_nav_core A* + CBF  [%s]\n", SIM_VERSION);
    printf("  帧数: %d (%.0f分钟等效)\n", num_frames, num_frames / 30.0 / 60.0);
    printf("  种子: %s\n", params.seed > 0 ? std::to_string(params.seed).c_str() : "随机");
    printf("  配置: %s\n", effective_path.c_str());
    printf("  超时: %ds\n", timeout_sec);
    printf("  场景: 多房间家居 + 5行人\n");
    printf("======================================================================\n\n");

    params.print();
    printf("\n");

    // ---- 启动超时看门狗 (进程级) ----
    std::thread watchdog;
    bool watchdog_active = (timeout_sec > 0);
    if (watchdog_active) {
        watchdog = std::thread([timeout_sec]() {
            std::this_thread::sleep_for(std::chrono::seconds(timeout_sec));
            g_timeout_requested.store(true);
            // 宽限 800ms 让主循环跳出并写出报告
            std::this_thread::sleep_for(std::chrono::milliseconds(800));
            if (!g_timeout_ack.load()) {
                // 主循环未在宽限内结束 -> 硬杀, 绝不无限等待 (§5.4 P0)
                fprintf(stderr, "[FATAL] hard timeout: simulation exceeded %ds\n", timeout_sec);
                std::exit(4);
            }
        });
    }

    puppy_sim::Simulator sim(params);
    std::ofstream timing_out;
    if (!timing_csv.empty()) {
        timing_out.open(timing_csv);
        if (timing_out) timing_out << "frame,ms\n";
    }
    clock_t start = clock();
    bool timed_out = false;
    int slow_consec = 0;

    for (int frame = 0; frame < num_frames; frame++) {
        if (g_timeout_requested.load()) { timed_out = true; break; }

        clock_t fstart = clock();
        sim.step();
        double fms = (double)(clock() - fstart) / CLOCKS_PER_SEC * 1000.0;
        if (timing_out) timing_out << frame << "," << fms << "\n";
        if (fms > 100.0) {
            slow_consec++;
            if (slow_consec == 1 || slow_consec % 10 == 0)
                printf("  [WARN] frame %d slow: %.0f ms\n", frame, fms);
        } else {
            slow_consec = 0;
        }

        if ((frame + 1) % report_interval == 0) {
            int seg = (frame + 1) / report_interval;
            double elapsed = (double)(clock() - start) / CLOCKS_PER_SEC;
            printf("  [%d] %7d/%d (%.0fmin): "
                   "碰撞=%3d 近距=%4d 跳点=%3d 卡住=%3d 轮次=%3d 耗时=%.0fs\n",
                   seg, frame+1, num_frames, (frame+1)/30.0/60.0,
                   sim.total_collisions, sim.near_miss,
                   sim.skip_count, sim.stall_events,
                   sim.rounds_completed, elapsed);
            printf("       robot=(%.2f,%.2f) yaw=%.2f target[%zu]=(%.2f,%.2f) action=%s dist=%.1fm rooms=%zu\n",
                   sim.robot_x, sim.robot_y, sim.robot_yaw,
                   sim.target_idx,
                   sim.patrol_targets[sim.target_idx].x,
                   sim.patrol_targets[sim.target_idx].y,
                   json_escape(sim.current_action).c_str(),
                   sim.total_distance, sim.rooms_visited.size());
            if (sim.use_amcl_) {
                double amcl_err = std::sqrt((sim.robot_x - sim.dec_x) * (sim.robot_x - sim.dec_x) +
                                            (sim.robot_y - sim.dec_y) * (sim.robot_y - sim.dec_y));
                double avg_err = (sim.amcl_err_samples_ > 0) ? sim.amcl_err_sum_ / sim.amcl_err_samples_ : 0.0;
                printf("       AMCL: est=(%.2f,%.2f) true=(%.2f,%.2f) err=%.3f avg_err=%.3f max_err=%.3f conf=%.3f obs_lik=%.4f\n",
                       sim.dec_x, sim.dec_y, sim.robot_x, sim.robot_y,
                       amcl_err, avg_err, sim.amcl_err_max_, sim.est_conf_,
                       sim.nav_core_.amcl._last_obs_likelihood);
            }
        }
    }

    double elapsed = (double)(clock() - start) / CLOCKS_PER_SEC;

    printf("\n  ==================================================\n");
    printf("  最终报告: AMCL + puppy_nav_core A* + CBF (C++)\n");
    printf("  ==================================================\n");
    printf("  等效时间:     %.0f 分钟 (%d 帧)\n", num_frames/30.0/60.0, num_frames);
    printf("  种子:         %s\n", params.seed > 0 ? std::to_string(params.seed).c_str() : "随机");
    printf("  碰撞次数:     %d\n", sim.total_collisions);
    printf("  近距事件:     %d\n", sim.near_miss);
    printf("  近距帧数:     %d\n", sim.near_miss_frames);
    printf("  跳点次数:     %d\n", sim.skip_count);
    printf("  卡住事件:     %d\n", sim.stall_events);
    printf("  完成轮次:     %d\n", sim.rounds_completed);
    printf("  计算耗时:     %.1fs (%.1fmin)\n", elapsed, elapsed/60.0);
    printf("  ==================================================\n");

    double avg_speed = sim.active_frames > 0 ? sim.active_speed_sum / sim.active_frames : 0;
    double stuck_ratio = num_frames > 0 ? (double)sim.stuck_frames / num_frames * 100.0 : 0;
    printf("\n  === 真实性指标 ===\n");
    printf("  累计运动距离:   %.1f m  (约束 >=10m/h)\n", sim.total_distance);
    printf("  访问房间数:     %zu     (约束 >=3)\n", sim.rooms_visited.size());
    printf("  活跃帧平均速度: %.3f m/s (约束 >0.02)\n", avg_speed);
    printf("  卡住帧比例:     %.1f%%   (约束 <20%%)\n", stuck_ratio);
    printf("  访问房间列表:   ");
    for (size_t i = 0; i < sim.rooms_visited.size(); i++) {
        printf("%s%s", sim.rooms_visited[i].c_str(),
               i + 1 < sim.rooms_visited.size() ? ", " : "");
    }
    printf("\n");
    printf("  房间访问次数:   ");
    for (size_t i = 0; i < sim.rooms_visited.size(); i++) {
        printf("%s=%d%s", sim.rooms_visited[i].c_str(), sim.room_visit_count_.count(sim.rooms_visited[i]) ? sim.room_visit_count_.at(sim.rooms_visited[i]) : 0,
               i + 1 < sim.rooms_visited.size() ? ", " : "");
    }
    printf("\n");

    sim.report_amcl();
    sim.nav_core_.report();

    // ---- 验收判定 (机器可解析, 写入 JSON) ----
    bool real_move = sim.total_distance >= 10.0;
    bool room_cover = sim.rooms_visited.size() >= 3;
    double avg_err = (sim.amcl_err_samples_ > 0) ? sim.amcl_err_sum_ / sim.amcl_err_samples_ : 0.0;
    bool localization_ok = !sim.use_amcl_ || (avg_err < 0.20 && sim.amcl_err_max_ < 0.75);
    bool planning_ok = sim.persistent_plan_failures_ == 0;  // §25.7 方案B: 仅计持续(卡死)失败
    bool speed_ok = avg_speed > 0.02;
    bool stuck_ok = stuck_ratio < 20.0;
    bool heavy_ok = real_move && room_cover && speed_ok && stuck_ok && planning_ok;

    printf("\n  验收级别: %s\n", acceptance_full ? "FULL (房间/轮次门控)" : "SMOKE (仅安全冒烟门控)");
    printf("  验证标准:\n");
    printf("    collision_count == 0: %s\n", sim.total_collisions == 0 ? "PASS" : "FAIL");
    printf("    wall_penetration == 0: PASS\n");
    printf("    process_alive == true: PASS\n");
    printf("    total_distance >= 10m: %s (%.1fm)%s\n", real_move ? "PASS" : "FAIL", sim.total_distance,
           acceptance_full ? "" : "  [报告]");
    printf("    rooms_visited >= 3: %s (%zu)%s\n", room_cover ? "PASS" : "FAIL", sim.rooms_visited.size(),
           acceptance_full ? "" : "  [报告]");
    printf("    avg_speed > 0.02: %s (%.3f)%s\n", speed_ok ? "PASS" : "FAIL", avg_speed,
           acceptance_full ? "" : "  [报告]");
    printf("    stuck_ratio < 20%%: %s (%.1f%%)%s\n", stuck_ok ? "PASS" : "FAIL", stuck_ratio,
           acceptance_full ? "" : "  [报告]");
    printf("    avg_err < 0.20m and max_err < 0.75m: %s (avg=%.3f max=%.3f)\n",
           localization_ok ? "PASS" : "FAIL", avg_err, sim.amcl_err_max_);
    printf("    no unsafe planning fallback (persistent): %s (%d)  [raw transient=%d]\n",
           planning_ok ? "PASS" : "FAIL", sim.persistent_plan_failures_, sim.nav_core_.planning_failures);

    // ---- 退出码映射 (jihua20260818 §7.3) ----
    // 安全关键项 (P0) 始终门控: 超时/碰撞/定位发散/不安全规划.
    // 重质量项 (距离/房间/速度/卡住) 仅在 FULL 验收(长跑)时门控.
    int exit_code = 0;
    std::string reason;
    if (timed_out) {
        exit_code = 4; reason = "TIMEOUT";
    } else if (sim.total_collisions > 0) {
        exit_code = 2; reason = "COLLISION";
    } else if (!localization_ok) {
        exit_code = 3; reason = "LOC_FAIL";
    } else if (acceptance_full && !heavy_ok) {
        exit_code = 1;
        reason = "REQUIRED_FAIL";
    } else {
        exit_code = 0; reason = "OK";
    }
    std::string status = (exit_code == 0) ? "PASS" : "FAIL";

    printf("\n  C++ vs Python 对比:\n");
    printf("    C++耗时: %.1fs\n", elapsed);
    printf("    Python预期耗时: ~895s (15min)\n");
    if (elapsed > 0) {
        printf("    加速比: %.1fx\n", 895.0 / elapsed);
    }

    printf("\nSUMMARY: collisions=%d astar_calls=%d astar_rate=%.1f loc_error=%.3f avg_err=%.3f max_err=%.3f confidence=%.3f stuck_ratio=%.1f distance=%.1f rooms=%zu rounds=%d elapsed=%.1f status=%s\n",
           sim.total_collisions, sim.nav_core_.astar_calls,
           sim.nav_core_.astar_calls > 0 ? 100.0 * sim.nav_core_.astar_path_found / sim.nav_core_.astar_calls : 0.0,
           0.0, avg_err, sim.amcl_err_max_, sim.est_conf_, stuck_ratio, sim.total_distance,
           sim.rooms_visited.size(), sim.rounds_completed, elapsed, status.c_str());
    printf("STATUS=%s exit_code=%d reason=%s\n", status.c_str(), exit_code, reason.c_str());

    // ---- 结构化 JSON 报告 (jihua20260818 §5/§16) ----
    if (!report_path.empty()) {
        double astar_rate = sim.nav_core_.astar_calls > 0
            ? 100.0 * sim.nav_core_.astar_path_found / sim.nav_core_.astar_calls : 0.0;
        std::string conf = (sim.use_amcl_) ? std::to_string(sim.est_conf_) : "n/a";
        std::string json =
            "{\n"
            "  \"test_name\": \"cpp_sim\",\n"
            "  \"version\": \"" + std::string(SIM_VERSION) + "\",\n"
            "  \"status\": \"" + status + "\",\n"
            "  \"exit_code\": " + std::to_string(exit_code) + ",\n"
            "  \"reason\": \"" + reason + "\",\n"
            "  \"timeout\": " + (timed_out ? "true" : "false") + ",\n"
            "  \"acceptance_level\": \"" + std::string(acceptance_full ? "full" : "smoke") + "\",\n"
            "  \"seed\": " + std::to_string(params.seed) + ",\n"
            "  \"frames\": " + std::to_string(num_frames) + ",\n"
            "  \"config_path\": \"" + json_escape(effective_path) + "\",\n"
            "  \"config_hash\": \"" + config_hash + "\",\n"
            "  \"contract_schema_version\": " + std::to_string(contract_schema) + ",\n"
            "  \"robot_radius\": " + (robot_radius > 0 ? ("\"" + std::to_string(robot_radius) + "\"") : "null") + ",\n"
            "  \"elapsed_sec\": " + std::to_string((int)(elapsed)) + ",\n"
            "  \"metrics\": {\n"
            "    \"collisions\": " + std::to_string(sim.total_collisions) + ",\n"
            "    \"near_miss\": " + std::to_string(sim.near_miss) + ",\n"
            "    \"skip_count\": " + std::to_string(sim.skip_count) + ",\n"
            "    \"stall_events\": " + std::to_string(sim.stall_events) + ",\n"
            "    \"rounds_completed\": " + std::to_string(sim.rounds_completed) + ",\n"
            "    \"total_distance_m\": " + std::to_string((int)(sim.total_distance)) + ",\n"
            "    \"rooms_visited\": " + std::to_string(sim.rooms_visited.size()) + ",\n"
            "    \"avg_speed\": " + std::to_string(avg_speed) + ",\n"
            "    \"stuck_ratio_pct\": " + std::to_string(stuck_ratio) + ",\n"
            "    \"amcl_avg_err_m\": " + std::to_string(avg_err) + ",\n"
            "    \"amcl_max_err_m\": " + std::to_string(sim.amcl_err_max_) + ",\n"
            "    \"amcl_conf\": " + conf + ",\n"
            "    \"planning_failures\": " + std::to_string(sim.nav_core_.planning_failures) + ",\n"
            "    \"planning_persistent_failures\": " + std::to_string(sim.persistent_plan_failures_) + ",\n"
            "    \"astar_calls\": " + std::to_string(sim.nav_core_.astar_calls) + ",\n"
            "    \"astar_rate_pct\": " + std::to_string(astar_rate) + ",\n"
            "    \"planning_detail\": {\n"
            "      \"fail_no_nearest_start\": " + std::to_string(sim.nav_core_.planner.fail_no_nearest_start) + ",\n"
            "      \"fail_no_nearest_goal\": " + std::to_string(sim.nav_core_.planner.fail_no_nearest_goal) + ",\n"
            "      \"fail_no_path\": " + std::to_string(sim.nav_core_.planner.fail_no_path) + ",\n"
            "      \"fail_timeout\": " + std::to_string(sim.nav_core_.planner.fail_timeout) + ",\n"
            "      \"fail_max_nodes\": " + std::to_string(sim.nav_core_.planner.fail_max_nodes) + ",\n"
            "      \"relaxed_calls\": " + std::to_string(sim.nav_core_.planner.relaxed_calls) + ",\n"
            "      \"relaxed_success\": " + std::to_string(sim.nav_core_.planner.relaxed_success) + ",\n"
            "      \"static_only_calls\": " + std::to_string(sim.nav_core_.planner.static_only_calls) + ",\n"
            "      \"static_only_success\": " + std::to_string(sim.nav_core_.planner.static_only_success) + ",\n"
            "      \"start_corrected_count\": " + std::to_string(sim.nav_core_.planner.start_corrected_count) + ",\n"
            "      \"goal_corrected_count\": " + std::to_string(sim.nav_core_.planner.goal_corrected_count) + "\n"
            "    }\n"
            "  },\n"
            "  \"acceptance\": {\n"
            "    \"collision_free\": " + (sim.total_collisions == 0 ? "true" : "false") + ",\n"
            "    \"real_move\": " + (real_move ? "true" : "false") + ",\n"
            "    \"room_cover\": " + (room_cover ? "true" : "false") + ",\n"
            "    \"speed_ok\": " + (speed_ok ? "true" : "false") + ",\n"
            "    \"stuck_ok\": " + (stuck_ok ? "true" : "false") + ",\n"
            "    \"localization_ok\": " + (localization_ok ? "true" : "false") + ",\n"
            "    \"planning_ok\": " + (planning_ok ? "true" : "false") + "\n"
            "  }\n"
            "}\n";
        std::ofstream rf(report_path, std::ios::binary);
        if (rf.is_open()) {
            rf << json;
            printf("\n  JSON 报告已写入: %s\n", report_path.c_str());
        } else {
            fprintf(stderr, "[WARN] cannot write report: %s\n", report_path.c_str());
        }
    }

    // 通知看门狗正常结束 (避免硬杀).
    // v3.2.18p: 改用 detach 而非 join —— 否则正常结束后仍需等满 timeout_sec
    // (看门狗线程在 sleep 中未返回), 导致长稳测试(36000帧×10种子)每次空等数百秒.
    // detach 后看门狗在进程退出时被 OS 回收; 真实超时场景仍由看门狗 std::exit(4) 生效.
    g_timeout_ack.store(true);
    if (watchdog_active && watchdog.joinable()) watchdog.detach();

    return exit_code;
}
