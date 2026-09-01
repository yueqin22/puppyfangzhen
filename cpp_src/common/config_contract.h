// config_contract.h — Puppy 仿真导航统一配置加载器 (C++ 侧)
// =====================================================================
// 依据 jihua20260818.md 模块一 (§4)。与 Python config/config_loader.py
// 共享同一份 config/simulation_contract.yaml，保证 Python / C++ / ROS /
// UE 读取同一来源。
//
// 本头文件零第三方依赖，自带最小 YAML 子集解析器 (嵌套段 / 标量 /
// 内联列表 / 块级列表)，逻辑与 Python 加载器对齐，便于两侧字段级 diff。
//
// 提供:
//   - parse_yaml(text)                 -> 通用节点树
//   - load_contract(path, Contract&)   -> 填充强类型结构体
//   - validate_contract(c, err, warn)  -> 一致性校验 (§4.3)
//   - snapshot(c)                      -> 扁平字段快照 (白盒 diff)
//   - sha256_hex(text)                 -> 文件指纹 (启动时打印)
#ifndef PUPPY_CONFIG_CONTRACT_H_
#define PUPPY_CONFIG_CONTRACT_H_

#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <cstdint>
#include <cmath>
#include <algorithm>

namespace puppy {

// ----------------------------------------------------------------------
// 强类型契约结构体
// ----------------------------------------------------------------------
struct FootprintPoint { double x = 0.0, y = 0.0; };

struct RobotCfg {
    std::string name;
    double radius = 0.0;
    double length = 0.0;
    double width = 0.0;
    double height = 0.0;
    double safety_margin = 0.0;
    std::vector<FootprintPoint> footprint;
};

struct MapCfg {
    double resolution = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    int width = 0;
    int height = 0;
    bool unknown_is_blocked = false;
};

struct PlanningCfg {
    int connected = 0;
    int lethal_cost = 0;
    int inscribed_cost = 0;
    int plan_blocked_cost = 0;
    double inflation_radius = 0.0;
    double min_path_clearance = 0.0;
    double goal_tolerance = 0.0;
    int max_planning_time_ms = 0;
};

struct LocalAvoidanceCfg {
    double max_speed = 0.0;
    double max_acceleration = 0.0;
    double max_angular_speed = 0.0;
    double dynamic_clearance = 0.0;
    double emergency_stop_distance = 0.0;
    double command_timeout_sec = 0.0;
};

struct SimulationCfg {
    int default_frames = 0;
    int max_frames = 0;
    int progress_interval_frames = 0;
    int hard_timeout_sec = 0;
    int seed = 0;
};

struct AcceptanceCfg {
    int max_collisions = 0;
    int max_wall_penetrations = 0;
    int max_planning_failures = 0;
    int max_goal_misses = 0;
    int max_stall_events = 0;
    double max_average_localization_error = 0.0;
    double max_peak_localization_error = 0.0;
    int min_required_rooms = 0;
    bool require_full_round = false;
};

struct Contract {
    int schema_version = 0;
    RobotCfg robot;
    std::vector<FootprintPoint> footprint_alias;  // placeholder (unused)
    MapCfg map;
    PlanningCfg planning;
    LocalAvoidanceCfg local_avoidance;
    SimulationCfg simulation;
    AcceptanceCfg acceptance;
};

// ----------------------------------------------------------------------
// 最小 YAML 解析
// ----------------------------------------------------------------------
struct YNode {
    enum Type { Scalar, List, Map } type = Scalar;
    std::string scalar;
    std::vector<YNode> list;
    std::map<std::string, YNode> map;
};

inline std::string strip_inline_comment(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (size_t i = 0; i < s.size(); ++i) {
        char ch = s[i];
        if (ch == '#' && (i == 0 || s[i - 1] == ' ' || s[i - 1] == '\t')) break;
        out.push_back(ch);
    }
    return out;
}

inline std::string trim(const std::string& s) {
    size_t b = s.find_first_not_of(" \t\r\n");
    size_t e = s.find_last_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    return s.substr(b, e - b + 1);
}

inline YNode parse_scalar(const std::string& s) {
    YNode n;
    std::string t = trim(strip_inline_comment(s));
    if (t.empty() || t == "null" || t == "~") return n;
    if (t == "true" || t == "True" || t == "TRUE") { n.scalar = "true"; return n; }
    if (t == "false" || t == "False" || t == "FALSE") { n.scalar = "false"; return n; }
    if (!t.empty() && t.front() == '[' && t.back() == ']') {
        std::string inner = t.substr(1, t.size() - 2);
        n.type = YNode::List;
        std::string cur;
        int depth = 0;
        for (char c : inner) {
            if (c == '[') ++depth;
            else if (c == ']') --depth;
            if (c == ',' && depth == 0) { n.list.push_back(parse_scalar(cur)); cur.clear(); }
            else cur.push_back(c);
        }
        if (!cur.empty()) n.list.push_back(parse_scalar(cur));
        return n;
    }
    n.scalar = t;
    return n;
}

inline std::pair<YNode, int> parse_block(const std::vector<std::string>& lines,
                                         int i, int indent) {
    YNode node;
    node.type = YNode::Map;
    while (i < (int)lines.size()) {
        const std::string& raw = lines[i];
        std::string stripped = trim(raw);
        if (stripped.empty()) { ++i; continue; }
        if (stripped.front() == '#') { ++i; continue; }
        int cur = (int)(raw.size() - stripped.size());
        if (cur < indent) break;
        if (cur > indent) { ++i; continue; }
        if (stripped.rfind("- ", 0) == 0) { ++i; continue; }
        size_t colon = stripped.find(':');
        if (colon == std::string::npos) { ++i; continue; }
        std::string key = trim(stripped.substr(0, colon));
        std::string rest = trim(strip_inline_comment(stripped.substr(colon + 1)));

        int nxt = i + 1;
        if (nxt < (int)lines.size()) {
            std::string nxt_raw = lines[nxt];
            std::string nxt_stripped = trim(nxt_raw);
            int nxt_indent = indent;
            { size_t b2 = nxt_raw.find_first_not_of(" \t\r\n");
              if (b2 != std::string::npos) nxt_indent = (int)b2; }

            // 块级列表: "- item"
            if (rest.empty() && !nxt_stripped.empty() && nxt_stripped.rfind("- ", 0) == 0) {
                YNode lst; lst.type = YNode::List;
                int j = nxt;
                while (j < (int)lines.size()) {
                    const std::string& lr = lines[j];
                    if (lr.find_first_not_of(" \t\r\n") == std::string::npos) { ++j; continue; }
                    std::string lrs = trim(lr);
                    int lcur = (int)(lr.size() - lrs.size());
                    if (lcur < nxt_indent || lrs.rfind("- ", 0) != 0) break;
                    YNode item_node;
                    item_node.scalar = lrs.substr(2);
                    lst.list.push_back(item_node);
                    ++j;
                }
                node.map[key] = lst;
                i = j;
                continue;
            }
            // 嵌套映射
            if (rest.empty() && nxt_indent > indent && !nxt_stripped.empty()) {
                YNode child; int after = 0;
                std::tie(child, after) = parse_block(lines, nxt, nxt_indent);
                node.map[key] = child;
                i = after;
                continue;
            }
        }
        node.map[key] = parse_scalar(rest);
        i = nxt;
    }
    return { node, i };
}

inline YNode parse_yaml(const std::string& text) {
    std::vector<std::string> lines;
    std::stringstream ss(text);
    std::string line;
    while (std::getline(ss, line)) lines.push_back(line);
    return parse_block(lines, 0, 0).first;
}

// ----------------------------------------------------------------------
// 类型取值辅助
// ----------------------------------------------------------------------
inline std::string node_scalar(const YNode& n) {
    return (n.type == YNode::Scalar) ? n.scalar : std::string();
}
inline double as_double(const YNode& n, double def = 0.0) {
    std::string s = node_scalar(n);
    if (s.empty()) return def;
    try { return std::stod(s); } catch (...) { return def; }
}
inline int as_int(const YNode& n, int def = 0) {
    std::string s = node_scalar(n);
    if (s.empty()) return def;
    try { return std::stoi(s); } catch (...) { return def; }
}
inline bool as_bool(const YNode& n, bool def = false) {
    std::string s = node_scalar(n);
    if (s == "true") return true;
    if (s == "false") return false;
    return def;
}
inline const YNode* map_get(const YNode& n, const std::string& k) {
    if (n.type != YNode::Map) return nullptr;
    auto it = n.map.find(k);
    return (it == n.map.end()) ? nullptr : &it->second;
}

// ----------------------------------------------------------------------
// 校验 (§4.3) —— 与 Python 对齐
// ----------------------------------------------------------------------
inline double polygon_area(const std::vector<FootprintPoint>& pts) {
    int n = (int)pts.size();
    if (n < 3) return 0.0;
    double s = 0.0;
    for (int i = 0; i < n; ++i) {
        const auto& a = pts[i];
        const auto& b = pts[(i + 1) % n];
        s += a.x * b.y - b.x * a.y;
    }
    return std::fabs(s) / 2.0;
}
inline bool seg_intersect(const FootprintPoint& p1, const FootprintPoint& p2,
                          const FootprintPoint& p3, const FootprintPoint& p4) {
    auto ccw = [](const FootprintPoint& a, const FootprintPoint& b, const FootprintPoint& c) {
        return (c.y - a.y) * (b.x - a.x) - (b.y - a.y) * (c.x - a.x);
    };
    double d1 = ccw(p3, p4, p1), d2 = ccw(p3, p4, p2);
    double d3 = ccw(p1, p2, p3), d4 = ccw(p1, p2, p4);
    return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0));
}
inline bool self_intersecting(const std::vector<FootprintPoint>& pts) {
    int n = (int)pts.size();
    for (int i = 0; i < n; ++i) {
        const auto& a1 = pts[i], a2 = pts[(i + 1) % n];
        for (int j = i + 2; j < n; ++j) {
            if (i == 0 && j == n - 1) continue;
            const auto& b1 = pts[j], b2 = pts[(j + 1) % n];
            if (seg_intersect(a1, a2, b1, b2)) return true;
        }
    }
    return false;
}

inline void validate_contract(const Contract& c,
                              std::vector<std::string>& errors,
                              std::vector<std::string>& warnings) {
    const RobotCfg& r = c.robot;
    if (!(r.radius > 0)) errors.push_back("robot.radius must be a positive number");

    if (r.footprint.size() < 3) {
        errors.push_back("robot.footprint must be a list of >=3 points");
    } else {
        double area = polygon_area(r.footprint);
        if (area <= 0) errors.push_back("robot.footprint area must be > 0");
        if (self_intersecting(r.footprint)) errors.push_back("robot.footprint is self-intersecting");
        double max_r = 0.0;
        for (const auto& p : r.footprint)
            max_r = std::max(max_r, std::sqrt(p.x * p.x + p.y * p.y));
        if (r.radius > 0 && max_r < r.radius - 1e-6)
            errors.push_back("robot.footprint circumscribed radius < robot.radius");
        if (!r.footprint.empty() &&
            (r.footprint[0].x != r.footprint.back().x ||
             r.footprint[0].y != r.footprint.back().y))
            warnings.push_back("robot.footprint is not explicitly closed");
    }

    if (c.planning.inflation_radius > 0 && r.radius > 0 && r.safety_margin > 0) {
        if (c.planning.inflation_radius < r.radius + r.safety_margin - 1e-9)
            errors.push_back("planning.inflation_radius < robot.radius + safety_margin");
    }
    if (c.planning.goal_tolerance > 0) {
        if (r.length > 0 && c.planning.goal_tolerance > r.length / 2.0 + 1e-9)
            errors.push_back("planning.goal_tolerance > half key-door-width");
    }
    if (!(c.map.resolution > 0)) errors.push_back("map.resolution must be > 0");
    if (!(c.map.width > 0)) errors.push_back("map.width must be > 0");
    if (!(c.map.height > 0)) errors.push_back("map.height must be > 0");
    if (!(c.simulation.default_frames > 0))
        errors.push_back("simulation.default_frames must be a positive integer");
    if (c.simulation.default_frames > 0 && c.simulation.max_frames > 0 &&
        c.simulation.default_frames > c.simulation.max_frames)
        errors.push_back("simulation.default_frames > max_frames");
}

// ----------------------------------------------------------------------
// 扁平快照 (白盒字段级 diff)
// ----------------------------------------------------------------------
inline std::map<std::string, std::string> snapshot(const Contract& c) {
    auto f = [](double v) { char buf[64]; std::snprintf(buf, sizeof(buf), "%.6g", v); return std::string(buf); };
    std::map<std::string, std::string> s;
    s["schema_version"] = std::to_string(c.schema_version);
    s["robot.radius"] = f(c.robot.radius);
    s["robot.length"] = f(c.robot.length);
    s["robot.width"] = f(c.robot.width);
    s["robot.safety_margin"] = f(c.robot.safety_margin);
    s["robot.footprint_size"] = std::to_string(c.robot.footprint.size());
    s["map.resolution"] = f(c.map.resolution);
    s["map.width"] = std::to_string(c.map.width);
    s["map.height"] = std::to_string(c.map.height);
    s["planning.inflation_radius"] = f(c.planning.inflation_radius);
    s["planning.goal_tolerance"] = f(c.planning.goal_tolerance);
    s["planning.min_path_clearance"] = f(c.planning.min_path_clearance);
    s["simulation.default_frames"] = std::to_string(c.simulation.default_frames);
    s["simulation.max_frames"] = std::to_string(c.simulation.max_frames);
    s["simulation.seed"] = std::to_string(c.simulation.seed);
    s["acceptance.max_average_localization_error"] = f(c.acceptance.max_average_localization_error);
    s["acceptance.max_peak_localization_error"] = f(c.acceptance.max_peak_localization_error);
    s["acceptance.min_required_rooms"] = std::to_string(c.acceptance.min_required_rooms);
    return s;
}

// ----------------------------------------------------------------------
// 加载契约文件 -> 强类型结构体
// ----------------------------------------------------------------------
inline bool load_contract(const std::string& path, Contract& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    std::stringstream ss; ss << f.rdbuf();
    std::string text = ss.str();
    YNode root = parse_yaml(text);

    if (const YNode* sv = map_get(root, "schema_version")) out.schema_version = as_int(*sv, 0);

    if (const YNode* robot = map_get(root, "robot")) {
        if (const YNode* v = map_get(*robot, "name")) out.robot.name = node_scalar(*v);
        if (const YNode* v = map_get(*robot, "radius")) out.robot.radius = as_double(*v);
        if (const YNode* v = map_get(*robot, "length")) out.robot.length = as_double(*v);
        if (const YNode* v = map_get(*robot, "width")) out.robot.width = as_double(*v);
        if (const YNode* v = map_get(*robot, "height")) out.robot.height = as_double(*v);
        if (const YNode* v = map_get(*robot, "safety_margin")) out.robot.safety_margin = as_double(*v);
        if (const YNode* fp = map_get(*robot, "footprint")) {
            if (fp->type == YNode::List) {
                for (const auto& item : fp->list) {
                    YNode inner = parse_scalar(item.scalar);
                    if (inner.type == YNode::List && inner.list.size() == 2) {
                        FootprintPoint p;
                        p.x = as_double(inner.list[0]);
                        p.y = as_double(inner.list[1]);
                        out.robot.footprint.push_back(p);
                    }
                }
            }
        }
    }
    if (const YNode* m = map_get(root, "map")) {
        if (const YNode* v = map_get(*m, "resolution")) out.map.resolution = as_double(*v);
        if (const YNode* v = map_get(*m, "origin_x")) out.map.origin_x = as_double(*v);
        if (const YNode* v = map_get(*m, "origin_y")) out.map.origin_y = as_double(*v);
        if (const YNode* v = map_get(*m, "width")) out.map.width = as_int(*v);
        if (const YNode* v = map_get(*m, "height")) out.map.height = as_int(*v);
        if (const YNode* v = map_get(*m, "unknown_is_blocked")) out.map.unknown_is_blocked = as_bool(*v);
    }
    if (const YNode* p = map_get(root, "planning")) {
        if (const YNode* v = map_get(*p, "connected")) out.planning.connected = as_int(*v);
        if (const YNode* v = map_get(*p, "lethal_cost")) out.planning.lethal_cost = as_int(*v);
        if (const YNode* v = map_get(*p, "inscribed_cost")) out.planning.inscribed_cost = as_int(*v);
        if (const YNode* v = map_get(*p, "plan_blocked_cost")) out.planning.plan_blocked_cost = as_int(*v);
        if (const YNode* v = map_get(*p, "inflation_radius")) out.planning.inflation_radius = as_double(*v);
        if (const YNode* v = map_get(*p, "min_path_clearance")) out.planning.min_path_clearance = as_double(*v);
        if (const YNode* v = map_get(*p, "goal_tolerance")) out.planning.goal_tolerance = as_double(*v);
        if (const YNode* v = map_get(*p, "max_planning_time_ms")) out.planning.max_planning_time_ms = as_int(*v);
    }
    if (const YNode* la = map_get(root, "local_avoidance")) {
        if (const YNode* v = map_get(*la, "max_speed")) out.local_avoidance.max_speed = as_double(*v);
        if (const YNode* v = map_get(*la, "max_acceleration")) out.local_avoidance.max_acceleration = as_double(*v);
        if (const YNode* v = map_get(*la, "max_angular_speed")) out.local_avoidance.max_angular_speed = as_double(*v);
        if (const YNode* v = map_get(*la, "dynamic_clearance")) out.local_avoidance.dynamic_clearance = as_double(*v);
        if (const YNode* v = map_get(*la, "emergency_stop_distance")) out.local_avoidance.emergency_stop_distance = as_double(*v);
        if (const YNode* v = map_get(*la, "command_timeout_sec")) out.local_avoidance.command_timeout_sec = as_double(*v);
    }
    if (const YNode* s = map_get(root, "simulation")) {
        if (const YNode* v = map_get(*s, "default_frames")) out.simulation.default_frames = as_int(*v);
        if (const YNode* v = map_get(*s, "max_frames")) out.simulation.max_frames = as_int(*v);
        if (const YNode* v = map_get(*s, "progress_interval_frames")) out.simulation.progress_interval_frames = as_int(*v);
        if (const YNode* v = map_get(*s, "hard_timeout_sec")) out.simulation.hard_timeout_sec = as_int(*v);
        if (const YNode* v = map_get(*s, "seed")) out.simulation.seed = as_int(*v);
    }
    if (const YNode* a = map_get(root, "acceptance")) {
        if (const YNode* v = map_get(*a, "max_collisions")) out.acceptance.max_collisions = as_int(*v);
        if (const YNode* v = map_get(*a, "max_wall_penetrations")) out.acceptance.max_wall_penetrations = as_int(*v);
        if (const YNode* v = map_get(*a, "max_planning_failures")) out.acceptance.max_planning_failures = as_int(*v);
        if (const YNode* v = map_get(*a, "max_goal_misses")) out.acceptance.max_goal_misses = as_int(*v);
        if (const YNode* v = map_get(*a, "max_stall_events")) out.acceptance.max_stall_events = as_int(*v);
        if (const YNode* v = map_get(*a, "max_average_localization_error")) out.acceptance.max_average_localization_error = as_double(*v);
        if (const YNode* v = map_get(*a, "max_peak_localization_error")) out.acceptance.max_peak_localization_error = as_double(*v);
        if (const YNode* v = map_get(*a, "min_required_rooms")) out.acceptance.min_required_rooms = as_int(*v);
        if (const YNode* v = map_get(*a, "require_full_round")) out.acceptance.require_full_round = as_bool(*v);
    }
    return true;
}

// ----------------------------------------------------------------------
// SHA-256 (FIPS-180-4)
// ----------------------------------------------------------------------
inline std::string sha256_hex(const std::string& data) {
    static const uint32_t K[64] = {
        0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
        0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
        0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
        0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
        0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
        0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
        0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
        0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};
    auto rotr = [](uint32_t x, uint32_t n) { return (x >> n) | (x << (32 - n)); };
    uint32_t h[8] = {0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19};
    std::vector<uint8_t> msg(data.begin(), data.end());
    uint64_t bitlen = (uint64_t)msg.size() * 8;
    msg.push_back(0x80);
    while (msg.size() % 64 != 56) msg.push_back(0x00);
    for (int i = 7; i >= 0; --i) msg.push_back((uint8_t)(bitlen >> (i * 8)));
    for (size_t off = 0; off < msg.size(); off += 64) {
        uint32_t w[64];
        for (int i = 0; i < 16; ++i)
            w[i] = ((uint32_t)msg[off+4*i]<<24)|((uint32_t)msg[off+4*i+1]<<16)|
                   ((uint32_t)msg[off+4*i+2]<<8)|((uint32_t)msg[off+4*i+3]);
        for (int i = 16; i < 64; ++i) {
            uint32_t s0 = rotr(w[i-15],7)^rotr(w[i-15],18)^(w[i-15]>>3);
            uint32_t s1 = rotr(w[i-2],17)^rotr(w[i-2],19)^(w[i-2]>>10);
            w[i] = w[i-16]+s0+w[i-7]+s1;
        }
        uint32_t a=h[0],b=h[1],c=h[2],d=h[3],e=h[4],f1=h[5],g=h[6],hh=h[7];
        for (int i = 0; i < 64; ++i) {
            uint32_t S1 = rotr(e,6)^rotr(e,11)^rotr(e,25);
            uint32_t ch = (e&f1)^((~e)&g);
            uint32_t t1 = hh + S1 + ch + K[i] + w[i];
            uint32_t S0 = rotr(a,2)^rotr(a,13)^rotr(a,22);
            uint32_t maj = (a&b)^(a&c)^(b&c);
            uint32_t t2 = S0 + maj;
            hh=g; g=f1; f1=e; e=d+t1; d=c; c=b; b=a; a=t1+t2;
        }
        h[0]+=a;h[1]+=b;h[2]+=c;h[3]+=d;h[4]+=e;h[5]+=f1;h[6]+=g;h[7]+=hh;
    }
    char hex[65];
    for (int i = 0; i < 8; ++i) std::snprintf(hex + i*8, 9, "%08x", h[i]);
    return std::string(hex, 64);
}

}  // namespace puppy

#endif  // PUPPY_CONFIG_CONTRACT_H_
