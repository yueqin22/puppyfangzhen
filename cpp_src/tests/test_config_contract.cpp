// test_config_contract.cpp — 模块一 白盒单元测试 (C++ 侧加载器)
// =====================================================================
// 覆盖:
//   - 内联契约解析 (嵌套段 / 块级列表 footprint / 标量类型)
//   - 一致性校验: 合法契约 0 错误; 坏 footprint(自交/面积0) 报错;
//     inflation_radius < radius+margin 报错
//   - 扁平快照字段值
//   - SHA-256 确定性
//   - 加载真实 config/simulation_contract.yaml 并与 Python 侧字段对齐
#include "config_contract.h"
#include "test_framework.h"
#include <cstdio>

using namespace puppy;

// 内联最小契约 (与 config/simulation_contract.yaml 关键字段一致)
static const char* kInlineContract = R"YAML(
schema_version: 1
robot:
  name: puppy
  radius: 0.35
  length: 0.70
  width: 0.50
  height: 0.55
  safety_margin: 0.08
  footprint:
    - [-0.35, -0.25]
    - [-0.35,  0.25]
    - [ 0.35,  0.25]
    - [ 0.35, -0.25]
map:
  resolution: 0.05
  width: 320
  height: 240
planning:
  inflation_radius: 0.43
  goal_tolerance: 0.35
  min_path_clearance: 0.08
simulation:
  default_frames: 300
  max_frames: 108000
  seed: 1
acceptance:
  max_average_localization_error: 0.20
  min_required_rooms: 8
)YAML";

TEST(config, parse_inline_contract) {
    Contract c;
    std::vector<std::string> err, warn;
    YNode root = parse_yaml(std::string(kInlineContract));
    // 直接经 load_contract 从内联文本无法 (需要文件), 故用 parse+手动填充替代:
    // 这里改为从临时文件加载以复用完整 load 逻辑。
    // 写法: 将内联文本写入临时文件再 load。
    const char* tmp = "config_contract_inline_tmp.yaml";
    { std::ofstream o(tmp, std::ios::binary); o << kInlineContract; }
    bool ok = load_contract(tmp, c);
    std::remove(tmp);
    ASSERT_TRUE(ok);
    ASSERT_EQ(c.schema_version, 1);
    ASSERT_NEAR(c.robot.radius, 0.35, 1e-9);
    ASSERT_NEAR(c.robot.length, 0.70, 1e-9);
    ASSERT_NEAR(c.robot.safety_margin, 0.08, 1e-9);
    ASSERT_EQ((int)c.robot.footprint.size(), 4);
    ASSERT_NEAR(c.robot.footprint[0].x, -0.35, 1e-9);
    ASSERT_NEAR(c.robot.footprint[0].y, -0.25, 1e-9);
    ASSERT_NEAR(c.robot.footprint[2].x, 0.35, 1e-9);
    ASSERT_NEAR(c.map.resolution, 0.05, 1e-9);
    ASSERT_EQ(c.map.width, 320);
    ASSERT_NEAR(c.planning.inflation_radius, 0.43, 1e-9);
    ASSERT_EQ(c.simulation.default_frames, 300);
    ASSERT_EQ(c.simulation.max_frames, 108000);
    ASSERT_EQ(c.acceptance.min_required_rooms, 8);
}

TEST(config, validate_good_contract) {
    const char* tmp = "config_contract_inline_tmp.yaml";
    { std::ofstream o(tmp, std::ios::binary); o << kInlineContract; }
    Contract c;
    ASSERT_TRUE(load_contract(tmp, c));
    std::remove(tmp);
    std::vector<std::string> err, warn;
    validate_contract(c, err, warn);
    ASSERT_EQ((int)err.size(), 0);
}

TEST(config, validate_bad_footprint_self_intersect) {
    const char* bad = R"YAML(
robot:
  radius: 0.35
  safety_margin: 0.08
  footprint:
    - [0.0, 0.0]
    - [1.0, 1.0]
    - [0.0, 1.0]
    - [1.0, 0.0]
map:
  resolution: 0.05
  width: 10
  height: 10
planning:
  inflation_radius: 0.50
simulation:
  default_frames: 100
  max_frames: 200
)YAML";
    const char* tmp = "config_contract_bad_fp.yaml";
    { std::ofstream o(tmp, std::ios::binary); o << bad; }
    Contract c;
    ASSERT_TRUE(load_contract(tmp, c));
    std::remove(tmp);
    std::vector<std::string> err, warn;
    validate_contract(c, err, warn);
    // 自交 + 面积0 应触发至少一个错误
    EXPECT_FALSE(err.empty());
    bool found = false;
    for (auto& e : err) if (e.find("self-intersecting") != std::string::npos ||
                            e.find("area must be > 0") != std::string::npos) found = true;
    EXPECT_TRUE(found);
}

TEST(config, validate_bad_inflation) {
    const char* bad = R"YAML(
robot:
  radius: 0.35
  safety_margin: 0.08
  footprint:
    - [-0.35, -0.25]
    - [-0.35,  0.25]
    - [ 0.35,  0.25]
    - [ 0.35, -0.25]
map:
  resolution: 0.05
  width: 10
  height: 10
planning:
  inflation_radius: 0.30
simulation:
  default_frames: 100
  max_frames: 200
)YAML";
    const char* tmp = "config_contract_bad_infl.yaml";
    { std::ofstream o(tmp, std::ios::binary); o << bad; }
    Contract c;
    ASSERT_TRUE(load_contract(tmp, c));
    std::remove(tmp);
    std::vector<std::string> err, warn;
    validate_contract(c, err, warn);
    bool found = false;
    for (auto& e : err) if (e.find("inflation_radius < robot.radius") != std::string::npos) found = true;
    EXPECT_TRUE(found);
}

TEST(config, snapshot_fields) {
    const char* tmp = "config_contract_inline_tmp.yaml";
    { std::ofstream o(tmp, std::ios::binary); o << kInlineContract; }
    Contract c;
    ASSERT_TRUE(load_contract(tmp, c));
    std::remove(tmp);
    auto s = snapshot(c);
    ASSERT_TRUE(s["robot.radius"] == "0.35");
    ASSERT_TRUE(s["robot.footprint_size"] == "4");
    ASSERT_TRUE(s["map.resolution"] == "0.05");
    ASSERT_TRUE(s["simulation.default_frames"] == "300");
}

TEST(config, sha256_deterministic) {
    std::string a = sha256_hex("puppy-config");
    std::string b = sha256_hex("puppy-config");
    ASSERT_TRUE(a == b);
    ASSERT_EQ(a.size(), 64u);
    // 已知向量: SHA-256("abc") = ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad
    ASSERT_TRUE(sha256_hex("abc") ==
                std::string("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"));
}

TEST(config, load_real_file_and_crosscheck) {
    // 加载真实契约文件并校验 (白盒: 与 Python config_loader 字段对齐)
    const char* candidates[] = {
        "config/simulation_contract.yaml",
        "../../config/simulation_contract.yaml",
        "E:/puppyfangzhen/config/simulation_contract.yaml",
        "E:\\puppyfangzhen\\config\\simulation_contract.yaml"
    };
    std::string path;
    for (auto c : candidates) {
        std::ifstream f(c);
        if (f) { path = c; break; }
    }
    if (path.empty()) {
        std::cout << "  [SKIP] real config file not found; inline tests still cover loader" << std::endl;
        return;
    }
    Contract c;
    ASSERT_TRUE(load_contract(path, c));
    ASSERT_NEAR(c.robot.radius, 0.35, 1e-9);
    ASSERT_EQ((int)c.robot.footprint.size(), 4);
    std::vector<std::string> err, warn;
    validate_contract(c, err, warn);
    ASSERT_EQ((int)err.size(), 0);
    auto s = snapshot(c);
    ASSERT_TRUE(s["robot.radius"] == "0.35");
    ASSERT_TRUE(s["planning.inflation_radius"] == "0.43");
    ASSERT_TRUE(s["map.resolution"] == "0.05");
}

int main() {
    return RUN_ALL_TESTS();
}
