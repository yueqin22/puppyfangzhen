// test_framework.h — 极简单元测试框架（零依赖）
// ====================================================
// 仅需 <iostream>，无 GoogleTest/CTest 依赖。
// 用法:
//   TEST(suite, name) { ASSERT_EQ(a, b); EXPECT_NEAR(a, b, eps); }
//   int main() { return RUN_ALL_TESTS(); }
//
// 实现说明:
//   每个测试在独立函数中执行，通过 per-test 的失败标志位记录断言结果。
//   ASSERT_* 失败时设置标志并 return；EXPECT_* 失败时仅设置标志继续执行。
//   测试函数执行完毕后再根据标志位记录最终结果，避免"先记录通过后失败"的 bug。
#pragma once
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace puppy_test {

struct TestResult {
    std::string suite;
    std::string name;
    bool passed;
    std::string message;
};

inline std::vector<TestResult>& get_results() {
    static std::vector<TestResult> results;
    return results;
}

inline int& test_count() { static int c = 0; return c; }
inline int& pass_count() { static int c = 0; return c; }

// per-test 失败标志（每个 TEST 在执行体前重置）
inline bool& current_test_failed() { static bool f = false; return f; }
inline std::string& current_test_msg() { static std::string m; return m; }

inline void reset_current() {
    current_test_failed() = false;
    current_test_msg().clear();
}

inline void record_result(const std::string& suite, const std::string& name,
                          bool passed, const std::string& msg = "") {
    get_results().push_back({suite, name, passed, msg});
    test_count()++;
    if (passed) pass_count()++;
}

inline int run_all_tests() {
    auto& results = get_results();
    int failures = 0;
    for (auto& r : results) {
        if (!r.passed) {
            std::cout << "[FAIL] " << r.suite << "::" << r.name;
            if (!r.message.empty()) std::cout << " — " << r.message;
            std::cout << std::endl;
            failures++;
        } else {
            std::cout << "[PASS] " << r.suite << "::" << r.name << std::endl;
        }
    }
    std::cout << "\n==== " << pass_count() << "/" << test_count()
              << " tests passed";
    if (failures > 0) std::cout << " (" << failures << " FAILED)";
    std::cout << " ====" << std::endl;
    return failures > 0 ? 1 : 0;
}

} // namespace puppy_test

// ---- 宏 ----
// TEST 宏: 注册一个静态函数指针到全局表，main 中通过 RUN_ALL_TESTS 执行。
// 执行体前重置失败标志，执行体后根据标志记录结果。
// 用 vector<function<void()>> 收集测试，避免静态初始化顺序问题。
#define TEST(suite, name) \
    static void suite##_##name##_impl(); \
    static bool suite##_##name##_registered = []() { \
        puppy_test::get_results(); /* 触发 results 初始化 */ \
        puppy_test::get_test_registry().push_back( \
            {#suite, #name, suite##_##name##_impl}); \
        return true; \
    }(); \
    static void suite##_##name##_impl()

// ASSERT_*: 失败时设置标志并 return（终止当前测试）
// 注: 消息用 cout 流式输出，避免 std::to_string 对 uint8_t 等类型无匹配重载
#define ASSERT_EQ(a, b) \
    do { if ((a) != (b)) { \
        puppy_test::current_test_failed() = true; \
        puppy_test::current_test_msg() = std::string(#a " != " #b); \
        std::cout << "  [ASSERT FAIL] " #a " != " #b \
                  << " (" << +(a) << " vs " << +(b) << ")" << std::endl; \
        return; \
    } } while(0)

#define ASSERT_TRUE(x) \
    do { if (!(x)) { \
        puppy_test::current_test_failed() = true; \
        puppy_test::current_test_msg() = #x " is false"; \
        std::cout << "  [ASSERT FAIL] " #x " is false" << std::endl; \
        return; \
    } } while(0)

#define ASSERT_FALSE(x) \
    do { if ((x)) { \
        puppy_test::current_test_failed() = true; \
        puppy_test::current_test_msg() = #x " is true"; \
        std::cout << "  [ASSERT FAIL] " #x " is true" << std::endl; \
        return; \
    } } while(0)

#define ASSERT_NEAR(a, b, eps) \
    do { if (std::abs((double)(a) - (double)(b)) > (eps)) { \
        puppy_test::current_test_failed() = true; \
        puppy_test::current_test_msg() = std::string("|" #a " - " #b "| > " #eps); \
        std::cout << "  [ASSERT FAIL] |" #a " - " #b "| > " #eps \
                  << " (" << (a) << " vs " << (b) << ")" << std::endl; \
        return; \
    } } while(0)

// EXPECT_*: 失败时设置标志但继续执行（不 return）
#define EXPECT_NEAR(a, b, eps) \
    do { if (std::abs((double)(a) - (double)(b)) > (eps)) { \
        puppy_test::current_test_failed() = true; \
        puppy_test::current_test_msg() = std::string("|" #a " - " #b "| > " #eps); \
        std::cout << "  [EXPECT FAIL] |" #a " - " #b "| > " #eps \
                  << " (" << (a) << " vs " << (b) << ")" << std::endl; \
    } } while(0)

#define EXPECT_TRUE(x) \
    do { if (!(x)) { \
        puppy_test::current_test_failed() = true; \
        puppy_test::current_test_msg() = #x " is false"; \
        std::cout << "  [EXPECT FAIL] " #x " is false" << std::endl; \
    } } while(0)

#define EXPECT_FALSE(x) \
    do { if ((x)) { \
        puppy_test::current_test_failed() = true; \
        puppy_test::current_test_msg() = #x " is true"; \
        std::cout << "  [EXPECT FAIL] " #x " is true" << std::endl; \
    } } while(0)

// -- P1-2.2 新增: 补充常用比较宏，便于 A* 单元测试 --
// ASSERT 系列: 失败 return
#define ASSERT_NE(a, b) \
    do { if ((a) == (b)) { \
        puppy_test::current_test_failed() = true; \
        puppy_test::current_test_msg() = std::string(#a " == " #b " (expected !=)"); \
        std::cout << "  [ASSERT FAIL] " #a " == " #b " (expected !=)" \
                  << " (" << +(a) << ")" << std::endl; \
        return; \
    } } while(0)

// EXPECT 系列: 失败继续
#define EXPECT_GE(a, b) \
    do { if ((double)(a) < (double)(b) - 1e-12) { \
        puppy_test::current_test_failed() = true; \
        puppy_test::current_test_msg() = std::string(#a " < " #b " (expected >=)"); \
        std::cout << "  [EXPECT FAIL] " #a " < " #b " (expected >=)" \
                  << " (" << (a) << " vs " << (b) << ")" << std::endl; \
    } } while(0)

#define EXPECT_GT(a, b) \
    do { if ((double)(a) <= (double)(b) + 1e-12) { \
        puppy_test::current_test_failed() = true; \
        puppy_test::current_test_msg() = std::string(#a " <= " #b " (expected >)"); \
        std::cout << "  [EXPECT FAIL] " #a " <= " #b " (expected >)" \
                  << " (" << (a) << " vs " << (b) << ")" << std::endl; \
    } } while(0)

#define EXPECT_LE(a, b) \
    do { if ((double)(a) > (double)(b) + 1e-12) { \
        puppy_test::current_test_failed() = true; \
        puppy_test::current_test_msg() = std::string(#a " > " #b " (expected <=)"); \
        std::cout << "  [EXPECT FAIL] " #a " > " #b " (expected <=)" \
                  << " (" << (a) << " vs " << (b) << ")" << std::endl; \
    } } while(0)

#define EXPECT_LT(a, b) \
    do { if ((double)(a) >= (double)(b) - 1e-12) { \
        puppy_test::current_test_failed() = true; \
        puppy_test::current_test_msg() = std::string(#a " >= " #b " (expected <)"); \
        std::cout << "  [EXPECT FAIL] " #a " >= " #b " (expected <)" \
                  << " (" << (a) << " vs " << (b) << ")" << std::endl; \
    } } while(0)

#define EXPECT_NE(a, b) \
    do { if ((a) == (b)) { \
        puppy_test::current_test_failed() = true; \
        puppy_test::current_test_msg() = std::string(#a " == " #b " (expected !=)"); \
        std::cout << "  [EXPECT FAIL] " #a " == " #b " (expected !=)" \
                  << " (" << (a) << ")" << std::endl; \
    } } while(0)

#define EXPECT_EQ(a, b) \
    do { if ((a) != (b)) { \
        puppy_test::current_test_failed() = true; \
        puppy_test::current_test_msg() = std::string(#a " != " #b " (expected ==)"); \
        std::cout << "  [EXPECT FAIL] " #a " != " #b " (expected ==)" \
                  << " (" << (a) << " vs " << (b) << ")" << std::endl; \
    } } while(0)

#define RUN_ALL_TESTS() puppy_test::run_all_tests_impl()

namespace puppy_test {

struct TestCase {
    const char* suite;
    const char* name;
    void (*fn)();
};

inline std::vector<TestCase>& get_test_registry() {
    static std::vector<TestCase> registry;
    return registry;
}

inline int run_all_tests_impl() {
    int failures = 0;
    for (auto& tc : get_test_registry()) {
        reset_current();
        tc.fn();
        bool passed = !current_test_failed();
        record_result(tc.suite, tc.name, passed, current_test_msg());
        if (!passed) failures++;
        else std::cout << "[PASS] " << tc.suite << "::" << tc.name << std::endl;
    }
    std::cout << "\n==== " << pass_count() << "/" << test_count()
              << " tests passed";
    if (failures > 0) std::cout << " (" << failures << " FAILED)";
    std::cout << " ====" << std::endl;
    return failures > 0 ? 1 : 0;
}

} // namespace puppy_test
