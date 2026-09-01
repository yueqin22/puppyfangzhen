// mini_yaml.h — Minimal YAML parser (zero dependency)
// ====================================================
// Supports a subset of YAML sufficient for sim config:
//   - key: value (string, int, double, bool)
//   - comments (#)
//   - one level of nesting (section:)
//   - no lists, no anchors, no multi-line strings
//
// Usage:
//   auto cfg = mini_yaml::parse_file("config/sim.yaml");
//   bool use_amcl = cfg.get_bool("navigation.use_amcl", true);
//   double speed = cfg.get_double("flee.speed_close", 1.2);
#pragma once
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace mini_yaml {

class Config {
public:
    // Flat key-value store: "section.key" -> string value
    std::unordered_map<std::string, std::string> values_;

    // Get with fallback; key is "section.field" or "field"
    bool get_bool(const std::string& key, bool fallback) const {
        auto it = values_.find(key);
        if (it == values_.end()) return fallback;
        const std::string& v = it->second;
        return (v == "1" || v == "true" || v == "True" || v == "TRUE" || v == "yes");
    }

    int get_int(const std::string& key, int fallback) const {
        auto it = values_.find(key);
        if (it == values_.end()) return fallback;
        try { return std::stoi(it->second); } catch (...) { return fallback; }
    }

    double get_double(const std::string& key, double fallback) const {
        auto it = values_.find(key);
        if (it == values_.end()) return fallback;
        try { return std::stod(it->second); } catch (...) { return fallback; }
    }

    std::string get_string(const std::string& key, const std::string& fallback) const {
        auto it = values_.find(key);
        if (it == values_.end()) return fallback;
        return it->second;
    }

    bool has(const std::string& key) const {
        return values_.find(key) != values_.end();
    }
};

inline std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

// Parse YAML text into Config; keys are prefixed with section name
inline Config parse_text(const std::string& text) {
    Config cfg;
    std::string section = "";
    std::istringstream iss(text);
    std::string line;

    while (std::getline(iss, line)) {
        // Strip comments
        size_t hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        std::string trimmed = trim(line);
        if (trimmed.empty()) continue;

        // Detect section header (key: with no value, ends with ':')
        // Note: "key: value" has ': ' but "section:" has ':' at end
        size_t colon = trimmed.find(':');
        if (colon == std::string::npos) continue;

        std::string key = trim(trimmed.substr(0, colon));
        std::string val = trim(trimmed.substr(colon + 1));

        if (val.empty()) {
            // Section header
            section = key;
        } else {
            // Key-value pair
            std::string full_key = section.empty() ? key : (section + "." + key);
            cfg.values_[full_key] = val;
        }
    }
    return cfg;
}

// Parse YAML file; return empty Config on failure
inline Config parse_file(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) return Config();
    std::stringstream ss;
    ss << f.rdbuf();
    return parse_text(ss.str());
}

} // namespace mini_yaml
