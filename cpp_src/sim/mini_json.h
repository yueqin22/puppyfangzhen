// mini_json.h — 极简JSON解析器 (仅用于 scene_home.json 加载)
// ================================================================
// 支持类型: object, array, string, number, bool, null
// 不支持: 注释(标准JSON无注释), 尾逗号
// 用法:
//   auto v = mini_json::parse(R"({"a": [1, 2.5, "hi"]})");
//   double d = v["a"][0].as_double();  // 1.0
//   std::string s = v["a"][2].as_string(); // "hi"
#pragma once
#include <string>
#include <vector>
#include <map>
#include <variant>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <cmath>

namespace mini_json {

class Value;
using Object = std::map<std::string, Value>;
using Array  = std::vector<Value>;

class Value {
public:
    enum Type { Null, Bool, Number, String, Array_, Object_ };
    Type type = Null;
    bool b = false;
    double num = 0.0;
    std::string str;
    std::shared_ptr<mini_json::Array> arr;
    std::shared_ptr<mini_json::Object> obj;

    Value() = default;

    bool is_null()   const { return type == Null; }
    bool is_bool()   const { return type == Bool; }
    bool is_number() const { return type == Number; }
    bool is_string() const { return type == String; }
    bool is_array()  const { return type == Array_; }
    bool is_object() const { return type == Object_; }

    double as_double(double def = 0.0) const {
        return is_number() ? num : def;
    }
    int as_int(int def = 0) const {
        return is_number() ? (int)num : def;
    }
    bool as_bool(bool def = false) const {
        return is_bool() ? b : def;
    }
    const std::string& as_string(const std::string& def = "") const {
        return is_string() ? str : def;
    }

    const Value& operator[](const std::string& key) const {
        static Value null_val;
        if (is_object() && obj->count(key)) return obj->at(key);
        return null_val;
    }
    const Value& operator[](size_t i) const {
        static Value null_val;
        if (is_array() && i < arr->size()) return (*arr)[i];
        return null_val;
    }
    size_t size() const {
        return is_array() ? arr->size() : 0;
    }
};

// ===== 解析器 =====
class Parser {
    const char* p_;
    const char* end_;

    void skip_ws() {
        while (p_ < end_ && (*p_ == ' ' || *p_ == '\t' || *p_ == '\n' || *p_ == '\r')) p_++;
    }

    char peek() { skip_ws(); return p_ < end_ ? *p_ : '\0'; }
    char next() { skip_ws(); return p_ < end_ ? *p_++ : '\0'; }

    Value parse_value() {
        char c = peek();
        if (c == '{') return parse_object();
        if (c == '[') return parse_array();
        if (c == '"') return parse_string();
        if (c == 't' || c == 'f') return parse_bool();
        if (c == 'n') return parse_null();
        return parse_number();
    }

    Value parse_object() {
        Value v; v.type = Value::Object_; v.obj = std::make_shared<Object>();
        next(); // skip '{'
        if (peek() == '}') { next(); return v; }
        while (true) {
            std::string key = parse_string_raw();
            if (peek() != ':') throw std::runtime_error("expected ':'");
            next(); // skip ':'
            (*v.obj)[key] = parse_value();
            if (peek() == ',') { next(); continue; }
            if (peek() == '}') { next(); break; }
            throw std::runtime_error("expected ',' or '}'");
        }
        return v;
    }

    Value parse_array() {
        Value v; v.type = Value::Array_; v.arr = std::make_shared<Array>();
        next(); // skip '['
        if (peek() == ']') { next(); return v; }
        while (true) {
            v.arr->push_back(parse_value());
            if (peek() == ',') { next(); continue; }
            if (peek() == ']') { next(); break; }
            throw std::runtime_error("expected ',' or ']'");
        }
        return v;
    }

    Value parse_string() {
        Value v; v.type = Value::String; v.str = parse_string_raw();
        return v;
    }

    std::string parse_string_raw() {
        if (peek() != '"') throw std::runtime_error("expected '\"'");
        next(); // skip '"'
        std::string s;
        while (p_ < end_ && *p_ != '"') {
            if (*p_ == '\\') {
                p_++;
                if (p_ >= end_) break;
                switch (*p_) {
                    case 'n': s += '\n'; break;
                    case 't': s += '\t'; break;
                    case 'r': s += '\r'; break;
                    case '"': s += '"'; break;
                    case '\\': s += '\\'; break;
                    case '/': s += '/'; break;
                    case 'u': {
                        // 简单处理: 跳过4位hex (中文等)
                        for (int i = 0; i < 4 && p_ + 1 < end_; ++i) p_++;
                        s += '?'; break;
                    }
                    default: s += *p_; break;
                }
                p_++;
            } else {
                s += *p_++;
            }
        }
        if (p_ < end_) p_++; // skip closing '"'
        return s;
    }

    Value parse_number() {
        std::string s;
        while (p_ < end_) {
            char c = *p_;
            if (c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E' ||
                (c >= '0' && c <= '9')) {
                s += c; p_++;
            } else break;
        }
        Value v; v.type = Value::Number; v.num = std::stod(s);
        return v;
    }

    Value parse_bool() {
        Value v; v.type = Value::Bool;
        if (p_ + 4 <= end_ && strncmp(p_, "true", 4) == 0) {
            v.b = true; p_ += 4;
        } else if (p_ + 5 <= end_ && strncmp(p_, "false", 5) == 0) {
            v.b = false; p_ += 5;
        }
        return v;
    }

    Value parse_null() {
        if (p_ + 4 <= end_ && strncmp(p_, "null", 4) == 0) p_ += 4;
        return Value();
    }

public:
    Parser(const std::string& text) : p_(text.c_str()), end_(text.c_str() + text.size()) {}

    Value parse() {
        return parse_value();
    }
};

inline Value parse(const std::string& text) {
    Parser p(text);
    return p.parse();
}

inline Value parse_file(const std::string& path) {
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) throw std::runtime_error("Cannot open: " + path);
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    std::string text(sz, '\0');
    fread(&text[0], 1, sz, f);
    fclose(f);
    return parse(text);
}

} // namespace mini_json
