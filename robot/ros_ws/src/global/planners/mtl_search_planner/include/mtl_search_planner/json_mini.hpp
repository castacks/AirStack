// =============================================================================
//  mtl_search_planner/json_mini.hpp  (vendored from cpp_planner/apps/json_mini.hpp;
//  only change: #include <limits>)
//
//  A small JSON reader/writer, used only by the mtl_plan tool.
//
//  WHY NOT nlohmann/json.  The planner library's entire dependency list is
//  "Eigen", and that is a feature: a host simulation vendors mtl::planner
//  without inheriting anything.  A command-line adapter is not a good reason to
//  add a dependency to the repository, and the subset of JSON a scenario file
//  needs - objects, arrays, numbers, strings, booleans, null - is a couple of
//  hundred lines of recursive descent.  It is deliberately strict: a malformed
//  scenario throws with a byte offset rather than silently parsing as an empty
//  object and producing an empty plan.
// =============================================================================
#ifndef MTL_APPS_JSON_MINI_HPP
#define MTL_APPS_JSON_MINI_HPP

#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace jsonmini {

class Value;
using Object = std::map<std::string, Value>;
using Array  = std::vector<Value>;

enum class Type { Null, Bool, Number, String, Array, Object };

class Value {
public:
    Value() = default;
    Value(bool b) : type_(Type::Bool), bool_(b) {}                        // NOLINT
    Value(double d) : type_(Type::Number), num_(d) {}                     // NOLINT
    Value(int i) : type_(Type::Number), num_(static_cast<double>(i)) {}   // NOLINT
    Value(const char* s) : type_(Type::String), str_(s) {}                // NOLINT
    Value(std::string s) : type_(Type::String), str_(std::move(s)) {}     // NOLINT
    Value(Array a) : type_(Type::Array), arr_(std::move(a)) {}            // NOLINT
    Value(Object o) : type_(Type::Object), obj_(std::move(o)) {}          // NOLINT

    Type type() const { return type_; }
    bool isNull() const { return type_ == Type::Null; }
    bool isObject() const { return type_ == Type::Object; }
    bool isArray() const { return type_ == Type::Array; }
    bool isNumber() const { return type_ == Type::Number; }

    const Array&  array() const { expect(Type::Array); return arr_; }
    const Object& object() const { expect(Type::Object); return obj_; }
    const std::string& str() const { expect(Type::String); return str_; }
    double number() const { expect(Type::Number); return num_; }
    bool   boolean() const { expect(Type::Bool); return bool_; }

    /// Member lookup.  Returns a null Value when absent, so `get(...)` chains
    /// read as "this field, if the scenario bothered to supply it".
    const Value& operator[](const std::string& key) const {
        static const Value kNull;
        if (type_ != Type::Object) return kNull;
        const auto it = obj_.find(key);
        return it == obj_.end() ? kNull : it->second;
    }

    bool has(const std::string& key) const {
        return type_ == Type::Object && obj_.find(key) != obj_.end();
    }

    // -- typed reads with a default -------------------------------------- //
    double num(double fallback) const { return type_ == Type::Number ? num_ : fallback; }
    bool   flag(bool fallback) const { return type_ == Type::Bool ? bool_ : fallback; }
    std::string text(const std::string& fallback) const {
        return type_ == Type::String ? str_ : fallback;
    }
    /// JSON has no infinity, so a null / absent budget field means "unbounded".
    double numOrInf() const {
        return type_ == Type::Number ? num_ : std::numeric_limits<double>::infinity();
    }

    std::vector<double> numbers() const {
        std::vector<double> out;
        if (type_ != Type::Array) return out;
        out.reserve(arr_.size());
        for (const Value& v : arr_) out.push_back(v.number());
        return out;
    }

    // -- writing ---------------------------------------------------------- //
    void dump(std::ostream& os) const {
        switch (type_) {
            case Type::Null: os << "null"; break;
            case Type::Bool: os << (bool_ ? "true" : "false"); break;
            case Type::Number: dumpNumber(os, num_); break;
            case Type::String: dumpString(os, str_); break;
            case Type::Array: {
                os << '[';
                for (std::size_t i = 0; i < arr_.size(); ++i) {
                    if (i) os << ',';
                    arr_[i].dump(os);
                }
                os << ']';
                break;
            }
            case Type::Object: {
                os << '{';
                bool first = true;
                for (const auto& kv : obj_) {
                    if (!first) os << ',';
                    first = false;
                    dumpString(os, kv.first);
                    os << ':';
                    kv.second.dump(os);
                }
                os << '}';
                break;
            }
        }
    }

    std::string dump() const {
        std::ostringstream os;
        dump(os);
        return os.str();
    }

private:
    void expect(Type t) const {
        if (type_ != t) throw std::runtime_error("json: value has the wrong type");
    }

    static void dumpNumber(std::ostream& os, double d) {
        // JSON has no NaN or Infinity literals; emitting them produces a file
        // Python's json module refuses. null is the honest encoding of both.
        if (!std::isfinite(d)) { os << "null"; return; }
        if (d == static_cast<double>(static_cast<long long>(d)) &&
            std::abs(d) < 1e15) {
            os << static_cast<long long>(d);
            return;
        }
        std::ostringstream tmp;
        tmp << std::setprecision(10) << d;
        os << tmp.str();
    }

    static void dumpString(std::ostream& os, const std::string& s) {
        os << '"';
        for (const char c : s) {
            switch (c) {
                case '"': os << "\\\""; break;
                case '\\': os << "\\\\"; break;
                case '\n': os << "\\n"; break;
                case '\r': os << "\\r"; break;
                case '\t': os << "\\t"; break;
                default:
                    if (static_cast<unsigned char>(c) < 0x20) {
                        os << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                           << static_cast<int>(c) << std::dec << std::setfill(' ');
                    } else {
                        os << c;
                    }
            }
        }
        os << '"';
    }

    Type        type_ = Type::Null;
    bool        bool_ = false;
    double      num_  = 0.0;
    std::string str_;
    Array       arr_;
    Object      obj_;
};

// --------------------------------------------------------------------------- //
class Parser {
public:
    explicit Parser(const std::string& text) : s_(text) {}

    Value parse() {
        skip();
        Value v = parseValue();
        skip();
        if (i_ != s_.size()) fail("trailing characters after the top-level value");
        return v;
    }

private:
    [[noreturn]] void fail(const std::string& what) const {
        throw std::runtime_error("json parse error at byte " + std::to_string(i_) + ": " + what);
    }

    void skip() {
        while (i_ < s_.size() &&
               (s_[i_] == ' ' || s_[i_] == '\t' || s_[i_] == '\n' || s_[i_] == '\r')) {
            ++i_;
        }
    }

    char peek() const {
        if (i_ >= s_.size()) throw std::runtime_error("json parse error: unexpected end of input");
        return s_[i_];
    }

    bool literal(const char* word) {
        const std::size_t n = std::string(word).size();
        if (s_.compare(i_, n, word) != 0) return false;
        i_ += n;
        return true;
    }

    Value parseValue() {
        switch (peek()) {
            case '{': return parseObject();
            case '[': return parseArray();
            case '"': return Value(parseString());
            case 't': if (literal("true")) return Value(true); fail("expected 'true'");
            case 'f': if (literal("false")) return Value(false); fail("expected 'false'");
            case 'n': if (literal("null")) return Value(); fail("expected 'null'");
            default: return Value(parseNumber());
        }
    }

    Value parseObject() {
        Object out;
        ++i_;  // '{'
        skip();
        if (peek() == '}') { ++i_; return Value(std::move(out)); }
        for (;;) {
            skip();
            if (peek() != '"') fail("object keys must be strings");
            std::string key = parseString();
            skip();
            if (peek() != ':') fail("expected ':' after an object key");
            ++i_;
            skip();
            out.emplace(std::move(key), parseValue());
            skip();
            const char c = peek();
            if (c == ',') { ++i_; continue; }
            if (c == '}') { ++i_; break; }
            fail("expected ',' or '}' in an object");
        }
        return Value(std::move(out));
    }

    Value parseArray() {
        Array out;
        ++i_;  // '['
        skip();
        if (peek() == ']') { ++i_; return Value(std::move(out)); }
        for (;;) {
            skip();
            out.push_back(parseValue());
            skip();
            const char c = peek();
            if (c == ',') { ++i_; continue; }
            if (c == ']') { ++i_; break; }
            fail("expected ',' or ']' in an array");
        }
        return Value(std::move(out));
    }

    std::string parseString() {
        ++i_;  // opening quote
        std::string out;
        while (i_ < s_.size()) {
            const char c = s_[i_++];
            if (c == '"') return out;
            if (c != '\\') { out.push_back(c); continue; }
            if (i_ >= s_.size()) fail("unterminated escape");
            const char esc = s_[i_++];
            switch (esc) {
                case '"': out.push_back('"'); break;
                case '\\': out.push_back('\\'); break;
                case '/': out.push_back('/'); break;
                case 'b': out.push_back('\b'); break;
                case 'f': out.push_back('\f'); break;
                case 'n': out.push_back('\n'); break;
                case 'r': out.push_back('\r'); break;
                case 't': out.push_back('\t'); break;
                case 'u': {
                    if (i_ + 4 > s_.size()) fail("truncated \\u escape");
                    // Scenario files are ASCII in practice; keep the codepoint if
                    // it fits in a byte and substitute '?' otherwise rather than
                    // pretending to do UTF-16 surrogate decoding.
                    const int cp = std::stoi(s_.substr(i_, 4), nullptr, 16);
                    i_ += 4;
                    out.push_back(cp < 0x80 ? static_cast<char>(cp) : '?');
                    break;
                }
                default: fail("unknown escape sequence");
            }
        }
        fail("unterminated string");
    }

    double parseNumber() {
        const std::size_t start = i_;
        if (i_ < s_.size() && (s_[i_] == '-' || s_[i_] == '+')) ++i_;
        while (i_ < s_.size() && (std::isdigit(static_cast<unsigned char>(s_[i_])) ||
                                  s_[i_] == '.' || s_[i_] == 'e' || s_[i_] == 'E' ||
                                  s_[i_] == '+' || s_[i_] == '-')) {
            ++i_;
        }
        if (i_ == start) fail("expected a value");
        try {
            return std::stod(s_.substr(start, i_ - start));
        } catch (const std::exception&) {
            fail("malformed number");
        }
    }

    const std::string& s_;
    std::size_t        i_ = 0;
};

inline Value parse(const std::string& text) { return Parser(text).parse(); }

}  // namespace jsonmini

#endif  // MTL_APPS_JSON_MINI_HPP
