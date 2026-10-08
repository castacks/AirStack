#pragma once

#include <cstddef>
#include <string>

namespace pid_controller {

// Bound raw bytes before escaping. Diagnostic strings never become control inputs.
inline std::string bounded_json_string(const std::string &value, std::size_t limit) {
  constexpr char hex[] = "0123456789abcdef";
  std::string result = "\"";
  for (std::size_t i = 0; i < value.size() && i < limit; ++i) {
    const unsigned char c = static_cast<unsigned char>(value[i]);
    if (c == '"' || c == '\\') {
      result += '\\';
      result += static_cast<char>(c);
    } else if (c < 0x20 || c >= 0x7f) {
      // ASCII-only output also keeps truncated multi-byte messages valid JSON.
      result += "\\u00";
      result += hex[c >> 4];
      result += hex[c & 15];
    } else {
      result += static_cast<char>(c);
    }
  }
  result += '"';
  return result;
}

}  // namespace pid_controller
