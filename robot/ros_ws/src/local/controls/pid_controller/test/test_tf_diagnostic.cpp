#include <gtest/gtest.h>
#include <pid_controller/tf_diagnostic.hpp>

TEST(TfDiagnostic, EscapesUntrustedExceptionAndFrameBytes) {
  EXPECT_EQ(pid_controller::bounded_json_string("a\"\\\n", 192), "\"a\\\"\\\\\\u000a\"");
  EXPECT_EQ(pid_controller::bounded_json_string(std::string(1, '\xff'), 512), "\"\\u00ff\"");
}

TEST(TfDiagnostic, BoundsRawBytesBeforeEscaping) {
  EXPECT_EQ(pid_controller::bounded_json_string("abcdef", 3), "\"abc\"");
  EXPECT_EQ(pid_controller::bounded_json_string("ignored", 0), "\"\"");
  EXPECT_LE(pid_controller::bounded_json_string(std::string(10000, '\n'), 512).size(), 3074u);
}
