// wheel_encoder_count_rad (ros2_controllers.yaml) is a COPY of the firmware's
// count size: the ESP32 computes the wheel position as
//   ((sign * count) / COUNTS_PER_OUTPUT_REV) * (2.0 * PI)
// (Software/microros/firmware/src/motor_controller.cpp), so one count is
// 2*pi / COUNTS_PER_OUTPUT_REV rad. This test reads the firmware SOURCES and
// fails when the two drift apart -- a firmware change to the encoder scaling
// must fail here, not silently mis-size HWR-30a's zero-count criterion.
//
// ASSUMPTION: the firmware tree sits at <this package>/../../../microros/firmware,
// i.e. Software/microros/firmware next to Software/ros2/src in the same checkout
// (true for the workspace and for the robot's checkout). CMake passes that path
// as GRIPPERX_FIRMWARE_DIR. If it is missing the test FAILS rather than skips:
// a skipped tie is indistinguishable from a satisfied one.

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <map>
#include <regex>
#include <sstream>
#include <string>

#include "active_config.hpp"

#ifndef GRIPPERX_FIRMWARE_DIR
#error "GRIPPERX_FIRMWARE_DIR must be defined by CMakeLists.txt"
#endif

namespace
{
std::string read_file(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    return {};
  }
  std::ostringstream out;
  out << in.rdbuf();
  return out.str();
}

/// `#define NAME <rest of line>` (comment stripped, whitespace collapsed).
std::map<std::string, std::string> defines(const std::string & text)
{
  std::map<std::string, std::string> out;
  const std::regex line(R"(^[ \t]*#define[ \t]+([A-Za-z_][A-Za-z0-9_]*)[ \t]+([^\n]*)$)");
  std::istringstream lines(text);
  std::string l;
  while (std::getline(lines, l)) {
    std::smatch m;
    if (std::regex_match(l, m, line)) {
      std::string value = m[2].str();
      const auto comment = value.find("//");
      if (comment != std::string::npos) {
        value.erase(comment);
      }
      std::string collapsed;
      for (char ch : value) {
        if (ch != ' ' && ch != '\t' && ch != '\r') {
          collapsed += ch;
        }
      }
      out[m[1].str()] = collapsed;
    }
  }
  return out;
}

std::string squash(const std::string & s)
{
  std::string out;
  for (char ch : s) {
    if (ch != ' ' && ch != '\t' && ch != '\r' && ch != '\n') {
      out += ch;
    }
  }
  return out;
}
}  // namespace

TEST(EncoderCountFirmware, YamlCountSizeEqualsTheFirmwareScaling)
{
  const std::string dir = GRIPPERX_FIRMWARE_DIR;
  const std::string header = read_file(dir + "/include/motor_controller.hpp");
  const std::string source = read_file(dir + "/src/motor_controller.cpp");
  ASSERT_FALSE(header.empty()) << "firmware header not found: " << dir
                               << "/include/motor_controller.hpp (see the ASSUMPTION above)";
  ASSERT_FALSE(source.empty()) << "firmware source not found: " << dir
                               << "/src/motor_controller.cpp";

  // The derivation this test evaluates. If the firmware changes HOW the count
  // size is built, this fails and the test has to be re-read, not guessed.
  const auto d = defines(header);
  ASSERT_EQ(d.count("COUNTS_PER_OUTPUT_REV"), 1u);
  ASSERT_EQ(d.at("COUNTS_PER_OUTPUT_REV"), "(ENCODER_CPR_PER_CHANNEL*4.0*GEAR_RATIO)");
  ASSERT_EQ(d.count("ENCODER_CPR_PER_CHANNEL"), 1u);
  ASSERT_EQ(d.count("GEAR_RATIO"), 1u);
  const double cpr = std::stod(d.at("ENCODER_CPR_PER_CHANNEL"));
  const double gear = std::stod(d.at("GEAR_RATIO"));
  const double counts_per_rev = cpr * 4.0 * gear;
  ASSERT_GT(counts_per_rev, 0.0);

  // ...and the position formula that makes one count 2*pi/COUNTS_PER_OUTPUT_REV.
  EXPECT_NE(
    squash(source).find("/COUNTS_PER_OUTPUT_REV)*(2.0*PI)"), std::string::npos)
    << "motor_controller.cpp no longer computes the position as "
       "(count / COUNTS_PER_OUTPUT_REV) * (2.0 * PI); re-derive wheel_encoder_count_rad";

  const double expected = 2.0 * 3.14159265358979323846 / counts_per_rev;
  const double yaml = active_config::stall_config().count_rad;
  EXPECT_NEAR(yaml, expected, 1e-12)
    << "wheel_encoder_count_rad = " << yaml << " but the firmware's count is 2*pi/"
    << counts_per_rev << " = " << expected;
}
