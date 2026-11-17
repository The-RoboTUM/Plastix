// GripperX: unit tests of the /scan stamp computation (scan_stamp.h).
#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "scan_stamp.h"

using scan_stamp::ComputeScanTiming;
using scan_stamp::ScanTiming;
using scan_stamp::StampFault;
using scan_stamp::StampSource;

namespace
{

constexpr int64_t kT0 = 1'762'300'000'000'000'000LL;  // arbitrary epoch ns
constexpr double kSpin = 10.0;                          // Hz

// A sweep as lipkg.cpp builds it: 12-point packets, point i of a packet stamped
// prev_arrival + i * (arrival - prev_arrival) / 11, angles rising from 0 deg.
ldlidar::Points2D MakeSweep(int packets, int64_t start_ns, int64_t packet_ns)
{
  ldlidar::Points2D out;
  const int n = packets * 12;
  for (int k = 0; k < packets; ++k) {
    const int64_t prev = start_ns + k * packet_ns;
    for (int i = 0; i < 12; ++i) {
      const int idx = k * 12 + i;
      const float angle = 360.0f * static_cast<float>(idx) / static_cast<float>(n);
      const uint64_t stamp = static_cast<uint64_t>(prev + (packet_ns * i) / 11);
      out.push_back(ldlidar::PointData(angle, 1000, 200, stamp));
    }
  }
  return out;
}

// 38 packets * 12 = 456 points over ~0.1 s, like an LD06 at 10 Hz / 4500 pts/s.
struct Fixture
{
  ldlidar::Points2D sweep = MakeSweep(38, kT0, 2'631'579);
  int beams = static_cast<int>(sweep.size());
  int64_t first = static_cast<int64_t>(sweep.front().stamp);
  int64_t last = static_cast<int64_t>(sweep.back().stamp);
  int64_t span = last - first;
  int64_t publish = last + 40'000'000;  // picked up 40 ms after the last point
  double interval = 0.1;
};

void ExpectLegacy(const ScanTiming & t, const Fixture & f)
{
  EXPECT_EQ(t.stamp_ns, f.publish);
  EXPECT_FLOAT_EQ(t.scan_time, static_cast<float>(f.interval));
  EXPECT_FLOAT_EQ(t.time_increment, static_cast<float>(f.interval / (f.beams - 1)));
}

}  // namespace

TEST(ScanStamp, ParseNames) {
  StampSource s;
  ASSERT_TRUE(scan_stamp::ParseStampSource("publish", &s));
  EXPECT_EQ(s, StampSource::kPublish);
  ASSERT_TRUE(scan_stamp::ParseStampSource("first_ray", &s));
  EXPECT_EQ(s, StampSource::kFirstRay);
  ASSERT_TRUE(scan_stamp::ParseStampSource("mid_sweep", &s));
  EXPECT_EQ(s, StampSource::kMidSweep);
  EXPECT_FALSE(scan_stamp::ParseStampSource("midsweep", &s));
  EXPECT_FALSE(scan_stamp::ParseStampSource("", &s));
}

TEST(ScanStamp, FixtureIsPlausible) {
  Fixture f;
  EXPECT_EQ(f.beams, 456);
  EXPECT_NEAR(f.span * 1e-9, 0.1, 0.001);
  EXPECT_EQ(scan_stamp::CheckPointStamps(f.sweep, kSpin, f.publish), StampFault::kNone);
}

TEST(ScanStamp, PublishIsLegacyEvenWithBadStamps) {
  for (bool mirrored : {true, false}) {
    Fixture f;
    ScanTiming t = ComputeScanTiming(StampSource::kPublish, f.sweep, f.beams, mirrored, kSpin,
      f.publish, f.interval);
    EXPECT_EQ(t.fault, StampFault::kNone);
    ExpectLegacy(t, f);
    f.sweep[5].stamp = 0;  // publish never looks at the points
    t = ComputeScanTiming(StampSource::kPublish, f.sweep, f.beams, mirrored, kSpin,
      f.publish, f.interval);
    EXPECT_EQ(t.fault, StampFault::kNone);
    ExpectLegacy(t, f);
  }
}

TEST(ScanStamp, MidSweepMirrored) {
  Fixture f;
  ScanTiming t = ComputeScanTiming(StampSource::kMidSweep, f.sweep, f.beams, true, kSpin,
    f.publish, f.interval);
  EXPECT_EQ(t.fault, StampFault::kNone);
  EXPECT_EQ(t.stamp_ns, f.first + f.span / 2);
  EXPECT_LT(t.time_increment, 0.0f);
  EXPECT_NEAR(t.time_increment, -f.span * 1e-9 / (f.beams - 1), 1e-9);
  EXPECT_NEAR(t.scan_time, f.span * 1e-9 * f.beams / (f.beams - 1), 1e-7);
  // Older than the publish stamp by the wait plus half the sweep.
  EXPECT_EQ(f.publish - t.stamp_ns, 40'000'000 + (f.span - f.span / 2));
}

TEST(ScanStamp, MidSweepNotMirrored) {
  Fixture f;
  ScanTiming t = ComputeScanTiming(StampSource::kMidSweep, f.sweep, f.beams, false, kSpin,
    f.publish, f.interval);
  EXPECT_EQ(t.fault, StampFault::kNone);
  EXPECT_EQ(t.stamp_ns, f.first + f.span / 2);
  EXPECT_GT(t.time_increment, 0.0f);
  EXPECT_NEAR(t.time_increment, f.span * 1e-9 / (f.beams - 1), 1e-9);
}

TEST(ScanStamp, FirstRayFollowsBeamOrder) {
  Fixture f;
  // Mirrored: ranges[0] is the last acquired point; stamp + i * ti walks back to the first.
  ScanTiming m = ComputeScanTiming(StampSource::kFirstRay, f.sweep, f.beams, true, kSpin,
    f.publish, f.interval);
  EXPECT_EQ(m.fault, StampFault::kNone);
  EXPECT_EQ(m.stamp_ns, f.last);
  const double end_m = m.stamp_ns * 1e-9 + (f.beams - 1) * static_cast<double>(m.time_increment);
  EXPECT_NEAR(end_m, f.first * 1e-9, 1e-6);
  // Not mirrored: ranges[0] is the first acquired point.
  ScanTiming n = ComputeScanTiming(StampSource::kFirstRay, f.sweep, f.beams, false, kSpin,
    f.publish, f.interval);
  EXPECT_EQ(n.stamp_ns, f.first);
  const double end_n = n.stamp_ns * 1e-9 + (f.beams - 1) * static_cast<double>(n.time_increment);
  EXPECT_NEAR(end_n, f.last * 1e-9, 1e-6);
  EXPECT_FLOAT_EQ(m.scan_time, n.scan_time);
  EXPECT_FLOAT_EQ(m.time_increment, -n.time_increment);
}

TEST(ScanStamp, StampsIndependentOfWait) {
  // The point of the fix: the loop's wait L must not move the stamp.
  Fixture f;
  for (int64_t wait_ns : {0LL, 33'000'000LL, 99'000'000LL}) {
    ScanTiming t = ComputeScanTiming(StampSource::kMidSweep, f.sweep, f.beams, true, kSpin,
      f.last + wait_ns, f.interval);
    EXPECT_EQ(t.stamp_ns, f.first + f.span / 2);
  }
}

namespace
{

void ExpectFallback(const Fixture & f, StampFault want, double spin = kSpin)
{
  for (StampSource src : {StampSource::kFirstRay, StampSource::kMidSweep}) {
    for (bool mirrored : {true, false}) {
      ScanTiming t = ComputeScanTiming(src, f.sweep, f.beams, mirrored, spin, f.publish,
        f.interval);
      EXPECT_EQ(t.fault, want) << scan_stamp::StampSourceName(src) << " mirrored=" << mirrored;
      // Dropped, not re-stamped with the publish time.
      scan_stamp::ScanStamper stamper(src);
      ScanTiming out;
      EXPECT_EQ(
        stamper.Process(f.sweep, f.beams, mirrored, spin, f.publish, f.interval, &out),
        scan_stamp::Verdict::kDropBadStamps);
      EXPECT_EQ(stamper.dropped_bad_stamps(), 1u);
      EXPECT_EQ(stamper.dropped_not_increasing(), 0u);
    }
  }
}

}  // namespace

TEST(ScanStampFallback, ZeroStamp) {
  Fixture f;
  f.sweep[100].stamp = 0;
  ExpectFallback(f, StampFault::kZeroStamp);
}

TEST(ScanStampFallback, NonMonotonic) {
  Fixture f;
  std::swap(f.sweep[100].stamp, f.sweep[200].stamp);
  ExpectFallback(f, StampFault::kNonMonotonic);
}

TEST(ScanStampFallback, EqualNeighboursAreAccepted) {
  Fixture f;  // MakeSweep already repeats every packet boundary stamp
  ASSERT_EQ(f.sweep[11].stamp, f.sweep[12].stamp);
  EXPECT_EQ(scan_stamp::CheckPointStamps(f.sweep, kSpin, f.publish), StampFault::kNone);
}

TEST(ScanStampFallback, SpanTooShort) {
  Fixture f;
  f.sweep = MakeSweep(38, kT0, 1'000'000);  // 38 ms for a 100 ms revolution
  f.beams = static_cast<int>(f.sweep.size());
  f.publish = static_cast<int64_t>(f.sweep.back().stamp) + 1'000'000;
  ExpectFallback(f, StampFault::kSpanImplausible);
}

TEST(ScanStampFallback, SpanTooLong) {
  Fixture f;
  f.sweep.back().stamp += 80'000'000;  // 0.18 s, e.g. a clock step inside the sweep
  f.publish = static_cast<int64_t>(f.sweep.back().stamp) + 1'000'000;
  ExpectFallback(f, StampFault::kSpanImplausible);
}

TEST(ScanStampFallback, SpinRateUnknown) {
  Fixture f;
  ExpectFallback(f, StampFault::kSpanImplausible, 0.0);
}

TEST(ScanStampFallback, AfterPublish) {
  Fixture f;
  f.publish = f.last - 1;
  ExpectFallback(f, StampFault::kAfterPublish);
}

TEST(ScanStampFallback, TooOld) {
  Fixture f;
  f.publish = f.last + 1'500'000'000LL;
  ExpectFallback(f, StampFault::kTooOld);
}

TEST(ScanStampFallback, TooFewPoints) {
  Fixture f;
  f.sweep.resize(1);
  f.beams = 1;
  for (StampSource src : {StampSource::kFirstRay, StampSource::kMidSweep}) {
    ScanTiming t = ComputeScanTiming(src, f.sweep, f.beams, true, kSpin, f.publish, f.interval);
    EXPECT_EQ(t.fault, StampFault::kNoPoints);
    scan_stamp::ScanStamper stamper(src);
    ScanTiming out;
    EXPECT_EQ(
      stamper.Process(f.sweep, f.beams, true, kSpin, f.publish, f.interval, &out),
      scan_stamp::Verdict::kDropBadStamps);
  }
}

// ---- Sequences of consecutive scans (ScanStamper) ----

namespace
{

using scan_stamp::ScanStamper;
using scan_stamp::Verdict;

constexpr int64_t kPeriod = 100'000'000;  // 10 Hz
constexpr int64_t kPacket = 2'631'579;    // 38 packets per revolution

// Scan k of a continuous run: sweep k starts at kT0 + k * kPeriod.
struct Scan
{
  ldlidar::Points2D sweep;
  int64_t publish;
};

Scan MakeScan(int k, int64_t wait_ns = 40'000'000)
{
  Scan s;
  s.sweep = MakeSweep(38, kT0 + k * kPeriod, kPacket);
  s.publish = static_cast<int64_t>(s.sweep.back().stamp) + wait_ns;
  return s;
}

Verdict Feed(ScanStamper & st, const Scan & s, ScanTiming * out)
{
  return st.Process(s.sweep, static_cast<int>(s.sweep.size()), true, kSpin, s.publish, 0.1, out);
}

}  // namespace

TEST(ScanStampSequence, GoodBadGoodNeverGoesBackwards) {
  // The audit case: a bad scan stamped at publish time would lie after the next good
  // mid-sweep stamp. It must be dropped, and the published stamps must rise.
  for (StampSource src : {StampSource::kFirstRay, StampSource::kMidSweep}) {
    ScanStamper st(src);
    ScanTiming t;
    std::vector<int64_t> published;
    for (int k = 0; k < 5; ++k) {
      Scan s = MakeScan(k, k == 2 ? 90'000'000 : 40'000'000);
      if (k == 2) {
        s.sweep[100].stamp = 0;
      }
      const Verdict v = Feed(st, s, &t);
      if (k == 2) {
        EXPECT_EQ(v, Verdict::kDropBadStamps);
      } else {
        ASSERT_EQ(v, Verdict::kPublish) << k;
        published.push_back(t.stamp_ns);
      }
    }
    ASSERT_EQ(published.size(), 4u);
    for (size_t i = 1; i < published.size(); ++i) {
      EXPECT_GT(published[i], published[i - 1]);
    }
    EXPECT_EQ(st.scans(), 5u);
    EXPECT_EQ(st.dropped_bad_stamps(), 1u);
    EXPECT_EQ(st.dropped_not_increasing(), 0u);
  }
}

TEST(ScanStampSequence, DuplicateFrameIsDropped) {
  // LiPkg::GetLaserScanData can hand out the same frame twice: identical stamps.
  ScanStamper st(StampSource::kMidSweep);
  ScanTiming t;
  Scan a = MakeScan(0);
  ASSERT_EQ(Feed(st, a, &t), Verdict::kPublish);
  const int64_t first_stamp = t.stamp_ns;
  Scan again = a;
  again.publish += kPeriod;  // picked up one loop tick later
  EXPECT_EQ(Feed(st, again, &t), Verdict::kDropNotIncreasing);
  EXPECT_EQ(Feed(st, MakeScan(1), &t), Verdict::kPublish);
  EXPECT_GT(t.stamp_ns, first_stamp);
  EXPECT_EQ(st.dropped_not_increasing(), 1u);
  EXPECT_EQ(st.dropped_bad_stamps(), 0u);
}

TEST(ScanStampSequence, LargeBackwardClockStepRecoversWithinOneScan) {
  // Clock steps forward 60 s, then back (chrony on a Pi without RTC): the first scan
  // after the back step is published and the run continues, no 60 s outage.
  for (StampSource src : {StampSource::kFirstRay, StampSource::kMidSweep}) {
    ScanStamper st(src);
    ScanTiming t;
    ASSERT_EQ(Feed(st, MakeScan(0), &t), Verdict::kPublish);
    ASSERT_EQ(Feed(st, MakeScan(600), &t), Verdict::kPublish);  // +60 s
    const int64_t jumped = t.stamp_ns;
    ASSERT_EQ(Feed(st, MakeScan(1), &t), Verdict::kPublishAfterClockStep);
    EXPECT_EQ(st.clock_step_resets(), 1u);
    EXPECT_EQ(st.last_clock_step_ns(), jumped - t.stamp_ns);
    EXPECT_NEAR(st.last_clock_step_ns() * 1e-9, 59.9, 1e-6);
    for (int k = 2; k < 12; ++k) {
      EXPECT_EQ(Feed(st, MakeScan(k), &t), Verdict::kPublish) << k;
    }
    EXPECT_EQ(st.dropped_not_increasing(), 0u);
    EXPECT_EQ(st.clock_step_resets(), 1u);
  }
}

TEST(ScanStampSequence, SmallBackwardStepIsDropped) {
  // Stamps 0.5 s back (below kClockStepThreshold_s): dropped, no reset.
  ScanStamper st(StampSource::kMidSweep);
  ScanTiming t;
  ASSERT_EQ(Feed(st, MakeScan(10), &t), Verdict::kPublish);
  const int64_t before = t.stamp_ns;
  EXPECT_EQ(Feed(st, MakeScan(5), &t), Verdict::kDropNotIncreasing);
  EXPECT_EQ(Feed(st, MakeScan(10), &t), Verdict::kDropNotIncreasing);  // equal is not later
  EXPECT_EQ(Feed(st, MakeScan(11), &t), Verdict::kPublish);
  EXPECT_GT(t.stamp_ns, before);
  EXPECT_EQ(st.dropped_not_increasing(), 2u);
  EXPECT_EQ(st.clock_step_resets(), 0u);
}

TEST(ScanStampSequence, StepAtThresholdIsDroppedJustAboveIsReset) {
  ScanStamper st(StampSource::kMidSweep);
  ScanTiming t;
  ASSERT_EQ(Feed(st, MakeScan(20), &t), Verdict::kPublish);
  EXPECT_EQ(Feed(st, MakeScan(10), &t), Verdict::kDropNotIncreasing);  // exactly 1.0 s back
  EXPECT_EQ(st.clock_step_resets(), 0u);
  EXPECT_EQ(Feed(st, MakeScan(9), &t), Verdict::kPublishAfterClockStep);  // 1.1 s back
  EXPECT_EQ(st.clock_step_resets(), 1u);
  EXPECT_EQ(st.dropped_not_increasing(), 1u);
}

TEST(ScanStampSequence, PublishModeNeverDrops) {
  // Legacy, bit for bit: bad stamps, duplicates and backward steps are all published
  // with the publish-time stamp.
  ScanStamper st(StampSource::kPublish);
  ScanTiming t;
  Scan bad = MakeScan(3);
  bad.sweep[7].stamp = 0;
  for (const Scan & s : {MakeScan(10), MakeScan(5), MakeScan(5), bad}) {
    EXPECT_EQ(Feed(st, s, &t), Verdict::kPublish);
    EXPECT_EQ(t.stamp_ns, s.publish);
    EXPECT_FLOAT_EQ(t.scan_time, 0.1f);
  }
  EXPECT_EQ(st.dropped_bad_stamps(), 0u);
  EXPECT_EQ(st.dropped_not_increasing(), 0u);
}

// The guards compare per-point stamps with node->now(): both must be one clock.
TEST(ScanStampClock, PointClockMatchesNodeNow) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("scan_stamp_clock_test");
  const int64_t a = static_cast<int64_t>(scan_stamp::SystemTimeStampNs());
  const int64_t now = node->now().nanoseconds();
  const int64_t b = static_cast<int64_t>(scan_stamp::SystemTimeStampNs());
  EXPECT_EQ(node->get_clock()->get_clock_type(), RCL_ROS_TIME);
  EXPECT_LE(a, now);
  EXPECT_LE(now, b);
  EXPECT_LT(b - a, 50'000'000);
  rclcpp::shutdown();
}
