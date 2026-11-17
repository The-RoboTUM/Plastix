#include "gripperx_swerve_controller/drive_fault_monitor.hpp"

#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>

namespace gripperx_swerve_controller
{

const char * drive_fault_reason_label(int reason)
{
  switch (reason) {
    case kDriveFaultNone:
      return "NONE";
    case kDriveFaultEncoderMissing:
      return "ENCODER_MISSING";
    case kDriveFaultStallRetrip:
      return "STALL_RETRIP";
    case kDriveFaultStallTripCount:
      return "STALL_TRIP_COUNT";
    case kDriveFaultStallMultiWheel:
      return "STALL_MULTI_WHEEL";
    case kDriveFaultStallLatchedHeld:
      return "STALL_LATCHED_HELD";
    case kDriveFaultFeedbackLost:
      return "FEEDBACK_LOST";
    default:
      return "UNKNOWN_REASON";
  }
}

bool drive_fault_reason_clearable(int reason)
{
  for (const int clearable : kOperatorClearableDriveFaultReasons) {
    if (reason == clearable) {
      return true;
    }
  }
  return false;
}

std::string serialize_drive_fault(const DriveFaultRecord & r)
{
  std::ostringstream out;
  out.precision(17);
  out << "format=gripperx_drive_fault_v1\n"
      << "reason=" << r.reason << "\n"
      << "reason_text=" << drive_fault_reason_label(r.reason) << "\n"
      << "wheel=" << r.wheel << "\n"
      << "other_wheel=" << r.other_wheel << "\n"
      << "trip_time_sec=" << r.trip_time_sec << "\n"
      << "commanded_rad_s=" << r.commanded_rad_s << "\n"
      << "provenance=" << r.provenance << "\n"
      << "accumulated_sec=" << r.accumulated_sec << "\n"
      << "stall_trip_count=" << r.stall_trip_count << "\n"
      << "since_release_sec=" << r.since_release_sec << "\n"
      << "latched_held_sec=" << r.latched_held_sec << "\n"
      << "zero_count=" << (r.zero_count ? 1 : 0) << "\n";
  return out.str();
}

bool parse_drive_fault(const std::string & text, DriveFaultRecord & record)
{
  DriveFaultRecord r;
  r.active = true;
  bool format_ok = false;
  bool reason_ok = false;
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    const auto eq = line.find('=');
    if (eq == std::string::npos) {
      continue;
    }
    const std::string key = line.substr(0, eq);
    const std::string value = line.substr(eq + 1);
    const char * v = value.c_str();
    char * end = nullptr;
    if (key == "format") {
      format_ok = value == "gripperx_drive_fault_v1";
      continue;
    }
    const double number = std::strtod(v, &end);
    if (end == v) {
      continue;  // text fields (reason_text) and garbage alike
    }
    if (key == "reason") {
      r.reason = static_cast<int>(number);
      reason_ok = r.reason >= kDriveFaultEncoderMissing && r.reason <= kDriveFaultFeedbackLost;
    } else if (key == "wheel") {
      r.wheel = static_cast<int>(number);
    } else if (key == "other_wheel") {
      r.other_wheel = static_cast<int>(number);
    } else if (key == "trip_time_sec") {
      r.trip_time_sec = number;
    } else if (key == "commanded_rad_s") {
      r.commanded_rad_s = number;
    } else if (key == "provenance") {
      r.provenance = static_cast<int>(number);
    } else if (key == "accumulated_sec") {
      r.accumulated_sec = number;
    } else if (key == "stall_trip_count") {
      r.stall_trip_count = static_cast<uint32_t>(number);
    } else if (key == "since_release_sec") {
      r.since_release_sec = number;
    } else if (key == "latched_held_sec") {
      r.latched_held_sec = number;
    } else if (key == "zero_count") {
      // Absent in markers written before 2026-10-22: stays false.
      r.zero_count = number != 0.0;
    }
  }
  if (!format_ok || !reason_ok) {
    record = DriveFaultRecord{};
    record.active = true;
    record.reason = kDriveFaultNone;
    return false;
  }
  record = r;
  return true;
}

DriveFaultMonitor::DriveFaultMonitor(const DriveFaultConfig & config) { configure(config); }

namespace
{
// A YAML can carry .nan / .inf. An infinite encoder_confirm_window_sec would
// leave trigger 1 (SR-18) armed in name and unable to fire; a NaN classification
// window would never call anything "half-way". Every check fails for both.
bool finite_at_least(double value, double low, bool strict, const char * name, std::string & error)
{
  if (std::isfinite(value) && (strict ? value > low : value >= low)) {
    return true;
  }
  error = std::string(name) + (strict ? " must be finite and > 0" : " must be finite and >= 0");
  return false;
}
}  // namespace

bool DriveFaultMonitor::validate(const DriveFaultConfig & config, std::string & error)
{
  return
    finite_at_least(
      config.encoder_confirm_min_command_rad_s, 0.0, true, "encoder_confirm_min_command_rad_s",
      error) &&
    finite_at_least(config.encoder_confirm_window_sec, 0.0, true, "encoder_confirm_window_sec",
      error) &&
    // 0 switches these two criteria off (documented); a negative value is not a
    // second spelling of "off" but a typo, and is refused.
    finite_at_least(config.stall_escalation_window_sec, 0.0, false, "stall_escalation_window_sec",
      error) &&
    finite_at_least(config.stall_escalation_latched_sec, 0.0, false,
      "stall_escalation_latched_sec", error) &&
    finite_at_least(config.stall_hold_min_command_rad_s, 0.0, false,
      "stall_hold_min_command_rad_s (stall_release_command_rad_s)", error) &&
    finite_at_least(config.stall_window_sec, 0.0, true, "stall_window_sec", error) &&
    finite_at_least(config.stall_silent_window_sec, 0.0, true, "stall_silent_window_sec", error);
}

void DriveFaultMonitor::configure(const DriveFaultConfig & config)
{
  // Deliberately NOTHING but the config. The latch and the encoder accumulators
  // survive a re-configure — see the header.
  config_ = config;
}

void DriveFaultMonitor::reset_activation_state()
{
  stall_trips_.fill(0);
  escalation_trips_.fill(0);
  released_once_.fill(false);
  last_release_sec_.fill(0.0);
  // Continuity only: the first cycle after an activation must not add the
  // whole inactive period to an accumulator. The accumulated time itself stays.
  unconfirmed_prev_.fill(false);
  have_prev_now_ = false;
  latched_held_sec_.fill(0.0);
  latched_held_prev_.fill(false);
}

void DriveFaultMonitor::restore(const DriveFaultRecord & record)
{
  if (record.active && !record_.active) {
    record_ = record;
  }
}

DriveFaultClearRefusal DriveFaultMonitor::clear_refusal(int expected_reason) const
{
  if (!record_.active) {
    return DriveFaultClearRefusal::kNotLatched;
  }
  if (!drive_fault_reason_clearable(record_.reason)) {
    return DriveFaultClearRefusal::kNotClearable;
  }
  if (expected_reason != kDriveFaultNone && expected_reason != record_.reason) {
    return DriveFaultClearRefusal::kExpectedReasonMismatch;
  }
  return DriveFaultClearRefusal::kNone;
}

bool DriveFaultMonitor::clear(double now_sec, int expected_reason)
{
  if (clear_refusal(expected_reason) != DriveFaultClearRefusal::kNone) {
    return false;
  }
  for (const int named : {record_.wheel, record_.other_wheel}) {
    if (named >= 0 && static_cast<std::size_t>(named) < kNumWheels) {
      const auto i = static_cast<std::size_t>(named);
      released_once_[i] = true;
      last_release_sec_[i] = now_sec;
      escalation_trips_[i] = 0;
    }
  }
  latched_held_sec_.fill(0.0);
  latched_held_prev_.fill(false);
  unconfirmed_prev_.fill(false);
  // Continuity only (as on activation): the first cycle after the clear must
  // not add the whole faulted period, during which update() did not advance
  // prev_now_sec_, to an accumulator. Accumulated time itself is kept.
  have_prev_now_ = false;
  record_ = DriveFaultRecord{};
  return true;
}

void DriveFaultMonitor::latch(const DriveFaultRecord & record)
{
  record_ = record;
  record_.active = true;
}

int DriveFaultMonitor::effective_provenance(
  const DriveFaultInput & input, std::size_t wheel) const
{
  if (config_.assume_live_until_provenance_received && !input.provenance_received) {
    return kStallProvenanceLive;
  }
  return input.provenance[wheel];
}

DriveFaultResult DriveFaultMonitor::update(
  const DriveFaultInput & input, const std::array<double, kNumWheels> & commands)
{
  DriveFaultResult result;
  result.commands = commands;

  if (!record_.active && config_.enabled) {
    const double now = input.now_sec;
    double dt = 0.0;
    if (have_prev_now_ && now > prev_now_sec_) {
      dt = now - prev_now_sec_;
    }

    // ------------------------------------------- trigger 1: encoder missing
    for (std::size_t i = 0; i < kNumWheels && !record_.active; ++i) {
      const int provenance = effective_provenance(input, i);
      if (provenance >= kStallProvenanceLive) {
        // Confirmed. The accumulated evidence is void from here on; if the
        // encoder ever drops back (an ESP32 reboot restarts at
        // LIVE_UNCONFIRMED) it is judged afresh.
        unconfirmed_sec_[i] = 0.0;
        unconfirmed_prev_[i] = false;
        continue;
      }
      const double requested = std::isfinite(input.requested[i]) ?
        std::fabs(input.requested[i]) : 0.0;
      // A tier-1 latched wheel is receiving exactly 0.0, so it is not being
      // driven. (A tier-1 latch needs LIVE provenance in the first place, so
      // this only matters under assume_live_provenance.)
      const bool driven = requested > config_.encoder_confirm_min_command_rad_s &&
        !input.drive_withheld && !input.stall_latched[i];
      // EVERY driven cycle counts the period since the previous cycle -- not
      // only cycles whose predecessor was driven too, which would drop one
      // period per rising edge and never trip a command flickering across the
      // threshold.
      if (driven) {
        unconfirmed_sec_[i] += dt;
      }
      unconfirmed_prev_[i] = driven;
      if (driven && unconfirmed_sec_[i] >= config_.encoder_confirm_window_sec) {
        // No wheel LIVE at all is not one bad encoder: on the robot it is what
        // a stale or absent /hw/joint_states looks like (GripperXInterface then
        // reports UNKNOWN on all four), or a lost encoder supply. Same
        // response, different name -- the text must not send someone to check
        // one encoder.
        bool any_live = false;
        for (std::size_t j = 0; j < kNumWheels; ++j) {
          any_live = any_live || effective_provenance(input, j) >= kStallProvenanceLive;
        }
        DriveFaultRecord record;
        record.reason = any_live ? kDriveFaultEncoderMissing : kDriveFaultFeedbackLost;
        record.wheel = any_live ? static_cast<int>(i) : -1;
        record.other_wheel = any_live ? -1 : static_cast<int>(i);
        record.trip_time_sec = now;
        record.commanded_rad_s = input.requested[i];
        record.provenance = provenance;
        record.accumulated_sec = unconfirmed_sec_[i];
        latch(record);
      }
    }

    // ------------------------------------- trigger 2: stall escalation
    // EVERY DRIVEN WHEEL FROZEN: when the multi-wheel criterion fires and every
    // wheel commanded above encoder_confirm_min_command_rad_s (where HWR-30a's
    // zero-count criterion watches a LIVE wheel; the displacement threshold is
    // not below it) is latched, completing a criterion this cycle, or at least
    // half-way into either of its windows (no movement, no count), that is what
    // a frozen /hw/joint_states looks like
    // (all positions freeze together; their windows complete within a cycle or
    // two of each other), not two independent stalls. Named FEEDBACK_LOST; the
    // response is identical. The half-window tolerance only CLASSIFIES -- it
    // can never make or prevent a trip.
    std::size_t driven = 0;
    std::size_t frozen = 0;
    for (std::size_t j = 0; j < kNumWheels; ++j) {
      const double req = std::isfinite(input.requested[j]) ? std::fabs(input.requested[j]) : 0.0;
      if (req <= config_.encoder_confirm_min_command_rad_s) {
        continue;
      }
      ++driven;
      const auto & e = input.stall_events[j];
      if (
        input.stall_latched[j] || e.tripped || e.multi_wheel_refused ||
        input.stall_no_motion_sec[j] >= 0.5 * config_.stall_window_sec ||
        input.stall_silent_sec[j] >= 0.5 * config_.stall_silent_window_sec)
      {
        ++frozen;
      }
    }
    const bool simultaneous_onset = driven >= 2 && frozen == driven;
    for (std::size_t i = 0; i < kNumWheels && !record_.active; ++i) {
      const auto & event = input.stall_events[i];
      if (event.released) {
        released_once_[i] = true;
        last_release_sec_[i] = now;
      }
      int other = -1;
      for (std::size_t j = 0; j < kNumWheels; ++j) {
        if (j != i && input.stall_latched[j]) {
          other = static_cast<int>(j);
          break;
        }
      }

      DriveFaultRecord record;
      record.wheel = static_cast<int>(i);
      record.trip_time_sec = now;
      record.commanded_rad_s = input.requested[i];
      record.provenance = effective_provenance(input, i);
      // An event on this cycle names its criterion; a standing latch (the
      // latched-and-held criterion) names the one that set it.
      record.zero_count = (event.tripped || event.multi_wheel_refused) ?
        event.zero_count : input.stall_latched_zero_count[i];

      if (event.tripped) {
        ++stall_trips_[i];
        ++escalation_trips_[i];
        record.stall_trip_count = stall_trips_[i];
        if (config_.stall_escalate_on_multi_wheel && other >= 0) {
          record.reason = kDriveFaultStallMultiWheel;
          record.other_wheel = other;
          latch(record);
        } else if (
          config_.stall_escalation_window_sec > 0.0 && released_once_[i] &&
          (now - last_release_sec_[i]) <= config_.stall_escalation_window_sec)
        {
          record.reason = kDriveFaultStallRetrip;
          record.since_release_sec = now - last_release_sec_[i];
          latch(record);
        } else if (
          config_.stall_escalation_trips > 0 &&
          escalation_trips_[i] >= config_.stall_escalation_trips)
        {
          record.reason = kDriveFaultStallTripCount;
          latch(record);
        }
      } else if (event.multi_wheel_refused && config_.stall_escalate_on_multi_wheel) {
        record.reason = simultaneous_onset ? kDriveFaultFeedbackLost : kDriveFaultStallMultiWheel;
        record.stall_trip_count = stall_trips_[i];
        record.other_wheel = other;
        if (simultaneous_onset) {
          record.other_wheel = record.wheel;
          record.wheel = -1;
        }
        latch(record);
      }

      // (c) OP-25 deadlock: latched, and the command that would have to drop
      // to release it is still held. CONTINUOUS time: the command dropping into
      // the release band, or the latch clearing, restarts it.
      if (!record_.active && config_.stall_escalation_latched_sec > 0.0) {
        const double requested = std::isfinite(input.requested[i]) ?
          std::fabs(input.requested[i]) : 0.0;
        const bool held =
          input.stall_latched[i] && requested > config_.stall_hold_min_command_rad_s;
        if (held && latched_held_prev_[i]) {
          latched_held_sec_[i] += dt;
        } else if (!held) {
          latched_held_sec_[i] = 0.0;
        }
        latched_held_prev_[i] = held;
        if (held && latched_held_sec_[i] >= config_.stall_escalation_latched_sec) {
          record.reason = kDriveFaultStallLatchedHeld;
          record.stall_trip_count = stall_trips_[i];
          record.latched_held_sec = latched_held_sec_[i];
          latch(record);
        }
      }
    }

    prev_now_sec_ = now;
    have_prev_now_ = true;
    result.tripped = record_.active;
  }

  if (record_.active) {
    result.commands.fill(0.0);
  }
  return result;
}

}  // namespace gripperx_swerve_controller
