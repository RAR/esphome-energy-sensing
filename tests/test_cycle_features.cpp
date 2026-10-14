// Host tests for the cycle features:
//   c++ -std=c++17 -Wall -Wextra -I components/load_events \
//     tests/test_cycle_features.cpp -o /tmp/test_cycles && /tmp/test_cycles

#include "cycle_features.h"
#include "step_detector.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

using esphome::load_events::CYCLE_HARMONICS;
using esphome::load_events::CycleFeatures;
using esphome::load_events::CycleSample;
using esphome::load_events::CycleStep;
using esphome::load_events::LoadEvent;

static int failures = 0;

#define CHECK(cond)                                                  \
  do {                                                               \
    if (!(cond)) {                                                   \
      std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);  \
      failures++;                                                    \
    }                                                                \
  } while (0)

#define NEAR(a, b, tol)                                                                                  \
  do {                                                                                                   \
    double a_ = (a), b_ = (b);                                                                           \
    if (!(std::fabs(a_ - b_) <= (tol))) {                                                                \
      std::printf("  FAIL %s:%d: %s = %.4f, want %.4f +- %.4f\n", __FILE__, __LINE__, #a, a_, b_, (double) (tol)); \
      failures++;                                                                                        \
    }                                                                                                    \
  } while (0)

static constexpr float PERIOD_MS = 1000.0f / 60.0f;
static constexpr float RAW_PER_W = 0.4f;  // the meter's raw power is 0.4 x watts: the scale is 2.5
static constexpr float AMPS_PER_W = 1.0f / 120.0f;

// A load: its real power, displacement power factor (lagging), harmonics over its fundamental (each at
// its own angle), and an inrush: `inrush` times its fundamental current for `inrush_cycles`, falling to 1.
struct Load {
  float w;
  float dpf = 1.0f;
  float h[3] = {0, 0, 0};
  float inrush = 1.0f;
  int inrush_cycles = 0;
  int on_at = -1, off_at = 1 << 30;  // cycles
};

static CycleSample cycle_at(const std::vector<Load> &loads, int n, unsigned seed) {
  CycleSample c;
  c.t_ms = static_cast<uint32_t>(n * PERIOD_MS);
  c.period_ms = PERIOD_MS;
  float w = 0;
  for (const Load &l : loads) {
    if (n < l.on_at || n >= l.off_at)
      continue;
    const int since = n - l.on_at;
    float boost = 1.0f;
    if (since < l.inrush_cycles)  // a straight fall from `inrush` to 1 over the inrush
      boost = l.inrush - (l.inrush - 1.0f) * since / l.inrush_cycles;
    const float i1 = l.w / l.dpf * AMPS_PER_W * boost;
    const float phi = std::acos(l.dpf);
    c.h_i[0] += i1 * std::cos(phi);
    c.h_q[0] += -i1 * std::sin(phi);
    for (int k = 1; k < CYCLE_HARMONICS; k++) {
      const float angle = 0.7f * k + 0.3f;
      c.h_i[k] += l.h[k - 1] * i1 * std::cos(angle);
      c.h_q[k] += l.h[k - 1] * i1 * std::sin(angle);
    }
    w += l.w * boost;
  }
  // A little wobble, as a real circuit has (+-2 W).
  seed = seed * 1103515245u + 12345u + n * 2654435761u;
  w += (static_cast<float>((seed >> 8) % 401) / 100.0f) - 2.0f;
  c.power_raw = w * RAW_PER_W;
  return c;
}

// Feeds `cycles` cycles, calibrating against the true watts every 13 cycles (a ~220 ms frame).
static void run(CycleFeatures &f, const std::vector<Load> &loads, int from, int cycles, unsigned seed = 1) {
  for (int n = from; n < from + cycles; n++) {
    f.feed(cycle_at(loads, n, seed));
    if (n % 13 == 12) {
      float watts = 0;
      for (int m = n - 12; m <= n; m++)
        watts += cycle_at(loads, m, seed).power_raw / RAW_PER_W;
      f.calibrate(watts / 13);
    }
  }
}

static void test_calibration() {
  std::printf("calibration\n");
  CycleFeatures f;
  run(f, {{500}}, 0, 130);
  NEAR(f.scale(), 2.5, 0.05);
  CycleFeatures quiet;
  run(quiet, {{5}}, 0, 130);  // too little power to scale from
  CHECK(quiet.scale() == 0);
  CHECK(quiet.kept() == 0);
}

static void test_heater_has_no_inrush_or_harmonics() {
  std::printf("heater\n");
  CycleFeatures f;
  Load base{400};
  Load heater{1500};
  heater.on_at = 200;
  run(f, {base, heater}, 0, 400);
  CHECK(f.kept() == 1);
  if (!f.kept())
    return;
  const CycleStep &s = f.kept_step(0);
  NEAR(s.dw, 1500, 15);
  NEAR(s.t_ms, 200 * PERIOD_MS, 1);
  NEAR(s.inrush, 1.0, 0.05);
  CHECK(s.inrush_ms == 0);
  NEAR(s.dpf, 1.0, 0.01);
  for (float h : s.h)
    NEAR(h, 0, 0.02);
}

static void test_motor_start() {
  std::printf("motor\n");
  CycleFeatures f;
  Load base{300};
  Load motor{800, 0.8f};
  motor.inrush = 5.0f;
  motor.inrush_cycles = 15;  // 250 ms
  motor.on_at = 150;
  run(f, {base, motor}, 0, 400);
  CHECK(f.kept() == 1);
  if (!f.kept())
    return;
  const CycleStep &s = f.kept_step(0);
  NEAR(s.dw, 800, 15);
  NEAR(s.inrush, 5.0, 0.15);
  // Over 1.15x until the fall from 5x reaches it: (5 - 1.15) / (4 / 15) = 14.4 cycles in.
  NEAR(s.inrush_ms, 15 * PERIOD_MS, PERIOD_MS + 1);
  NEAR(s.dpf, 0.8, 0.02);
}

static void test_power_supply_harmonics_on_and_off() {
  std::printf("power supply\n");
  CycleFeatures f;
  Load base{1000};
  Load psu{200, 0.98f, {0.8f, 0.6f, 0.4f}};
  psu.on_at = 150;
  psu.off_at = 400;
  run(f, {base, psu}, 0, 600);
  CHECK(f.kept() == 2);
  if (f.kept() != 2)
    return;
  const CycleStep &on = f.kept_step(0), &off = f.kept_step(1);
  NEAR(on.dw, 200, 10);
  NEAR(on.h[0], 0.8, 0.03);
  NEAR(on.h[1], 0.6, 0.03);
  NEAR(on.h[2], 0.4, 0.03);
  NEAR(on.dpf, 0.98, 0.02);
  NEAR(off.dw, -200, 10);
  NEAR(off.h[0], 0.8, 0.03);  // the load that went away, not its negative
  NEAR(off.dpf, 0.98, 0.02);
  CHECK(std::isnan(off.inrush));
  NEAR(off.t_ms, 400 * PERIOD_MS, 1);
}

static void test_a_pulse_is_not_a_step() {
  std::printf("pulse\n");
  CycleFeatures f;
  Load base{600};
  Load blip{900};
  blip.on_at = 150;
  blip.off_at = 154;
  run(f, {base, blip}, 0, 400);
  CHECK(f.kept() == 0);
}

static void test_take_matches_sign_size_and_time() {
  std::printf("take\n");
  CycleFeatures f(30);
  Load base{400};
  Load kettle{1200};
  kettle.on_at = 150;
  run(f, {base, kettle}, 0, 300);
  CHECK(f.kept() == 1);
  CycleStep s;
  const uint32_t step_ms = static_cast<uint32_t>(150 * PERIOD_MS);
  CHECK(!f.take(-1200, step_ms + 500, &s));  // the wrong way
  CHECK(!f.take(400, step_ms + 500, &s));    // far too small
  CHECK(!f.take(1200, step_ms + 5000, &s));  // too long after
  CHECK(f.take(1150, step_ms + 600, &s));    // the event's frames run behind
  NEAR(s.dw, 1200, 15);
  CHECK(f.kept() == 0);
  CHECK(!f.take(1150, step_ms + 600, &s));  // taken once
}

static void test_event_json_carries_the_cycle() {
  std::printf("json\n");
  LoadEvent e;
  e.key = "total";
  std::string plain = e.to_json(0);
  CHECK(plain.find("\"cycle\"") == std::string::npos);
  e.has_cycle = true;
  e.cycle_dw = 812.5f;
  e.inrush = 4.96f;
  e.inrush_ms = 233;
  e.dpf = 0.8f;
  e.harmonics[0] = 0.05f;
  e.cycle_lead_ms = 640;
  std::string out = e.to_json(0);
  CHECK(out.find("\"cycle\":{\"lead_ms\":640,\"inrush_ms\":233,\"dw\":812.5,\"inrush\":4.96,\"dpf\":0.800,"
                 "\"h\":[0.050,null,null]}") != std::string::npos);
}

int main() {
  test_event_json_carries_the_cycle();
  test_calibration();
  test_heater_has_no_inrush_or_harmonics();
  test_motor_start();
  test_power_supply_harmonics_on_and_off();
  test_a_pulse_is_not_a_step();
  test_take_matches_sign_size_and_time();
  std::printf(failures ? "%d FAILED\n" : "all passed\n", failures);
  return failures ? 1 : 0;
}
