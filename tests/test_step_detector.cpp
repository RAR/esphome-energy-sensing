// Host tests for the step detector:
//   c++ -std=c++17 -Wall -Wextra -I components/load_events \
//     tests/test_step_detector.cpp -o /tmp/test_step && /tmp/test_step

#include "step_detector.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using esphome::load_events::LoadEvent;
using esphome::load_events::StepConfig;
using esphome::load_events::StepDetector;

static int failures = 0;

#define CHECK(cond)                                                  \
  do {                                                               \
    if (!(cond)) {                                                   \
      std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);  \
      failures++;                                                    \
    }                                                                \
  } while (0)

#define NEAR(a, b, tol) CHECK(std::fabs((double) (a) - (double) (b)) <= (tol))

static constexpr uint32_t FRAME_MS = 200;

// Feeds the frames 200 ms apart and returns every event, in order.
struct Run {
  StepDetector det;
  uint32_t ms = 0;
  std::vector<LoadEvent> events;
  explicit Run(StepConfig cfg = {}) : det("test", cfg) {}
  // pf gives each frame an apparent power of w / pf.
  void feed(std::vector<float> ws, float pf = NAN) {
    for (float w : ws) {
      if (det.feed(w, std::isnan(pf) ? NAN : w / pf, ms)) {
        events.push_back(det.event());
        while (det.next())
          events.push_back(det.event());
      }
      ms += FRAME_MS;
    }
  }
  void hold(float w, int frames, float pf = NAN, float jitter = 0) {
    for (int i = 0; i < frames; i++)
      feed({w + ((i % 2) ? jitter : -jitter)}, pf);
  }
};

static void a_heater_is_a_clean_step_up_and_down() {
  Run r;
  r.hold(46, 10, 0.98f, 1);
  r.hold(1530, 10, 1.0f, 2);
  r.hold(46, 10, 0.98f, 1);
  CHECK(r.events.size() == 2);
  if (r.events.size() != 2)
    return;
  const LoadEvent &on = r.events[0];
  NEAR(on.from_w, 46, 2);
  NEAR(on.to_w, 1530, 3);
  NEAR(on.peak_w, 1532, 0.5);
  CHECK(on.settled);
  CHECK(on.settle_ms == 0);                // the first frame was already the new level
  NEAR(on.pf_from(), 0.98, 0.01);
  NEAR(on.pf_to(), 1.0, 0.01);
  NEAR(on.va_to, 1530, 3);
  CHECK(on.start_ms == 10 * FRAME_MS);
  CHECK(on.count >= 3 + 4);                // the lead-in frames, then the step
  NEAR(on.samples[0], 46, 2);
  NEAR(on.samples[3], 1528, 0.5);
  CHECK(!on.pulse);
  const LoadEvent &off = r.events[1];
  NEAR(off.from_w, 1530, 3);
  NEAR(off.to_w, 46, 2);
}

static void a_motor_shows_its_inrush_and_how_long_it_took_to_settle() {
  Run r;
  r.hold(10, 10, 0.5f);
  r.feed({2400, 1200, 700, 650, 652, 648, 651, 650, 650}, 0.8f);
  CHECK(r.events.size() == 1);
  if (r.events.empty())
    return;
  const LoadEvent &e = r.events[0];
  NEAR(e.peak_w, 2400, 0.1);
  NEAR(e.to_w, 650, 3);
  CHECK(e.settled);
  CHECK(e.settle_ms == 3 * FRAME_MS);      // 650 was the first of the agreeing frames
  NEAR(e.pf_to(), 0.8, 0.01);
  NEAR(e.va_to, 650 / 0.8, 5);
}

static void one_odd_frame_is_not_a_step() {
  Run r;
  r.hold(10, 10);
  r.feed({400});
  r.hold(10, 10);
  CHECK(r.events.empty());
  NEAR(r.det.level(), 10, 0.5);
}

static void noise_and_slow_drift_are_not_steps() {
  Run r;
  r.hold(400, 50, NAN, 12);                // ±12 W is under 30 W
  for (int i = 0; i < 6000; i++)
    r.feed({400 + i * 0.01f});             // 60 W over 20 minutes: a heater sagging, not a load switching
  CHECK(r.events.empty());
  NEAR(r.det.level(), 460, 5);
}

static void a_load_fading_off_is_reported_once_as_a_gradual_change() {
  Run r;
  r.hold(330, 50, 0.99f, 3);
  for (int i = 0; i < 150; i++)            // 80 W fading off over 30 s, too slowly to make a step
    r.feed({330 - i * 80 / 150.0f}, 0.99f);
  r.hold(250, 150, 0.99f, 3);
  CHECK(r.events.size() == 1);
  if (r.events.empty())
    return;
  const LoadEvent &e = r.events[0];
  CHECK(e.drift && !e.pulse && !e.settled);
  NEAR(e.from_w, 330, 6); std::printf("  fade from %.1f to %.1f\n", e.from_w, e.to_w);
  CHECK(e.to_w < 300 && e.to_w > 240);
  CHECK(e.start_ms >= 50 * FRAME_MS && e.start_ms < 200 * FRAME_MS);   // dated from the fade, not the report
  CHECK(e.json().find("\"drift\":true") != std::string::npos);
  NEAR(e.pf_from(), 0.99f, 0.02f);         // apparent power averaged the same way as real power
  NEAR(e.pf_to(), 0.99f, 0.02f);
}

static void a_ramp_is_not_learned_as_wobble() {
  Run r;
  r.hold(200, 20, 1.0f, 2);
  for (int i = 0; i < 60; i++)             // an EV charger ramping 120 W a frame up to 7400 W
    r.feed({200 + i * 120.0f}, 1.0f);
  r.hold(7400, 100, 1.0f, 5);
  size_t up = r.events.size();
  CHECK(up >= 1 && up <= 3);
  std::printf("  %zu events for a 60-frame ramp\n", up);
  CHECK(r.det.noise() < 50);
  r.hold(200, 30, 1.0f, 2);                // and its switching off is one clean step
  CHECK(r.events.size() == up + 1);
}

static void steady_creep_is_not_a_stream_of_small_changes() {
  Run r;
  r.hold(500, 20, 1.0f, 2);
  for (int i = 0; i < 18000; i++)          // 10 W a minute for an hour (frames 200 ms apart)
    r.feed({500 + i * (10.0f / 300.0f)}, 1.0f);
  std::printf("  %zu events in an hour of creep\n", r.events.size());
  CHECK(r.events.size() <= 2);
}

static void a_load_switching_off_a_jittery_circuit_is_still_seen() {
  Run r;
  r.hold(250, 20, 0.99f, 3);
  for (int i = 0; i < 600; i++)            // a 3D printer's bed switching every few frames: ±60 W
    r.feed({330 + ((i / 3) % 2 ? 60.0f : -60.0f)}, 0.99f);
  size_t before = r.events.size();
  r.hold(250, 150, 0.99f, 3);              // unplugged
  CHECK(r.events.size() > before);
  if (r.events.size() > before)
    CHECK(r.events.back().to_w < 290);
}

static void a_brief_load_comes_back_as_a_pulse() {
  Run r;
  r.hold(10, 10);
  r.feed({800, 800});
  r.hold(10, 6);
  CHECK(r.events.size() == 1);
  if (r.events.empty())
    return;
  NEAR(r.events[0].peak_w, 800, 0.1);
  NEAR(r.events[0].to_w, 10, 0.5);
  CHECK(r.events[0].pulse);
}

static void a_load_that_never_settles_is_reported_when_it_times_out() {
  Run r;
  r.hold(10, 10);
  for (int i = 0; i < 60; i++)
    r.feed({(i % 2) ? 500.0f : 700.0f});
  CHECK(r.events.size() >= 1);
  if (r.events.empty())
    return;
  const LoadEvent &e = r.events[0];
  CHECK(!e.settled);
  CHECK(e.settle_ms >= 10000 - 8 * FRAME_MS);  // reported at the timeout, its last frames as the level
  NEAR(e.to_w, 600, 1);
  CHECK(e.count == LoadEvent::MAX_SAMPLES);  // the capture stops filling, it does not overflow
}

static void steps_follow_on_from_each_other() {
  Run r;
  r.hold(100, 10);
  r.hold(600, 8);
  r.hold(1800, 8);
  r.hold(100, 8);
  CHECK(r.events.size() == 3);
  if (r.events.size() != 3)
    return;
  NEAR(r.events[1].from_w, 600, 1);
  NEAR(r.events[1].to_w, 1800, 1);
  NEAR(r.events[2].from_w, 1800, 1);
  NEAR(r.events[2].samples[0], 1800, 1);   // the lead-in is the level it left
}

static void a_big_or_wobbly_level_needs_a_bigger_step() {
  Run r;
  r.hold(3000, 40, NAN, 20);
  r.hold(3050, 10, NAN, 20);               // under 2 %, and under 4 x its 20 W wobble
  CHECK(r.events.empty());
  r.hold(3400, 10, NAN, 20);
  CHECK(r.events.size() == 1);
}

static void the_event_reads_as_json() {
  StepDetector det("hot \"tub\"");
  uint32_t ms = 0;
  for (int i = 0; i < 5; i++)
    det.feed(46, NAN, ms += FRAME_MS);
  bool done = false;
  for (int i = 0; i < 6 && !done; i++)
    done = det.feed(1530, NAN, ms += FRAME_MS);
  CHECK(done);
  std::string json = det.event().to_json(ms + 1000);
  std::printf("  %s\n", json.c_str());
  CHECK(json.rfind("{\"key\":\"hot \\\"tub\\\"\",\"age_ms\":", 0) == 0);
  CHECK(json.find("\"from_w\":46.0") != std::string::npos);
  CHECK(json.find("\"to_w\":1530.0") != std::string::npos);
  CHECK(json.find("\"va_from\":null,\"va_to\":null,\"pf_from\":null,\"pf_to\":null") != std::string::npos);
  CHECK(json.find("\"settled\":true,\"pulse\":false") != std::string::npos);
  CHECK(json.find("\"dt_ms\":200,\"samples\":[46,46,46,1530,1530,1530,1530]}") != std::string::npos);
}

static void unknown_frames_are_skipped() {
  Run r;
  r.hold(10, 5);
  r.feed({NAN, NAN});
  r.hold(10, 5);
  CHECK(r.events.empty());
}

// A fixed pseudo-random sequence, so the noise tests always see the same frames.
struct Lcg {
  uint32_t state = 12345;
  float gauss() {  // roughly normal, mean 0, sd 1 (sum of 12 uniforms)
    float sum = 0;
    for (int i = 0; i < 12; i++) {
      state = state * 1664525u + 1013904223u;
      sum += (state >> 8) / 16777216.0f;
    }
    return sum - 6.0f;
  }
};

static void a_noisy_load_learns_its_wobble_and_stays_quiet() {
  Run r;
  Lcg rng;
  for (int i = 0; i < 7200; i++)           // an hour at 2 frames a second, sd 15 W
    r.feed({430 + 15 * rng.gauss()});
  CHECK(r.events.size() <= 3);
  std::printf("  %zu events in an hour of 15 W noise, wobble %.1f W\n", r.events.size(), r.det.noise());
  r.hold(1630, 10);                        // a real 1200 W step is still seen
  CHECK(!r.events.empty() && std::fabs(r.events.back().to_w - 1630) < 5);
}

static void an_oscillating_load_reports_once_not_every_timeout() {
  Run r;
  r.hold(10, 10);
  for (int i = 0; i < 1200; i++)           // 10 minutes of 500, 500, 700, 700, ...
    r.feed({(i / 2) % 2 ? 700.0f : 500.0f});
  CHECK(r.events.size() <= 2);
  std::printf("  %zu events in 10 minutes of oscillation\n", r.events.size());
  r.hold(10, 10);                          // and it switching off is still seen
  CHECK(!r.events.empty() && std::fabs(r.events.back().to_w - 10) < 5);
}

static void a_load_is_seen_switching_off_a_busy_circuit_as_it_was_switching_on() {
  Run r;
  r.hold(2880, 10);
  r.hold(3000, 10);                        // on: 120 W onto 2880 W
  r.hold(2880, 10);                        // off: 120 W off 3000 W
  CHECK(r.events.size() == 2);
}


// A 3D printer's bed: 265 W on for 2 s, off for 3 s (frames 200 ms apart), over a 250 W circuit.
static void bed(Run &r, int frames, float base = 250) {
  for (int i = 0; i < frames; i++)
    r.feed({((i % 25) < 10 ? base + 265 : base) + ((i % 2) ? 3.0f : -3.0f)}, 0.99f);
}

static void a_cycling_load_is_one_start_and_one_end() {
  Run r;
  r.hold(250, 30, 0.99f, 3);
  bed(r, 1500);                              // 5 minutes
  r.hold(250, 100, 0.99f, 3);
  int starts = 0, ends = 0, steps = 0;
  const LoadEvent *start = nullptr, *end = nullptr;
  for (auto &e : r.events) {
    if (e.cycling == 1) { starts++; start = &e; }
    else if (e.cycling == 2) { ends++; end = &e; }
    else steps++;
  }
  std::printf("  %d steps, %d start, %d end\n", steps, starts, ends);
  CHECK(starts == 1 && ends == 1 && steps <= 4);   // only the switches before it was recognised
  if (!start || !end)
    return;
  NEAR(start->from_w, 250, 8);
  CHECK(start->spans_ms > 0 && start->spans_ms < 30000);
  NEAR(start->amp_w, 265, 15);
  NEAR(end->held_w, 250 + 265 * 0.4f, 15);   // its true mean: on 40 % of the time
  NEAR(end->duty, 0.4f, 0.08f);
  NEAR(end->period_ms, 5000, 500);
  NEAR(end->to_w, 250, 8);
  CHECK(end->json().find("\"cycling\":\"end\"") != std::string::npos);
}

static void another_load_switching_mid_cycle_is_still_reported() {
  Run r;
  r.hold(250, 30, 0.99f, 3);
  bed(r, 500);
  r.hold(250, 10, 0.99f, 3);                 // the bed off for a moment...
  r.hold(370, 10, 0.99f, 3);                 // ...as a 120 W PC comes on
  bed(r, 500, 370);
  r.hold(370, 100, 0.99f, 3);
  bool ended = false, pc = false;
  int starts = 0;
  for (auto &e : r.events) {
    if (e.cycling == 2)
      ended = true;
    if (e.cycling == 1)
      starts++;
    if (!e.cycling && std::fabs(e.from_w - 250) < 10 && std::fabs(e.to_w - 370) < 10)
      pc = true;
  }
  CHECK(ended && pc && starts == 2);
}

static void long_runs_are_never_cycling() {
  Run r;
  for (int k = 0; k < 6; k++) {              // a hot tub heater: 27 minutes on, then off
    r.hold(46, 300, 0.98f, 1);
    r.hold(957, 300, 1.0f, 2);
  }
  for (auto &e : r.events)
    CHECK(e.cycling == 0);
}

static void a_step_reports_the_stretch_it_ended_and_its_stages() {
  Run r;
  r.hold(226, 30, 0.95f, 2);
  r.hold(270, 3, 0.95f, 1);                  // a PC waking: +44 W, then its screen +124 W
  r.hold(394, 30, 0.95f, 2);
  CHECK(r.events.size() == 1);
  if (r.events.empty())
    return;
  const LoadEvent &e = r.events[0];
  NEAR(e.held_w, 226, 3);
  NEAR(e.held_ms, 30 * FRAME_MS, 2 * FRAME_MS);
  CHECK(e.held_span_w < 10);
  CHECK(e.stage_count == 1);
  if (e.stage_count)
    NEAR(e.stage_w[0], 270, 5);
  CHECK(e.json().find("\"stages\":[[270,") != std::string::npos);
}


static void a_cycle_s_mean_leaves_out_the_step_that_ended_it_and_the_quiet_after() {
  Run r;
  r.hold(250, 30, 0.99f, 3);
  bed(r, 510);                               // ends 10 frames into an on: the bed is high...
  r.hold(250 + 265 + 3000, 60, 1.0f, 5);     // ...as a 3 kW oven comes on
  const LoadEvent *end = nullptr;
  for (auto &e : r.events)
    if (e.cycling == 2)
      end = &e;
  CHECK(end != nullptr);
  if (end)
    NEAR(end->held_w, 250 + 265 * 0.4f, 15);
  Run q;                                     // and stopping on its own: not the quiet tail
  q.hold(250, 30, 0.99f, 3);
  bed(q, 500);
  q.hold(250, 400, 0.99f, 3);
  end = nullptr;
  for (auto &e : q.events)
    if (e.cycling == 2)
      end = &e;
  CHECK(end != nullptr);
  if (end)
    NEAR(end->held_w, 250 + 265 * 0.4f, 12);
}

static void a_long_first_on_ends_with_its_own_step_down() {
  Run r;
  r.hold(250, 30, 0.99f, 3);
  r.hold(515, 200, 0.99f, 3);                // a bed heating up from cold for 40 s
  for (int i = 0; i < 500; i++)              // then holding: off first
    r.feed({(((i + 10) % 25) < 10 ? 515.0f : 250.0f) + ((i % 2) ? 3.0f : -3.0f)}, 0.99f);
  const LoadEvent *down = nullptr, *start = nullptr;
  for (auto &e : r.events) {
    if (!down && !e.cycling && e.from_w > 500 && e.to_w < 270)
      down = &e;
    if (e.cycling == 1)
      start = &e;
  }
  CHECK(down != nullptr && start != nullptr);
  const LoadEvent *end = nullptr;
  r.hold(250, 400, 0.99f, 3);
  for (auto &e : r.events)
    if (e.cycling == 2)
      end = &e;
  CHECK(end != nullptr);
  if (start && end) {
    // One session from the heat-up on, its mean counting the heat-up at full power.
    NEAR(start->start_ms, 30 * FRAME_MS, FRAME_MS);
    float expect = 250 + 265 * (40.0f + 0.4f * 100) / 140.0f;   // 40 s on, then 100 s at 40 %
    NEAR(end->held_w, expect, 15);
  }
}

static void a_motor_s_settling_is_not_a_stage() {
  Run r;
  r.hold(10, 20, 0.6f, 1);
  r.feed({520, 96, 91, 70, 64, 58, 57, 56}, 0.68f);
  r.hold(56, 20, 0.68f, 1);
  CHECK(r.events.size() == 1);
  if (!r.events.empty())
    CHECK(r.events[0].stage_count == 0);
}


static void the_component_s_marks_read_in_the_json() {
  LoadEvent e;
  e.key = "hot_tub";
  e.from_w = 46;
  e.to_w = 957;
  std::string plain = e.to_json(0);
  CHECK(plain.find("\"seq\"") == std::string::npos && plain.find("\"with\"") == std::string::npos);
  e.seq = 42;
  e.replay = true;
  e.with_keys[e.with_count++] = "bar_wall";
  std::string json = e.to_json(0);
  CHECK(json.find("\"seq\":42") != std::string::npos);
  CHECK(json.find("\"replay\":true") != std::string::npos);
  CHECK(json.find("\"with\":[\"bar_wall\"]") != std::string::npos);
}

int main() {
  struct {
    const char *name;
    void (*fn)();
  } tests[] = {
      {"a heater is a clean step up and down", a_heater_is_a_clean_step_up_and_down},
      {"a motor shows its inrush and how long it took to settle", a_motor_shows_its_inrush_and_how_long_it_took_to_settle},
      {"one odd frame is not a step", one_odd_frame_is_not_a_step},
      {"noise and slow drift are not steps", noise_and_slow_drift_are_not_steps},
      {"a load fading off is reported once as a gradual change", a_load_fading_off_is_reported_once_as_a_gradual_change},
      {"a load switching off a jittery circuit is still seen", a_load_switching_off_a_jittery_circuit_is_still_seen},
      {"a ramp is not learned as wobble", a_ramp_is_not_learned_as_wobble},
      {"steady creep is not a stream of small changes", steady_creep_is_not_a_stream_of_small_changes},
      {"a cycling load is one start and one end", a_cycling_load_is_one_start_and_one_end},
      {"another load switching mid-cycle is still reported", another_load_switching_mid_cycle_is_still_reported},
      {"long runs are never cycling", long_runs_are_never_cycling},
      {"a step reports the stretch it ended and its stages", a_step_reports_the_stretch_it_ended_and_its_stages},
      {"a cycle's mean leaves out the step that ended it and the quiet after", a_cycle_s_mean_leaves_out_the_step_that_ended_it_and_the_quiet_after},
      {"a long first on ends with its own step down", a_long_first_on_ends_with_its_own_step_down},
      {"a motor's settling is not a stage", a_motor_s_settling_is_not_a_stage},
      {"the component's marks read in the json", the_component_s_marks_read_in_the_json},
      {"a brief load comes back as a pulse", a_brief_load_comes_back_as_a_pulse},
      {"a load that never settles is reported when it times out", a_load_that_never_settles_is_reported_when_it_times_out},
      {"steps follow on from each other", steps_follow_on_from_each_other},
      {"a big or wobbly level needs a bigger step", a_big_or_wobbly_level_needs_a_bigger_step},
      {"the event reads as json", the_event_reads_as_json},
      {"unknown frames are skipped", unknown_frames_are_skipped},
      {"a noisy load learns its wobble and stays quiet", a_noisy_load_learns_its_wobble_and_stays_quiet},
      {"an oscillating load reports once, not every timeout", an_oscillating_load_reports_once_not_every_timeout},
      {"a load is seen switching off a busy circuit as it was switching on", a_load_is_seen_switching_off_a_busy_circuit_as_it_was_switching_on},
  };
  for (auto &t : tests) {
    int before = failures;
    t.fn();
    std::printf("%s %s\n", failures == before ? "ok  " : "FAIL", t.name);
  }
  std::printf("%d failure(s)\n", failures);
  return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
