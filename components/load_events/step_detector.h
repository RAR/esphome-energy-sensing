#pragma once

// Finds steps in one channel's power, frame by frame: a load switching on or
// off, its inrush peak, how long it took to settle, and the real and apparent
// power on either side. Plain C++ with no ESPHome dependencies, so it is
// tested on the host (tests/test_step_detector.cpp).
//
// The thresholds are counted in frames, so their time depends on the meter's
// rate: an Emporia Vue's stock I2C frames come every 500 ms (the defaults then
// mean a step must hold for 1 s and settle over 2 s); its SPI windows every ~220 ms.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

namespace esphome {
namespace load_events {

// Below this apparent power per clamp the current reading is mostly the
// clamp's noise floor (an idle clamp reads ~0.12 A), so a power factor means nothing.
static constexpr float MIN_POWER_FACTOR_VA = 25.0f;

// The power factor of real and apparent power, or NAN when there is too little to tell.
inline float power_factor(float watts, float va, float min_va = MIN_POWER_FACTOR_VA) {
  if (std::isnan(watts) || std::isnan(va) || va < min_va)
    return NAN;
  return std::min(1.0f, std::fabs(watts) / va);
}

struct StepConfig {
  float min_step_w = 30.0f;    // a step must move at least this far...
  float rel_step = 0.02f;      // ...and this fraction of the smaller of the two levels...
  float noise_factor = 4.0f;   // ...and this many times the level's usual wobble...
  float max_noise_w = 0.0f;    // ...held to this if set (never under 1.5x the wobble); 0 no cap
  float settle_w = 10.0f;      // settled once the last frames span no more than this...
  float settle_rel = 0.03f;    // ...or this fraction of their level, or twice the wobble
  uint8_t confirm = 2;         // frames away from the level before it counts as a step
  uint8_t settle_frames = 4;   // frames that must agree to call it settled (at most 8)
  uint32_t timeout_ms = 10000; // a step that has not settled by then is reported as is
  float min_va = MIN_POWER_FACTOR_VA;  // power factor is unknown below this (per clamp, summed for a circuit)
  // A change too gradual or too jittery to make a step (a load fading off, or switching off on a circuit
  // whose wobble hides it) is reported once a slow average has stayed a step away from the level the
  // last event left for this many frames; 0 never.
  uint8_t drift_frames = 20;
  // A load switching on and off over and over (a 3D printer's bed, a bang-bang element) is reported as
  // one cycling event at its start and one at its end, not every switch: once this many steps in a row
  // alternate up and down, alike within cycle_tolerance of their size, within cycle_window_ms. 0 never.
  uint8_t cycle_steps = 4;
  float cycle_tolerance = 0.35f;
  uint32_t cycle_window_ms = 120000;  // at least 1.5 of its periods (an oven element's are 30-90 s)
};

struct LoadEvent {
  static constexpr uint8_t MAX_SAMPLES = 48;

  const char *key{""};
  float from_w{0}, to_w{0}, peak_w{0};
  float va_from{NAN}, va_to{NAN};  // apparent power either side; NAN without a current reading
  float noise_w{0};                // the level's usual wobble (mean absolute deviation)
  float min_va{MIN_POWER_FACTOR_VA};
  uint32_t start_ms{0};            // the first frame that moved
  uint32_t settle_ms{0};           // from start_ms to the first of the settled frames
  bool settled{false};             // false when it timed out still moving
  bool pulse{false};               // it came back to where it was: an inrush or a brief load
  bool drift{false};               // a gradual change, not a step: from and to are slow averages
  // The stretch this event ended: its mean, how long, and its span (max - min), from the last event on.
  float held_w{NAN}, held_span_w{NAN};
  uint32_t held_ms{0};
  // Levels it passed through on the way, held for two frames or more (a load's stages): [w, frames].
  static constexpr uint8_t MAX_STAGES = 3;
  uint8_t stage_count{0};
  float stage_w[MAX_STAGES]{};
  uint8_t stage_frames[MAX_STAGES]{};
  // Cycling: 0 not, 1 its start (from the low level to its mean so far), 2 its end (from its mean to
  // the level after). With the swing, the time from one switch on to the next, the share of that on,
  // and (at the start) how long since the first switch, so the switches already reported can be set aside.
  uint8_t cycling{0};
  float amp_w{NAN}, duty{NAN};
  uint32_t period_ms{0}, spans_ms{0};
  // Set by the component as it reports: a number counting up across the node's events (from 1 at boot),
  // whether this is a repeat sent after the connection came back, and other channels whose event began
  // within a frame of this one's (a 240 V load on both legs, a branch step also seen on the mains).
  uint32_t seq{0};
  bool replay{false};
  static constexpr uint8_t MAX_WITH = 2;
  uint8_t with_count{0};
  const char *with_keys[MAX_WITH]{};
  // From the cycle features, when the meter hands over each mains cycle (cycle_features.h): the step as
  // the cycles saw it, its inrush and the load's harmonics. has_cycle false otherwise.
  bool has_cycle{false};
  float cycle_dw{NAN}, inrush{NAN}, dpf{NAN};
  float harmonics[3]{NAN, NAN, NAN};  // |I3|, |I5|, |I7| over |I1|
  uint16_t inrush_ms{0};
  int32_t cycle_lead_ms{0};  // start_ms less the step's first cycle (the frames' delay)
  uint32_t first_ms{0}, last_ms{0};  // the first and last stored samples
  uint32_t reported_ms{0};           // when it was handed on (set by the component)
  uint8_t count{0};
  float samples[MAX_SAMPLES]{};    // frames from just before the step onwards (W)

  float pf_from() const { return power_factor(this->from_w, this->va_from, this->min_va); }
  float pf_to() const { return power_factor(this->to_w, this->va_to, this->min_va); }

  // As JSON, with age_ms counted to when it was reported.
  std::string json() const { return this->to_json(this->reported_ms); }

  std::string to_json(uint32_t now_ms) const {
    std::string out;
    out.reserve(360 + 7 * this->count);
    char buf[96];
    auto num = [&](const char *name, float v, const char *fmt) {
      if (std::isnan(v)) {
        snprintf(buf, sizeof(buf), ",\"%s\":null", name);
      } else {
        int n = snprintf(buf, sizeof(buf), ",\"%s\":", name);
        snprintf(buf + n, sizeof(buf) - n, fmt, v);
      }
      out += buf;
    };
    out += "{\"key\":\"";
    for (const char *c = this->key; *c; c++) {
      if (*c == '"' || *c == '\\')
        out += '\\';
      out += *c;
    }
    out += '"';
    snprintf(buf, sizeof(buf), ",\"age_ms\":%u", (unsigned) (now_ms - this->start_ms));
    out += buf;
    num("from_w", this->from_w, "%.1f");
    num("to_w", this->to_w, "%.1f");
    num("peak_w", this->peak_w, "%.1f");
    num("va_from", this->va_from, "%.1f");
    num("va_to", this->va_to, "%.1f");
    num("pf_from", this->pf_from(), "%.2f");
    num("pf_to", this->pf_to(), "%.2f");
    num("noise_w", this->noise_w, "%.1f");
    snprintf(buf, sizeof(buf), ",\"settle_ms\":%u,\"settled\":%s,\"pulse\":%s,\"drift\":%s", (unsigned) this->settle_ms,
             this->settled ? "true" : "false", this->pulse ? "true" : "false", this->drift ? "true" : "false");
    out += buf;
    num("held_w", this->held_w, "%.1f");
    num("held_span_w", this->held_span_w, "%.1f");
    snprintf(buf, sizeof(buf), ",\"held_ms\":%u,\"stages\":[", (unsigned) this->held_ms);
    out += buf;
    for (uint8_t i = 0; i < this->stage_count; i++) {
      snprintf(buf, sizeof(buf), i ? ",[%.0f,%u]" : "[%.0f,%u]", this->stage_w[i], (unsigned) this->stage_frames[i]);
      out += buf;
    }
    out += ']';
    if (this->seq) {
      snprintf(buf, sizeof(buf), ",\"seq\":%u", (unsigned) this->seq);
      out += buf;
    }
    if (this->replay)
      out += ",\"replay\":true";
    if (this->with_count) {
      out += ",\"with\":[";
      for (uint8_t i = 0; i < this->with_count; i++) {
        out += i ? ",\"" : "\"";
        out += this->with_keys[i];
        out += '"';
      }
      out += ']';
    }
    if (this->cycling) {
      snprintf(buf, sizeof(buf), ",\"cycling\":\"%s\"", this->cycling == 1 ? "start" : "end");
      out += buf;
      num("amp_w", this->amp_w, "%.1f");
      num("duty", this->duty, "%.2f");
      snprintf(buf, sizeof(buf), ",\"period_ms\":%u,\"spans_ms\":%u", (unsigned) this->period_ms, (unsigned) this->spans_ms);
      out += buf;
    }
    if (this->has_cycle) {
      out += ",\"cycle\":{";
      snprintf(buf, sizeof(buf), "\"lead_ms\":%d,\"inrush_ms\":%u", (int) this->cycle_lead_ms, (unsigned) this->inrush_ms);
      out += buf;
      num("dw", this->cycle_dw, "%.1f");
      num("inrush", this->inrush, "%.2f");
      num("dpf", this->dpf, "%.3f");
      out += ",\"h\":[";
      for (uint8_t i = 0; i < 3; i++) {
        if (std::isnan(this->harmonics[i]))
          snprintf(buf, sizeof(buf), i ? ",null" : "null");
        else
          snprintf(buf, sizeof(buf), i ? ",%.3f" : "%.3f", this->harmonics[i]);
        out += buf;
      }
      out += "]}";
    }
    unsigned dt = this->count > 1 ? (unsigned) ((this->last_ms - this->first_ms) / (this->count - 1)) : 0;
    snprintf(buf, sizeof(buf), ",\"dt_ms\":%u,\"samples\":[", dt);
    out += buf;
    for (uint8_t i = 0; i < this->count; i++) {
      snprintf(buf, sizeof(buf), i ? ",%.0f" : "%.0f", this->samples[i]);
      out += buf;
    }
    out += "]}";
    return out;
  }
};

class StepDetector {
 public:
  static constexpr uint8_t PRE = 3;     // steady frames kept to lead into a step
  static constexpr uint8_t WINDOW = 8;  // the most frames the settle check looks back over
  static constexpr float DRIFT_GAIN = 0.1f;      // the slow average's weight on each frame (~10 frames)
  static constexpr float ANCHOR_GAIN = 1 / 600.0f;  // the anchor follows change slower than ~600 frames (5 min)
  static constexpr uint8_t HISTORY = 8;             // the most steps kept to see a load cycling

  explicit StepDetector(const char *key = "", StepConfig cfg = {}) : cfg_(cfg) {
    this->event_.key = key;
    this->event_.min_va = this->cfg_.min_va;
    this->cyc_event_.key = key;
    this->cyc_event_.min_va = this->cfg_.min_va;
    this->cfg_.cycle_steps = std::min<uint8_t>(this->cfg_.cycle_steps, HISTORY);
    this->cfg_.settle_frames = std::max<uint8_t>(2, std::min<uint8_t>(this->cfg_.settle_frames, WINDOW));
    this->cfg_.confirm = std::max<uint8_t>(1, std::min<uint8_t>(this->cfg_.confirm, WINDOW));
  }

  const StepConfig &config() const { return this->cfg_; }
  // The event feed() reported; next() moves on to a second from the same frame (a cycling load's end,
  // then the step that ended it), false when there is none.
  const LoadEvent &event() const { return *this->out_[this->out_i_]; }
  bool next() {
    if (this->out_i_ + 1 >= this->out_n_)
      return false;
    this->out_i_++;
    return true;
  }
  bool cycling() const { return this->cycling_; }
  // Watts a step must also clear, set each frame by the caller (a remainder's calibration mismatch
  // grows with what it is the remainder of).
  void set_extra_threshold(float w) { this->extra_w_ = w; }
  float level() const { return this->base_; }
  float noise() const { return this->noise_; }

  // Feed one frame (real power, apparent power or NAN); true when an event is
  // ready (read it from event(), then next() for any second one).
  bool feed(float w, float va, uint32_t ms) {
    this->out_n_ = this->out_i_ = 0;
    if (std::isnan(w))
      return false;
    // A cycling load's mean is weighted by time (W x ms), so the switches before it was recognised
    // can be counted in by how long each level lasted.
    uint32_t dt = this->last_ms_ ? std::min<uint32_t>(ms - this->last_ms_, 5000) : 0;
    this->last_ms_ = ms;
    this->ring_w_[this->ring_head_] = w * dt;
    this->ring_dt_[this->ring_head_] = dt;
    this->ring_head_ = (this->ring_head_ + 1) % WINDOW;
    if (this->cycling_) {
      this->cyc_sum_ += (double) w * dt;
      this->cyc_n_ += dt;
    }
    if (this->step(w, va, ms))
      this->route(ms);
    else if (this->cycling_)
      this->cycle_idle(ms);
    return this->out_n_ > 0;
  }

 protected:
  // The step detector itself: true when a step (or a gradual change) has finished, in event_.
  bool step(float w, float va, uint32_t ms) {
    if (!this->primed_) {
      this->primed_ = true;
      this->base_ = w;
      this->base_va_ = va;
      this->anchor(w, va);
      this->remember(w, va, ms);
      return false;
    }
    this->slow_ += DRIFT_GAIN * (w - this->slow_);
    if (!std::isnan(va))
      this->slow_va_ = std::isnan(this->slow_va_) ? va : this->slow_va_ + DRIFT_GAIN * (va - this->slow_va_);
    if (!this->moving_) {
      float off = std::fabs(w - this->base_);
      if (off <= this->threshold(this->base_, w)) {
        this->pending_ = 0;
        // Learn the level's usual wobble, and follow slow drift.
        this->noise_ += 0.05f * (off - this->noise_);
        this->base_ += 0.25f * (w - this->base_);
        if (!std::isnan(va))
          this->base_va_ = std::isnan(this->base_va_) ? va : this->base_va_ + 0.25f * (va - this->base_va_);
        this->remember(w, va, ms);
        this->held_add(w, ms);
        if (this->cycling_) {
          // A cycling load's mean moves as its duty does: that is not a change to report.
          this->anchor(this->slow_, this->slow_va_);
          return false;
        }
        return this->drifted(ms);
      }
      // Away from the level: wait for `confirm` frames in a row before calling it a step.
      this->pend_w_[this->pending_] = w;
      this->pend_va_[this->pending_] = va;
      this->pend_ms_[this->pending_] = ms;
      if (++this->pending_ < this->cfg_.confirm)
        return false;
      this->start();
      return false;
    }
    this->capture(w, va, ms);
    return this->check(ms);
  }

  // How far a frame must be from the level to count: symmetric, so a load
  // switching on and the same load switching off are both seen.
  float threshold(float level, float w) const {
    float smaller = std::min(std::fabs(level), std::fabs(w));
    // The wobble's part is capped so a jittery level does not hide a load the size of the cap switching,
    // but stays past the wobble itself, so the jitter is never a step.
    float wobble = this->cfg_.noise_factor * this->noise_;
    if (this->cfg_.max_noise_w > 0)
      wobble = std::min(wobble, std::max(this->cfg_.max_noise_w, 1.5f * this->noise_));
    return std::max({this->cfg_.min_step_w, this->cfg_.rel_step * smaller, wobble, this->extra_w_});
  }
  // What a slow average must move to be reported: twice a step by the level alone, without the wobble,
  // so steady creep the anchor lags behind does not come out as a stream of minimum-sized changes.
  float drift_threshold(float level, float w) const {
    return 2.0f * std::max({this->cfg_.min_step_w, this->cfg_.rel_step * std::min(std::fabs(level), std::fabs(w)), this->extra_w_});
  }

  // The level an event left: where gradual change is measured from.
  void anchor(float w, float va) {
    this->anchor_w_ = w;
    this->anchor_va_ = va;
    this->slow_ = w;
    this->slow_va_ = va;
    this->drift_n_ = 0;
  }

  // True when the slow average has stayed a step away from the anchor for drift_frames: reported as an
  // event from the anchor to the slow average, dated from when it first moved away.
  bool drifted(uint32_t ms) {
    if (this->cfg_.drift_frames == 0)
      return false;
    float gap = std::fabs(this->slow_ - this->anchor_w_), limit = this->drift_threshold(this->anchor_w_, this->slow_);
    if (gap <= limit) {
      // Change over many minutes (a heater sagging as it warms) is not a load switching: followed while
      // within a step, but not once a change is under way, which is measured from where it began.
      if (gap <= limit / 2)
        this->anchor_w_ += ANCHOR_GAIN * (this->slow_ - this->anchor_w_);
      if (gap <= limit / 2 && !std::isnan(this->slow_va_))
        this->anchor_va_ = std::isnan(this->anchor_va_) ? this->slow_va_ : this->anchor_va_ + ANCHOR_GAIN * (this->slow_va_ - this->anchor_va_);
      this->drift_n_ = 0;
      return false;
    }
    if (this->drift_n_++ == 0) {
      this->drift_ms_ = ms;
      this->drift_mid_ = this->slow_;
    }
    if (this->drift_n_ < this->cfg_.drift_frames)
      return false;
    // Still on its way (a fade not finished): wait for it to stop, up to six timeouts, so it is one
    // event. Stopped is moving less than a quarter of a step over the last drift_frames.
    float still = this->drift_threshold(this->anchor_w_, this->slow_) / 4.0f;
    if (std::fabs(this->slow_ - this->drift_mid_) > still && ms - this->drift_ms_ < 6 * this->cfg_.timeout_ms) {
      this->drift_n_ = 1;
      this->drift_mid_ = this->slow_;
      return false;
    }
    LoadEvent &e = this->event_;
    e.from_w = this->anchor_w_;
    e.va_from = this->anchor_va_;
    e.to_w = this->slow_;
    e.va_to = this->slow_va_;
    e.peak_w = this->slow_;
    e.noise_w = this->noise_;
    e.start_ms = this->drift_ms_;
    e.settle_ms = ms - this->drift_ms_;
    e.settled = false;
    e.pulse = false;
    e.drift = true;
    e.stage_count = 0;
    this->held_fill(e, this->drift_ms_);
    e.count = 0;
    for (uint8_t i = 0; i < this->pre_n_; i++) {
      uint8_t at = (this->pre_head_ + PRE - this->pre_n_ + i) % PRE;
      if (i == 0)
        e.first_ms = this->pre_ms_[at];
      e.samples[e.count++] = this->pre_w_[at];
      e.last_ms = this->pre_ms_[at];
    }
    this->base_ = this->slow_;
    if (!std::isnan(this->slow_va_))
      this->base_va_ = this->slow_va_;
    this->anchor(this->slow_, this->slow_va_);
    return true;
  }
  float tolerance(float level) const {
    return std::max({this->cfg_.settle_w, this->cfg_.settle_rel * std::fabs(level), 2.0f * this->noise_});
  }

  void remember(float w, float va, uint32_t ms) {
    this->pre_w_[this->pre_head_] = w;
    this->pre_va_[this->pre_head_] = va;
    this->pre_ms_[this->pre_head_] = ms;
    this->pre_head_ = (this->pre_head_ + 1) % PRE;
    this->pre_n_ = std::min<uint8_t>(this->pre_n_ + 1, PRE);
  }

  void start() {
    LoadEvent &e = this->event_;
    e.from_w = this->base_;
    e.va_from = this->base_va_;
    e.noise_w = this->noise_;
    e.peak_w = this->base_;
    e.start_ms = this->pend_ms_[0];
    e.drift = false;
    this->held_fill(e, e.start_ms);
    e.count = 0;
    for (uint8_t i = 0; i < this->pre_n_; i++) {
      uint8_t at = (this->pre_head_ + PRE - this->pre_n_ + i) % PRE;
      if (i == 0)
        e.first_ms = this->pre_ms_[at];
      e.samples[e.count++] = this->pre_w_[at];
      e.last_ms = this->pre_ms_[at];
    }
    if (e.count == 0)
      e.first_ms = this->pend_ms_[0];
    this->moving_ = true;
    this->win_n_ = 0;
    uint8_t n = this->pending_;
    this->pending_ = 0;
    this->lead_ = e.count;
    // A cycling load's mean stops where this step began: if the step ends the cycling, its frames are
    // not the cycling load's.
    this->step_sum_ = this->cyc_sum_;
    this->step_n_ = this->cyc_n_;
    for (uint8_t i = 0; i < n; i++) {  // the confirm frames, the latest n fed
      uint8_t at = (this->ring_head_ + WINDOW - 1 - i) % WINDOW;
      this->step_sum_ -= this->ring_w_[at];
      this->step_n_ = this->step_n_ > this->ring_dt_[at] ? this->step_n_ - this->ring_dt_[at] : 0;
    }
    for (uint8_t i = 0; i < n; i++)
      this->capture(this->pend_w_[i], this->pend_va_[i], this->pend_ms_[i]);
  }

  void capture(float w, float va, uint32_t ms) {
    LoadEvent &e = this->event_;
    if (e.count < LoadEvent::MAX_SAMPLES) {
      e.samples[e.count++] = w;
      e.last_ms = ms;
    }
    if (std::fabs(w - e.from_w) > std::fabs(e.peak_w - e.from_w))
      e.peak_w = w;
    uint8_t at = this->win_head_;
    this->win_w_[at] = w;
    this->win_va_[at] = va;
    this->win_ms_[at] = ms;
    this->win_head_ = (at + 1) % WINDOW;
    this->win_n_ = std::min<uint8_t>(this->win_n_ + 1, WINDOW);
  }

  // --- the stretch between events -------------------------------------------

  void held_add(float w, uint32_t ms) {
    if (this->held_n_ == 0) {
      this->held_ms_ = ms;
      this->held_lo_ = this->held_hi_ = w;
      this->held_sum_ = 0;
    }
    this->held_sum_ += w;
    this->held_n_++;
    this->held_lo_ = std::min(this->held_lo_, w);
    this->held_hi_ = std::max(this->held_hi_, w);
  }
  void held_fill(LoadEvent &e, uint32_t end_ms) {
    e.held_w = this->held_n_ ? this->held_sum_ / this->held_n_ : NAN;
    e.held_span_w = this->held_n_ ? this->held_hi_ - this->held_lo_ : NAN;
    e.held_ms = this->held_n_ ? end_ms - this->held_ms_ : 0;
    this->held_n_ = 0;
  }

  // Levels the step passed through for two frames or more, between where it started and where it settled.
  void stages(LoadEvent &e, uint8_t settled_frames) {
    e.stage_count = 0;
    int end = (int) e.count - settled_frames;
    float run_sum = 0;
    uint8_t run_n = 0;
    auto close = [&]() {
      if (run_n >= 2 && e.stage_count < LoadEvent::MAX_STAGES) {
        float m = run_sum / run_n;
        // Strictly between where it started and settled: an inrush's or overshoot's decay lies beyond.
        bool between = (m - e.from_w) * (e.to_w - m) > 0;
        if (between && std::fabs(m - e.from_w) > this->tolerance(e.from_w) && std::fabs(m - e.to_w) > this->tolerance(e.to_w)) {
          e.stage_w[e.stage_count] = m;
          e.stage_frames[e.stage_count++] = run_n;
        }
      }
      run_sum = 0;
      run_n = 0;
    };
    for (int i = this->lead_; i < end; i++) {
      float s = e.samples[i];
      if (run_n && std::fabs(s - run_sum / run_n) > this->tolerance(run_sum / run_n))
        close();
      run_sum += s;
      run_n++;
    }
    close();
  }

  // --- cycling ----------------------------------------------------------------

  void push(const LoadEvent *e) {
    if (this->out_n_ < 2)
      this->out_[this->out_n_++] = e;
  }

  // What to report of a step just found: itself, or nothing while a cycling load goes on switching,
  // or a cycling load's end and then the step that ended it, or the step and then cycling's start.
  void route(uint32_t ms) {
    LoadEvent &e = this->event_;
    e.cycling = 0;
    if (e.drift) {
      this->push(&e);
      return;
    }
    if (this->cycling_) {
      if (e.pulse) {
        if (this->short_on(e))
          this->pulse_toggle(e);
        return;
      }
      if (this->fits(e)) {
        this->toggle(e);
        return;
      }
      this->end_cycle(e.from_w, e.start_ms, this->step_sum_, this->step_n_);
      this->push(&e);
      this->remember_step(e);
      return;
    }
    this->push(&e);
    if (e.pulse)
      return;
    this->remember_step(e);
    if (this->cfg_.cycle_steps >= 2 && this->cycles())
      this->begin_cycle(ms);
  }

  void remember_step(const LoadEvent &e) {
    if (this->hist_n_ == HISTORY) {
      for (uint8_t i = 1; i < HISTORY; i++) {
        this->hist_ms_[i - 1] = this->hist_ms_[i];
        this->hist_dw_[i - 1] = this->hist_dw_[i];
        this->hist_low_[i - 1] = this->hist_low_[i];
      }
      this->hist_n_--;
    }
    this->hist_ms_[this->hist_n_] = e.start_ms;
    this->hist_dw_[this->hist_n_] = e.to_w - e.from_w;
    this->hist_low_[this->hist_n_++] = std::min(e.from_w, e.to_w);
  }

  // The last cycle_steps steps alternate up and down, alike in size, within the window.
  bool cycles() const {
    const uint8_t n = this->cfg_.cycle_steps;
    if (this->hist_n_ < n)
      return false;
    const uint8_t first = this->hist_n_ - n;
    if (this->hist_ms_[this->hist_n_ - 1] - this->hist_ms_[first] > this->cfg_.cycle_window_ms)
      return false;
    // Counted from a switch on: a long first on (a bed heating up from cold) then ends with a step down
    // that is still reported, so it is a run of its own.
    if (this->hist_dw_[first] <= 0)
      return false;
    float size = 0;
    for (uint8_t i = first; i < this->hist_n_; i++) {
      if (i > first && (this->hist_dw_[i] > 0) == (this->hist_dw_[i - 1] > 0))
        return false;
      size += std::fabs(this->hist_dw_[i]);
    }
    size /= n;
    float low = this->hist_low_[this->hist_n_ - 1];
    for (uint8_t i = first; i < this->hist_n_; i++) {
      if (std::fabs(std::fabs(this->hist_dw_[i]) - size) > this->cfg_.cycle_tolerance * size)
        return false;
      // ...and all between the same two levels (not a load switching on as the cycling one does).
      if (std::fabs(this->hist_low_[i] - low) > std::max(this->cfg_.min_step_w, 0.15f * size))
        return false;
    }
    return true;
  }

  void begin_cycle(uint32_t ms) {
    const uint8_t n = this->cfg_.cycle_steps, first = this->hist_n_ - n;
    this->cycling_ = true;
    this->cyc_low_ = this->hist_low_[this->hist_n_ - 1];
    this->cyc_amp_ = 0;
    this->cyc_high_ms_ = this->cyc_total_ms_ = 0;
    for (uint8_t i = first; i < this->hist_n_; i++) {
      this->cyc_amp_ += std::fabs(this->hist_dw_[i]) / n;
      if (i + 1 < this->hist_n_) {
        uint32_t gap = this->hist_ms_[i + 1] - this->hist_ms_[i];
        this->cyc_total_ms_ += gap;
        if (this->hist_dw_[i] > 0)
          this->cyc_high_ms_ += gap;
      }
    }
    this->cyc_toggles_ = n - 1;
    this->cyc_start_ms_ = this->hist_ms_[first];
    this->last_toggle_ms_ = this->hist_ms_[this->hist_n_ - 1];
    this->last_up_ = this->hist_dw_[this->hist_n_ - 1] > 0;
    // Counted in from its first switch: each level for as long as it lasted (a long heat-up at full power).
    this->cyc_sum_ = 0;
    for (uint8_t i = first; i < this->hist_n_; i++) {
      float level = this->hist_low_[i] + (this->hist_dw_[i] > 0 ? std::fabs(this->hist_dw_[i]) : 0.0f);
      uint32_t until = i + 1 < this->hist_n_ ? this->hist_ms_[i + 1] : ms;
      this->cyc_sum_ += (double) level * (until - this->hist_ms_[i]);
    }
    this->cyc_n_ = ms - this->hist_ms_[first];
    this->idle_sum_ = this->cyc_sum_;
    this->idle_n_ = this->cyc_n_;
    LoadEvent &c = this->cycle_event(1, this->cyc_low_, this->cyc_low_ + this->cyc_amp_ * this->duty_now(), this->cyc_start_ms_);
    c.spans_ms = ms - this->cyc_start_ms_;
    this->push(&c);
  }

  // A step that carries the cycling on: the other way from the last, about its size, and landing back on
  // the cycle's low or high level. Anything else (another load switching as the cycling one does) ends it.
  bool fits(const LoadEvent &e) const {
    float dw = e.to_w - e.from_w;
    if ((dw > 0) == this->last_up_ || std::fabs(std::fabs(dw) - this->cyc_amp_) > this->cfg_.cycle_tolerance * this->cyc_amp_)
      return false;
    float level = dw > 0 ? this->cyc_low_ + this->cyc_amp_ : this->cyc_low_;
    return std::fabs(e.to_w - level) <= std::max(this->cfg_.min_step_w, 0.15f * this->cyc_amp_);
  }

  // A pulse from the low level up to the high and back: an on too short to settle (a bed near its setpoint).
  bool short_on(const LoadEvent &e) const {
    float near = std::max(this->cfg_.min_step_w, 0.15f * this->cyc_amp_);
    return !this->last_up_ && std::fabs(e.from_w - this->cyc_low_) <= near &&
           std::fabs(e.peak_w - (this->cyc_low_ + this->cyc_amp_)) <= near;
  }
  void pulse_toggle(const LoadEvent &e) {
    uint32_t width = e.settle_ms;
    this->cyc_total_ms_ += (e.start_ms - this->last_toggle_ms_) + width;
    this->cyc_high_ms_ += width;
    this->cyc_toggles_ += 2;
    this->last_toggle_ms_ = e.start_ms + width;
    this->idle_sum_ = this->cyc_sum_;
    this->idle_n_ = this->cyc_n_;
  }

  void toggle(const LoadEvent &e) {
    uint32_t gap = e.start_ms - this->last_toggle_ms_;
    this->cyc_total_ms_ += gap;
    if (this->last_up_)
      this->cyc_high_ms_ += gap;
    this->cyc_toggles_++;
    this->last_toggle_ms_ = e.start_ms;
    this->last_up_ = !this->last_up_;
    if (!this->last_up_)
      this->cyc_low_ += 0.25f * (e.to_w - this->cyc_low_);
    this->cyc_amp_ += 0.2f * (std::fabs(e.to_w - e.from_w) - this->cyc_amp_);
    this->idle_sum_ = this->cyc_sum_;
    this->idle_n_ = this->cyc_n_;
  }

  // Off at the low level, and no switch for three of its usual gaps (or a timeout): it has stopped.
  // On at the high level it is still going, however long (a bed holding full power).
  void cycle_idle(uint32_t ms) {
    if (this->last_up_)
      return;
    uint32_t usual = this->cyc_toggles_ ? this->cyc_total_ms_ / this->cyc_toggles_ : 0;
    // Three usual gaps, but never longer than the window (one long hold at full power makes "usual" huge).
    uint32_t wait = std::max<uint32_t>(std::min<uint32_t>(3 * usual, this->cfg_.cycle_window_ms), this->cfg_.timeout_ms);
    if (ms - this->last_toggle_ms_ > wait)
      this->end_cycle(this->base_, this->last_toggle_ms_, this->idle_sum_, this->idle_n_);  // not the quiet tail
  }

  // Ends the cycling at at_ms; sum and n are its frames up to then.
  void end_cycle(float to_w, uint32_t at_ms, double sum, uint32_t n) {
    float mean = n ? (float) (sum / n) : this->cyc_low_ + this->cyc_amp_ * this->duty_now();
    LoadEvent &c = this->cycle_event(2, mean, to_w, at_ms);
    c.held_w = mean;
    c.held_span_w = this->cyc_amp_;
    c.held_ms = at_ms - this->cyc_start_ms_;
    this->push(&c);
    this->cycling_ = false;
    this->hist_n_ = 0;
    this->held_n_ = 0;
    this->anchor(this->base_, this->base_va_);
  }

  float duty_now() const { return this->cyc_total_ms_ ? (float) this->cyc_high_ms_ / this->cyc_total_ms_ : 0.5f; }

  LoadEvent &cycle_event(uint8_t kind, float from_w, float to_w, uint32_t start_ms) {
    LoadEvent &c = this->cyc_event_;
    c.cycling = kind;
    c.from_w = from_w;
    c.to_w = to_w;
    c.peak_w = this->cyc_low_ + this->cyc_amp_;
    c.va_from = c.va_to = NAN;  // a switching load's apparent power averages to nothing useful
    c.noise_w = this->noise_;
    c.start_ms = start_ms;
    c.settle_ms = 0;
    c.settled = true;
    c.pulse = c.drift = false;
    c.held_w = c.held_span_w = NAN;
    c.held_ms = 0;
    c.stage_count = 0;
    c.count = 0;
    c.first_ms = c.last_ms = start_ms;
    c.amp_w = this->cyc_amp_;
    c.duty = this->duty_now();
    c.period_ms = this->cyc_toggles_ ? 2 * this->cyc_total_ms_ / this->cyc_toggles_ : 0;
    c.spans_ms = 0;
    return c;
  }

  // How many times the last `n` captured frames turned back (rising then falling, or the reverse).
  uint8_t turns(uint8_t n) const {
    uint8_t count = 0;
    float last = 0;
    for (uint8_t i = 1; i < n; i++) {
      float d = this->win_w_[this->recent(i, n)] - this->win_w_[this->recent(i - 1, n)];
      if (d != 0 && last != 0 && (d > 0) != (last > 0))
        count++;
      if (d != 0)
        last = d;
    }
    return count;
  }

  // The last `n` captured frames, oldest first.
  uint8_t recent(uint8_t i, uint8_t n) const { return (this->win_head_ + WINDOW - n + i) % WINDOW; }

  bool check(uint32_t ms) {
    LoadEvent &e = this->event_;
    const uint8_t n = this->cfg_.settle_frames;
    bool timed_out = ms - e.start_ms >= this->cfg_.timeout_ms;
    if (this->win_n_ < n && !timed_out)
      return false;
    uint8_t have = std::min(this->win_n_, n);
    float lo = INFINITY, hi = -INFINITY, sum = 0, va_sum = 0;
    uint8_t va_n = 0;
    for (uint8_t i = 0; i < have; i++) {
      uint8_t at = this->recent(i, have);
      lo = std::min(lo, this->win_w_[at]);
      hi = std::max(hi, this->win_w_[at]);
      sum += this->win_w_[at];
      if (!std::isnan(this->win_va_[at])) {
        va_sum += this->win_va_[at];
        va_n++;
      }
    }
    float mean = sum / have;
    bool settled = have == n && hi - lo <= this->tolerance(mean);
    if (!settled && !timed_out)
      return false;
    e.to_w = mean;
    e.va_to = va_n ? va_sum / va_n : NAN;
    e.settled = settled;
    e.pulse = std::fabs(mean - e.from_w) < this->threshold(e.from_w, mean);
    e.settle_ms = this->win_ms_[this->recent(0, have)] - e.start_ms;
    this->stages(e, have);
    if (e.pulse || !settled)  // a step still moving at the timeout has no levels to tell
      e.stage_count = 0;
    // The settled frames are the new level and lead into the next step.
    this->base_ = mean;
    if (va_n)
      this->base_va_ = e.va_to;
    this->anchor(mean, this->base_va_);
    // A level that never settled wobbles at least this much: an oscillating
    // load would otherwise fire a timed-out event every timeout.
    // A ramp (an EV charger stepping up) times out too, but is not wobble: learning it would make
    // every later stretch of the ramp a "settled" step. Wobble turns back and forth.
    if (!settled && this->turns(this->win_n_) >= 2)
      this->noise_ = std::max(this->noise_, (hi - lo) / 2.0f);
    this->pre_n_ = 0;
    this->pre_head_ = 0;
    for (uint8_t i = std::max<int>(0, have - PRE); i < have; i++) {
      uint8_t at = this->recent(i, have);
      this->remember(this->win_w_[at], this->win_va_[at], this->win_ms_[at]);
    }
    this->moving_ = false;
    return true;
  }

  StepConfig cfg_;
  LoadEvent event_;
  LoadEvent cyc_event_;
  const LoadEvent *out_[2]{&event_, &event_};
  uint8_t out_n_{0}, out_i_{0};
  uint8_t lead_{0};                        // frames before the step at the head of its samples
  float held_sum_{0}, held_lo_{0}, held_hi_{0};
  uint32_t held_n_{0}, held_ms_{0};
  uint32_t hist_ms_[HISTORY]{};
  float hist_dw_[HISTORY]{}, hist_low_[HISTORY]{};
  uint8_t hist_n_{0};
  bool cycling_{false}, last_up_{false};
  float cyc_low_{0}, cyc_amp_{0};
  double cyc_sum_{0}, step_sum_{0}, idle_sum_{0};  // W x ms: all of it, where the latest step began, at its last switch
  uint32_t step_n_{0}, idle_n_{0};                 // and the ms each covers
  uint32_t last_ms_{0};
  float ring_w_[WINDOW]{};                         // the latest frames' W x ms and ms, to take the confirm frames back out
  uint32_t ring_dt_[WINDOW]{};
  uint8_t ring_head_{0};
  uint32_t cyc_n_{0}, cyc_start_ms_{0}, last_toggle_ms_{0}, cyc_high_ms_{0}, cyc_total_ms_{0}, cyc_toggles_{0};
  bool primed_{false};
  bool moving_{false};
  float base_{0};
  float base_va_{NAN};
  float noise_{0};
  float extra_w_{0};
  float slow_{0}, slow_va_{NAN};          // the slow average of every frame
  float anchor_w_{0}, anchor_va_{NAN};    // the level the last event left
  uint8_t drift_n_{0};                    // frames the slow average has been a step away from it
  uint32_t drift_ms_{0};                  // when it first was
  float drift_mid_{0};                    // the slow average drift_frames ago, to see it has stopped moving
  uint8_t pending_{0};
  float pend_w_[WINDOW]{}, pend_va_[WINDOW]{};
  uint32_t pend_ms_[WINDOW]{};
  float pre_w_[PRE]{}, pre_va_[PRE]{};
  uint32_t pre_ms_[PRE]{};
  uint8_t pre_head_{0}, pre_n_{0};
  float win_w_[WINDOW]{}, win_va_[WINDOW]{};
  uint32_t win_ms_[WINDOW]{};
  uint8_t win_head_{0}, win_n_{0};
};

}  // namespace load_events
}  // namespace esphome
