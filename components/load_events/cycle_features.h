#pragma once

// A channel's steps found one mains cycle at a time, with what a frame-level
// step cannot see: the cycle it began in, its inrush (the peak cycle's change
// in fundamental current against the settled change) and how long that took to
// pass, and the load's own current harmonics (3rd, 5th, 7th against the
// fundamental) and displacement power factor, taken from the change in the
// current's harmonic vectors across the step. Plain C++ with no ESPHome
// dependencies, tested on the host (tests/test_cycle_features.cpp).
//
// It is fed one CycleSample per mains cycle, with the current's harmonics
// already referenced to its leg's voltage (in phase and consuming: +i), so the
// legs of a 240 V load add. Power arrives in the meter's raw units and is
// scaled to watts by calibrate(), from the watts the meter publishes.
//
// Not thread safe: the caller serialises feed() (the meter's task) with
// calibrate() and take() (the main loop).

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace esphome {
namespace load_events {

static constexpr uint8_t CYCLE_HARMONICS = 4;  // orders 1, 3, 5, 7
static constexpr uint8_t CYCLE_HARMONIC_ORDER[CYCLE_HARMONICS] = {1, 3, 5, 7};

struct CycleSample {
  uint32_t t_ms{0};
  float power_raw{NAN};   // the cycle's mean power, in the meter's raw units (consuming: +)
  float period_ms{0};
  float h_i[CYCLE_HARMONICS]{}, h_q[CYCLE_HARMONICS]{};  // current harmonics (amplitude), referenced to the voltage
};

struct CycleStep {
  uint32_t t_ms{0};         // the first cycle that moved
  float dw{NAN};            // after - before (W)
  float inrush{NAN};        // peak |dI1| over the settled |dI1|; NAN for a step down
  uint16_t inrush_ms{0};    // from the first cycle until |dI1| stays within INRUSH_OVER of settled
  float h[3]{NAN, NAN, NAN};  // the load's |I3|, |I5|, |I7| over |I1|
  float dpf{NAN};           // the load's displacement power factor (cos of its current's angle)
};

class CycleFeatures {
 public:
  static constexpr uint8_t RING = 128;        // ~2.1 s of 60 Hz cycles
  static constexpr uint8_t BASE = 12;         // cycles that make the level before
  static constexpr uint8_t AFTER_FROM = 40;   // the settled level: cycles 40..59 after the start (0.67-1 s)
  static constexpr uint8_t AFTER_TO = 60;
  static constexpr uint8_t KEEP = 4;          // steps kept until an event takes them
  static constexpr float NOISE_FACTOR = 4.0f;
  static constexpr float INRUSH_OVER = 1.15f;
  static constexpr float MIN_FUNDAMENTAL = 1e-6f;

  explicit CycleFeatures(float min_step_w = 30.0f) : min_step_w_(min_step_w) {}

  float scale() const { return this->scale_; }
  void set_scale(float w_per_raw) { this->scale_ = w_per_raw; }

  // The watts the meter published for this channel's latest frame: matched against the cycles fed since
  // the last call, it keeps the raw-to-watts scale (only over steady stretches with enough power).
  void calibrate(float watts) {
    const uint16_t n = this->since_n_;
    const float mean = n ? this->since_sum_ / n : NAN;
    const float spread = this->since_max_ - this->since_min_;
    this->since_n_ = 0;
    this->since_sum_ = 0;
    if (std::isnan(watts) || n < 4 || std::fabs(watts) < CALIBRATE_MIN_W || std::fabs(mean) <= 0)
      return;
    if (spread > CALIBRATE_STEADY * std::fabs(mean))
      return;
    const float k = watts / mean;
    if (!(k > 0) || !std::isfinite(k))
      return;
    this->scale_ = this->scale_ > 0 ? this->scale_ + CALIBRATE_RATE * (k - this->scale_) : k;
  }

  void feed(const CycleSample &c) {
    if (std::isnan(c.power_raw))
      return;
    if (this->since_n_ == 0) {
      this->since_min_ = this->since_max_ = c.power_raw;
    } else {
      this->since_min_ = std::min(this->since_min_, c.power_raw);
      this->since_max_ = std::max(this->since_max_, c.power_raw);
    }
    this->since_sum_ += c.power_raw;
    this->since_n_ = static_cast<uint16_t>(std::min<int>(this->since_n_ + 1, 60000));

    this->ring_[this->head_] = c;
    this->head_ = static_cast<uint8_t>((this->head_ + 1) % RING);
    if (this->count_ < RING)
      this->count_++;
    if (this->cooldown_ > 0)
      this->cooldown_--;
    if (this->scale_ <= 0)
      return;

    if (this->watching_) {
      this->watched_++;
      if (this->watched_ >= AFTER_TO)
        this->finish_();
      return;
    }
    if (this->cooldown_ > 0 || this->count_ < BASE + 1)
      return;
    float base, noise;
    this->level_(1, BASE, &base, &noise);
    const float w = c.power_raw * this->scale_;
    if (std::fabs(w - base) > this->threshold_(noise)) {
      this->watching_ = true;
      this->watched_ = 1;  // cycles seen from the start, itself the first: it is watched_ - 1 back
      this->base_w_ = base;
      this->base_noise_ = noise;
      this->vector_mean_(1, BASE, this->base_i_, this->base_q_);
    }
  }

  // The kept step that fits an event of dw watts which began near near_ms (the event's clock runs behind
  // the cycles' by its frames' delay), the closest in size; removed once taken.
  bool take(float dw, uint32_t near_ms, CycleStep *out) {
    int best = -1;
    float best_off = 0;
    for (uint8_t i = 0; i < this->kept_n_; i++) {
      const CycleStep &s = this->kept_[i];
      if ((s.dw > 0) != (dw > 0))
        continue;
      const int32_t lead = static_cast<int32_t>(near_ms - s.t_ms);
      if (lead < -MATCH_AFTER_MS || lead > MATCH_BEFORE_MS)
        continue;
      const float off = std::fabs(s.dw - dw);
      if (off > std::max(MATCH_REL * std::fabs(dw), 2.0f * this->min_step_w_))
        continue;
      if (best < 0 || off < best_off) {
        best = i;
        best_off = off;
      }
    }
    if (best < 0)
      return false;
    *out = this->kept_[best];
    for (uint8_t i = static_cast<uint8_t>(best); i + 1 < this->kept_n_; i++)
      this->kept_[i] = this->kept_[i + 1];
    this->kept_n_--;
    return true;
  }

  uint8_t kept() const { return this->kept_n_; }
  const CycleStep &kept_step(uint8_t i) const { return this->kept_[i]; }

 protected:
  static constexpr float CALIBRATE_MIN_W = 20.0f;
  static constexpr float CALIBRATE_STEADY = 0.10f;  // the cycles' spread over their mean
  static constexpr float CALIBRATE_RATE = 0.1f;
  static constexpr int32_t MATCH_BEFORE_MS = 3000;  // the event's start after the step's first cycle, at most
  static constexpr int32_t MATCH_AFTER_MS = 1000;   // ...or before it, at most
  static constexpr float MATCH_REL = 0.35f;

  // The cycle `back` cycles before the newest (0 = the newest).
  const CycleSample &at_(uint8_t back) const { return this->ring_[(this->head_ + RING - 1 - back) % RING]; }

  float threshold_(float noise) const { return std::max(this->min_step_w_, NOISE_FACTOR * noise); }

  // Mean watts and mean absolute deviation of the cycles from `from` back, `n` of them.
  void level_(uint8_t from, uint8_t n, float *mean, float *noise) const {
    float sum = 0;
    for (uint8_t i = 0; i < n; i++)
      sum += this->at_(from + i).power_raw;
    *mean = sum / n * this->scale_;
    float dev = 0;
    for (uint8_t i = 0; i < n; i++)
      dev += std::fabs(this->at_(from + i).power_raw * this->scale_ - *mean);
    *noise = dev / n;
  }

  void vector_mean_(uint8_t from, uint8_t n, float *mi, float *mq) const {
    for (uint8_t k = 0; k < CYCLE_HARMONICS; k++) {
      mi[k] = 0;
      mq[k] = 0;
    }
    for (uint8_t i = 0; i < n; i++) {
      const CycleSample &c = this->at_(from + i);
      for (uint8_t k = 0; k < CYCLE_HARMONICS; k++) {
        mi[k] += c.h_i[k];
        mq[k] += c.h_q[k];
      }
    }
    for (uint8_t k = 0; k < CYCLE_HARMONICS; k++) {
      mi[k] /= n;
      mq[k] /= n;
    }
  }

  void finish_() {
    this->watching_ = false;
    this->cooldown_ = BASE;
    // The newest cycle is AFTER_TO - 1 after the start: the start is AFTER_TO - 1 back.
    const uint8_t start_back = AFTER_TO - 1;
    float after, after_noise;
    this->level_(0, AFTER_TO - AFTER_FROM, &after, &after_noise);
    const float dw = after - this->base_w_;
    if (std::fabs(dw) <= this->threshold_(std::max(this->base_noise_, after_noise)))
      return;  // it came back (a pulse, an inrush with nothing after) or was noise
    float ai[CYCLE_HARMONICS], aq[CYCLE_HARMONICS];
    this->vector_mean_(0, AFTER_TO - AFTER_FROM, ai, aq);
    // The load's own current: what appeared (a step up) or what went away (a step down).
    const float sign = dw > 0 ? 1.0f : -1.0f;
    float di[CYCLE_HARMONICS], dq[CYCLE_HARMONICS], mag[CYCLE_HARMONICS];
    for (uint8_t k = 0; k < CYCLE_HARMONICS; k++) {
      di[k] = sign * (ai[k] - this->base_i_[k]);
      dq[k] = sign * (aq[k] - this->base_q_[k]);
      mag[k] = std::sqrt(di[k] * di[k] + dq[k] * dq[k]);
    }
    CycleStep s;
    s.t_ms = this->at_(start_back).t_ms;
    s.dw = dw;
    if (mag[0] > MIN_FUNDAMENTAL) {
      for (uint8_t k = 1; k < CYCLE_HARMONICS; k++)
        s.h[k - 1] = mag[k] / mag[0];
      s.dpf = std::max(-1.0f, std::min(1.0f, di[0] / mag[0]));
      if (dw > 0) {
        // Each cycle's change in fundamental current from before, against the settled change.
        float peak = 0;
        uint8_t last_over = 0;
        float elapsed = 0, until = 0;
        for (uint8_t j = 0; j < AFTER_FROM; j++) {
          const CycleSample &c = this->at_(static_cast<uint8_t>(start_back - j));
          const float ci = c.h_i[0] - this->base_i_[0], cq = c.h_q[0] - this->base_q_[0];
          const float d = std::sqrt(ci * ci + cq * cq) / mag[0];
          elapsed += c.period_ms;
          peak = std::max(peak, d);
          if (d > INRUSH_OVER) {
            last_over = static_cast<uint8_t>(j + 1);
            until = elapsed;
          }
        }
        s.inrush = peak;
        s.inrush_ms = last_over ? static_cast<uint16_t>(until + 0.5f) : 0;
      }
    }
    if (this->kept_n_ == KEEP) {
      for (uint8_t i = 0; i + 1 < KEEP; i++)
        this->kept_[i] = this->kept_[i + 1];
      this->kept_n_--;
    }
    this->kept_[this->kept_n_++] = s;
  }

  float min_step_w_;
  float scale_{0};
  CycleSample ring_[RING]{};
  uint8_t head_{0}, count_{0};
  uint8_t cooldown_{0};
  bool watching_{false};
  uint8_t watched_{0};
  float base_w_{0}, base_noise_{0};
  float base_i_[CYCLE_HARMONICS]{}, base_q_[CYCLE_HARMONICS]{};
  CycleStep kept_[KEEP]{};
  uint8_t kept_n_{0};
  // The cycles since the last calibrate(): their sum, count and range (raw).
  float since_sum_{0}, since_min_{0}, since_max_{0};
  uint16_t since_n_{0};
};

}  // namespace load_events
}  // namespace esphome
