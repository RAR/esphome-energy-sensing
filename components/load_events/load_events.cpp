#include "load_events.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cmath>
#include <cstring>

namespace esphome {
namespace load_events {

static const char *const TAG = "load_events";

static constexpr uint32_t WITH_WITHIN_MS = 600;     // other channels' events this close are tagged as "with"
static constexpr uint32_t REPLAY_AFTER_MS = 2000;   // mqtt on_connect runs before a publish can go out
static constexpr float REMAINDER_MISMATCH = 0.02f;  // meters differ by a few percent: a remainder's step must clear it

Channel::Input *Channel::watch_(sensor::Sensor *s) {
  auto *input = new Input{s};  // NOLINT
  s->add_on_raw_state_callback([this, input](float value) {
    input->value = value;
    this->dirty_ = true;
  });
  return input;
}

float Channel::power() const {
  float sum = 0;
  for (auto *input : this->power_) {
    if (std::isnan(input->value))
      return NAN;
    sum += input->value;
  }
  return this->power_.empty() ? NAN : sum;
}

float Channel::apparent_power() const {
  float sum = 0;
  for (auto *input : this->apparent_) {
    if (std::isnan(input->value))
      return NAN;
    sum += input->value;
  }
  return this->apparent_.empty() ? NAN : sum;
}

#ifdef USE_LOAD_EVENTS_CYCLES
void Channel::feed_cycle(const emporiavue::MeteringCycle &cycle) {
  CycleSample sample;
  sample.t_ms = cycle.timestamp_ms;
  sample.period_ms = cycle.period_ms;
  float power = 0;
  for (uint8_t port : this->cycle_ports_) {
    if (port >= 19)
      return;
    const emporiavue::MeteringCycleClamp &clamp = cycle.clamps[port];
    if (clamp.samples == 0)
      return;
    // Its leg: the phase its power is read against, or (several, while lines are detected) the one it
    // draws most against this cycle.
    uint8_t phase = 0;
    const uint8_t mask = clamp.voltage_mask;
    if (mask == 1 || mask == 2 || mask == 4) {
      phase = mask == 1 ? 0 : mask == 2 ? 1 : 2;
    } else {
      for (uint8_t p = 1; p < 3; p++) {
        if (std::fabs(clamp.power_raw_by_phase[p]) > std::fabs(clamp.power_raw_by_phase[phase]))
          phase = p;
      }
    }
    power += clamp.power_raw_by_phase[phase];
    // Each harmonic turned back by k times its leg voltage's angle, and negated (the meter counts
    // power as -(i * v)), so a consuming load's current in phase with its voltage reads +i.
    const float angle = std::atan2(cycle.voltage_q[phase], cycle.voltage_i[phase]);
    for (uint8_t k = 0; k < CYCLE_HARMONICS; k++) {
      const float turn = CYCLE_HARMONIC_ORDER[k] * angle;
      const float c = std::cos(turn), s = std::sin(turn);
      const float hi = clamp.harmonic_i[k], hq = clamp.harmonic_q[k];
      sample.h_i[k] -= hi * c + hq * s;
      sample.h_q[k] -= hq * c - hi * s;
    }
  }
  sample.power_raw = power;
  LockGuard guard(this->cycle_lock_);
  this->features_->feed(sample);
}

void Channel::calibrate_cycles(float watts) {
  LockGuard guard(this->cycle_lock_);
  this->features_->calibrate(watts);
}

void Channel::attach_cycle(LoadEvent &event) {
  if (event.cycling || event.drift)
    return;
  CycleStep step;
  {
    LockGuard guard(this->cycle_lock_);
    if (!this->features_->take(event.to_w - event.from_w, event.start_ms, &step))
      return;
  }
  event.has_cycle = true;
  event.cycle_dw = step.dw;
  event.inrush = step.inrush;
  event.inrush_ms = step.inrush_ms;
  event.dpf = step.dpf;
  for (uint8_t i = 0; i < 3; i++)
    event.harmonics[i] = step.h[i];
  event.cycle_lead_ms = static_cast<int32_t>(event.start_ms - step.t_ms);
}
#endif

#ifdef USE_LOAD_EVENTS_CYCLES
void LoadEventsComponent::set_meter(emporiavue::EmporiaVueComponent *meter) {
  // Runs in the meter's SPI task, once per mains cycle; the channels are all added before it starts.
  meter->add_on_cycle_callback([this](const emporiavue::MeteringCycle &cycle) {
    for (auto *channel : this->channels_) {
      if (channel->has_cycles())
        channel->feed_cycle(cycle);
    }
  });
}
#endif

void LoadEventsComponent::loop() {
  bool any = false;
  for (auto *channel : this->channels_)
    any |= channel->dirty();
  if (!any)
    return;
  const uint32_t now = millis();
  for (auto *channel : this->channels_) {
    if (!channel->dirty())
      continue;
    StepDetector &detector = channel->detector();
#ifdef USE_LOAD_EVENTS_CYCLES
    if (channel->has_cycles())
      channel->calibrate_cycles(channel->power());
#endif
    if (detector.feed(channel->power(), channel->apparent_power(), now)) {
      do {
        LoadEvent event = detector.event();
#ifdef USE_LOAD_EVENTS_CYCLES
        if (channel->has_cycles())
          channel->attach_cycle(event);
#endif
        this->report_(event);
      } while (detector.next());
    }
  }
  if (this->remainder_ != nullptr && this->remainder_whole_->dirty()) {
    float watts = this->remainder_whole_->power(), parts = 0;
    for (auto *part : this->remainder_parts_) {
      float w = part->power();
      if (!std::isnan(w)) {
        watts -= w;
        parts += std::fabs(w);
      }
    }
    // A 7 kW load on a metered breaker would otherwise leave a 150 W "step" in the remainder.
    this->remainder_->set_extra_threshold(REMAINDER_MISMATCH * parts);
    if (this->remainder_->feed(watts, NAN, now)) {
      do
        this->report_(this->remainder_->event());
      while (this->remainder_->next());
    }
  }
  for (auto *channel : this->channels_)
    channel->clean();
}

void LoadEventsComponent::report_(const LoadEvent &event) {
  ESP_LOGD(TAG, "Load event on %s: %.0f W -> %.0f W (peak %.0f W, settled in %u ms)", event.key, event.from_w,
           event.to_w, event.peak_w, (unsigned) event.settle_ms);
  LoadEvent reported = event;
  reported.reported_ms = millis();
  reported.seq = ++this->seq_;
  reported.replay = false;
  // Other channels' events that began within a frame or so of this one, from those just sent.
  reported.with_count = 0;
  for (uint8_t i = 0; i < this->sent_n_ && reported.with_count < LoadEvent::MAX_WITH; i++) {
    const LoadEvent &other = this->sent_[(this->sent_head_ + REPLAY - 1 - i) % REPLAY];
    uint32_t apart = other.start_ms > event.start_ms ? other.start_ms - event.start_ms : event.start_ms - other.start_ms;
    if (apart <= WITH_WITHIN_MS && strcmp(other.key, event.key) != 0 && !other.cycling && !event.cycling)
      reported.with_keys[reported.with_count++] = other.key;
  }
  this->sent_[this->sent_head_] = reported;
  this->sent_head_ = (this->sent_head_ + 1) % REPLAY;
  this->sent_n_ = std::min<uint8_t>(this->sent_n_ + 1, REPLAY);
  this->callback_.call(reported);
}

void LoadEventsComponent::replay() {
  this->set_timeout("replay", REPLAY_AFTER_MS, [this]() { this->replay_now_(); });
}

void LoadEventsComponent::replay_now_() {
  ESP_LOGD(TAG, "Replaying the last %u load events", (unsigned) this->sent_n_);
  const uint32_t now = millis();
  for (uint8_t i = 0; i < this->sent_n_; i++) {
    LoadEvent again = this->sent_[(this->sent_head_ + REPLAY - this->sent_n_ + i) % REPLAY];
    again.replay = true;
    again.reported_ms = now;  // its age is still counted from when it began
    this->callback_.call(again);
  }
}

}  // namespace load_events
}  // namespace esphome
