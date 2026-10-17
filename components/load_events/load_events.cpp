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
static constexpr uint32_t TELL_EVERY_MS = 60000;    // how often missing frames are logged

#ifdef USE_LOAD_EVENTS_EMPORIAVUE
void LoadEventsComponent::set_meter(emporiavue::EmporiaVueComponent *meter) {
  this->framed_ = true;
  // Called just before the meter publishes the frame's sensors (or skips publishing a backlogged one).
  meter->add_on_frame_callback([this](const emporiavue::EmporiaVueComponent::MeteringFrame &frame) {
    if (this->have_seq_ &&
        (frame.acquisition_epoch != this->last_epoch_ || frame.sequence - this->last_seq_ != 1U))
      this->gaps_++;
    this->have_seq_ = true;
    this->last_seq_ = frame.sequence;
    this->last_epoch_ = frame.acquisition_epoch;
    this->frame_ms_ = frame.timestamp_ms;
    this->frames_++;
  });
}
#endif

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

void LoadEventsComponent::loop() {
  if (this->framed_ && (this->unpublished_ || this->gaps_) && millis() - this->told_ms_ >= TELL_EVERY_MS) {
    ESP_LOGD(TAG, "Last minute: %u meter windows not published (each folded into the next step), %u gaps in its frames",
             (unsigned) this->unpublished_, (unsigned) this->gaps_);
    this->unpublished_ = this->gaps_ = 0;
    this->told_ms_ = millis();
  }
  bool any = false;
  for (auto *channel : this->channels_)
    any |= channel->dirty();
  if (!any)
    return;
  // With the meter's frames, the step is timed by when its frame was measured.
  const uint32_t now = this->framed_ && this->frames_ ? this->frame_ms_ : millis();
  if (this->frames_ > 1)
    this->unpublished_ += this->frames_ - 1;
  this->frames_ = 0;
  for (auto *channel : this->channels_) {
    if (!channel->dirty())
      continue;
    StepDetector &detector = channel->detector();
    if (detector.feed(channel->power(), channel->apparent_power(), now)) {
      do
        this->report_(detector.event());
      while (detector.next());
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
