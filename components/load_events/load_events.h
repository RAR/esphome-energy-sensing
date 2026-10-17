#pragma once

// Load events from any meter's sensors: each channel sums the power (and apparent
// power) sensors it is given, frame by frame, and finds the steps in it with the
// step detector. A frame is the set of readings a meter publishes together; the
// component takes the sensors' raw states (before filters) and runs the detectors
// once per loop after any of them changed, so one frame is one detector step.
//
// Given an emporiavue meter (emporiavue_id), it also listens to that meter's
// frame hook: each step is then timed by when its frame was measured, not when
// the loop got to it, and windows the meter measured but did not publish (it
// publishes only the last of a backlog) or never delivered are counted and logged.

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

#include "step_detector.h"

#ifdef USE_LOAD_EVENTS_EMPORIAVUE
#include "esphome/components/emporiavue/emporiavue.h"
#endif

#include <vector>

namespace esphome {
namespace load_events {

class Channel {
 public:
  Channel(const char *key, StepConfig cfg) : detector_(key, cfg) {}

  void add_power(sensor::Sensor *s) { this->power_.push_back(this->watch_(s)); }
  void add_apparent_power(sensor::Sensor *s) { this->apparent_.push_back(this->watch_(s)); }

  // True when a sensor of this channel published since the last frame.
  bool dirty() const { return this->dirty_; }
  // This frame's sums: NAN when any power input has no reading yet (or reads NAN);
  // apparent power NAN when it has no inputs or any of them reads NAN.
  float power() const;
  float apparent_power() const;
  StepDetector &detector() { return this->detector_; }
  void clean() { this->dirty_ = false; }

 protected:
  struct Input {
    sensor::Sensor *sensor;
    float value{NAN};
  };
  Input *watch_(sensor::Sensor *s);

  StepDetector detector_;
  std::vector<Input *> power_, apparent_;
  bool dirty_{false};
};

class LoadEventsComponent : public Component {
 public:
  void add_channel(Channel *channel) { this->channels_.push_back(channel); }
  // A remainder: the first channel's power less the others' (loads on breakers with no meter of their own).
  void set_remainder(const char *key, StepConfig cfg, Channel *whole, std::vector<Channel *> parts) {
    this->remainder_ = new StepDetector(key, cfg);  // NOLINT
    this->remainder_whole_ = whole;
    this->remainder_parts_ = std::move(parts);
  }

  void loop() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
#ifdef USE_LOAD_EVENTS_EMPORIAVUE
  void set_meter(emporiavue::EmporiaVueComponent *meter);
#endif

  void add_on_load_event_callback(std::function<void(const LoadEvent &)> &&callback) {
    this->callback_.add(std::move(callback));
  }
  // The last events again, marked as repeats: call from mqtt on_connect so none are lost to a reconnect.
  void replay();

 protected:
  void report_(const LoadEvent &event);
  void replay_now_();

  std::vector<Channel *> channels_;
  StepDetector *remainder_{nullptr};
  Channel *remainder_whole_{nullptr};
  std::vector<Channel *> remainder_parts_;

  CallbackManager<void(const LoadEvent &)> callback_;

  // The meter's frames (emporiavue_id): when the frame being published was measured, the frames since the
  // last detector step, and what went missing, told in the log once a minute.
  bool framed_{false};
  uint32_t frame_ms_{0};
  uint32_t frames_{0};
  bool have_seq_{false};
  uint32_t last_seq_{0}, last_epoch_{0};
  uint32_t unpublished_{0}, gaps_{0}, told_ms_{0};
  uint32_t seq_{0};
  static constexpr uint8_t REPLAY = 8;
  LoadEvent sent_[REPLAY];
  uint8_t sent_head_{0}, sent_n_{0};
};

class LoadEventTrigger : public Trigger<const LoadEvent &> {
 public:
  explicit LoadEventTrigger(LoadEventsComponent *parent) {
    parent->add_on_load_event_callback([this](const LoadEvent &event) { this->trigger(event); });
  }
};

template<typename... Ts> class ReplayAction : public Action<Ts...> {
 public:
  explicit ReplayAction(LoadEventsComponent *parent) : parent_(parent) {}
  void play(const Ts &...x) override { this->parent_->replay(); }

 protected:
  LoadEventsComponent *parent_;
};

}  // namespace load_events
}  // namespace esphome
