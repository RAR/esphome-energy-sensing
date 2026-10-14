#pragma once

// Load events from any meter's sensors: each channel sums the power (and apparent
// power) sensors it is given, frame by frame, and finds the steps in it with the
// step detector. A frame is the set of readings a meter publishes together; the
// component takes the sensors' raw states (before filters) and runs the detectors
// once per loop after any of them changed, so one frame is one detector step.

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

#include "step_detector.h"

#ifdef USE_LOAD_EVENTS_CYCLES
#include "esphome/components/emporiavue/emporiavue.h"
#include "esphome/core/helpers.h"

#include "cycle_features.h"
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

#ifdef USE_LOAD_EVENTS_CYCLES
  // The meter's inputs this channel's power comes from (0-2 the mains, 3-18 circuits 1-16): with them,
  // each mains cycle is summed into the cycle features, and events carry what they found.
  void add_cycle_input(uint8_t port) {
    if (this->features_ == nullptr)
      this->features_ = new CycleFeatures(this->detector_.config().min_step_w);  // NOLINT
    this->cycle_ports_.push_back(port);
  }
  bool has_cycles() const { return this->features_ != nullptr; }
  // From the meter's task: this channel's part of one mains cycle.
  void feed_cycle(const emporiavue::MeteringCycle &cycle);
  // From the main loop: the frame's published watts (the raw-to-watts scale), and an event's cycle step.
  void calibrate_cycles(float watts);
  void attach_cycle(LoadEvent &event);
#endif

 protected:
  struct Input {
    sensor::Sensor *sensor;
    float value{NAN};
  };
  Input *watch_(sensor::Sensor *s);

  StepDetector detector_;
  std::vector<Input *> power_, apparent_;
  bool dirty_{false};
#ifdef USE_LOAD_EVENTS_CYCLES
  CycleFeatures *features_{nullptr};
  std::vector<uint8_t> cycle_ports_;
  Mutex cycle_lock_;
#endif
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
#ifdef USE_LOAD_EVENTS_CYCLES
  // Registered while the app is built, before the meter's SPI task starts (it reads the callback list).
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
