# esphome-energy-sensing

ESPHome components for making sense of a home's energy readings.

## load_events

An ESPHome component that finds load events (a load switching on or off, its inrush,
how long it took to settle, the real and apparent power either side) in any meter's
power sensors, and hands each to an `on_load_event` automation as JSON.

It reads the sensors' raw states, before filters, and steps its detectors once per
loop after any of them changed: one meter frame is one detector step. So it works on
top of a meter component without changing it, for example
[rosenrot00/emporiavue](https://github.com/rosenrot00/emporiavue) on an Emporia Vue
(stock I2C frames every 500 ms, or SPI windows every ~220 ms).

The step detector came from RAR/esphome-emporia-vue (`better-sensing`), unchanged.

```yaml
external_components:
  - source: github://RAR/esphome-energy-sensing@main
    components: [load_events]

load_events:
  step_detection:          # thresholds counted in frames: scale them to the meter's rate
    confirm_frames: 4
    settle_frames: 8
    drift_frames: 45
  channels:
    - key: total                              # the events' "key"
      power: [line_1_power, line_2_power]     # summed each frame
      apparent_power: [line_1_va, line_2_va]  # optional: power factor either side
  # remainder:                                # optional: loads with no meter of their own
  #   of: total
  #   less: [dryer, oven]
  on_load_event:
    - mqtt.publish:
        topic: my-meter/load_event
        payload: !lambda return event.json();

mqtt:
  on_connect:
    - load_events.replay:   # the last 8 again, marked "replay", after a reconnect
```

Host tests for the step detector:

    c++ -std=c++17 -Wall -Wextra -I components/load_events tests/test_step_detector.cpp -o test_step && ./test_step
