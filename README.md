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

### Per-cycle detail (experimental)

With a meter that hands over each mains cycle's raw sums, a channel's events also carry a
`cycle` object: the step as the cycles saw it (`dw`), its `inrush` (the peak cycle's change
in fundamental current over the settled change) and `inrush_ms`, the load's own current
harmonics `h` (3rd, 5th, 7th over the fundamental) and its displacement power factor `dpf`,
and `lead_ms` (how far the frames ran behind the cycles). The steps themselves are still
found by the frame-level detector; the cycles only add detail
(`components/load_events/cycle_features.h`).

```yaml
load_events:
  emporiavue_id: vue            # needs an emporiavue build with the per-cycle hook (not upstream)
  channels:
    - key: total
      power: [line_1_power, line_2_power]
      cycle_inputs: [A, B]      # the meter inputs that power comes from: A-C mains, 1-16 circuits
```

The hook (`add_on_cycle_callback`) is a local patch to rosenrot00/emporiavue's SPI transport
for now; without `emporiavue_id` nothing here depends on it.

Host tests:

    c++ -std=c++17 -Wall -Wextra -I components/load_events tests/test_step_detector.cpp -o test_step && ./test_step
    c++ -std=c++17 -Wall -Wextra -I components/load_events tests/test_cycle_features.cpp -o test_cycles && ./test_cycles
