"""Load events from any meter's power sensors (see load_events.h)."""

from esphome import automation
import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_KEY, CONF_POWER, CONF_TIMEOUT, CONF_TRIGGER_ID

CODEOWNERS = ["@RAR"]
DEPENDENCIES = ["sensor"]

CONF_APPARENT_POWER = "apparent_power"
CONF_CHANNELS = "channels"
CONF_CYCLE_INPUTS = "cycle_inputs"
CONF_EMPORIAVUE_ID = "emporiavue_id"
CONF_CONFIRM_FRAMES = "confirm_frames"
CONF_CYCLE_STEPS = "cycle_steps"
CONF_CYCLE_TOLERANCE = "cycle_tolerance"
CONF_CYCLE_WINDOW = "cycle_window"
CONF_DRIFT_FRAMES = "drift_frames"
CONF_LESS = "less"
CONF_MAX_NOISE = "max_noise"
CONF_MIN_STEP = "min_step"
CONF_MIN_STEP_PERCENT = "min_step_percent"
CONF_NOISE_FACTOR = "noise_factor"
CONF_OF = "of"
CONF_ON_LOAD_EVENT = "on_load_event"
CONF_REMAINDER = "remainder"
CONF_SETTLE_FRAMES = "settle_frames"
CONF_SETTLE_WITHIN = "settle_within"
CONF_SETTLE_WITHIN_PERCENT = "settle_within_percent"
CONF_STEP_DETECTION = "step_detection"

load_events_ns = cg.esphome_ns.namespace("load_events")
LoadEventsComponent = load_events_ns.class_("LoadEventsComponent", cg.Component)
Channel = load_events_ns.class_("Channel")
StepConfig = load_events_ns.struct("StepConfig")
LoadEvent = load_events_ns.struct("LoadEvent")
LoadEventTrigger = load_events_ns.class_(
    "LoadEventTrigger", automation.Trigger.template(LoadEvent.operator("const").operator("ref"))
)
ReplayAction = load_events_ns.class_("ReplayAction", automation.Action)


# The emporiavue component's input ports.
CYCLE_INPUT_PORTS = {"A": 0, "B": 1, "C": 2, **{str(n): n + 2 for n in range(1, 17)}}
EmporiaVueComponent = cg.esphome_ns.namespace("emporiavue").class_("EmporiaVueComponent", cg.Component)


def watts(value):
    return cv.float_with_unit("power", "(W|w)?")(value)


STEP_DETECTION_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_MIN_STEP, default="30W"): watts,
        cv.Optional(CONF_MIN_STEP_PERCENT, default="2%"): cv.percentage,
        cv.Optional(CONF_NOISE_FACTOR, default=4.0): cv.float_range(min=0, max=20),
        # The wobble's part of the threshold is held to this if set (never under 1.5x the wobble); 0W no cap.
        cv.Optional(CONF_MAX_NOISE, default="0W"): watts,
        # Frames a gradual change must hold before it is reported; 0 never.
        cv.Optional(CONF_DRIFT_FRAMES, default=20): cv.int_range(min=0, max=250),
        # A load switching on and off over and over is one cycling start and end: once this many steps
        # in a row alternate, alike within cycle_tolerance, within cycle_window. 0 never.
        cv.Optional(CONF_CYCLE_STEPS, default=4): cv.Any(cv.one_of(0, int=True), cv.int_range(min=3, max=8)),
        cv.Optional(CONF_CYCLE_TOLERANCE, default="35%"): cv.percentage,
        cv.Optional(CONF_CYCLE_WINDOW, default="120s"): cv.All(
            cv.positive_time_period_milliseconds,
            cv.Range(min=cv.TimePeriod(seconds=5), max=cv.TimePeriod(minutes=10)),
        ),
        cv.Optional(CONF_SETTLE_WITHIN, default="10W"): watts,
        cv.Optional(CONF_SETTLE_WITHIN_PERCENT, default="3%"): cv.percentage,
        cv.Optional(CONF_CONFIRM_FRAMES, default=2): cv.int_range(min=1, max=8),
        cv.Optional(CONF_SETTLE_FRAMES, default=4): cv.int_range(min=2, max=8),
        cv.Optional(CONF_TIMEOUT, default="10s"): cv.All(
            cv.positive_time_period_milliseconds, cv.Range(max=cv.TimePeriod(seconds=60))
        ),
    }
)

CHANNEL_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Channel),
        # What the events are published under: the dashboard matches it to the meter's circuit names.
        cv.Required(CONF_KEY): cv.string_strict,
        # Summed each frame: a 240 V circuit on two clamps, or a panel's two legs.
        cv.Required(CONF_POWER): cv.All(cv.ensure_list(cv.use_id(sensor.Sensor)), cv.Length(min=1, max=3)),
        cv.Optional(CONF_APPARENT_POWER): cv.All(cv.ensure_list(cv.use_id(sensor.Sensor)), cv.Length(min=1, max=3)),
        cv.Optional(CONF_MIN_STEP): watts,
        # The meter inputs its power comes from (A-C the mains, 1-16 circuits), for per-cycle detail on its
        # events (needs emporiavue_id: an emporiavue meter on SPI with the cycle hook).
        cv.Optional(CONF_CYCLE_INPUTS): cv.All(
            cv.ensure_list(cv.one_of(*CYCLE_INPUT_PORTS, upper=True, string=True)), cv.Length(min=1, max=3)
        ),
    }
)


def _unique_keys(config):
    keys = [c[CONF_KEY] for c in config[CONF_CHANNELS]]
    if len(set(keys)) != len(keys):
        raise cv.Invalid("each channel needs its own key")
    if CONF_REMAINDER in config:
        rem = config[CONF_REMAINDER]
        for key in [rem[CONF_OF], *rem[CONF_LESS]]:
            if key not in keys:
                raise cv.Invalid(f"remainder: no channel with key {key}")
        if rem[CONF_KEY] in keys:
            raise cv.Invalid("remainder: its key must differ from the channels'")
    if any(CONF_CYCLE_INPUTS in c for c in config[CONF_CHANNELS]) and CONF_EMPORIAVUE_ID not in config:
        raise cv.Invalid("cycle_inputs needs emporiavue_id")
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(LoadEventsComponent),
            cv.Optional(CONF_STEP_DETECTION, default={}): STEP_DETECTION_SCHEMA,
            # An emporiavue meter on SPI whose mains cycles feed the channels' cycle_inputs.
            cv.Optional(CONF_EMPORIAVUE_ID): cv.use_id(EmporiaVueComponent),
            cv.Required(CONF_CHANNELS): cv.All(cv.ensure_list(CHANNEL_SCHEMA), cv.Length(min=1)),
            # What one channel carries that the listed others do not: loads with no meter of their own.
            cv.Optional(CONF_REMAINDER): cv.Schema(
                {
                    cv.Optional(CONF_KEY, default="unmetered"): cv.string_strict,
                    cv.Required(CONF_OF): cv.string_strict,
                    cv.Required(CONF_LESS): cv.All(cv.ensure_list(cv.string_strict), cv.Length(min=1)),
                    cv.Optional(CONF_MIN_STEP, default="80W"): watts,
                }
            ),
            cv.Required(CONF_ON_LOAD_EVENT): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(LoadEventTrigger)}
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _unique_keys,
)


def _step_config(events, min_step=None):
    return cg.StructInitializer(
        StepConfig,
        ("min_step_w", events[CONF_MIN_STEP] if min_step is None else min_step),
        ("rel_step", events[CONF_MIN_STEP_PERCENT]),
        ("noise_factor", events[CONF_NOISE_FACTOR]),
        ("max_noise_w", events[CONF_MAX_NOISE]),
        ("settle_w", events[CONF_SETTLE_WITHIN]),
        ("settle_rel", events[CONF_SETTLE_WITHIN_PERCENT]),
        ("confirm", events[CONF_CONFIRM_FRAMES]),
        ("settle_frames", events[CONF_SETTLE_FRAMES]),
        ("timeout_ms", events[CONF_TIMEOUT].total_milliseconds),
        ("drift_frames", events[CONF_DRIFT_FRAMES]),
        ("cycle_steps", events[CONF_CYCLE_STEPS]),
        ("cycle_tolerance", events[CONF_CYCLE_TOLERANCE]),
        ("cycle_window_ms", events[CONF_CYCLE_WINDOW].total_milliseconds),
    )


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    events = config[CONF_STEP_DETECTION]
    if CONF_EMPORIAVUE_ID in config:
        cg.add_define("USE_LOAD_EVENTS_CYCLES")
        cg.add(var.set_meter(await cg.get_variable(config[CONF_EMPORIAVUE_ID])))

    by_key = {}
    for ch in config[CONF_CHANNELS]:
        channel = cg.new_Pvariable(ch[CONF_ID], ch[CONF_KEY], _step_config(events, ch.get(CONF_MIN_STEP)))
        for sensor_id in ch[CONF_POWER]:
            cg.add(channel.add_power(await cg.get_variable(sensor_id)))
        for sensor_id in ch.get(CONF_APPARENT_POWER, []):
            cg.add(channel.add_apparent_power(await cg.get_variable(sensor_id)))
        for port in ch.get(CONF_CYCLE_INPUTS, []):
            cg.add(channel.add_cycle_input(CYCLE_INPUT_PORTS[port]))
        cg.add(var.add_channel(channel))
        by_key[ch[CONF_KEY]] = channel

    if CONF_REMAINDER in config:
        rem = config[CONF_REMAINDER]
        cg.add(
            var.set_remainder(
                rem[CONF_KEY],
                _step_config(events, rem[CONF_MIN_STEP]),
                by_key[rem[CONF_OF]],
                [by_key[k] for k in rem[CONF_LESS]],
            )
        )

    for conf in config[CONF_ON_LOAD_EVENT]:
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(LoadEvent.operator("const").operator("ref"), "event")], conf)


@automation.register_action(
    "load_events.replay",
    ReplayAction,
    cv.Schema({cv.GenerateID(): cv.use_id(LoadEventsComponent)}),
)
async def replay_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, parent)
