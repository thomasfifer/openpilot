import pyray as rl
from collections.abc import Callable
from cereal import log

from openpilot.system.ui.widgets.scroller import Scroller
from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl, BigMultiParamToggle, BigMultiToggle
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import NavWidget
from openpilot.selfdrive.ui.layouts.settings.common import restart_needed_callback
from openpilot.selfdrive.ui.ui_state import ui_state

PERSONALITY_TO_INT = log.LongitudinalPersonality.schema.enumerants

TIMEOUT_OPTIONS = ["no sleep", "2s", "3s", "5s", "10s"]
TIMEOUT_VALUES = [0, 2, 3, 5, 10]

BRIGHTNESS_OPTIONS = ["auto", "0.1%", "0.5%", "1%"]
BRIGHTNESS_VALUES = [0, 1, 5, 10]


class TogglesLayoutMici(NavWidget):
  def __init__(self, back_callback: Callable):
    super().__init__()
    self.set_back_callback(back_callback)

    self._personality_toggle = BigMultiParamToggle("driving personality", "LongitudinalPersonality", ["aggressive", "standard", "relaxed"])
    self._experimental_btn = BigParamControl("experimental mode", "ExperimentalMode")
    is_metric_toggle = BigParamControl("use metric units", "IsMetric")
    ldw_toggle = BigParamControl("lane departure warnings", "IsLdwEnabled")
    always_on_dm_toggle = BigParamControl("always-on driver monitor", "AlwaysOnDM")
    record_front = BigParamControl("record & upload driver camera", "RecordFront", toggle_callback=restart_needed_callback)
    record_mic = BigParamControl("record & upload mic audio", "RecordAudio", toggle_callback=restart_needed_callback)
    enable_openpilot = BigParamControl("enable openpilot", "OpenpilotEnabledToggle", toggle_callback=restart_needed_callback)

    # Volvo-specific toggles
    volvo_double_tap = BigParamControl("double-tap cruise engage", "VolvoDoubleTapCruise", toggle_callback=restart_needed_callback)
    volvo_spoof_pa = BigParamControl("PA: spoof hands on wheel", "VolvoSpoofPAHandsOnWheel", toggle_callback=restart_needed_callback)

    # Screen settings
    current_timeout = ui_state.params.get("OnroadScreenSleepTimeout", return_default=True) or 0
    try:
      timeout_idx = TIMEOUT_VALUES.index(current_timeout)
    except (ValueError, TypeError):
      timeout_idx = 0
    self._screen_sleep_toggle = BigMultiToggle("screen sleep", TIMEOUT_OPTIONS, select_callback=self._set_screen_sleep)
    self._screen_sleep_toggle.set_value(TIMEOUT_OPTIONS[timeout_idx])

    current_brightness = ui_state.params.get("OnroadBrightnessPercent", return_default=True) or 0
    try:
      brightness_idx = BRIGHTNESS_VALUES.index(current_brightness)
    except (ValueError, TypeError):
      brightness_idx = 0
    self._brightness_toggle = BigMultiToggle("brightness", BRIGHTNESS_OPTIONS, select_callback=self._set_brightness)
    self._brightness_toggle.set_value(BRIGHTNESS_OPTIONS[brightness_idx])

    self._scroller = Scroller([
      self._personality_toggle,
      self._experimental_btn,
      is_metric_toggle,
      ldw_toggle,
      always_on_dm_toggle,
      record_front,
      record_mic,
      volvo_double_tap,
      volvo_spoof_pa,
      self._screen_sleep_toggle,
      self._brightness_toggle,
      enable_openpilot,
    ], snap_items=False)

    # Toggle lists
    self._refresh_toggles = (
      ("ExperimentalMode", self._experimental_btn),
      ("IsMetric", is_metric_toggle),
      ("IsLdwEnabled", ldw_toggle),
      ("AlwaysOnDM", always_on_dm_toggle),
      ("RecordFront", record_front),
      ("RecordAudio", record_mic),
      ("OpenpilotEnabledToggle", enable_openpilot),
      ("VolvoDoubleTapCruise", volvo_double_tap),
      ("VolvoSpoofPAHandsOnWheel", volvo_spoof_pa),
    )

    enable_openpilot.set_enabled(lambda: not ui_state.engaged)
    record_front.set_enabled(False if ui_state.params.get_bool("RecordFrontLock") else (lambda: not ui_state.engaged))
    record_mic.set_enabled(lambda: not ui_state.engaged)
    volvo_double_tap.set_enabled(lambda: not ui_state.engaged)
    volvo_spoof_pa.set_enabled(lambda: not ui_state.engaged)

    if ui_state.params.get_bool("ShowDebugInfo"):
      gui_app.set_show_touches(True)
      gui_app.set_show_fps(True)

    ui_state.add_engaged_transition_callback(self._update_toggles)

  def _set_screen_sleep(self, value: str):
    idx = TIMEOUT_OPTIONS.index(value)
    ui_state.params.put("OnroadScreenSleepTimeout", TIMEOUT_VALUES[idx])

  def _set_brightness(self, value: str):
    idx = BRIGHTNESS_OPTIONS.index(value)
    ui_state.params.put("OnroadBrightnessPercent", BRIGHTNESS_VALUES[idx])

  def _update_state(self):
    super()._update_state()

    if ui_state.sm.updated["selfdriveState"]:
      personality = PERSONALITY_TO_INT[ui_state.sm["selfdriveState"].personality]
      if personality != ui_state.personality and ui_state.started:
        self._personality_toggle.set_value(self._personality_toggle._options[personality])
      ui_state.personality = personality

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()

    # CP gating for experimental mode
    if ui_state.CP is not None:
      if ui_state.has_longitudinal_control:
        self._experimental_btn.set_visible(True)
        self._personality_toggle.set_visible(True)
      else:
        # no long for now
        self._experimental_btn.set_visible(False)
        self._experimental_btn.set_checked(False)
        self._personality_toggle.set_visible(False)
        ui_state.params.remove("ExperimentalMode")

    # Refresh toggles from params to mirror external changes
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))

  def _render(self, rect: rl.Rectangle):
    self._scroller.render(rect)
