"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached

BMW_REGEN_ONLY_HINT = "regen only — cover brake"
_HINT_COLOR = rl.Color(255, 255, 255, 200)


def draw_bmw_regen_only_hint(font, rect: rl.Rectangle, y: float, font_size: int = 36) -> None:
  if ui_state.CP is None or ui_state.CP.brand != "bmw":
    return

  text = tr(BMW_REGEN_ONLY_HINT)
  text_width = measure_text_cached(font, text, font_size).x
  rl.draw_text_ex(
    font,
    text,
    rl.Vector2(rect.x + (rect.width - text_width) / 2, y),
    font_size,
    0,
    _HINT_COLOR,
  )
