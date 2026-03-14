"""SP NavScroller — exposes _Scroller API and fixes show_event scroll state.

Upstream Scroller doesn't expose _Scroller's public API (add_widget, scroll_panel, etc.)
and show_event doesn't reset scroll velocity/state, causing momentum carryover between
panel navigations. See memory/scroller-snap-bug.md for details.
"""

from openpilot.system.ui.lib.scroll_panel2 import ScrollState
from openpilot.system.ui.widgets.scroller import NavScroller as _NavScroller


class NavScroller(_NavScroller):
  """NavScroller with _Scroller API forwarding and show_event scroll state reset."""

  def add_widget(self, item):
    self._scroller.add_widget(item)

  def add_widgets(self, items):
    self._scroller.add_widgets(items)

  def set_scrolling_enabled(self, enabled):
    self._scroller.set_scrolling_enabled(enabled)

  def set_reset_scroll_at_show(self, scroll):
    self._scroller.set_reset_scroll_at_show(scroll)

  def clear_widgets(self):
    self._scroller._items.clear()

  @property
  def scroll_panel(self):
    return self._scroller.scroll_panel

  def show_event(self):
    super().show_event()
    self._scroller._scroll_snap_filter.x = 0.0
    self._scroller.scroll_panel._state = ScrollState.STEADY
    self._scroller.scroll_panel._velocity = 0.0
    self._scroller.scroll_panel._velocity_buffer.clear()
