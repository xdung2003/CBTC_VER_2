from __future__ import annotations

from GUI.main_gui import *


class CurvePlot(tk.Canvas):
    """Canvas primitive for plotting ATP/ATO speed curves."""

    def __init__(self, master: tk.Widget, **kwargs):
        kwargs.setdefault("background", APP_THEME["card_alt"])
        kwargs.setdefault("highlightthickness", 1)
        kwargs.setdefault("highlightbackground", APP_THEME["border"])
        super().__init__(master, **kwargs)

    def clear(self) -> None:
        self.delete("all")


__all__ = ["CurvePlot"]
