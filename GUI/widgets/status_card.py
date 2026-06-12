from __future__ import annotations

from GUI.main_gui import *


class StatusCard(ttk.Frame):
    """Reusable title/value status card for compact GUI summaries."""

    def __init__(self, master: tk.Widget, title: str, value: str = ""):
        super().__init__(master, style="Card.TFrame")
        self.value_var = tk.StringVar(value=value)
        ttk.Label(self, text=title, style="CardTitle.TLabel").pack(anchor="w")
        ttk.Label(self, textvariable=self.value_var, font=("Consolas", 12, "bold")).pack(anchor="w")

    def set_value(self, value: str) -> None:
        self.value_var.set(value)


__all__ = ["StatusCard"]
