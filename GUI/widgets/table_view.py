from __future__ import annotations

from GUI.main_gui import *


class TableView(ttk.Treeview):
    """Thin Treeview wrapper for tabular GUI data."""

    def replace_rows(self, rows: list[tuple]) -> None:
        yview = self.yview()
        for item in self.get_children():
            self.delete(item)
        for row in rows:
            self.insert("", "end", values=row)
        if yview:
            self.yview_moveto(yview[0])


__all__ = ["TableView"]
