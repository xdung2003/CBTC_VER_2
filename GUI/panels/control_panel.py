from __future__ import annotations

from GUI.main_gui import *

class ControlPanel(ttk.Frame):
    def __init__(self, master: tk.Widget, on_apply_psr, on_add_tsr, on_clear_tsr, scale_factor: float):
        super().__init__(master, padding=int(4 * scale_factor), borderwidth=1, relief="solid", width=int(170 * scale_factor), height=int(210 * scale_factor))
        self.on_apply_psr = on_apply_psr
        self.on_add_tsr = on_add_tsr
        self.on_clear_tsr = on_clear_tsr
        self.segment_label = None

        ttk.Label(self, text="ATS RaSTA Commands", font=("Segoe UI", int(11 * scale_factor), "bold")).pack(anchor="w")

        psr_frame = ttk.LabelFrame(self, text="PSR Segment")
        psr_frame.pack(fill="x", pady=(int(8 * scale_factor), int(6 * scale_factor)))
        self.segment_label = ttk.Label(psr_frame, text="Segment")
        self.segment_label.grid(row=0, column=0, sticky="w", padx=int(4 * scale_factor), pady=int(2 * scale_factor))
        ttk.Label(psr_frame, text="PSR km/h").grid(row=1, column=0, sticky="w", padx=int(4 * scale_factor), pady=int(2 * scale_factor))
        self.psr_segment = tk.StringVar(value="0")
        self.psr_value = tk.StringVar(value="40")
        ttk.Entry(psr_frame, textvariable=self.psr_segment, width=8).grid(row=0, column=1, padx=4, pady=2)
        ttk.Entry(psr_frame, textvariable=self.psr_value, width=8).grid(row=1, column=1, padx=4, pady=2)
        ttk.Button(psr_frame, text="Issue PSR", command=self._apply_psr).grid(row=2, column=0, columnspan=2, sticky="we", padx=4, pady=4)

        tsr_frame = ttk.LabelFrame(self, text="TSR Zone")
        tsr_frame.pack(fill="x", pady=(4, 6))
        ttk.Label(tsr_frame, text="Start (m)").grid(row=0, column=0, sticky="w", padx=4, pady=2)
        ttk.Label(tsr_frame, text="End (m)").grid(row=1, column=0, sticky="w", padx=4, pady=2)
        ttk.Label(tsr_frame, text="Speed km/h").grid(row=2, column=0, sticky="w", padx=4, pady=2)
        self.tsr_start = tk.StringVar(value="600")
        self.tsr_end = tk.StringVar(value="900")
        self.tsr_speed = tk.StringVar(value="25")
        ttk.Entry(tsr_frame, textvariable=self.tsr_start, width=8).grid(row=0, column=1, padx=4, pady=2)
        ttk.Entry(tsr_frame, textvariable=self.tsr_end, width=8).grid(row=1, column=1, padx=4, pady=2)
        ttk.Entry(tsr_frame, textvariable=self.tsr_speed, width=8).grid(row=2, column=1, padx=4, pady=2)
        ttk.Button(tsr_frame, text="Issue TSR", command=self._add_tsr).grid(row=3, column=0, columnspan=2, sticky="we", padx=4, pady=4)
        ttk.Button(tsr_frame, text="Clear TSR", command=self._clear_tsr).grid(row=4, column=0, columnspan=2, sticky="we", padx=4, pady=(0, 4))

    def _apply_psr(self):
        self.on_apply_psr(self.psr_segment.get(), self.psr_value.get())

    def _add_tsr(self):
        self.on_add_tsr(self.tsr_start.get(), self.tsr_end.get(), self.tsr_speed.get())

    def _clear_tsr(self):
        self.on_clear_tsr()

    def update_track_profile(self, track_profile: List[Tuple[float, float, float, float]]):
        if not track_profile:
            self.segment_label.config(text="Segment")
            return
        self.segment_label.config(text=f"Segment (0-{len(track_profile) - 1})")


class SpeedLimitsPanel(ttk.Frame):
    def __init__(self, master: tk.Widget, scale_factor: float):
        super().__init__(master, padding=int(4 * scale_factor), borderwidth=1, relief="solid", width=int(240 * scale_factor), height=int(260 * scale_factor))
        ttk.Label(self, text="Speed Limits", font=("Segoe UI", int(12 * scale_factor), "bold")).pack(anchor="w")
        content = ttk.Frame(self)
        content.pack(fill="both", expand=True, pady=(int(4 * scale_factor), 0))
        self.text = tk.Text(
            content,
            width=int(30 * scale_factor),
            height=int(14 * scale_factor),
            state="disabled",
            font=("Consolas", int(9 * scale_factor)),
            wrap="none",
            background=APP_THEME["log_bg"],
            foreground=APP_THEME["text"],
            insertbackground=APP_THEME["text"],
            highlightthickness=1,
            highlightbackground=APP_THEME["border"],
        )
        self.text.grid(row=0, column=0, sticky="nsew")
        vscroll = ttk.Scrollbar(content, orient="vertical", command=self.text.yview)
        vscroll.grid(row=0, column=1, sticky="ns")
        self.text.configure(yscrollcommand=vscroll.set)
        hscroll = ttk.Scrollbar(content, orient="horizontal", command=self.text.xview)
        hscroll.grid(row=1, column=0, sticky="ew")
        self.text.configure(xscrollcommand=hscroll.set)
        content.columnconfigure(0, weight=1)
        content.rowconfigure(0, weight=1)

    def update_limits(self, track_profile, tsr_zones):
        lines = []
        lines.append("Track segments:")
        for i, (start, end, grad, psr) in enumerate(track_profile):
            lines.append(f"  {i}: {start:.0f}-{end:.0f} m  grad={grad:+.3f}  PSR={psr:.0f} km/h")
        lines.append("")
        lines.append("TSR (temporary):")
        if not tsr_zones:
            lines.append("  (none)")
        else:
            for idx, z in enumerate(tsr_zones, 1):
                lines.append(f"  {idx}: {z['start']:.0f}-{z['end']:.0f} m  {z['speed']:.0f} km/h")

        yview = self.text.yview()
        xview = self.text.xview()
        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        self.text.insert("1.0", "\n".join(lines))
        if yview:
            self.text.yview_moveto(yview[0])
        if xview:
            self.text.xview_moveto(xview[0])
        self.text.configure(state="disabled")


