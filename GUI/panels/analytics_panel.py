from __future__ import annotations

from GUI.main_gui import *

class AnalyticsPanel(ttk.Frame):
    def __init__(self, master: tk.Widget, scale_factor: float):
        super().__init__(master, padding=int(8 * scale_factor), style="Panel.TFrame")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)
        ttk.Label(self, text="ATS Supervision Summary", style="SectionTitle.TLabel").grid(row=0, column=0, sticky="w")
        self.summary_var = tk.StringVar(value="")
        ttk.Label(self, textvariable=self.summary_var, style="Muted.TLabel").grid(row=1, column=0, sticky="w", pady=(int(2 * scale_factor), int(6 * scale_factor)))
        text_frame = ttk.Frame(self, style="Panel.TFrame")
        text_frame.grid(row=2, column=0, sticky="nsew")
        text_frame.columnconfigure(0, weight=1)
        text_frame.rowconfigure(0, weight=1)
        self.text = tk.Text(
            text_frame,
            height=int(18 * scale_factor),
            state="disabled",
            wrap="none",
            font=("Consolas", int(8 * scale_factor)),
            background=APP_THEME["log_bg"],
            foreground=APP_THEME["text"],
            insertbackground=APP_THEME["text"],
            highlightthickness=1,
            highlightbackground=APP_THEME["border"],
        )
        self.text.grid(row=0, column=0, sticky="nsew")
        vscroll = ttk.Scrollbar(text_frame, orient="vertical", command=self.text.yview)
        vscroll.grid(row=0, column=1, sticky="ns")
        hscroll = ttk.Scrollbar(text_frame, orient="horizontal", command=self.text.xview)
        hscroll.grid(row=1, column=0, sticky="ew")
        self.text.configure(yscrollcommand=vscroll.set, xscrollcommand=hscroll.set)

    def update_data(self, sim: Simulation):
        train_states = dict(getattr(sim, "ats_received_train_state", {}) or {})
        total_trains = len(train_states)
        speeds = [ms_to_kmh(float(state.get("speed_mps", 0.0) or 0.0)) for state in train_states.values()]
        avg_speed = sum(speeds) / len(speeds) if speeds else 0.0
        moving = sum(1 for speed in speeds if speed > 0.5)
        dcs_faults = sum(1 for state in train_states.values() if bool((state.get("fault_flags", {}) or {}).get("DCS", False)))
        ebi_active = sum(
            1
            for state in train_states.values()
            if bool((state.get("fault_flags", {}) or {}).get("EMERGENCY", False))
            or str(state.get("atp_state", "")) in ("ATP_EMERGENCY", "ATP_TRIP")
        )
        self.summary_var.set(
            f"ATS packets only  |  trains={total_trains}  moving={moving}  "
            f"avg speed={avg_speed:.1f} km/h  EBI/TRIP={ebi_active}  DCS fault={dcs_faults}"
        )

        lines = [
            "ATS TRAIN/ZC/STATION/DCS/WAYSIDE status summary",
            "-" * 88,
            f"Simulation time        : {sim.sim_time_s:,.1f} s",
            f"WAYSIDE_STATUS         : {getattr(sim, 'ats_wayside_freshness', 'LOST')}",
            f"ZC_STATUS              : {getattr(sim, 'ats_zc_freshness', 'LOST')}",
            f"STATION_STATUS         : {getattr(sim, 'ats_station_freshness', 'LOST')}",
            f"DCS_STATUS             : {getattr(sim, 'ats_dcs_freshness', 'LOST')}",
            f"Received train packets : {total_trains}",
            f"Moving trains          : {moving}",
            f"Average reported speed : {avg_speed:.1f} km/h",
            f"Emergency/trip states  : {ebi_active}",
            f"DCS fault flags        : {dcs_faults}",
            "",
            "Train  Fresh    Mode   Pos(m)   Speed   Door          DCS     ATP",
            "-" * 88,
        ]
        freshness_map = dict(getattr(sim, "ats_train_freshness", {}) or {})
        for train_id, state in sorted(train_states.items()):
            fault_flags = dict(state.get("fault_flags", {}) or {})
            dcs_text = "FAULT" if fault_flags.get("DCS") else "OK"
            lines.append(
                f"{train_id:<5} {freshness_map.get(train_id, 'LOST'):<8} {str(state.get('mode', '--')):<6} "
                f"{float(state.get('position_m', 0.0)):>7.1f} "
                f"{ms_to_kmh(float(state.get('speed_mps', 0.0))):>6.1f} "
                f"{str(state.get('door_state', '--')):<13} {dcs_text:<7} {str(state.get('atp_state', '--'))}"
            )

        wayside = dict(getattr(sim, "ats_received_wayside_state", {}) or {})
        zc_state = dict(getattr(sim, "ats_received_zc_state", {}) or {})
        station_state = dict(getattr(sim, "ats_received_station_state", {}) or {})
        lines.extend(["", "Wayside records", "-" * 88])
        lines.append(f"Track segments         : {len(wayside.get('track_profile', []) or [])}")
        lines.append(f"Temporary restrictions : {len(zc_state.get('tsr_zones', []) or [])}")
        lines.append(f"Scheduled stops        : {len(wayside.get('scheduled_stops', []) or [])}")
        lines.append(f"Line conditions        : {len(wayside.get('line_conditions', []) or [])}")
        lines.append(f"Station routes         : {len(station_state.get('station_route_states', []) or [])}")

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


