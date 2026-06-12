from __future__ import annotations

from GUI.main_gui import *

class InfrastructurePanel(ttk.Frame):
    def __init__(self, master: tk.Widget, scale_factor: float):
        super().__init__(master, padding=int(8 * scale_factor), style="Panel.TFrame")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        ttk.Label(self, text="ATS Wayside Status", style="SectionTitle.TLabel").grid(row=0, column=0, sticky="w")
        text_frame = ttk.Frame(self, style="Panel.TFrame")
        text_frame.grid(row=1, column=0, sticky="nsew", pady=(int(6 * scale_factor), 0))
        text_frame.columnconfigure(0, weight=1)
        text_frame.rowconfigure(0, weight=1)
        self.text = tk.Text(
            text_frame,
            height=int(13 * scale_factor),
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
        self._last_content = ""
        vscroll = ttk.Scrollbar(text_frame, orient="vertical", command=self.text.yview)
        vscroll.grid(row=0, column=1, sticky="ns")
        hscroll = ttk.Scrollbar(text_frame, orient="horizontal", command=self.text.xview)
        hscroll.grid(row=1, column=0, sticky="ew")
        self.text.configure(yscrollcommand=vscroll.set, xscrollcommand=hscroll.set)

    def update_data(self, sim: Simulation):
        wayside = dict(getattr(sim, "ats_received_wayside_state", {}) or {})
        zc_state = dict(getattr(sim, "ats_received_zc_state", {}) or {})
        station_state = dict(getattr(sim, "ats_received_station_state", {}) or {})
        dcs_state = dict(getattr(sim, "ats_received_dcs_state", {}) or {})
        train_states = dict(getattr(sim, "ats_received_train_state", {}) or {})
        track_profile = [tuple(segment) for segment in wayside.get("track_profile", [])]
        tsr_zones = [dict(zone) for zone in zc_state.get("tsr_zones", [])]
        lines = ["ATS status-packet supervision view"]
        lines.append("-" * 72)
        lines.append(f"WAYSIDE_STATUS freshness: {getattr(sim, 'ats_wayside_freshness', 'LOST')}")
        lines.append(f"ZC_STATUS freshness     : {getattr(sim, 'ats_zc_freshness', 'LOST')}")
        lines.append(f"STATION_STATUS freshness: {getattr(sim, 'ats_station_freshness', 'LOST')}")
        lines.append(f"DCS_STATUS freshness    : {getattr(sim, 'ats_dcs_freshness', 'LOST')}")
        lines.append(f"TRAIN_STATUS packets    : {len(train_states)}")
        zc_status = dict(zc_state.get("zc_status", {}) or {})
        if zc_status:
            lines.append(
                f"ZC availability         : {zc_status.get('zc_id', 'ZC')} {zc_status.get('availability', '--')}  "
                f"fresh PR={zc_status.get('fresh_position_reports', 0)}/{zc_status.get('valid_position_reports', 0)}"
            )
        lines.append("")
        secondary_sections = [dict(item) for item in zc_state.get("secondary_detection_sections", [])]
        for idx, (start, end, gradient, psr) in enumerate(track_profile, 1):
            section = secondary_sections[idx - 1] if idx - 1 < len(secondary_sections) else {}
            occupied = bool(section.get("occupied", False))
            occupied_by = ",".join(str(item) for item in section.get("occupied_by_train_ids", []) or [])
            signal = "RED" if occupied else "GREEN"
            axle = "OCC" if occupied else "CLEAR"
            lines.append(
                f"SEG-{idx:02d} {start:>5.0f}-{end:<5.0f}  {axle:<20} "
                f"gradient={gradient:+.3f} SSP={psr:.0f} km/h occupied_by={occupied_by or '--'}"
            )
        rap_items = [dict(item) for item in wayside.get("radio_access_points", [])]
        if rap_items:
            lines.append("")
            lines.append("Radio access points")
            lines.append("-" * 72)
            for rap in rap_items:
                lines.append(
                    f"{str(rap.get('id', 'RAP')):<8} "
                    f"{float(rap.get('start_m', 0.0)):>5.0f}-{float(rap.get('end_m', 0.0)):<5.0f}  DCS RADIO COVERAGE"
                )
        balises = [dict(balise) for balise in wayside.get("balises", [])]
        if balises:
            lines.append("")
            lines.append("Balise / Beacon Layout")
            lines.append("-" * 72)
            for balise in balises:
                lines.append(f"{str(balise.get('id', 'BALISE')):<8} pos={float(balise['pos_m']):>6.1f}m")
        if tsr_zones:
            lines.append("")
            lines.append("Temporary speed restrictions")
            lines.append("-" * 72)
            for idx, zone in enumerate(tsr_zones, 1):
                lines.append(
                    f"TSR-{idx:02d} {float(zone['start']):>5.0f}-{float(zone['end']):<5.0f}  ACTIVE               limit={float(zone['speed']):.0f} km/h"
                )
        lines.append("")
        lines.append("ATS Train Status")
        lines.append("-" * 72)
        for train_id, state in sorted(train_states.items()):
            fault_flags = dict(state.get("fault_flags", {}) or {})
            lines.append(
                f"{train_id:<10} pos={float(state.get('position_m', 0.0)):>7.1f}m  "
                f"speed={ms_to_kmh(float(state.get('speed_mps', 0.0))):>5.1f}km/h  "
                f"mode={str(state.get('mode', '--')):<6} ATP={str(state.get('atp_state', '--')):<14} faults={fault_flags}"
            )
        lines.append("")
        lines.append("ZC Protection / ESA / Overlap")
        lines.append("-" * 72)
        protection_zones = [dict(item) for item in zc_state.get("protection_zones", [])]
        if not protection_zones:
            lines.append("No protection-zone records in ZC_STATUS.")
        for zone in protection_zones:
            esa = "ESA_ACTIVE" if zone.get("esa_active") else "NORMAL"
            lines.append(
                f"{str(zone.get('train_id', '--')):<8} EOA={float(zone.get('eoa_m', 0.0)):>7.1f}m "
                f"SVL={float(zone.get('svl_m', 0.0)):>7.1f}m overlap={float(zone.get('overlap_m', 0.0)):>5.1f}m "
                f"{esa:<10} zone={zone.get('protection_zone_id') or '--'}"
            )
        dcs_transport_state = dict(dcs_state.get("dcs_transport_state", {}) or {})
        if dcs_transport_state:
            lines.append("")
            lines.append("DCS Transport Status")
            lines.append("-" * 72)
            lines.append(f"active_path={dcs_transport_state.get('active_path', '--')} last_fault={dcs_transport_state.get('last_fault', '--')}")
            lines.append(f"faults={dcs_transport_state.get('faults', {})}")
        content = "\n".join(lines)
        if content == self._last_content:
            return
        yview = self.text.yview()
        xview = self.text.xview()
        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        self.text.insert("1.0", content)
        if yview:
            self.text.yview_moveto(yview[0])
        if xview:
            self.text.xview_moveto(xview[0])
        self.text.configure(state="disabled")
        self._last_content = content


