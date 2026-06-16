from __future__ import annotations

import json
import tkinter as tk
from collections import deque
from tkinter import ttk
from typing import Dict

from CONFIG.config import (
    ATP_ADHESION_FACTOR,
    ATP_BRAKE_BUILDUP_S,
    ATP_EBI_REACTION_S,
    ATP_EMERGENCY_BRAKE_FACTOR,
    ATP_P_REACTION_S,
    ATP_SBI_REACTION_S,
    ATP_SERVICE_BRAKE_FACTOR,
    ATP_W_REACTION_S,
    BALISE_POS_UNCERT_M,
    DCS_DELAY_MAX_S,
    DCS_DELAY_MIN_S,
    DCS_TIMEOUT_S,
    MAX_JERK_MS3,
    ODOMETER_ERROR_RATE,
    OVERLAP_M,
    POS_UNCERT_M,
    PRECISE_STOP_POS_UNCERT_M,
    STATION_POS_UNCERT_M,
)
from SUBSYSTEMS.runtime import Simulation

APP_THEME = {
    "canvas": "#fbf3ef",
    "canvas_grid": "#dec8bf",
    "border": "#6b2e35",
    "text": "#5a2630",
    "muted": "#8a5b52",
    "accent": "#b85c2d",
    "warning": "#f0a132",
    "log_bg": "#fffaf2",
}

class TimeDistancePanel(ttk.Frame):
    def __init__(self, master: tk.Widget):
        super().__init__(master, padding=8, style="Panel.TFrame")
        self.columnconfigure(0, weight=1)
        ttk.Label(self, text="Time-Distance Graph", style="SectionTitle.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(
            self,
            text="Internal simulated running graph; not an ATS/OCC telemetry path",
            style="Muted.TLabel",
        ).grid(row=1, column=0, sticky="w", pady=(2, 6))
        self.canvas = tk.Canvas(self, height=300, background=APP_THEME["canvas"], highlightthickness=1, highlightbackground=APP_THEME["border"])
        self.canvas.grid(row=2, column=0, sticky="ew")

    def update_data(self, sim: Simulation, time_history: deque, position_history: Dict[str, deque]):
        c = self.canvas
        w = max(320, c.winfo_width())
        h = max(180, c.winfo_height())
        c.delete("all")
        c.create_rectangle(1, 1, w - 1, h - 1, outline=APP_THEME["border"], fill=APP_THEME["canvas"])
        if len(time_history) < 2:
            return

        min_t = time_history[0]
        max_t = time_history[-1]
        span_t = max(1.0, max_t - min_t)
        span_pos = max(1.0, sim.track_max_m - sim.track_min_m)
        left = 46
        right = w - 16
        top = 16
        bottom = h - 28

        c.create_text(left, 8, anchor="w", text="Distance", fill=APP_THEME["accent"], font=("Consolas", 8, "bold"))
        c.create_text(right, h - 10, anchor="e", text="Time", fill=APP_THEME["accent"], font=("Consolas", 8, "bold"))
        c.create_line(left, top, left, bottom, fill=APP_THEME["border"])
        c.create_line(left, bottom, right, bottom, fill=APP_THEME["border"])

        for label in sim.track_labels:
            y = bottom - ((label - sim.track_min_m) / span_pos) * (bottom - top)
            c.create_line(left, y, right, y, fill="#ead9d2")
            c.create_text(left - 6, y, anchor="e", text=f"{int(label)}", fill=APP_THEME["muted"], font=("Consolas", 8))

        for train in sim.trains:
            series = position_history.get(train.id)
            if series is None or len(series) < 2:
                continue
            pts = []
            for idx, pos in enumerate(series):
                x = left + ((time_history[idx] - min_t) / span_t) * (right - left)
                y = bottom - ((pos - sim.track_min_m) / span_pos) * (bottom - top)
                pts.extend([x, y])
            c.create_line(*pts, fill=train.color, width=2)
            c.create_text(pts[-2] + 4, pts[-1], anchor="w", text=train.id, fill=train.color, font=("Consolas", 8, "bold"))


class EngineeringPanel(ttk.Frame):
    def __init__(self, master: tk.Widget, scale_factor: float):
        super().__init__(master, padding=int(8 * scale_factor), style="Panel.TFrame")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=0)
        self.rowconfigure(1, weight=1)

        ttk.Label(self, text="Engineering & Config", style="SectionTitle.TLabel").grid(row=0, column=0, sticky="w")

        text_frame = ttk.Frame(self, style="Panel.TFrame")
        text_frame.grid(row=1, column=0, sticky="nsew", pady=(int(6 * scale_factor), 0))
        text_frame.columnconfigure(0, weight=1)
        text_frame.rowconfigure(0, weight=1)

        self.text = tk.Text(
            text_frame,
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
        self.packet_events = []
        self.text.bind("<Button-1>", self._on_packet_row_click)

        vscroll = ttk.Scrollbar(text_frame, orient="vertical", command=self.text.yview)
        vscroll.grid(row=0, column=1, sticky="ns")
        hscroll = ttk.Scrollbar(text_frame, orient="horizontal", command=self.text.xview)
        hscroll.grid(row=1, column=0, sticky="ew")

        self.text.configure(yscrollcommand=vscroll.set, xscrollcommand=hscroll.set)

    def _on_packet_row_click(self, event):
        index = self.text.index(f"@{event.x},{event.y}")
        tags = self.text.tag_names(index)
        for tag in tags:
            if tag.startswith("packet_event_"):
                try:
                    event_index = int(tag.rsplit("_", 1)[1])
                except ValueError:
                    return None
                if 0 <= event_index < len(self.packet_events):
                    self._open_packet_inspector(self.packet_events[event_index])
                    return "break"
        return None

    def _open_packet_inspector(self, event):
        window = tk.Toplevel(self)
        window.title(f"Packet Inspector - {event.protocol} {event.msg_type} #{event.sequence_number}")
        window.geometry(f"{int(860 * self.scale_factor)}x{int(720 * self.scale_factor)}")
        window.columnconfigure(0, weight=1)
        window.rowconfigure(0, weight=1)
        text = tk.Text(
            window,
            wrap="word",
            font=("Consolas", int(9 * self.scale_factor)),
            background=APP_THEME["log_bg"],
            foreground=APP_THEME["text"],
        )
        text.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(window, orient="vertical", command=text.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        text.configure(yscrollcommand=scroll.set)
        details = event.details or {}
        sections = {
            "A. Route": details.get("route", {}),
            "B. Message layer": details.get("message", {}),
            "C. Frame / packet layer": details.get("frame", {}),
            "D. Protection layer": {
                "result": event.result,
                "action": event.action,
                "reason": event.reason,
                **details.get("protection", {}),
            },
            "E. Radio / modulation layer": details.get("radio", {}),
            "Transformation chain": details.get("chain", []),
        }
        lines = []
        for title, payload in sections.items():
            lines.append(title)
            lines.append("-" * len(title))
            lines.append(json.dumps(payload, indent=2, sort_keys=True, default=str))
            lines.append("")
        text.insert("1.0", "\n".join(lines))
        text.configure(state="disabled")

    def update_data(self, sim: Simulation):
        lines = [
            "Static Guideway Database",
            "-" * 72,
        ]
        for idx, (start, end, gradient, psr) in enumerate(sim.track_profile, 1):
            lines.append(
                f"SEG-{idx:02d}  start={start:>6.1f}m  end={end:>6.1f}m  gradient={gradient:+.3f}  SSP={psr:>5.1f} km/h"
            )
        lines.append("")
        lines.append("Balise / Beacon Layout")
        lines.append("-" * 72)
        for balise in getattr(sim, "balises", []):
            lines.append(f"{str(balise.get('id', 'BALISE')):<8} pos={float(balise['pos_m']):>6.1f}m")
        lines.append("")
        lines.append("Train Configuration")
        lines.append("-" * 72)
        for train in sim.trains:
            adhesion_pct = ATP_ADHESION_FACTOR * 100.0
            mute_count = len(train.dcs_mute_windows)
            lines.append(
                f"{train.id:<4} mass={train.mass:>8.0f}kg  length={train.length:>5.1f}m  "
                f"mode={train.drive_mode:<5} manual_cap={train.max_manual_speed_kmh:>4.0f}km/h  "
                f"mute_windows={mute_count}  jerk={MAX_JERK_MS3:.2f}m/s3  adhesion={adhesion_pct:.0f}%"
            )
        lines.append("")
        lines.append("Safety Logic Baseline")
        lines.append("-" * 72)
        lines.extend(
            [
                f"Service brake model : factor={ATP_SERVICE_BRAKE_FACTOR:.2f}  buildup={ATP_BRAKE_BUILDUP_S:.2f}s",
                f"Emergency brake     : factor={ATP_EMERGENCY_BRAKE_FACTOR:.2f}  overlap={OVERLAP_M:.1f}m",
                f"Reaction delays      : P={ATP_P_REACTION_S:.1f}s  W={ATP_W_REACTION_S:.1f}s  SBI={ATP_SBI_REACTION_S:.1f}s  EBI={ATP_EBI_REACTION_S:.1f}s",
                f"Odometer error       : rate={ODOMETER_ERROR_RATE:.2f} m/m  base CI={POS_UNCERT_M:.1f}m  balise CI={BALISE_POS_UNCERT_M:.1f}m  station CI={STATION_POS_UNCERT_M:.1f}m  precise CI={PRECISE_STOP_POS_UNCERT_M:.1f}m",
                f"DCS transmission     : min={DCS_DELAY_MIN_S:.2f}s  max={DCS_DELAY_MAX_S:.2f}s  timeout={DCS_TIMEOUT_S:.1f}s",
            ]
        )
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


class DataFlowPanel(ttk.Frame):
    def __init__(self, master: tk.Widget, scale_factor: float):
        super().__init__(master, padding=int(8 * scale_factor), style="Panel.TFrame")
        self.scale_factor = scale_factor
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)
        self.rowconfigure(4, weight=1)
        ttk.Label(self, text="Dataflow Monitor", style="SectionTitle.TLabel").grid(row=0, column=0, sticky="w")
        self.summary_var = tk.StringVar(value="")
        ttk.Label(self, textvariable=self.summary_var, style="Muted.TLabel").grid(
            row=1,
            column=0,
            sticky="w",
            pady=(int(2 * scale_factor), int(6 * scale_factor)),
        )
        diagram_frame = ttk.Frame(self, style="Panel.TFrame")
        diagram_frame.grid(row=2, column=0, sticky="nsew")
        diagram_frame.columnconfigure(0, weight=1)
        diagram_frame.rowconfigure(0, weight=1)
        self.canvas = tk.Canvas(
            diagram_frame,
            height=int(820 * scale_factor),
            background=APP_THEME["canvas"],
            highlightthickness=1,
            highlightbackground=APP_THEME["border"],
        )
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.canvas_hscroll = ttk.Scrollbar(diagram_frame, orient="horizontal", command=self.canvas.xview)
        self.canvas_hscroll.grid(row=1, column=0, sticky="ew")
        self.canvas.configure(xscrollcommand=self.canvas_hscroll.set)
        self.canvas.bind("<Button-1>", self._on_canvas_packet_click)
        self.canvas_packet_events = []
        self._last_sim = None
        self._resize_after_id = None
        self.canvas.bind("<Configure>", self._on_canvas_resize, add="+")
        self.bind("<Destroy>", self._on_destroy, add="+")

        filter_frame = ttk.Frame(self, style="Panel.TFrame")
        filter_frame.grid(row=3, column=0, sticky="ew", pady=(int(6 * scale_factor), 0))
        filter_frame.columnconfigure(5, weight=1)
        self.packet_filter_var = tk.StringVar(value="All")
        self.packet_flow_var = tk.StringVar(value="All flows")
        self.packet_flow_combo = None
        filter_values = ("All", "Vital only", "OPC UA only", "Rejected only")
        ttk.Label(filter_frame, text="Filter", style="Muted.TLabel").grid(row=0, column=0, sticky="w", padx=(0, 4))
        filter_combo = ttk.Combobox(filter_frame, textvariable=self.packet_filter_var, values=filter_values, width=14, state="readonly")
        filter_combo.grid(row=0, column=1, sticky="w", padx=(0, 8))
        filter_combo.bind("<<ComboboxSelected>>", self._on_filter_changed)
        ttk.Label(filter_frame, text="Flow", style="Muted.TLabel").grid(row=0, column=2, sticky="w", padx=(0, 4))
        self.packet_flow_combo = ttk.Combobox(
            filter_frame,
            textvariable=self.packet_flow_var,
            values=("All flows",),
            width=58,
            state="readonly",
        )
        self.packet_flow_combo.grid(row=0, column=3, sticky="ew")
        self.packet_flow_combo.bind("<<ComboboxSelected>>", self._on_filter_changed)

        text_frame = ttk.Frame(self, style="Panel.TFrame")
        text_frame.grid(row=4, column=0, sticky="nsew", pady=(int(6 * scale_factor), 0))
        text_frame.columnconfigure(0, weight=1)
        text_frame.rowconfigure(0, weight=1)
        self.text = tk.Text(
            text_frame,
            height=int(10 * scale_factor),
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
        self.packet_events = []
        self.text.bind("<Button-1>", self._on_packet_row_click)
        vscroll = ttk.Scrollbar(text_frame, orient="vertical", command=self.text.yview)
        vscroll.grid(row=0, column=1, sticky="ns")
        hscroll = ttk.Scrollbar(text_frame, orient="horizontal", command=self.text.xview)
        hscroll.grid(row=1, column=0, sticky="ew")
        self.text.configure(yscrollcommand=vscroll.set, xscrollcommand=hscroll.set)

    def _on_canvas_resize(self, _event):
        if self._last_sim is None:
            return
        if self._resize_after_id is not None:
            try:
                self.after_cancel(self._resize_after_id)
            except tk.TclError:
                pass
        self._resize_after_id = self.after(120, self._redraw_last_sim)

    def _on_destroy(self, _event):
        if self._resize_after_id is not None:
            try:
                self.after_cancel(self._resize_after_id)
            except tk.TclError:
                pass
            self._resize_after_id = None

    def _redraw_last_sim(self):
        self._resize_after_id = None
        if self._last_sim is not None and self.winfo_exists():
            self.update_data(self._last_sim)

    def _on_canvas_packet_click(self, event):
        item = self.canvas.find_closest(self.canvas.canvasx(event.x), self.canvas.canvasy(event.y))
        if not item:
            return None
        for tag in self.canvas.gettags(item[0]):
            if tag.startswith("canvas_packet_event_"):
                try:
                    event_index = int(tag.rsplit("_", 1)[1])
                except ValueError:
                    return None
                if 0 <= event_index < len(self.canvas_packet_events):
                    self._open_packet_inspector(self.canvas_packet_events[event_index])
                    return "break"
        return None

    def _on_packet_row_click(self, event):
        index = self.text.index(f"@{event.x},{event.y}")
        tags = self.text.tag_names(index)
        for tag in tags:
            if tag.startswith("packet_event_"):
                try:
                    event_index = int(tag.rsplit("_", 1)[1])
                except ValueError:
                    return None
                if 0 <= event_index < len(self.packet_events):
                    self._open_packet_inspector(self.packet_events[event_index])
                    return "break"
        return None

    def _open_packet_inspector(self, event):
        window = tk.Toplevel(self)
        window.title(f"Packet Inspector - {event.protocol} {event.msg_type} #{event.sequence_number}")
        window.geometry(f"{int(900 * self.scale_factor)}x{int(760 * self.scale_factor)}")
        window.columnconfigure(0, weight=1)
        window.rowconfigure(0, weight=1)
        text = tk.Text(
            window,
            wrap="word",
            font=("Consolas", int(9 * self.scale_factor)),
            background=APP_THEME["log_bg"],
            foreground=APP_THEME["text"],
        )
        text.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(window, orient="vertical", command=text.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        text.configure(yscrollcommand=scroll.set)
        details = event.details or {}
        frame = details.get("frame", {})
        sections = {
            "Route": details.get("route", {}),
            "Message": details.get("message", {}),
            "Frame/Header": {
                "header": frame.get("received_header") or frame.get("header") or frame.get("sent_header") or frame,
                "session_id": (frame.get("received_header") or frame.get("header") or {}).get("session_id", ""),
                "sequence_number": event.sequence_number,
                "timestamp": (frame.get("received_header") or frame.get("header") or {}).get("timestamp_ms", frame.get("timestamp_ms", "")),
                "ttl": (frame.get("received_header") or frame.get("header") or {}).get("ttl_ms", frame.get("timeout_ms", "")),
                "packet_uuid": frame.get("safety", {}).get("packet_uuid", ""),
                "payload_length": (frame.get("received_header") or frame.get("header") or {}).get("payload_length", ""),
            },
            "Encryption": {
                "encryption_enabled": frame.get("encryption_enabled", False),
                "encryption_algorithm": frame.get("encryption_algorithm", ""),
                "key_id": frame.get("key_id", frame.get("safety", {}).get("key_id", "")),
                "encrypted_payload": frame.get("encrypted_payload", ""),
                "payload_format": frame.get("payload_format", ""),
            },
            "Protection": {
                "crc32": frame.get("safety", {}).get("crc32", ""),
                "hmac_sha256": frame.get("safety", {}).get("hmac_sha256", ""),
                "validation_result": event.result,
                "reject_reason": event.reason if event.result not in ("ACCEPTED", "DELIVERED", "FAILOVER") else "",
                **details.get("protection", {}),
            },
            "Radio/RAP": details.get("radio", {}),
            "Validation result": {
                "result": event.result,
                "action": event.action,
                "reason": event.reason,
            },
            "State update result": {
                "updated": event.result in ("ACCEPTED", "DELIVERED") and event.action not in ("ignored", "rejected"),
                "action": event.action,
                "reason": event.reason,
            },
            "Transformation chain": details.get("chain", []),
        }
        lines = []
        for title, payload in sections.items():
            lines.append(title)
            lines.append("-" * len(title))
            lines.append(json.dumps(payload, indent=2, sort_keys=True, default=str))
            lines.append("")
        text.insert("1.0", "\n".join(lines))
        text.configure(state="disabled")

    def _on_filter_changed(self, _event=None):
        self.event_generate("<<DataflowFilterChanged>>")
        if self._last_sim is not None and self.winfo_exists():
            self.update_data(self._last_sim)

    def _event_flow_label(self, event) -> str:
        return (
            f"{event.source_id} -> {event.destination_id} | "
            f"{event.protocol} | {event.path} | {event.msg_type}"
        )

    def _refresh_flow_options(self, events):
        flow_values = ["All flows"]
        seen = set()
        for event in events:
            label = self._event_flow_label(event)
            if label in seen:
                continue
            seen.add(label)
            flow_values.append(label)
        if self.packet_flow_combo is not None and tuple(self.packet_flow_combo["values"]) != tuple(flow_values):
            self.packet_flow_combo.configure(values=tuple(flow_values))
        if self.packet_flow_var.get() not in flow_values:
            self.packet_flow_var.set("All flows")

    def _filtered_packet_events(self, events):
        mode = self.packet_filter_var.get() if hasattr(self, "packet_filter_var") else "All"
        selected_flow = self.packet_flow_var.get() if hasattr(self, "packet_flow_var") else "All flows"
        rejected_results = {"REJECTED", "TIMEOUT", "REPLAY", "CRC_ERROR", "HMAC_ERROR", "OUT_OF_ORDER", "DECRYPT_ERROR"}
        filtered = []
        for event in events:
            if mode == "Vital only" and event.protocol != "RASTA_VITAL":
                continue
            if mode == "OPC UA only" and event.protocol != "OPCUA_SUPERVISION":
                continue
            if mode == "Rejected only" and event.result not in rejected_results and event.action not in ("ignored", "rejected"):
                continue
            if selected_flow != "All flows" and self._event_flow_label(event) != selected_flow:
                continue
            filtered.append(event)
        return filtered

    def _scale(self, value: float) -> int:
        scale = getattr(self, "_diagram_scale", self.scale_factor)
        return int(round(value * scale))

    def _function_block(self, x: float, y: float, w: float, h: float, title: str, body: str, fill: str) -> dict[str, float]:
        c = self.canvas
        c.create_rectangle(x, y, x + w, y + h, fill=fill, outline="#1f2933", width=2)
        c.create_text(
            x + w / 2,
            y + self._scale(18),
            text=title,
            fill=APP_THEME["text"],
            font=("Consolas", self._scale(10), "bold"),
        )
        if body:
            c.create_text(
                x + self._scale(12),
                y + self._scale(38),
                anchor="nw",
                text=body,
                fill=APP_THEME["muted"],
                font=("Consolas", self._scale(8)),
                width=max(self._scale(90), w - self._scale(24)),
            )
        return {
            "left": x,
            "right": x + w,
            "top": y,
            "bottom": y + h,
            "cx": x + w / 2,
            "cy": y + h / 2,
            "w": w,
            "h": h,
        }

    def _diagram_box(
        self,
        x: float,
        y: float,
        w: float,
        h: float,
        title: str,
        bullets: list[str],
        outline: str,
        fill: str,
        title_color: str | None = None,
    ) -> dict[str, float]:
        c = self.canvas
        title_color = outline if title_color is None else title_color
        self._rounded_rect(x, y, x + w, y + h, self._scale(10), fill=fill, outline=outline, width=2)
        c.create_text(
            x + w / 2,
            y + self._scale(20),
            text=title,
            fill=title_color,
            font=("Segoe UI", self._scale(12), "bold"),
        )
        for idx, bullet in enumerate(bullets):
            c.create_text(
                x + self._scale(16),
                y + self._scale(48) + idx * self._scale(22),
                anchor="w",
                text=f"• {bullet}",
                fill="#111827",
                font=("Segoe UI", self._scale(9)),
            )
        return {
            "left": x,
            "right": x + w,
            "top": y,
            "bottom": y + h,
            "cx": x + w / 2,
            "cy": y + h / 2,
            "w": w,
            "h": h,
        }

    def _process_box(self, x: float, y: float, w: float, h: float, text: str, color: str) -> dict[str, float]:
        c = self.canvas
        self._rounded_rect(x, y, x + w, y + h, self._scale(8), fill="#ffffff", outline=color, width=2)
        c.create_text(
            x + w / 2,
            y + h / 2,
            text=text,
            fill=color,
            font=("Segoe UI", self._scale(11), "bold"),
        )
        return {"left": x, "right": x + w, "top": y, "bottom": y + h, "cx": x + w / 2, "cy": y + h / 2}

    def _rounded_rect(self, x1: float, y1: float, x2: float, y2: float, radius: float, **kwargs):
        radius = max(1.0, min(radius, (x2 - x1) / 2, (y2 - y1) / 2))
        points = [
            x1 + radius, y1,
            x2 - radius, y1,
            x2, y1,
            x2, y1 + radius,
            x2, y2 - radius,
            x2, y2,
            x2 - radius, y2,
            x1 + radius, y2,
            x1, y2,
            x1, y2 - radius,
            x1, y1 + radius,
            x1, y1,
        ]
        return self.canvas.create_polygon(points, smooth=True, splinesteps=12, **kwargs)

    def _route_label(self, x: float, y: float, text: str, color: str, anchor: str = "center"):
        # Label không có nền để tránh che line phía sau.
        # Giảm nhẹ cỡ chữ để vùng trung tâm đỡ rối.
        self.canvas.create_text(
            x,
            y,
            text=text,
            fill=color,
            anchor=anchor,
            font=("Consolas", self._scale(7), "bold"),
        )
        
    def _lane_arrow(
        self,
        points: list[tuple[float, float]],
        label: str,
        color: str,
        label_segment: int = 0,
        label_offset: float = -16,
        dash: tuple[int, int] | None = None,
        width_px: int = 3,
    ):
        if len(points) < 2:
            return
        flat = [coord for point in points for coord in point]
        self.canvas.create_line(
            *flat,
            fill=color,
            width=width_px,
            arrow=tk.LAST,
            dash=dash,
            capstyle=tk.ROUND,
            joinstyle=tk.ROUND,
        )
        if label:
            segment_index = min(max(0, label_segment), len(points) - 2)
            start = points[segment_index]
            end = points[segment_index + 1]
            lx = (start[0] + end[0]) / 2
            ly = (start[1] + end[1]) / 2
            self._route_label(lx, ly + label_offset, label, color)

    def _update_canvas_scrollregion(self, padding: int | None = None):
        padding = self._scale(70) if padding is None else padding
        bbox = self.canvas.bbox("all")
        if bbox is None:
            width = max(self._scale(980), int(self.canvas.winfo_width() or self._scale(980)))
            height = self._scale(620)
            self.canvas.configure(scrollregion=(0, 0, width, height))
            return
        x1, y1, x2, y2 = bbox
        scrollregion = (
            min(0, x1 - padding),
            min(0, y1 - padding),
            max(int(self.canvas.winfo_width() or 0), x2 + padding),
            y2 + padding,
        )
        self.canvas.configure(scrollregion=scrollregion)
        if hasattr(self, "canvas_hscroll"):
            canvas_width = int(self.canvas.winfo_width() or 0)
            needs_hscroll = scrollregion[2] - scrollregion[0] > canvas_width + 2
            if needs_hscroll:
                self.canvas_hscroll.grid()
            else:
                self.canvas_hscroll.grid_remove()

    def _draw_legend(self, x: float, y: float, compact: bool = False):
        rows = [
            ("#5b6fd6", "Wireless/RAP link through DCS", (6, 4)),
            ("#1f8a8a", "Wired/backbone link through DCS", None),
            ("#ff2020", "ATS operation command to CC/Train", (6, 4)),
            ("#ff5a00", "ATS operation command to ZC", (6, 4)),
            ("#6b7280", "Scenario/internal infrastructure state", None),
        ]
        row_h = self._scale(17 if compact else 20)
        w = self._scale(330 if compact else 360)
        h = self._scale(30) + len(rows) * row_h
        self.canvas.create_rectangle(x, y, x + w, y + h, fill=APP_THEME["canvas"], outline=APP_THEME["border"])
        self.canvas.create_text(
            x + self._scale(10),
            y + self._scale(8),
            anchor="nw",
            text="Legend",
            fill=APP_THEME["text"],
            font=("Consolas", self._scale(8), "bold"),
        )
        for idx, (color, text, dash) in enumerate(rows):
            ly = y + self._scale(34) + idx * row_h
            self.canvas.create_line(x + self._scale(12), ly, x + self._scale(48), ly, fill=color, width=3, dash=dash)
            self.canvas.create_text(
                x + self._scale(58),
                ly,
                anchor="w",
                text=text,
                fill=APP_THEME["muted"],
                font=("Consolas", self._scale(7)),
            )

    def _draw_basic_dataflow_canvas(self, sim: Simulation, events):
        c = self.canvas
        c.delete("all")
        self.canvas_packet_events = []

        canvas_w = max(960, int(c.winfo_width() or self.winfo_width() or 0))
        design_w = 1456.0
        fit_scale = min(self.scale_factor, max(0.58 * self.scale_factor, (canvas_w - 12) / design_w))
        previous_scale = getattr(self, "_diagram_scale", self.scale_factor)
        self._diagram_scale = fit_scale
        width = self._scale(1456)
        height = self._scale(856)
        c.configure(height=height)

        blue = "#0b4bd3"
        orange = "#ff5a00"
        purple = "#6d20d8"
        green = "#0a7a16"
        teal = "#00838c"
        red = "#ff2020"
        gray = "#5c6670"
        dark_green = "#0b6b1c"
        brown = "#a54200"

        train = self._diagram_box(
            self._scale(60),
            self._scale(28),
            self._scale(280),
            self._scale(150),
            "TÀU / CC ONBOARD",
            ["CC_A / CC_B", "Tạo POSITION_REPORT", "Nhận MA_UPDATE", "Gửi TRAIN_STATUS"],
            blue,
            "#f7fbff",
        )
        ats = self._diagram_box(
            self._scale(60),
            self._scale(384),
            self._scale(250),
            self._scale(196),   # tăng chiều cao để các line DCS->ATS nằm gọn trong box
            "ATS / OCC",
            ["Nhận trạng thái non-vital", "Giám sát", "Gửi ATS_OPERATION_COMMAND", "Không tạo MA/EOA"],
            brown,
            "#fffaf4",
        )
        dcs = self._diagram_box(
            self._scale(630),
            self._scale(252),
            self._scale(300),
            self._scale(320),
            "DCS TRANSPORT",
            [],
            purple,
            "#fbf7ff",
            title_color=purple,
        )
        vital = self._process_box(dcs["left"] + self._scale(26), dcs["top"] + self._scale(92), self._scale(248), self._scale(72), "transport_vital()", red)
        supervision = self._process_box(dcs["left"] + self._scale(26), dcs["top"] + self._scale(218), self._scale(248), self._scale(72), "transport_supervision()", blue)
        c.create_text(
            dcs["cx"],
            dcs["bottom"] + self._scale(300),
            text="",
            fill=APP_THEME["canvas"],
            font=("Segoe UI", self._scale(10), "bold"),
        )
        zc = self._diagram_box(
            self._scale(1180),
            self._scale(18),
            self._scale(260),
            self._scale(260),
            "ZC",
            ["Nhận POSITION_REPORT", "Tính EOA / SvL / MA", "Build safe packets", "Xuất MA_UPDATE"],
            dark_green,
            "#f5fff5",
        )
        c.create_line(zc["left"] + self._scale(16), zc["top"] + self._scale(148), zc["right"] - self._scale(16), zc["top"] + self._scale(148), fill=APP_THEME["border"], width=1)
        for idx, text in enumerate(("APPLY_PSR", "ADD/UPDATE", "REMOVE/CLEAR_TSR")):
            c.create_text(
                zc["left"] + self._scale(32),
                zc["top"] + self._scale(176 + idx * 26),
                anchor="w",
                text=f"• {text}",
                fill=APP_THEME["text"],
                font=("Segoe UI", self._scale(10)),
            )
        sgd = self._diagram_box(
            self._scale(1210),
            self._scale(605),
            self._scale(230),
            self._scale(118),
            "STATION",
            ["Track profile / balise", "Route / capacity", "Line conditions"],
            brown,
            "#fffaf4",
        )
        nms = self._diagram_box(
            self._scale(620),
            self._scale(668),
            self._scale(280),
            self._scale(86),
            "DCS_NMS / DCS TRANSPORT",
            ["RED/BLUE status", "Counters / faults"],
            gray,
            "#f9fafb",
        )
        # Vital train-to-ground and ground-to-train paths.
        self._lane_arrow(
            [(train["right"], self._scale(92)), (dcs["cx"] - self._scale(18), self._scale(92)), (dcs["cx"] - self._scale(18), dcs["top"])],
            "POSITION_REPORT / RaSTA_VITAL",
            orange,
            label_segment=0,
            label_offset=-self._scale(16),
            dash=(6, 4),
        )
        self._lane_arrow(
            [(dcs["cx"] + self._scale(22), dcs["top"]), (dcs["cx"] + self._scale(22), self._scale(92)), (zc["left"], self._scale(92))],
            "",
            orange,
            dash=(6, 4),
        )
        self._lane_arrow(
            [(zc["left"], self._scale(148)), (dcs["cx"] + self._scale(78), self._scale(148)), (dcs["cx"] + self._scale(78), dcs["top"])],
            "",
            blue,
            dash=(6, 4),
        )
        self._lane_arrow(
            [(dcs["left"] + self._scale(70), dcs["top"]), (dcs["left"] + self._scale(70), self._scale(148)), (train["right"], self._scale(148))],
            "MA_UPDATE / RaSTA_VITAL",
            blue,
            label_segment=1,
            label_offset=-self._scale(16),
            dash=(6, 4),
        )

        # =========================
        # Clean lane layout for ATS / DCS / TRAIN / ZC area
        # =========================

        # Lane Y positions: sắp lại để các line DCS -> ATS đi ngang, nằm gọn trong khối ATS
        train_status_y = self._scale(350)
        ats_cmd_train_y = self._scale(430)
        zc_status_y = self._scale(466)
        ats_cmd_zc_y = self._scale(500)
        wayside_y = self._scale(532)
        dcs_status_y = self._scale(564)

        # ---------- TRAIN_STATUS : TRAIN -> DCS ----------
        # Tách riêng lane vào DCS và lane ra ATS.
        # Đường DCS -> ATS đi ngang thẳng vào cạnh phải ATS, không gập lên đỉnh ATS.
        train_status_in_y = self._scale(350)
        train_status_out_y = self._scale(398)

        self._lane_arrow(
            [
                (train["cx"], train["bottom"]),
                (train["cx"], train_status_in_y),
                (dcs["left"], train_status_in_y),
            ],
            "TRAIN_STATUS / OPCUA",
            purple,
            label_segment=1,
            label_offset=-self._scale(14),
            dash=(6, 4),
        )

        # ---------- TRAIN_STATUS : DCS -> ATS ----------
        self._lane_arrow(
            [
                (dcs["left"], train_status_out_y),
                (ats["right"], train_status_out_y),
            ],
            "",
            purple,
            dash=(6, 4),
        )

        # ---------- ATS command to TRAIN : ATS -> DCS ----------
        self._lane_arrow(
            [
                (ats["right"], ats_cmd_train_y),
                (dcs["left"], ats_cmd_train_y),
            ],
            "ATS_CMD→TRAIN / VITAL",
            red,
            label_segment=0,
            label_offset=-self._scale(14),
            dash=(6, 4),
        )

        # ---------- ATS command to TRAIN : DCS -> CC/TRAIN ----------
        # Đi vòng gọn lên mép phải tàu, tránh cắt qua bó line giữa
        train_cmd_x = train["right"] + self._scale(18)
        dcs_train_branch_y = self._scale(214)

        self._lane_arrow(
            [
                (dcs["left"] + self._scale(34), dcs["top"]),
                (dcs["left"] + self._scale(34), dcs_train_branch_y),
                (train_cmd_x, dcs_train_branch_y),
                (train_cmd_x, train["bottom"] - self._scale(10)),
                (train["right"], train["bottom"] - self._scale(10)),
            ],
            "→ CC/TRAIN",
            red,
            label_segment=1,
            label_offset=-self._scale(14),
            dash=(6, 4),
        )

        # ---------- ZC_STATUS : ZC -> DCS ----------
        # Giữ nhánh này không gắn label để tránh lặp chữ 2 lần
        self._lane_arrow(
            [
                (zc["left"] + self._scale(28), zc["bottom"]),
                (zc["left"] + self._scale(28), self._scale(336)),
                (dcs["right"], self._scale(336)),
            ],
            "",
            teal,
        )

        # ---------- ZC_STATUS : DCS -> ATS ----------
        self._lane_arrow(
            [
                (dcs["left"], zc_status_y),
                (ats["right"], zc_status_y),
            ],
            "ZC_STATUS",
            teal,
            label_segment=0,
            label_offset=-self._scale(14),
        )

        # ---------- ATS command to ZC : ATS -> DCS ----------
        self._lane_arrow(
            [
                (ats["right"], ats_cmd_zc_y),
                (dcs["left"], ats_cmd_zc_y),
            ],
            "ATS_CMD→ZC / VITAL",
            orange,
            label_segment=0,
            label_offset=-self._scale(14),
            dash=(6, 4),
        )

        # ---------- ATS command to ZC : DCS -> ZC ----------
        self._lane_arrow(
            [
                (dcs["right"], ats_cmd_zc_y),
                (zc["left"] + self._scale(70), ats_cmd_zc_y),
                (zc["left"] + self._scale(70), zc["bottom"]),
            ],
            "PSR / TSR",
            orange,
            label_segment=0,
            label_offset=-self._scale(14),
            dash=(6, 4),
        )

        # ---------- WAYSIDE + STATION : STATION -> DCS ----------
        # Nhánh nguồn không cần label để giảm rối
        self._lane_arrow(
            [
                (sgd["left"], self._scale(632)),
                (self._scale(1080), self._scale(632)),
                (self._scale(1080), wayside_y),
                (dcs["right"], wayside_y),
            ],
            "",
            green,
        )

        # ---------- WAYSIDE + STATION : DCS -> ATS ----------
        self._lane_arrow(
            [
                (dcs["left"], wayside_y),
                (ats["right"], wayside_y),
            ],
            "WAYSIDE + STATION",
            green,
            label_segment=0,
            label_offset=-self._scale(14),
        )

        # ---------- STATION_STATUS : STATION -> ZC ----------
        station_status_x = (
            max(sgd["left"], zc["left"])
            + min(sgd["right"], zc["right"])
        ) / 2

        self._lane_arrow(
            [
                (station_status_x, sgd["top"]),
                (station_status_x, zc["bottom"]),
            ],
            "STATION_STATUS",
            green,
            label_segment=0,
            label_offset=0,
        )

        # ---------- DCS_STATUS : NMS -> DCS ----------
        # Nhánh nguồn không cần label để tránh lặp
        self._lane_arrow(
            [
                (nms["cx"], nms["top"]),
                (nms["cx"], dcs["bottom"]),
            ],
            "",
            gray,
        )

        # ---------- DCS_STATUS : DCS -> ATS ----------
        self._lane_arrow(
            [
                (dcs["left"], dcs_status_y),
                (ats["right"], dcs_status_y),
            ],
            "DCS_STATUS",
            gray,
            label_segment=0,
            label_offset=-self._scale(14),
        )

        c.create_line(
            vital["cx"],
            vital["bottom"],
            vital["cx"],
            supervision["top"],
            fill=gray,
            width=2,
            arrow=tk.BOTH,
            dash=(2, 3),
        )
        c.create_line(vital["cx"], vital["bottom"], vital["cx"], supervision["top"], fill=gray, width=2, arrow=tk.BOTH, dash=(2, 3))

        self._update_canvas_scrollregion(padding=self._scale(18))
        self._diagram_scale = previous_scale

    def update_data(self, sim: Simulation):
        self._last_sim = sim
        trains = sorted(sim.trains, key=lambda item: item.id)
        transport = getattr(sim, "dcs_transport", None)
        raw_events = list(getattr(transport, "events", []))[-120:] if transport is not None else []
        self._refresh_flow_options(raw_events)
        events = self._filtered_packet_events(raw_events)[-80:]
        self.summary_var.set(
            f"Logical dataflow view  |  CC={len(trains)}  stations={len(sim.scheduled_stops)}  "
            f"segments={len(sim.track_profile)}  packet log events={len(events)}  DCS details in packet log"
        )
        self._draw_basic_dataflow_canvas(sim, events)

        header_lines = [
            "time    | from        | to          | protocol          | path       | msg_type        | seq   | latency | ttl | result       | action        | reason",
            "-" * 156,
        ]
        event_lines = []
        if events:
            for event in events:
                event_lines.append(
                    f"{event.time_s:7.3f} | "
                    f"{event.source_id[:11]:<11} | "
                    f"{event.destination_id[:11]:<11} | "
                    f"{event.protocol[:17]:<17} | "
                    f"{event.path[:10]:<10} | "
                    f"{event.msg_type[:15]:<15} | "
                    f"{event.sequence_number:<5} | "
                    f"{event.latency_ms:>6.0f}ms | "
                    f"{event.ttl_state:<3} | "
                    f"{event.result[:12]:<12} | "
                    f"{event.action[:13]:<13} | "
                    f"{event.reason}"
                )
        else:
            event_lines.append("(no packet events yet)")
        footer_lines = ["", "Network Health / DCS-NMS", "-" * 72]
        if transport is not None:
            red = transport.paths.get("RED")
            blue = transport.paths.get("BLUE")
            total_timeout = sum(path.timeout_count for path in transport.paths.values())
            total_lost = sum(path.lost_count for path in transport.paths.values())
            total_sent = sum(path.sent_count for path in transport.paths.values())
            total_loss_pct = (total_lost / total_sent * 100.0) if total_sent else 0.0
            active_path = getattr(transport, "active_path", "")
            rap_text = ", ".join(f"{train_id}={rap_id}" for train_id, rap_id in sorted(getattr(transport, "last_rap_by_train", {}).items())) or "none"
            active = transport.paths.get(active_path)
            latency = f"{active.base_latency_ms:.0f}ms" if active is not None else "n/a"
            jitter = f"{active.jitter_ms:.0f}ms" if active is not None else "n/a"
            footer_lines.extend(
                [
                    f"RED={red.state.value if red else 'N/A':<8} BLUE={blue.state.value if blue else 'N/A':<8} active={active_path:<4} RAP={rap_text}",
                    f"latency={latency:<6} jitter={jitter:<6} packet_loss={total_loss_pct:>5.1f}%  timeout_count={total_timeout:<4} handover_count={getattr(transport, 'handover_count', 0):<4}",
                    f"last_fault={getattr(transport, 'last_fault', '') or 'none'}",
                ]
            )
        else:
            footer_lines.append("DCS transport unavailable")
        footer_lines.extend(["", "Train vital data freshness", "-" * 72])
        for train in trains:
            link = "MUTE" if train.dcs_muted else "OK" if train.safe_packet_valid else "TIMEOUT"
            controller_status = train.controller_status_payload()
            footer_lines.append(
                f"{train.id:<10} MA={getattr(train, 'ma_freshness', 'FRESH'):<7} "
                f"link={link:<8} result={getattr(train, 'vital_packet_result', ''):<12} "
                f"active={controller_status['active_controller']:<4} "
                f"CC_A={controller_status['cc_a_status']:<12} CC_B={controller_status['cc_b_status']:<12} "
                f"switches={controller_status['switch_counter']:<3} reason={getattr(train, 'vital_packet_reason', '')}"
            )
        controller_events = []
        for train in trains:
            controller_events.extend(
                record
                for record in getattr(train, "event_records", [])
                if record.get("event") == "CONTROLLER_SWITCH"
            )
        controller_events.sort(key=lambda record: float(record.get("sim_time", 0.0)))
        footer_lines.extend(["", "Controller switchover events", "-" * 72])
        if controller_events:
            for record in controller_events[-12:]:
                footer_lines.append(
                    f"{float(record.get('sim_time', 0.0)):7.2f}s  "
                    f"{record.get('train_id', '--'):<8} CONTROLLER_SWITCH  {record.get('reason', '')}"
                )
        else:
            footer_lines.append("(no controller switch events)")
        yview = self.text.yview()
        xview = self.text.xview()
        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        self.text.insert("end", "\n".join(header_lines) + "\n")
        self.packet_events = events
        for idx, line in enumerate(event_lines):
            tag = f"packet_event_{idx}"
            if events:
                self.text.insert("end", line + "\n", (tag,))
            else:
                self.text.insert("end", line + "\n")
            self.text.tag_configure(tag, foreground=APP_THEME["text"], underline=False)
        self.text.insert("end", "\n".join(footer_lines))
        if yview:
            self.text.yview_moveto(yview[0])
        if xview:
            self.text.xview_moveto(xview[0])
        self.text.configure(state="disabled")
