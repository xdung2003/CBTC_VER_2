from __future__ import annotations

import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, ttk
import random
import time
import math
import sys
import threading
import queue
from dataclasses import dataclass, replace
from datetime import datetime, timezone, timedelta
from typing import Any, Dict, List, Tuple
from collections import deque
import ctypes

from CONFIG.config import (
    AW3_MASS_KG,
    DT,
    OVERLAP_M,
    SAFETY_MARGIN_M,
    BRAKE_FORCE_N,
    EMERGENCY_FORCE_N,
    BRAKE_BUILDUP_S,
    MAX_JERK_MS3,
    DEPARTURE_RELEASE_MIN_AUTHORITY_M,
    LINE_CENTER_SPACING_M,
    MIN_PASSENGER_DWELL_S,
    PARALLEL_RELEASE_MARGIN_M,
    PARALLEL_ROMAN_LABELS,
    SOURCE_RELEASE_LOCK_S,
    SOURCE_TRAIN_EXIT_M,
    SOURCE_TRAIN_LENGTH_M,
    SOURCE_TRAIN_SPACING_M,
    SOURCE_TRAIN_STAGING_CLEARANCE_M,
    SOURCE_TRAIN_START_M,
    SOURCE_VISIBLE_ACTIVE_TRAINS,
    STATION_ROUTE_APPROACH_M,
    TURNOUT_LOCK_S,
)
from SUBSYSTEMS.physics import (
    kmh_to_ms,
    ms_to_kmh,
    braking_distance_m,
    traction_acceleration_ms2,
    running_resistance_accel_ms2,
    equivalent_mass_adjusted_accel,
    limit_jerk,
)
from CONFIG.scenario_loader import DEFAULT_SCENARIO_PATH, load_scenario
from OPERATION.headway_manager import HeadwayManager
from SUBSYSTEMS import control_common as _control_common
from SUBSYSTEMS.dcs import DCSWatchdog, OnboardControlCenter, RedundantOnboardControlCenter
from SUBSYSTEMS.signalling import (
    AuthorityManager,
    MovementAuthorityLimit,
    SafeMovementPacket,
    VitalBrakeModel,
    braking_curve_profile,
    conservative_brake_decel_ms2,
    get_track_info,
    gradient_adjusted_decel_ms2,
    max_entry_speed_with_buildup,
    max_speed_with_buildup,
    stopping_distance_with_buildup,
    vital_delay_margin_m,
    worst_gradient_in_range,
)
from SUBSYSTEMS.atp import ATPEnvelopeEngine, ATPEnvelopeResult
from SUBSYSTEMS.ato import ATOPilotingEngine, ATOPilotingResult
from SUBSYSTEMS.train import Train, train_color
from SUBSYSTEMS.zc import ZoneController
from SUBSYSTEMS.runtime import Simulation

for _name in _control_common.__all__:
    globals()[_name] = getattr(_control_common, _name)
del _name

TSR_COLOR = "#c94a36"

APP_THEME = {
    "bg": "#f5ebe9",
    "workspace": "#fff4ef",
    "panel": "#ffe2c2",
    "panel_alt": "#f4ba72",
    "card": "#fff8ed",
    "card_alt": "#ffe7c7",
    "canvas": "#fbf3ef",
    "canvas_grid": "#dec8bf",
    "border": "#6b2e35",
    "text": "#5a2630",
    "muted": "#8a5b52",
    "button": "#f4a63c",
    "button_hover": "#ffc15c",
    "button_pressed": "#d8782d",
    "button_active": "#ffd06f",
    "accent": "#b85c2d",
    "accent_pressed": "#8f3f24",
    "run_active": "#f0a132",
    "pause_active": "#ffd166",
    "button_inactive": "#d9a876",
    "danger": "#d84b3c",
    "danger_pressed": "#9b2c2b",
    "ok": "#74b65d",
    "warning": "#f0a132",
    "log_bg": "#fffaf2",
}

CURVE_COLORS = {
    "actual": "#5a2630",
    "P": "#2f7f8f",
    "I": "#8a4f9f",
    "W": "#b86f00",
    "SBD": "#4f8f3a",
    "EBD": "#c94a36",
}

ACTION_COLORS = {
    "WARN": CURVE_COLORS["W"],
    "OFF": "#b86f00",
    "SBI": CURVE_COLORS["SBD"],
    "EBI": CURVE_COLORS["EBD"],
}

from GUI.panels.train_panel import TrainPanel
from GUI.panels.ats_overview_panel import ATSOverviewPanel
from GUI.panels.infrastructure_panel import InfrastructurePanel
from GUI.panels.engineering_panel import DataFlowPanel
from GUI.panels.analytics_panel import AnalyticsPanel
from GUI.panels.control_panel import ControlPanel, SpeedLimitsPanel

class App(tk.Tk):
    def __init__(self):
        super().__init__()

        # Detect DPI and set scaling for responsiveness
        try:
            user32 = ctypes.windll.user32
            user32.SetProcessDPIAware()
            dpi = user32.GetDpiForSystem()
            scale_factor = dpi / 96.0  # 96 is default DPI
            self.tk.call('tk', 'scaling', scale_factor)
        except:
            # Fallback if DPI detection fails
            scale_factor = 1.0

        self.scale_factor = scale_factor
        self.scenario = load_scenario()
        self.title(self.scenario["window_title"])
        # Set fullscreen mode
        self.state('zoomed')  # Windows fullscreen
        self.resizable(True, True)
        self.configure(background=APP_THEME["bg"])
        self._configure_styles()

        self.sim = Simulation(self.scenario)
        self.time_scale = 1
        self.sim_paused = False
        self.selected_train_id: str | None = None
        self.operation_mechanism_var = tk.StringVar(value="")
        self.operation_selected_status_var = tk.StringVar(value="")
        self.headway_target_var = tk.StringVar(value=str(self.scenario.get("headway", {}).get("target_headway_s", 180.0)))
        self.time_scale_buttons: Dict[int, ttk.Button] = {}
        self.time_history = deque(maxlen=240)
        self.position_history: Dict[str, deque] = {}
        self.event_log = deque(maxlen=30)
        self._last_ui_refresh_real_s = 0.0
        self._running_ui_refresh_interval_s = DT
        self._idle_ui_refresh_interval_s = 1.00
        self.prev_train_snapshot: Dict[str, Tuple[str, str, bool, bool, bool]] = {}
        self.child_windows: Dict[str, tk.Toplevel] = {}
        self._reset_runtime_buffers()
        self.pending_line_extension: Tuple[float, float] | None = None
        self.pending_station_prompt_after_limit = False
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)

        container = ttk.Frame(self, style="Shell.TFrame")
        container.grid(row=0, column=0, sticky="nsew")
        container.grid_columnconfigure(0, weight=1)
        container.grid_rowconfigure(0, weight=1)

        self.scroll_canvas = tk.Canvas(container, background=APP_THEME["bg"], highlightthickness=0)
        self.scroll_canvas.grid(row=0, column=0, sticky="nsew")
        vscroll = ttk.Scrollbar(container, orient="vertical", command=self.scroll_canvas.yview)
        vscroll.grid(row=0, column=1, sticky="ns")
        self.scroll_canvas.configure(yscrollcommand=vscroll.set)

        # Bind mouse wheel scrolling
        self.scroll_canvas.bind("<MouseWheel>", self._on_mousewheel)
        self.scroll_canvas.bind("<Button-4>", self._on_mousewheel)
        self.scroll_canvas.bind("<Button-5>", self._on_mousewheel)

        self.content = ttk.Frame(self.scroll_canvas, padding=6, style="Shell.TFrame")
        self.content.grid_columnconfigure(0, weight=1)
        self.content.grid_rowconfigure(5, weight=1)
        self.scroll_canvas_frame = self.scroll_canvas.create_window((0, 0), window=self.content, anchor="nw")

        self.content.bind(
            "<Configure>",
            lambda event: self.scroll_canvas.configure(scrollregion=self.scroll_canvas.bbox("all")),
        )
        self.scroll_canvas.bind(
            "<Configure>",
            lambda event: self.scroll_canvas.itemconfig(self.scroll_canvas_frame, width=event.width, height=event.height),
        )

        header = ttk.Frame(self.content, padding=(6, 4, 6, 2), style="Shell.TFrame")
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)
        btns = ttk.Frame(header, style="Shell.TFrame")
        btns.grid(row=0, column=0, sticky="ew")
        btns.columnconfigure(0, weight=5)
        sim_group = self._make_button_group(btns, "Simulation Control", 0)
        scenario_group = self._make_button_group(btns, "Line Config I/O", 2)
        headway_group = self._make_button_group(btns, "Headway Target", 3)
        tools_group = self._make_button_group(btns, "Tools", 4)

        self.start_btn = ttk.Button(sim_group, text="Start", command=self.on_start, style="Accent.TButton")
        self.start_btn.grid(row=1, column=0, padx=3, pady=(2, 4), sticky="ew")
        self.stop_btn = ttk.Button(sim_group, text="II", command=self.on_stop, style="History.TButton", width=3)
        self.stop_btn.grid(row=1, column=1, padx=3, pady=(2, 4), sticky="ew")
        self.reset_sim_btn = ttk.Button(sim_group, text="⟳", command=self.on_reset_simulation, style="History.TButton", width=3)
        self.reset_sim_btn.grid(row=1, column=2, padx=3, pady=(2, 4), sticky="ew")
        for idx, scale in enumerate((1, 2, 5, 10), start=3):
            button = ttk.Button(sim_group, text=f"x{scale}", command=lambda value=scale: self.set_time_scale(value))
            button.grid(row=1, column=idx, padx=2, pady=(2, 4), sticky="ew")
            self.time_scale_buttons[scale] = button
        self._update_time_scale_buttons()

        self._update_run_pause_buttons()
        self.start_btn.grid_configure(column=0)
        self.stop_btn.grid_configure(column=1)
        self.reset_sim_btn.configure(text="⟳")
        self.reset_sim_btn.grid_configure(column=2)
        for idx, scale in enumerate((1, 2, 5, 10), start=5):
            self.time_scale_buttons[scale].grid_configure(column=idx - 2)
        for idx in range(10):
            sim_group.columnconfigure(idx, weight=1)
        for idx in range(3):
            scenario_group.columnconfigure(idx, weight=1)
        headway_group.columnconfigure(0, weight=0)
        headway_group.columnconfigure(1, weight=1)
        headway_group.columnconfigure(2, weight=0)
        scenario_group.grid_configure(column=1)
        headway_group.grid_configure(column=2)
        tools_group.grid_configure(column=3)
        btns.columnconfigure(1, weight=1)
        btns.columnconfigure(2, weight=0)
        btns.columnconfigure(3, weight=0)
        self._update_run_pause_buttons()

        self.load_btn = ttk.Button(scenario_group, text="Load Line Config", command=self.on_load_scenario)
        self.load_btn.grid(row=1, column=0, padx=3, pady=(2, 4), sticky="ew")
        self.load_btn.grid(row=1, column=0, columnspan=3, padx=3, pady=(2, 4), sticky="ew")

        self.target_label = ttk.Label(headway_group, text="Target seconds", style="Muted.TLabel")
        self.target_label.grid(row=1, column=0, padx=(3, 4), pady=(2, 4), sticky="w")
        self.target_entry = ttk.Entry(headway_group, textvariable=self.headway_target_var, width=10)
        self.target_entry.grid(row=1, column=1, padx=(0, 6), pady=(2, 4), sticky="ew")
        ttk.Button(headway_group, text="Apply + Reset", command=self.apply_headway_block_settings).grid(
            row=1, column=2, padx=(0, 3), pady=(2, 4), sticky="ew"
        )
        ttk.Button(tools_group, text="Dataflow", command=self.open_dataflow_window).grid(
            row=1, column=0, padx=2, pady=(2, 4), sticky="ew"
        )
        ttk.Button(tools_group, text="Speed Restriction", command=self.open_speed_restriction_dialog).grid(
            row=1, column=1, padx=2, pady=(2, 4), sticky="ew"
        )
        tools_group.columnconfigure(0, weight=1)
        tools_group.columnconfigure(1, weight=1)

        self.status_var = tk.StringVar(value="Status: stopped")
        self.scenario_var = tk.StringVar(value=f"Line config: {self.scenario['name']}")
        self.summary_var = tk.StringVar(value="")
        status_row = ttk.Frame(self.content, padding=(6, 0, 6, 0), style="Shell.TFrame")
        status_row.grid(row=1, column=0, sticky="ew")
        status_row.grid_columnconfigure(0, weight=1)
        ttk.Label(status_row, textvariable=self.status_var, style="Status.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(status_row, textvariable=self.scenario_var, style="Status.TLabel").grid(row=0, column=1, sticky="e")

        summary_frame = ttk.Frame(self.content, padding=(6, 3, 6, 0), style="Shell.TFrame")
        summary_frame.grid(row=2, column=0, sticky="ew")
        ttk.Label(summary_frame, textvariable=self.summary_var, style="Status.TLabel").pack(anchor="w")

        self._refresh_operation_mode_controls()

        workspace = ttk.PanedWindow(self.content, orient=tk.HORIZONTAL)
        workspace.grid(row=3, column=0, sticky="nsew", padx=4, pady=(4, 0))
        self.workspace = workspace
        workspace.bind("<Configure>", lambda _event: self.after_idle(self._fit_workspace_panes), add="+")
        self.content.grid_rowconfigure(3, weight=1)
        self.content.grid_rowconfigure(4, weight=0)

        side_shell = ttk.Frame(workspace, padding=(0, 0, 3, 0), style="Shell.TFrame")
        side_shell.columnconfigure(0, weight=1)
        side_shell.rowconfigure(0, weight=1)
        self.side_toolbar_canvas = tk.Canvas(side_shell, background=APP_THEME["workspace"], highlightthickness=0, width=int(138 * self.scale_factor))
        self.side_toolbar_canvas.grid(row=0, column=0, sticky="nsew")
        side_scrollbar = ttk.Scrollbar(side_shell, orient="vertical", command=self.side_toolbar_canvas.yview)
        side_scrollbar.grid(row=0, column=1, sticky="ns")
        self.side_toolbar_canvas.configure(yscrollcommand=side_scrollbar.set)
        side_toolbar = ttk.Frame(self.side_toolbar_canvas, style="Shell.TFrame")
        self.side_toolbar_window = self.side_toolbar_canvas.create_window((0, 0), window=side_toolbar, anchor="nw")
        side_toolbar.bind("<Configure>", lambda _event: self.side_toolbar_canvas.configure(scrollregion=self.side_toolbar_canvas.bbox("all")))
        self.side_toolbar_canvas.bind("<Configure>", lambda event: self.side_toolbar_canvas.itemconfigure(self.side_toolbar_window, width=event.width))
        side_toolbar.columnconfigure(0, weight=1)
        self._bind_side_toolbar_scroll(self.side_toolbar_canvas)

        faults_side = ttk.LabelFrame(side_toolbar, text="Train Commands / Fault Scenarios", padding=4)
        faults_side.grid(row=0, column=0, sticky="ew", pady=(0, 5))
        faults_side.columnconfigure(0, weight=1)
        ttk.Button(faults_side, text="Clear", command=self.clear_all_faults, style="Inactive.TButton").grid(row=0, column=0, sticky="ew", padx=1, pady=1)
        self.emergency_fault_frame = ttk.LabelFrame(faults_side, text="EBI", padding=3)
        self.emergency_fault_frame.grid(row=1, column=0, sticky="ew", padx=1, pady=(4, 1))
        self.train_fault_frame = ttk.LabelFrame(faults_side, text="Train Fault", padding=3)
        self.train_fault_frame.grid(row=2, column=0, sticky="ew", padx=1, pady=1)
        for frame in (self.emergency_fault_frame, self.train_fault_frame):
            frame.columnconfigure(0, weight=1)
        self.train_fault_buttons: Dict[str, Dict[str, ttk.Button]] = {}

        comm_faults_side = ttk.LabelFrame(side_toolbar, text="DCS Fault Commands", padding=4)
        comm_faults_side.grid(row=1, column=0, sticky="ew", pady=(0, 5))
        comm_faults_side.columnconfigure(0, weight=1)
        comm_fault_buttons = [
            ("RED Lost", lambda: self.toggle_communication_path_loss("RED")),
            ("BLUE Lost", lambda: self.toggle_communication_path_loss("BLUE")),
            ("Both Lost", self.toggle_both_communication_paths_loss),
            ("Radio Loss", lambda: self.toggle_communication_fault("radio_coverage_loss", "Radio coverage loss")),
            ("Handover Fail", lambda: self.toggle_communication_fault("handover_failure", "RAP handover failure")),
            ("High Latency", lambda: self.toggle_communication_fault("high_latency", "High latency")),
            ("Packet Loss", lambda: self.toggle_communication_fault("packet_loss", "Packet loss")),
            ("CRC Corrupt", lambda: self.toggle_communication_fault("crc_corruption", "CRC corruption")),
            ("HMAC Corrupt", lambda: self.toggle_communication_fault("hmac_corruption", "HMAC corruption")),
            ("BER Corrupt", lambda: self.toggle_communication_fault("ber_corruption", "BER corruption")),
            ("Replay", lambda: self.toggle_communication_fault("replay_attack", "Replay attack")),
            ("Out-of-order", lambda: self.toggle_communication_fault("out_of_order_packet", "Out-of-order packet")),
            ("OPC UA Loss", lambda: self.toggle_communication_fault("opcua_loss", "OPC UA supervision loss")),
        ]
        for row, (label, command) in enumerate(comm_fault_buttons):
            ttk.Button(comm_faults_side, text=label, command=command, style="Danger.TButton").grid(row=row, column=0, sticky="ew", padx=1, pady=1)
        self.comm_faults_side = comm_faults_side
        self._bind_side_toolbar_tree(side_toolbar)

        workspace.add(side_shell, weight=0)

        canvas_shell = ttk.Frame(workspace, style="Shell.TFrame")
        canvas_shell.columnconfigure(0, weight=1)
        canvas_shell.rowconfigure(0, weight=1)
        self.ats_overview_panel = ATSOverviewPanel(
            canvas_shell,
            self.scale_factor,
            on_select=self.on_ats_element_selected,
            on_edit=None,
        )
        self.ats_overview_panel.grid(row=0, column=0, sticky="nsew")
        workspace.add(canvas_shell, weight=4)

        dock_tabs = ttk.Notebook(workspace, style="Shell.TNotebook")
        self.dock_tabs = dock_tabs
        workspace.add(dock_tabs, weight=2)
        trains_tab = ttk.Frame(dock_tabs, style="Shell.TFrame")
        infra_tab = ttk.Frame(dock_tabs, style="Shell.TFrame")
        self.dataflow_window = None
        self.dataflow_panel = None
        analytics_tab = ttk.Frame(dock_tabs, style="Shell.TFrame")
        for tab in (trains_tab, infra_tab, analytics_tab):
            tab.columnconfigure(0, weight=1)
            tab.rowconfigure(0, weight=1)
        trains_tab.rowconfigure(1, weight=0)
        self.trains_canvas = tk.Canvas(trains_tab, background=APP_THEME["workspace"], highlightthickness=0)
        self.trains_scrollbar = ttk.Scrollbar(trains_tab, orient="horizontal", command=self.trains_canvas.xview)
        self.trains_scrollable_frame = ttk.Frame(self.trains_canvas, style="Shell.TFrame")
        self.trains_scrollable_frame.bind(
            "<Configure>",
            lambda _event: self.trains_canvas.configure(scrollregion=self.trains_canvas.bbox("all")),
        )
        self.trains_window = self.trains_canvas.create_window((0, 0), window=self.trains_scrollable_frame, anchor="nw")
        self.trains_canvas.bind("<Configure>", self._resize_train_boards, add="+")
        self.trains_canvas.configure(xscrollcommand=self.trains_scrollbar.set)
        self._bind_train_horizontal_scroll(self.trains_canvas)
        self._bind_train_horizontal_scroll(self.trains_scrollable_frame)
        self.trains_canvas.grid(row=0, column=0, sticky="nsew")
        self.trains_scrollbar.grid(row=1, column=0, sticky="ew")
        self.infrastructure_panel = InfrastructurePanel(infra_tab, self.scale_factor)
        self.infrastructure_panel.grid(row=0, column=0, sticky="nsew", padx=int(6 * self.scale_factor), pady=(int(6 * self.scale_factor), int(3 * self.scale_factor)))
        self.analytics_panel = AnalyticsPanel(analytics_tab, self.scale_factor)
        self.analytics_panel.grid(row=0, column=0, sticky="nsew", padx=int(6 * self.scale_factor), pady=int(6 * self.scale_factor))
        self.limits_panel = SpeedLimitsPanel(infra_tab, self.scale_factor)
        dock_tabs.add(trains_tab, text="Train")
        dock_tabs.add(infra_tab, text="Infrastructure")
        dock_tabs.add(analytics_tab, text="Analytics")

        button_row = ttk.Frame(self.content, padding=(0, 8, 0, 0), style="Shell.TFrame")
        button_row.grid(row=6, column=0, sticky="ew")
        button_row.grid_columnconfigure(0, weight=1)

        self.panels: Dict[str, TrainPanel] = {}
        self._create_aux_windows()
        self.ats_overview_panel.update_data(self.sim)
        self.infrastructure_panel.update_data(self.sim)
        self.analytics_panel.update_data(self.sim)
        self.limits_panel.update_limits(self._ats_track_profile(), self._ats_tsr_zones())
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.bind_all("<MouseWheel>", self._on_global_mousewheel, add="+")
        self.bind_all("<Button-4>", self._on_global_mousewheel, add="+")
        self.bind_all("<Button-5>", self._on_global_mousewheel, add="+")
        self.after(100, self.tick)
        self.after_idle(self._fit_workspace_panes)

    def _make_button_group(self, master: tk.Widget, title: str, column: int) -> ttk.Frame:
        group = ttk.Frame(master, padding=(6, 2, 6, 2), style="ToolbarGroup.TFrame")
        group.grid(row=0, column=column, sticky="ew", padx=(0, 5))
        master.columnconfigure(column, weight=1 if title == "Simulation Control" else 0)
        ttk.Label(group, text=title, style="ToolbarTitle.TLabel").grid(row=0, column=0, columnspan=8, sticky="w")
        return group

    def open_dataflow_window(self):
        if self.dataflow_window is None or not self.dataflow_window.winfo_exists():
            window = tk.Toplevel(self)
            window.title("Dataflow Monitor")
            window.configure(background=APP_THEME["bg"])
            window.geometry(
                f"{int(1120 * self.scale_factor)}x{int(720 * self.scale_factor)}"
            )
            window.minsize(int(760 * self.scale_factor), int(460 * self.scale_factor))
            window.columnconfigure(0, weight=1)
            window.rowconfigure(0, weight=1)
            self.dataflow_window = window
            outer_canvas = tk.Canvas(window, background=APP_THEME["bg"], highlightthickness=0)
            outer_scroll = ttk.Scrollbar(window, orient="vertical", command=outer_canvas.yview)
            outer_canvas.configure(yscrollcommand=outer_scroll.set)
            outer_canvas.grid(row=0, column=0, sticky="nsew")
            outer_scroll.grid(row=0, column=1, sticky="ns")
            outer_frame = ttk.Frame(outer_canvas, style="Shell.TFrame")
            outer_frame.columnconfigure(0, weight=1)
            outer_frame.rowconfigure(0, weight=1)
            outer_window = outer_canvas.create_window((0, 0), window=outer_frame, anchor="nw")
            outer_frame.bind(
                "<Configure>",
                lambda _event, canvas=outer_canvas: canvas.configure(scrollregion=canvas.bbox("all")),
            )
            outer_canvas.bind(
                "<Configure>",
                lambda event, canvas=outer_canvas, item=outer_window: canvas.itemconfigure(item, width=event.width),
            )

            def _dataflow_mousewheel(event, canvas=outer_canvas):
                if getattr(event, "num", None) == 4:
                    delta = -3
                elif getattr(event, "num", None) == 5:
                    delta = 3
                else:
                    raw = getattr(event, "delta", 0)
                    delta = -3 * int(raw / 120) if raw else 0
                if delta:
                    canvas.yview_scroll(delta, "units")
                return "break"

            outer_canvas.bind("<MouseWheel>", _dataflow_mousewheel)
            outer_canvas.bind("<Button-4>", _dataflow_mousewheel)
            outer_canvas.bind("<Button-5>", _dataflow_mousewheel)
            outer_frame.bind("<MouseWheel>", _dataflow_mousewheel)
            outer_frame.bind("<Button-4>", _dataflow_mousewheel)
            outer_frame.bind("<Button-5>", _dataflow_mousewheel)
            self.dataflow_panel = DataFlowPanel(outer_frame, self.scale_factor)
            self.dataflow_panel.grid(row=0, column=0, sticky="nsew", padx=int(6 * self.scale_factor), pady=int(6 * self.scale_factor))

            def _bind_dataflow_scroll_tree(widget: tk.Widget):
                widget.bind("<MouseWheel>", _dataflow_mousewheel, add="+")
                widget.bind("<Button-4>", _dataflow_mousewheel, add="+")
                widget.bind("<Button-5>", _dataflow_mousewheel, add="+")
                for child in widget.winfo_children():
                    _bind_dataflow_scroll_tree(child)

            _bind_dataflow_scroll_tree(outer_frame)
            window.protocol("WM_DELETE_WINDOW", self._close_dataflow_window)
        if self.dataflow_panel is not None:
            self.dataflow_panel.update_data(self.sim)
        self.dataflow_window.deiconify()
        self.dataflow_window.lift()
        self.dataflow_window.focus_force()

    def _close_dataflow_window(self):
        if self.dataflow_window is not None:
            try:
                self.dataflow_window.destroy()
            except tk.TclError:
                pass
        self.dataflow_window = None
        self.dataflow_panel = None

    def _bind_side_toolbar_scroll(self, widget: tk.Widget):
        if getattr(widget, "_cbtc_side_scroll_bound", False):
            return
        setattr(widget, "_cbtc_side_scroll_bound", True)
        widget.bind("<MouseWheel>", self._on_side_toolbar_mousewheel, add="+")
        widget.bind("<Button-4>", self._on_side_toolbar_mousewheel, add="+")
        widget.bind("<Button-5>", self._on_side_toolbar_mousewheel, add="+")

    def _bind_side_toolbar_tree(self, widget: tk.Widget):
        self._bind_side_toolbar_scroll(widget)
        for child in widget.winfo_children():
            self._bind_side_toolbar_tree(child)

    def _on_side_toolbar_mousewheel(self, event):
        if not hasattr(self, "side_toolbar_canvas"):
            return None
        if getattr(event, "num", None) == 4:
            delta = -4
        elif getattr(event, "num", None) == 5:
            delta = 4
        else:
            delta_raw = getattr(event, "delta", 0)
            if not delta_raw:
                return "break"
            delta = -4 if delta_raw > 0 else 4
        self.side_toolbar_canvas.yview_scroll(delta, "units")
        return "break"

    def _fit_workspace_panes(self):
        if not hasattr(self, "workspace"):
            return
        self.update_idletasks()
        width = max(1, self.workspace.winfo_width())
        left_w = int(156 * self.scale_factor)
        right_w = max(int(390 * self.scale_factor), min(int(520 * self.scale_factor), int(width * 0.28)))
        try:
            self.workspace.sashpos(0, left_w)
            self.workspace.sashpos(1, max(left_w + 480, width - right_w))
        except tk.TclError:
            pass
        self._resize_train_boards()

    def _update_operation_mode_status(self):
        if not hasattr(self, "operation_selected_status_var"):
            return
        self.operation_selected_status_var.set("Mode: moving-block headway target")

    def _refresh_operation_mode_controls(self):
        self.operation_mechanism_var.set("moving_block")
        self._update_operation_mode_status()

    def reload_headway_block_values(self):
        headway = self.scenario.get("headway", {}) if isinstance(self.scenario.get("headway", {}), dict) else {}
        self.headway_target_var.set(str(headway.get("target_headway_s", 180.0)))
        self._refresh_operation_mode_controls()
        self.status_var.set("Status: operation mode values reloaded")

    def apply_headway_block_settings(self):
        try:
            target_headway_s = max(1.0, float(self.headway_target_var.get()))
        except ValueError:
            target_headway_s = 180.0
            self.headway_target_var.set("180.0")
        headway = {"mode": "fixed", "target_headway_s": target_headway_s}
        self.scenario["block_mode"] = "moving_block"
        self.scenario["headway"] = headway
        self.on_reset_simulation()
        self.ats_overview_panel.update_data(self.sim)
        self.status_var.set("Status: applied operation mode and reset simulation")

    def _configure_styles(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        # Scale font sizes based on DPI scaling
        base_font_size = int(9 * self.scale_factor)
        button_font_size = int(11 * self.scale_factor)
        history_button_font_size = int(15 * self.scale_factor)
        header_font_size = int(16 * self.scale_factor)
        section_font_size = int(11 * self.scale_factor)
        status_font_size = int(9 * self.scale_factor)
        muted_font_size = int(8 * self.scale_factor)
        card_title_font_size = int(11 * self.scale_factor)
        tab_font_size = int(10 * self.scale_factor)

        style.configure("TFrame", background=APP_THEME["workspace"])
        style.configure("TLabel", background=APP_THEME["workspace"], foreground=APP_THEME["text"], font=("Consolas", base_font_size))
        style.configure("Shell.TFrame", background=APP_THEME["workspace"])
        style.configure("Panel.TFrame", background=APP_THEME["workspace"])
        style.configure("Card.TFrame", background=APP_THEME["card"], relief="solid", borderwidth=1)
        style.configure("SubCard.TFrame", background=APP_THEME["card_alt"])
        style.configure("ToolbarGroup.TFrame", background=APP_THEME["panel"], relief="ridge", borderwidth=1)
        style.configure("TLabelframe", background=APP_THEME["panel"], foreground=APP_THEME["text"], bordercolor=APP_THEME["border"], lightcolor=APP_THEME["border"], darkcolor=APP_THEME["border"])
        style.configure("TLabelframe.Label", background=APP_THEME["panel"], foreground=APP_THEME["accent"], font=("Consolas", base_font_size, "bold"))
        style.configure("Vertical.TScrollbar", background=APP_THEME["button"], troughcolor=APP_THEME["bg"], bordercolor=APP_THEME["border"], arrowcolor=APP_THEME["border"])
        style.configure("Horizontal.TScrollbar", background=APP_THEME["button"], troughcolor=APP_THEME["bg"], bordercolor=APP_THEME["border"], arrowcolor=APP_THEME["border"])
        style.configure("HeaderTitle.TLabel", background=APP_THEME["workspace"], foreground=APP_THEME["accent"], font=("Consolas", header_font_size, "bold"))
        style.configure("SectionTitle.TLabel", background=APP_THEME["workspace"], foreground=APP_THEME["accent"], font=("Consolas", section_font_size, "bold"))
        style.configure("CardTitle.TLabel", background=APP_THEME["card"], foreground=APP_THEME["accent"], font=("Consolas", card_title_font_size, "bold"))
        style.configure("ToolbarTitle.TLabel", background=APP_THEME["panel"], foreground=APP_THEME["accent"], font=("Consolas", base_font_size, "bold"))
        style.configure("Shell.TNotebook", background=APP_THEME["workspace"], borderwidth=0)
        style.configure(
            "Shell.TNotebook.Tab",
            padding=(int(10 * self.scale_factor), int(6 * self.scale_factor)),
            font=("Consolas", tab_font_size, "bold"),
            background=APP_THEME["panel_alt"],
            foreground=APP_THEME["muted"],
        )
        style.map(
            "Shell.TNotebook.Tab",
            background=[("selected", APP_THEME["card"]), ("active", APP_THEME["button_hover"])],
            foreground=[("selected", APP_THEME["accent"]), ("active", APP_THEME["text"])],
        )
        style.configure("Muted.TLabel", background=APP_THEME["workspace"], foreground=APP_THEME["muted"], font=("Consolas", muted_font_size))
        style.configure("Status.TLabel", background=APP_THEME["workspace"], foreground=APP_THEME["text"], font=("Consolas", status_font_size))
        style.configure(
            "TEntry",
            fieldbackground=APP_THEME["log_bg"],
            foreground=APP_THEME["text"],
            bordercolor=APP_THEME["border"],
            lightcolor=APP_THEME["button_hover"],
            darkcolor=APP_THEME["border"],
        )
        style.configure(
            "TCombobox",
            fieldbackground=APP_THEME["log_bg"],
            foreground=APP_THEME["text"],
            background=APP_THEME["button"],
            bordercolor=APP_THEME["border"],
            arrowcolor=APP_THEME["border"],
        )
        style.map(
            "TCombobox",
            fieldbackground=[("readonly", APP_THEME["log_bg"])],
            background=[("active", APP_THEME["button_hover"]), ("readonly", APP_THEME["button"])],
        )
        style.configure(
            "TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(9, 5),
            background=APP_THEME["button"],
            foreground=APP_THEME["text"],
            bordercolor=APP_THEME["border"],
            lightcolor="#ffd979",
            darkcolor="#b85c2d",
            relief="raised",
        )
        style.map(
            "TButton",
            background=[("pressed", APP_THEME["button_pressed"]), ("active", APP_THEME["button_hover"]), ("disabled", APP_THEME["button_inactive"])],
            foreground=[("disabled", APP_THEME["muted"]), ("!disabled", APP_THEME["text"])],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "Accent.TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(10, 5),
            background=APP_THEME["accent"],
            foreground="#fff8ed",
            bordercolor=APP_THEME["border"],
            relief="raised",
        )
        style.map(
            "Accent.TButton",
            background=[("pressed", APP_THEME["accent_pressed"]), ("active", "#d8782d")],
            foreground=[("disabled", APP_THEME["muted"]), ("!disabled", "#fff8ed")],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "Active.TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(10, 5),
            background=APP_THEME["button_active"],
            foreground=APP_THEME["text"],
            bordercolor=APP_THEME["border"],
            relief="ridge",
        )
        style.map(
            "Active.TButton",
            background=[("pressed", APP_THEME["button_pressed"]), ("active", "#ffe08a"), ("!disabled", APP_THEME["button_active"])],
            relief=[("pressed", "sunken"), ("!pressed", "ridge")],
        )
        style.configure(
            "History.TButton",
            font=("Consolas", history_button_font_size, "bold"),
            padding=(10, 1),
            background=APP_THEME["button"],
            foreground=APP_THEME["text"],
            bordercolor=APP_THEME["border"],
            relief="raised",
        )
        style.map(
            "History.TButton",
            background=[("pressed", APP_THEME["button_pressed"]), ("active", APP_THEME["button_hover"]), ("disabled", APP_THEME["button_inactive"])],
            foreground=[("disabled", APP_THEME["muted"]), ("!disabled", APP_THEME["text"])],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "RunActive.TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(10, 5),
            background=APP_THEME["run_active"],
            foreground="#fff8ed",
            bordercolor=APP_THEME["border"],
            relief="raised",
        )
        style.map(
            "RunActive.TButton",
            background=[("pressed", APP_THEME["accent_pressed"]), ("active", "#ffc15c"), ("!disabled", APP_THEME["run_active"])],
            foreground=[("!disabled", "#fff8ed")],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "PauseActive.TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(10, 5),
            background=APP_THEME["pause_active"],
            foreground="#1f2730",
            bordercolor=APP_THEME["border"],
            relief="raised",
        )
        style.map(
            "PauseActive.TButton",
            background=[("pressed", "#d8782d"), ("active", "#ffe08a"), ("!disabled", APP_THEME["pause_active"])],
            foreground=[("!disabled", "#1f2730")],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "PauseHistoryActive.TButton",
            font=("Consolas", history_button_font_size, "bold"),
            padding=(10, 1),
            background=APP_THEME["pause_active"],
            foreground=APP_THEME["text"],
            bordercolor=APP_THEME["border"],
            relief="raised",
        )
        style.map(
            "PauseHistoryActive.TButton",
            background=[("pressed", "#d8782d"), ("active", "#ffe08a"), ("!disabled", APP_THEME["pause_active"])],
            foreground=[("!disabled", APP_THEME["text"])],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "Danger.TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(9, 5),
            background=APP_THEME["danger"],
            foreground="#fff8ed",
            bordercolor=APP_THEME["danger_pressed"],
            relief="raised",
        )
        style.map(
            "Danger.TButton",
            background=[("pressed", APP_THEME["danger_pressed"]), ("active", "#e85d4f"), ("!disabled", APP_THEME["danger"])],
            foreground=[("disabled", "#f0caca"), ("!disabled", "#fff8ed")],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "Inactive.TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(10, 5),
            background=APP_THEME["button_inactive"],
            foreground=APP_THEME["muted"],
            bordercolor=APP_THEME["border"],
            relief="raised",
        )
        style.map(
            "Inactive.TButton",
            background=[("pressed", "#c18a58"), ("active", "#e7bd8c"), ("!disabled", APP_THEME["button_inactive"])],
            foreground=[("!disabled", APP_THEME["muted"])],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "StartInactive.TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(10, 5),
            background=APP_THEME["button_inactive"],
            foreground=APP_THEME["text"],
            bordercolor=APP_THEME["border"],
            relief="raised",
        )
        style.map(
            "StartInactive.TButton",
            background=[("pressed", "#c18a58"), ("active", "#e7bd8c"), ("!disabled", APP_THEME["button_inactive"])],
            foreground=[("!disabled", APP_THEME["text"])],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )

    def _reset_runtime_buffers(self):
        self.time_history.clear()
        self.position_history = {
            train_id: deque(maxlen=240)
            for train_id in getattr(self.sim, "ats_received_train_state", {})
        }
        self.event_log.clear()
        self.prev_train_snapshot = {}

    def _ats_wayside(self) -> Dict[str, Any]:
        return dict(getattr(self.sim, "ats_received_wayside_state", {}) or {})

    def _ats_zc(self) -> Dict[str, Any]:
        return dict(getattr(self.sim, "ats_received_zc_state", {}) or {})

    def _ats_station(self) -> Dict[str, Any]:
        return dict(getattr(self.sim, "ats_received_station_state", {}) or {})

    def _ats_dcs(self) -> Dict[str, Any]:
        return dict(getattr(self.sim, "ats_received_dcs_state", {}) or {})

    def _ats_track_profile(self) -> List[Tuple[float, float, float, float]]:
        return [tuple(segment) for segment in self._ats_wayside().get("track_profile", [])]

    def _ats_tsr_zones(self) -> List[Dict[str, Any]]:
        return [dict(zone) for zone in self._ats_zc().get("tsr_zones", [])]

    def _on_mousewheel(self, event):
        """Handle mouse wheel scroll events (Windows and Linux)."""
        # Windows uses event.delta (positive = up, negative = down)
        # Linux uses event.num (4 = up, 5 = down)
        if event.num == 4:  # Linux scroll up
            self.scroll_canvas.yview_scroll(-3, "units")
        elif event.num == 5:  # Linux scroll down
            self.scroll_canvas.yview_scroll(3, "units")
        elif event.delta > 0:  # Windows scroll up
            self.scroll_canvas.yview_scroll(-3, "units")
        elif event.delta < 0:  # Windows scroll down
            self.scroll_canvas.yview_scroll(3, "units")
        return "break"  # Prevent default scrolling

    def _on_global_mousewheel(self, event):
        widget = event.widget
        if isinstance(widget, str):
            try:
                widget = self.nametowidget(widget)
            except (KeyError, tk.TclError):
                return None
        if isinstance(widget, tk.Text):
            return None
        try:
            if widget.winfo_toplevel() is not self:
                return None
        except (AttributeError, tk.TclError):
            return None
        if getattr(event, "num", None) == 4:
            delta = -3
        elif getattr(event, "num", None) == 5:
            delta = 3
        else:
            delta = -3 * int(event.delta / 120) if getattr(event, "delta", 0) else 0
        if delta:
            self.scroll_canvas.yview_scroll(delta, "units")
        return None

    def _record_runtime_history(self):
        self.time_history.append(self.sim.sim_time_s)
        for train_id, state in getattr(self.sim, "ats_received_train_state", {}).items():
            if train_id not in self.position_history:
                self.position_history[train_id] = deque(maxlen=240)
            self.position_history[train_id].append(float(state.get("position_m", 0.0)))

    def _update_event_log(self):
        sim_stamp = f"{self.sim.sim_time_s:7.1f}s"
        for train_id, state in getattr(self.sim, "ats_received_train_state", {}).items():
            fault_flags = dict(state.get("fault_flags", {}) or {})
            snapshot = (
                str(state.get("atp_state", "")),
                tuple(sorted((key, bool(value)) for key, value in fault_flags.items())),
                str(state.get("door_state", "")),
                bool(state.get("departure_hold", False)),
                getattr(self.sim, "ats_train_freshness", {}).get(train_id, "LOST"),
            )
            previous = self.prev_train_snapshot.get(train_id)
            if previous is None:
                self.prev_train_snapshot[train_id] = snapshot
                continue
            if previous[0] != snapshot[0]:
                self.event_log.appendleft(f"{sim_stamp}  {train_id}  ATP state {previous[0] or 'NONE'} -> {snapshot[0] or 'NONE'}")
            if previous[1] != snapshot[1]:
                self.event_log.appendleft(f"{sim_stamp}  {train_id}  fault flags changed via TRAIN_STATUS")
            if previous[2] != snapshot[2]:
                self.event_log.appendleft(f"{sim_stamp}  {train_id}  door {snapshot[2] or '--'}")
            if previous[3] != snapshot[3]:
                self.event_log.appendleft(f"{sim_stamp}  {train_id}  departure hold {'active' if snapshot[3] else 'released'}")
            if previous[4] != snapshot[4]:
                self.event_log.appendleft(f"{sim_stamp}  {train_id}  TRAIN_STATUS {snapshot[4]}")
            self.prev_train_snapshot[train_id] = snapshot

    def _create_aux_windows(self):
        for window in list(self.child_windows.values()):
            try:
                window.destroy()
            except tk.TclError:
                pass
        self.child_windows = {}

        self.rebuild_train_panels()
        self._update_control_track_profile()

    def _update_control_track_profile(self):
        panel = getattr(self, "control_panel", None)
        if panel is not None:
            panel.update_track_profile(self._ats_track_profile())

    def rebuild_train_panels(self):
        # Clear existing panels
        for panel in self.panels.values():
            panel.destroy()
        self.panels = {}
        self.position_history = {train.id: deque(maxlen=240) for train in self.sim.trains}

        # Clear the scrollable frame
        for widget in self.trains_scrollable_frame.winfo_children():
            widget.destroy()

        for i, train in enumerate(self.sim.trains):
            wrapper = ttk.Frame(self.trains_scrollable_frame, padding=8, style="Shell.TFrame")
            board_w, board_h = self._train_board_dimensions()
            wrapper.config(width=board_w, height=board_h)
            wrapper.pack(side="left", fill="y", padx=(0, 10))
            wrapper.pack_propagate(False)
            self._bind_train_horizontal_scroll(wrapper)
            panel = TrainPanel(
                wrapper,
                train.id,
                self.toggle_train,
                self.resume_train,
                self.instant_stop_train,
                self.precise_jog_train,
                train.color,
                self.scale_factor,
            )
            panel.config(width=max(280, board_w - 20), height=max(200, board_h - 20))
            panel.pack(fill="both", expand=True)
            panel.pack_propagate(False)
            self._bind_train_horizontal_scroll_tree(wrapper)
            panel.set_track_range(self.sim.track_max_m)
            self.panels[train.id] = panel
        self._rebuild_train_fault_buttons()
        self._resize_train_boards()
        self._update_root_summary()

    def sync_train_panels(self):
        current_ids = {train.id for train in self.sim.trains}
        for train_id, panel in list(self.panels.items()):
            if train_id not in current_ids:
                wrapper = panel.master
                panel.destroy()
                try:
                    wrapper.destroy()
                except tk.TclError:
                    pass
                self.panels.pop(train_id, None)
                self.position_history.pop(train_id, None)

        for train in self.sim.trains:
            if train.id in self.panels:
                self.panels[train.id].set_track_range(self.sim.track_max_m)
                continue
            wrapper = ttk.Frame(self.trains_scrollable_frame, padding=8, style="Shell.TFrame")
            board_w, board_h = self._train_board_dimensions()
            wrapper.config(width=board_w, height=board_h)
            wrapper.pack(side="left", fill="y", padx=(0, 10))
            wrapper.pack_propagate(False)
            self._bind_train_horizontal_scroll(wrapper)
            panel = TrainPanel(
                wrapper,
                train.id,
                self.toggle_train,
                self.resume_train,
                self.instant_stop_train,
                self.precise_jog_train,
                train.color,
                self.scale_factor,
            )
            panel.config(width=max(280, board_w - 20), height=max(200, board_h - 20))
            panel.pack(fill="both", expand=True)
            panel.pack_propagate(False)
            self._bind_train_horizontal_scroll_tree(wrapper)
            panel.set_track_range(self.sim.track_max_m)
            self.panels[train.id] = panel
            if train.id not in self.position_history:
                self.position_history[train.id] = deque(maxlen=240)
        self._rebuild_train_fault_buttons()
        self._resize_train_boards()
        self._update_root_summary()

    def _train_board_dimensions(self) -> Tuple[int, int]:
        canvas_w = max(1, int(getattr(self, "trains_canvas", self).winfo_width() or 0))
        canvas_h = max(1, int(getattr(self, "trains_canvas", self).winfo_height() or 0))
        count = max(1, len(getattr(self.sim, "trains", [])))
        visible_count = min(count, 4)
        gap_px = int(10 * self.scale_factor)
        available_w = max(1, canvas_w - gap_px * max(0, visible_count - 1) - int(16 * self.scale_factor))
        board_w = max(int(360 * self.scale_factor), int(available_w / visible_count))
        board_h = max(int(220 * self.scale_factor), canvas_h - int(8 * self.scale_factor))
        return board_w, board_h

    def _resize_train_boards(self, _event=None):
        if not hasattr(self, "trains_scrollable_frame"):
            return
        board_w, board_h = self._train_board_dimensions()
        for panel in self.panels.values():
            wrapper = panel.master
            try:
                wrapper.config(width=board_w, height=board_h)
                panel.config(width=max(280, board_w - 20), height=max(200, board_h - 20))
            except tk.TclError:
                continue
        self.trains_scrollable_frame.update_idletasks()
        try:
            self.trains_canvas.itemconfigure(self.trains_window, height=max(1, self.trains_canvas.winfo_height()))
        except tk.TclError:
            pass
        self.trains_canvas.configure(scrollregion=self.trains_canvas.bbox("all"))

    def _bind_train_horizontal_scroll(self, widget: tk.Widget):
        if getattr(widget, "_cbtc_train_scroll_bound", False):
            return
        setattr(widget, "_cbtc_train_scroll_bound", True)
        widget.bind("<MouseWheel>", self._on_train_horizontal_mousewheel, add="+")
        widget.bind("<Shift-MouseWheel>", self._on_train_horizontal_mousewheel, add="+")
        widget.bind("<Button-4>", self._on_train_horizontal_mousewheel, add="+")
        widget.bind("<Button-5>", self._on_train_horizontal_mousewheel, add="+")

    def _bind_train_horizontal_scroll_tree(self, widget: tk.Widget):
        self._bind_train_horizontal_scroll(widget)
        for child in widget.winfo_children():
            self._bind_train_horizontal_scroll_tree(child)

    def _on_train_horizontal_mousewheel(self, event):
        if not hasattr(self, "trains_canvas"):
            return None
        if getattr(event, "num", None) == 4:
            delta = -1
        elif getattr(event, "num", None) == 5:
            delta = 1
        else:
            delta_raw = getattr(event, "delta", 0)
            if not delta_raw:
                return "break"
            delta = -1 if delta_raw > 0 else 1
        if delta:
            self.trains_canvas.xview_scroll(delta * 12, "units")
        return "break"

    def _rebuild_train_fault_buttons(self):
        if not hasattr(self, "emergency_fault_frame"):
            return
        for frame in (self.emergency_fault_frame, self.train_fault_frame):
            for child in frame.winfo_children():
                child.destroy()
        self.train_fault_buttons = {}
        emergency_btn = ttk.Button(
            self.emergency_fault_frame,
            text="Apply EBI",
            command=self.instant_stop_selected_trains,
            style="Danger.TButton",
        )
        emergency_btn.grid(row=0, column=0, sticky="ew", padx=1, pady=1)
        fault_buttons = (
            ("ATO CC_A", lambda: self.toggle_selected_train_fault("ATO_CC_A_FAULT")),
            ("ATO CC_B", lambda: self.toggle_selected_train_fault("ATO_CC_B_FAULT")),
            ("ATP CC_A", lambda: self.toggle_selected_train_fault("ATP_CC_A_FAULT")),
            ("ATP CC_B", lambda: self.toggle_selected_train_fault("ATP_CC_B_FAULT")),
            ("INTEGRITY", lambda: self.toggle_selected_train_fault("INTEGRITY")),
        )
        for row, (label, command) in enumerate(fault_buttons):
            btn = ttk.Button(self.train_fault_frame, text=label, command=command)
            btn.grid(row=row, column=0, sticky="ew", padx=1, pady=1)
            self._bind_side_toolbar_scroll(btn)
        self._bind_side_toolbar_scroll(emergency_btn)
        self._bind_side_toolbar_tree(self.emergency_fault_frame)
        self._bind_side_toolbar_tree(self.train_fault_frame)

    def _hide_all_child_windows(self):
        for window in self.child_windows.values():
            try:
                window.withdraw()
            except tk.TclError:
                pass

    def _any_child_visible(self) -> bool:
        return any(str(window.state()) != "withdrawn" for window in self.child_windows.values())

    def _hide_window(self, window: tk.Toplevel):
        try:
            window.withdraw()
        except tk.TclError:
            return
        if not self._any_child_visible():
            try:
                self.deiconify()
            except tk.TclError:
                pass

    def _show_window(self, key: str):
        window = self.child_windows.get(key)
        if window is None:
            return
        self._hide_all_child_windows()
        try:
            self.withdraw()
        except tk.TclError:
            pass
        window.deiconify()
        window.lift()
        window.focus_force()
        try:
            window.state("zoomed")
        except tk.TclError:
            screen_w = window.winfo_screenwidth()
            screen_h = window.winfo_screenheight()
            window.geometry(f"{screen_w}x{screen_h}+0+0")

    def _update_root_summary(self):
        ats_states = getattr(self.sim, "ats_received_train_state", {})
        total_trains = len(ats_states)
        moving = sum(1 for state in ats_states.values() if float(state.get("speed_mps", 0.0)) > 0.1)
        dcs_ok = sum(1 for state in ats_states.values() if not bool((state.get("fault_flags", {}) or {}).get("DCS", False)))
        emergency = sum(
            1
            for state in ats_states.values()
            if bool((state.get("fault_flags", {}) or {}).get("EMERGENCY", False))
            or str(state.get("atp_state", "")) == "ATP_TRIP"
        )
        wayside = getattr(self.sim, "ats_wayside_freshness", "LOST")
        self.summary_var.set(
            f"ATS trains={total_trains}  moving={moving}  DCS healthy={dcs_ok}/{total_trains}  "
            f"EBI active={emergency}  WAYSIDE_STATUS={wayside}  ZC_STATUS={getattr(self.sim, 'ats_zc_freshness', 'LOST')}"
        )

    def _on_close(self):
        self._close_dataflow_window()
        for window in list(self.child_windows.values()):
            try:
                window.destroy()
            except tk.TclError:
                pass
        self.destroy()

    def on_load_scenario(self):
        selected = filedialog.askopenfilename(
            title="Load Line Configuration YAML",
            filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")],
            initialdir=str(DEFAULT_SCENARIO_PATH.parent),
        )
        if not selected:
            return
        was_running = self.sim.running
        self.sim.stop()
        try:
            self.scenario = load_scenario(selected)
        except Exception as exc:
            self.status_var.set(f"Status: failed to load line config ({exc})")
            if was_running:
                self.sim.start()
            return
        self.sim.load_scenario(self.scenario)
        self.title(self.scenario["window_title"])
        self.scenario_var.set(f"Line config: {self.scenario['name']}")
        self.reload_headway_block_values()
        self._update_control_track_profile()
        self._reset_runtime_buffers()
        self.rebuild_train_panels()
        self.status_var.set(f"Status: loaded line config from {selected}")
        if was_running:
            self.sim.start()
        self.sim_paused = False
        self._update_run_pause_buttons()

    def on_start(self):
        self.sim.start()
        self.sim_paused = False
        self._update_run_pause_buttons()
        self.status_var.set("Status: running")

    def on_stop(self):
        self.sim.stop()
        self.sim_paused = True
        self._update_run_pause_buttons()
        self.status_var.set("Status: paused")

    def on_reset_simulation(self):
        was_running = self.sim.running
        self.sim.stop()
        self.sim.load_scenario(self.scenario)
        self._update_control_track_profile()
        self._reset_runtime_buffers()
        self.rebuild_train_panels()
        self.ats_overview_panel.update_data(self.sim)
        self.infrastructure_panel.update_data(self.sim)
        self.limits_panel.update_limits(self._ats_track_profile(), self._ats_tsr_zones())
        self._update_operation_mode_status()
        if was_running:
            self.sim.start()
        self.sim_paused = False
        self._update_run_pause_buttons()
        self.status_var.set("Status: simulation reset")

    def set_time_scale(self, scale: int):
        self.time_scale = max(1, int(scale))
        self._update_time_scale_buttons()
        self.status_var.set(f"Status: time scale x{self.time_scale}")

    def _update_time_scale_buttons(self):
        for scale, button in self.time_scale_buttons.items():
            button.configure(style="Active.TButton" if scale == self.time_scale else "TButton")

    def _update_run_pause_buttons(self):
        if hasattr(self, "start_btn"):
            self.start_btn.configure(style="RunActive.TButton" if self.sim.running else "StartInactive.TButton")
        if hasattr(self, "stop_btn"):
            pause_active = self.sim_paused and not self.sim.running
            self.stop_btn.configure(style="PauseHistoryActive.TButton" if pause_active else "History.TButton")

    def toggle_train(self, train_id: str, emergency: bool = False):
        if not emergency:
            for train in self.sim.trains:
                if train.id == train_id:
                    if train.ato_recovery_state == "READY":
                        train.confirm_ato_ready()
                        self.status_var.set(f"Status: ATO ready confirmed for {train_id}")
                    elif train.ato_recovery_state == "CONFIRMED":
                        train.start_ato_after_ready()
                        self.status_var.set(f"Status: ATO start requested for {train_id}")
                    else:
                        self.status_var.set(f"Status: no onboard action is available for {train_id}")
                    return
            self.status_var.set(f"Status: unknown train {train_id}")
            return
        for train in self.sim.trains:
            if train.id == train_id:
                train.emg_ack = True
                train.acknowledge_emergency_safe()
                self.status_var.set(f"Status: onboard safe confirmed for {train_id}")
                return
        self.status_var.set(f"Status: unknown train {train_id}")

    def resume_train(self, train_id: str):
        for train in self.sim.trains:
            if train.id == train_id:
                train.resume_after_emergency()
                self.status_var.set(f"Status: onboard resume requested for {train_id}")
                return
        self.status_var.set(f"Status: unknown train {train_id}")

    def instant_stop_train(self, train_id: str):
        self.sim.dispatch_ats_operation_command("EMERGENCY_STOP", train_id, reason="ats_panel")

    def _selected_train_target(self) -> str:
        selected = self.selected_train_id or ""
        if selected and any(train.id == selected for train in self.sim.trains):
            return selected
        return ""

    def _target_label(self, train_id: str) -> str:
        return train_id if train_id else "all trains"

    def instant_stop_selected_trains(self):
        train_id = self._selected_train_target()
        ok = self.sim.dispatch_ats_operation_command("EMERGENCY_STOP", train_id, reason="ats_panel")
        self.status_var.set(
            f"Status: queued ATS EBI command for {self._target_label(train_id)}"
            if ok
            else f"Status: failed to send ATS EBI command for {self._target_label(train_id)}"
        )

    def precise_jog_train(self, train_id: str):
        if self.sim.dispatch_ats_operation_command("PRECISE_JOG", train_id, reason="ats_panel"):
            self.status_var.set(f"Status: queued ATS precise jog command for {train_id}")
        else:
            self.status_var.set(f"Status: failed to send ATS precise jog command for {train_id}")

    def toggle_train_fault(self, train_id: str, subsystem: str):
        subsystem = subsystem.upper()
        ok = self.sim.dispatch_ats_operation_command(
            "TOGGLE_TRAIN_FAULT",
            train_id,
            {"subsystem": subsystem},
            reason="ats_panel",
        )
        self.status_var.set(
            f"Status: queued RaSTA {subsystem} fault command for {train_id}"
            if ok
            else f"Status: failed to send RaSTA {subsystem} fault command for {train_id}"
        )

    def toggle_selected_train_fault(self, subsystem: str):
        train_id = self._selected_train_target()
        subsystem = subsystem.upper()
        ok = self.sim.dispatch_ats_operation_command(
            "TOGGLE_TRAIN_FAULT",
            train_id,
            {"subsystem": subsystem},
            reason="ats_panel",
        )
        self.status_var.set(
            f"Status: queued RaSTA {subsystem} fault command for {self._target_label(train_id)}"
            if ok
            else f"Status: failed to send RaSTA {subsystem} fault command for {self._target_label(train_id)}"
        )

    def clear_all_faults(self):
        train_id = self._selected_train_target()
        ok = self.sim.dispatch_ats_operation_command("CLEAR_TRAIN_FAULTS", train_id, reason="ats_panel")
        comm_ok = self.sim.dispatch_ats_operation_command("CLEAR_COMM_FAULTS", "", reason="ats_panel")
        self.status_var.set(
            f"Status: queued RaSTA clear command for {self._target_label(train_id)} and DCS"
            if ok and comm_ok
            else "Status: failed to send one or more RaSTA clear commands"
        )

    def toggle_communication_path_loss(self, path_name: str):
        key = path_name.upper()
        transport_state = dict(self._ats_dcs().get("dcs_transport_state", {}) or {})
        path = dict((transport_state.get("paths", {}) or {}).get(key, {}) or {})
        if not path:
            self.status_var.set(f"Status: unknown DCS path {key}")
            return
        new_state = "OK" if str(path.get("state", "OK")).upper() == "LOST" else "LOST"
        ok = self.sim.dispatch_ats_operation_command(
            "SET_DCS_PATH_STATE",
            "",
            {"path": key, "state": new_state},
            reason="ats_panel",
        )
        self.status_var.set(
            f"Status: queued RaSTA {key} path {new_state}"
            if ok
            else f"Status: failed to send RaSTA {key} path {new_state}"
        )

    def toggle_both_communication_paths_loss(self):
        transport_state = dict(self._ats_dcs().get("dcs_transport_state", {}) or {})
        paths = dict(transport_state.get("paths", {}) or {})
        if not paths:
            self.status_var.set("Status: DCS transport state is not available in DCS_STATUS")
            return
        both_lost = all(str(path.get("state", "OK")).upper() == "LOST" for path in paths.values())
        new_state = "OK" if both_lost else "LOST"
        results = []
        for key in ("RED", "BLUE"):
            results.append(self.sim.dispatch_ats_operation_command(
                "SET_DCS_PATH_STATE",
                "",
                {"path": key, "state": new_state},
                reason="ats_panel",
            ))
        self.status_var.set(
            f"Status: queued RaSTA RED/BLUE paths {new_state}"
            if all(results)
            else f"Status: failed to send one or more RaSTA RED/BLUE path {new_state} commands"
        )

    def toggle_communication_fault(self, fault: str, label: str):
        transport_state = dict(self._ats_dcs().get("dcs_transport_state", {}) or {})
        active = not bool((transport_state.get("faults", {}) or {}).get(fault, False))
        ok = self.sim.dispatch_ats_operation_command(
            "SET_DCS_FAULT",
            "",
            {"fault": fault, "active": active},
            reason="ats_panel",
        )
        state_text = "ON" if active else "OFF"
        self.status_var.set(
            f"Status: queued RaSTA {label} {state_text}"
            if ok
            else f"Status: failed to send RaSTA {label} {state_text}"
        )

    def clear_communication_faults(self):
        ok = self.sim.dispatch_ats_operation_command("CLEAR_COMM_FAULTS", "", reason="ats_panel")
        self.status_var.set(
            "Status: queued RaSTA all communication faults clear command"
            if ok
            else "Status: failed to send RaSTA clear communication faults command"
        )

    def on_ats_element_selected(self, element_key: str):
        if element_key.startswith("train:"):
            self.selected_train_id = element_key.split(":", 1)[1]
            self.status_var.set(f"Status: selected train {self.selected_train_id}; fault buttons target this train")
            return
        self.selected_train_id = None
        self.status_var.set(
            f"Status: selected ATS/OCC element {element_key}"
        )

    def open_canvas_speed_limit_editor(self, kind: str, index: int):
        if kind == "track_segment":
            self.open_psr_editor(index)
        elif kind == "tsr":
            self.open_tsr_editor(index)
        else:
            self.status_var.set(f"Status: {kind}:{index} is not a speed-limit item")

    def open_psr_editor(self, segment_index: int):
        track_profile = self._ats_track_profile()
        if segment_index < 0 or segment_index >= len(track_profile):
            self.status_var.set("Status: invalid PSR segment")
            return
        start, end, _gradient, current_psr = track_profile[segment_index]
        value = simpledialog.askfloat(
            "Edit PSR",
            f"Segment {segment_index}: {start:.0f}-{end:.0f} m\nPSR km/h:",
            initialvalue=float(current_psr),
            minvalue=1.0,
            parent=self,
        )
        if value is None:
            return
        self.apply_psr(str(segment_index), str(value))

    def open_speed_restriction_dialog(self):
        choice = simpledialog.askstring(
            "Speed Restriction",
            "Choose restriction type: PSR or TSR",
            initialvalue="PSR",
            parent=self,
        )
        if choice is None:
            return
        normalized = choice.strip().upper()
        if normalized == "PSR":
            self.open_psr_dialog()
        elif normalized == "TSR":
            self.open_add_tsr_dialog()
        else:
            self.status_var.set("Status: choose PSR or TSR")

    def open_psr_dialog(self):
        track_profile = self._ats_track_profile()
        default_segment = "SEG-01"
        if self.ats_overview_panel.selected_element:
            kind, index = self.ats_overview_panel._element_lookup.get(self.ats_overview_panel.selected_element, ("", -1))
            if kind == "track_segment" and 0 <= index < len(track_profile):
                default_segment = f"SEG-{index + 1:02d}"
        segment = simpledialog.askstring(
            "Apply PSR",
            "Track segment (SEG-xx or index):",
            initialvalue=default_segment,
            parent=self,
        )
        if segment is None:
            return
        try:
            idx = self._parse_segment_index(segment)
        except ValueError:
            self.status_var.set("Status: invalid PSR segment")
            return
        if idx < 0 or idx >= len(track_profile):
            self.status_var.set("Status: invalid PSR segment")
            return
        start, end, _gradient, current_psr = track_profile[idx]
        psr = simpledialog.askfloat(
            "Apply PSR",
            f"SEG-{idx + 1:02d}: {start:.0f}-{end:.0f} m\nPSR km/h:",
            initialvalue=float(current_psr),
            minvalue=1.0,
            parent=self,
        )
        if psr is None:
            return
        self.apply_psr(str(idx), str(psr))

    def open_add_tsr_dialog(self):
        default_start = 0.0
        wayside = self._ats_wayside()
        track_profile = self._ats_track_profile()
        default_end = min(500.0, float(wayside.get("track_end_m", 500.0) or 500.0))
        if self.ats_overview_panel.selected_element:
            kind, index = self.ats_overview_panel._element_lookup.get(self.ats_overview_panel.selected_element, ("", -1))
            if kind == "track_segment" and 0 <= index < len(track_profile):
                default_start, default_end, _gradient, _psr = track_profile[index]
        start = simpledialog.askfloat("Add Temporary TSR", "Start position (m):", initialvalue=float(default_start), parent=self)
        if start is None:
            return
        end = simpledialog.askfloat("Add Temporary TSR", "End position (m):", initialvalue=float(default_end), parent=self)
        if end is None:
            return
        speed = simpledialog.askfloat("Add Temporary TSR", "TSR speed (km/h):", initialvalue=25.0, minvalue=1.0, parent=self)
        if speed is None:
            return
        self.add_tsr(str(start), str(end), str(speed))

    def open_tsr_editor(self, tsr_index: int):
        tsr_zones = self._ats_tsr_zones()
        if tsr_index < 0 or tsr_index >= len(tsr_zones):
            self.status_var.set("Status: invalid TSR zone")
            return
        zone = tsr_zones[tsr_index]
        if messagebox.askyesno("Temporary TSR", "Remove this TSR zone?", parent=self):
            ok = self.sim.dispatch_ats_operation_command(
                "REMOVE_TSR",
                "",
                {"index": tsr_index},
                reason="ats_panel",
            )
            self.status_var.set(
                f"Status: queued RaSTA remove TSR {tsr_index + 1}"
                if ok
                else f"Status: failed to send RaSTA remove TSR {tsr_index + 1}"
            )
            return
        speed = simpledialog.askfloat(
            "Edit Temporary TSR",
            f"TSR {tsr_index + 1}: {float(zone['start']):.0f}-{float(zone['end']):.0f} m\nSpeed km/h:",
            initialvalue=float(zone["speed"]),
            minvalue=1.0,
            parent=self,
        )
        if speed is None:
            return
        ok = self.sim.dispatch_ats_operation_command(
            "UPDATE_TSR",
            "",
            {"index": tsr_index, "speed": float(speed)},
            reason="ats_panel",
        )
        self.status_var.set(
            f"Status: queued RaSTA TSR {tsr_index + 1} update to {speed:.0f} km/h"
            if ok
            else f"Status: failed to send RaSTA TSR {tsr_index + 1} update"
        )

    def _refresh_speed_limit_views(self):
        self._update_control_track_profile()
        self.limits_panel.update_limits(self._ats_track_profile(), self._ats_tsr_zones())
        self.ats_overview_panel.update_data(self.sim)
        self.infrastructure_panel.update_data(self.sim)

    def _parse_segment_index(self, segment_str: str) -> int:
        text = str(segment_str).strip().upper()
        if text.startswith("SEG-"):
            return int(text.split("-", 1)[1]) - 1
        if text.startswith("SEG"):
            return int(text[3:]) - 1
        return int(text)

    def apply_psr(self, segment_str: str, psr_str: str):
        try:
            idx = self._parse_segment_index(segment_str)
            psr = float(psr_str)
            if idx < 0 or idx >= len(self._ats_track_profile()):
                self.status_var.set("Status: invalid PSR segment")
                return
        except ValueError:
            return
        ok = self.sim.dispatch_ats_operation_command(
            "APPLY_PSR",
            "",
            {"segment": idx, "psr_kmh": psr},
            reason="ats_panel",
        )
        self.status_var.set(
            f"Status: queued RaSTA PSR command for segment {idx} to {psr:.0f} km/h"
            if ok
            else f"Status: failed to send RaSTA PSR command for segment {idx}"
        )

    def add_tsr(self, start_str: str, end_str: str, speed_str: str):
        try:
            start = float(start_str)
            end = float(end_str)
            speed = float(speed_str)
            if end < start:
                start, end = end, start
            if end <= start:
                self.status_var.set("Status: TSR end must be greater than start")
                return
        except ValueError:
            return
        ok = self.sim.dispatch_ats_operation_command(
            "ADD_TSR",
            "",
            {"start": start, "end": end, "speed": speed},
            reason="ats_panel",
        )
        self.status_var.set(
            f"Status: queued RaSTA TSR command {start:.0f}-{end:.0f} m at {speed:.0f} km/h"
            if ok
            else f"Status: failed to send RaSTA TSR command {start:.0f}-{end:.0f} m"
        )

    def clear_tsr(self):
        if not self._ats_tsr_zones():
            return
        ok = self.sim.dispatch_ats_operation_command("CLEAR_TSR", "", reason="ats_panel")
        self.status_var.set(
            "Status: queued RaSTA clear TSR command"
            if ok
            else "Status: failed to send RaSTA clear TSR command"
        )

    def tick(self):
        if self.sim.running:
            steps_this_tick = max(1, int(self.time_scale))
            for _ in range(steps_this_tick):
                self.sim.step()
            if self.sim.train_generation_changed:
                self.sync_train_panels()
        now = time.monotonic()
        refresh_interval = self._running_ui_refresh_interval_s if self.sim.running else self._idle_ui_refresh_interval_s
        if now - self._last_ui_refresh_real_s >= refresh_interval:
            self._last_ui_refresh_real_s = now
            if self.sim.running:
                self._record_runtime_history()
            self._update_event_log()
            self._update_root_summary()
            ats_states = getattr(self.sim, "ats_received_train_state", {})
            freshness_map = getattr(self.sim, "ats_train_freshness", {})
            for train_id, status in sorted(ats_states.items()):
                if train_id not in self.panels:
                    self.sync_train_panels()
                panel = self.panels.get(train_id)
                if panel is not None:
                    panel.update_from_status(
                        status,
                        freshness=freshness_map.get(train_id, "LOST"),
                        append_history=self.sim.running,
                    )
            self.limits_panel.update_limits(self._ats_track_profile(), self._ats_tsr_zones())
            self.ats_overview_panel.update_data(self.sim)
            self.infrastructure_panel.update_data(self.sim)
            if self.dataflow_panel is not None:
                self.dataflow_panel.update_data(self.sim)
            self.analytics_panel.update_data(self.sim)
        self.after(int(DT * 1000), self.tick)


# GUI startup is kept in run.py. This module remains import-compatible for tests
# and for code that still imports the legacy combined module during refactoring.
