from __future__ import annotations

from GUI.main_gui import *

class TrainPanel(ttk.Frame):
    def __init__(self, master: tk.Widget, train_id: str, on_toggle, on_resume, on_instant_stop, on_precise_jog, color: str, scale_factor: float):
        super().__init__(master, style="Card.TFrame")
        self.train_id = train_id
        self.on_toggle = on_toggle
        self.on_resume = on_resume
        self.on_instant_stop = on_instant_stop
        self.on_precise_jog = on_precise_jog
        self.color = color
        self.scale_factor = scale_factor
        self.track_max_m = 2000.0
        self.emg_state = 0
        self.history_len = 180
        self.hist_actual = deque(maxlen=self.history_len)
        self.hist_curves = {
            "P": deque(maxlen=self.history_len),
            "I": deque(maxlen=self.history_len),
            "W": deque(maxlen=self.history_len),
            "SBI": deque(maxlen=self.history_len),
            "SBD": deque(maxlen=self.history_len),
            "EBI": deque(maxlen=self.history_len),
            "EBD": deque(maxlen=self.history_len),
        }
        self.curve_vars = {
            "actual": tk.StringVar(value="0.0 km/h"),
            "P": tk.StringVar(value="0.0 km/h"),
            "I": tk.StringVar(value="0.0 km/h"),
            "W": tk.StringVar(value="0.0 km/h"),
            "SBI": tk.StringVar(value="0.0 km/h"),
            "SBD": tk.StringVar(value="0.0 km/h"),
            "EBI": tk.StringVar(value="0.0 km/h"),
            "EBD": tk.StringVar(value="0.0 km/h"),
        }
        self.current_release_speed = None

        # Use direct frame container instead of internal scrolling so train panels show full content.
        self.config(width=600, height=420)
        self.pack_propagate(False)
        self.grid_propagate(False)
        container = self
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(1, weight=0)
        self.rowconfigure(2, weight=1)
        self.rowconfigure(3, weight=0)
        self.rowconfigure(4, weight=0)
        self.rowconfigure(5, weight=0)
        container.columnconfigure(0, weight=0, minsize=int(180 * scale_factor))
        container.columnconfigure(1, weight=1)

        # Header
        header = ttk.Frame(container, style="Card.TFrame")
        header.grid(row=0, column=0, columnspan=2, sticky="ew", padx=int(10 * scale_factor), pady=(int(10 * scale_factor), 0))
        header.columnconfigure(0, weight=1)
        ttk.Label(
            header,
            text=f"Train {train_id}",
            style="CardTitle.TLabel",
            foreground=self.color,
        ).grid(row=0, column=0, sticky="w")
        self.mode_var = tk.StringVar(value="A")
        self.mode_label = ttk.Label(
            header,
            textvariable=self.mode_var,
            font=("Consolas", int(12 * scale_factor), "bold"),
            foreground=APP_THEME["accent"],
        )
        self.mode_label.grid(row=0, column=1)
        self.badge_var = tk.StringVar(value="OK")
        self.badge_label = tk.Label(
            header,
            textvariable=self.badge_var,
            bg=APP_THEME["ok"],
            fg="#243018",
            font=("Consolas", int(8 * scale_factor), "bold"),
            padx=int(8 * scale_factor),
            pady=int(3 * scale_factor),
        )
        self.badge_label.grid(row=0, column=2, sticky="e")

        # Main content: target/operations on the left, speed curves on the right.
        target_frame = ttk.Frame(container, style="Card.TFrame")
        target_frame.grid(row=1, column=0, sticky="new", padx=(int(10 * scale_factor), int(5 * scale_factor)), pady=(int(6 * scale_factor), int(4 * scale_factor)))
        ttk.Label(target_frame, text="Target Distance", style="CardTitle.TLabel").pack()
        self.target_distance_var = tk.StringVar(value="0 m")
        ttk.Label(target_frame, textvariable=self.target_distance_var, font=("Consolas", int(16 * scale_factor), "bold")).pack()
        self.target_speed_var = tk.StringVar(value="0 km/h")
        ttk.Label(target_frame, textvariable=self.target_speed_var, font=("Consolas", int(12 * scale_factor))).pack()

        # Operational Info under target distance.
        op_frame = ttk.Frame(target_frame, style="Card.TFrame")
        op_frame.pack(fill="x", pady=(int(8 * scale_factor), 0))
        ttk.Label(op_frame, text="Operational Status", style="CardTitle.TLabel").pack()
        self.door_var = tk.StringVar(value="Closed")
        ttk.Label(op_frame, textvariable=self.door_var, font=("Consolas", int(10 * scale_factor))).pack()
        self.docking_var = tk.StringVar(value="Not Docked")
        ttk.Label(op_frame, textvariable=self.docking_var, font=("Consolas", int(10 * scale_factor))).pack()
        self.headway_var = tk.StringVar(value="-- s")
        ttk.Label(op_frame, text="Headway:", font=("Consolas", int(9 * scale_factor))).pack()
        ttk.Label(op_frame, textvariable=self.headway_var, font=("Consolas", int(10 * scale_factor), "bold")).pack()
        self.controller_var = tk.StringVar(value="Active CC: CC_A")
        self.cc_a_var = tk.StringVar(value="CC_A: HEALTHY")
        self.cc_b_var = tk.StringVar(value="CC_B: HEALTHY")
        ttk.Label(op_frame, textvariable=self.controller_var, font=("Consolas", int(9 * scale_factor), "bold")).pack(pady=(int(6 * scale_factor), 0))
        ttk.Label(op_frame, textvariable=self.cc_a_var, font=("Consolas", int(9 * scale_factor))).pack()
        ttk.Label(op_frame, textvariable=self.cc_b_var, font=("Consolas", int(9 * scale_factor))).pack()

        # Curves Table (Right)
        curves_frame = ttk.Frame(container, style="Card.TFrame")
        curves_frame.grid(row=1, column=1, sticky="new", padx=(int(5 * scale_factor), int(10 * scale_factor)), pady=(int(6 * scale_factor), int(4 * scale_factor)))
        curves_frame.columnconfigure(0, weight=0)
        curves_frame.columnconfigure(1, weight=1)
        ttk.Label(curves_frame, text="Speed Curves", style="CardTitle.TLabel").grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, int(6 * scale_factor)))
        
        # Create variables for curves
        self.curve_vars = {
            "Actual": tk.StringVar(value="0.0 km/h"),
            "Permitted": tk.StringVar(value="0.0 km/h"),
            "Warning": tk.StringVar(value="0.0 km/h"),
            "SBI": tk.StringVar(value="0.0 km/h"),
            "EBI": tk.StringVar(value="0.0 km/h"),
        }
        
        for row, (curve_name, var) in enumerate(self.curve_vars.items(), start=1):
            ttk.Label(curves_frame, text=f"{curve_name}:", font=("Consolas", int(10 * scale_factor))).grid(row=row, column=0, sticky="w", pady=1)
            ttk.Label(curves_frame, textvariable=var, font=("Consolas", int(11 * scale_factor), "bold")).grid(row=row, column=1, sticky="w", padx=(int(8 * scale_factor), 0), pady=1)

        # Braking Curves Chart
        chart_frame = ttk.Frame(container, style="Card.TFrame")
        chart_frame.grid(row=2, column=0, columnspan=2, sticky="nsew", padx=int(10 * scale_factor), pady=(0, int(4 * scale_factor)))
        ttk.Label(chart_frame, text="Braking Curves & Speed", style="CardTitle.TLabel").pack(pady=(0, int(5 * scale_factor)))
        self.chart_canvas = tk.Canvas(
            chart_frame,
            width=int(560 * scale_factor),
            height=int(150 * scale_factor),
            background=APP_THEME["card_alt"],
            highlightthickness=1,
            highlightbackground=APP_THEME["border"],
        )
        self.chart_canvas.pack(fill="both", expand=True)

        # Message Area
        self.message_var = tk.StringVar(value="")
        message_label = ttk.Label(
            container,
            textvariable=self.message_var,
            font=("Consolas", int(10 * scale_factor)),
            foreground="#6b2e35",
            background="#ffd6c9",
            padding=int(5 * scale_factor),
        )
        self.release_var = tk.StringVar(value="")
        self.release_label = ttk.Label(
            container,
            textvariable=self.release_var,
            font=("Consolas", int(10 * scale_factor), "bold"),
            foreground="#7a3d1c",
            background="#ffe8b5",
            padding=int(8 * scale_factor),
        )
        self.release_label.grid(row=3, column=0, columnspan=2, sticky="ew", padx=int(10 * scale_factor), pady=(0, int(10 * scale_factor)))
        self.release_label.grid_remove()

        message_label.grid(row=4, column=0, columnspan=2, sticky="ew", padx=int(10 * scale_factor), pady=(0, int(10 * scale_factor)))

        # Jogging Status (hidden by default)
        self.jog_var = tk.StringVar(value="")
        self.jog_label = ttk.Label(
            container,
            textvariable=self.jog_var,
            font=("Consolas", int(8 * scale_factor), "bold"),
            foreground=APP_THEME["accent"],
            background=APP_THEME["card_alt"],
            padding=int(3 * scale_factor),
        )
        self.jog_label.grid_remove()

        # Buttons
        btn_row = ttk.Frame(container, style="Card.TFrame")
        btn_row.grid(row=5, column=0, columnspan=2, sticky="ew", padx=int(10 * scale_factor), pady=(int(8 * scale_factor), int(10 * scale_factor)))
        btn_row.columnconfigure(0, weight=1)
        btn_row.columnconfigure(1, weight=1)
        self.toggle_btn = ttk.Button(btn_row, text="", command=self._toggle, style="Accent.TButton")
        self.toggle_btn.grid(row=0, column=0, sticky="ew", padx=(0, int(4 * scale_factor)))
        self.more_btn = ttk.Button(btn_row, text="More Details", command=self._show_details)
        self.more_btn.grid(row=0, column=1, sticky="ew", padx=(int(4 * scale_factor), 0))
        self._set_recovery_button_state(0)

    def _on_mousewheel(self, event):
        self.canvas.yview_scroll(int(-1*(event.delta/120)), "units")

    def _bind_detail_scroll(self, window, canvas, scrollable_frame, window_id):
        def sync_scrollregion(_event=None):
            canvas.configure(scrollregion=canvas.bbox("all"))

        def fit_frame_width(event):
            canvas.itemconfigure(window_id, width=event.width)

        def on_mousewheel(event):
            if getattr(event, "num", None) == 4:
                delta = -1
            elif getattr(event, "num", None) == 5:
                delta = 1
            else:
                delta = int(-1 * (event.delta / 120)) if getattr(event, "delta", 0) else 0
            if delta:
                canvas.yview_scroll(delta, "units")

        scrollable_frame.bind("<Configure>", sync_scrollregion)
        canvas.bind("<Configure>", fit_frame_width)

        for widget in (window, canvas, scrollable_frame):
            widget.bind("<MouseWheel>", on_mousewheel, add="+")
            widget.bind("<Button-4>", on_mousewheel, add="+")
            widget.bind("<Button-5>", on_mousewheel, add="+")

    def _show_details(self):
        # Create a dialog window for detailed information
        details_window = tk.Toplevel(self)
        details_window.title(f"Train {self.train_id} - Detailed Information")
        details_window.geometry("312x647")
        details_window.minsize(312, 647)
        details_window.resizable(True, True)
        
        # Bind close event to cleanup
        details_window.protocol("WM_DELETE_WINDOW", lambda: self._cleanup_details(details_window))
        
        # Create scrollable frame for details
        canvas = tk.Canvas(details_window, highlightthickness=0)
        scrollbar = tk.Scrollbar(
            details_window,
            orient="vertical",
            command=canvas.yview,
            width=16,
            cursor="hand2",
            activebackground="#9db6d8",
        )
        scrollable_frame = ttk.Frame(canvas)

        detail_window_id = canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        self._bind_detail_scroll(details_window, canvas, scrollable_frame, detail_window_id)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Add detailed information
        ttk.Label(scrollable_frame, text="Detailed Train Information", style="CardTitle.TLabel").pack(pady=10)
        
        # Metrics
        metrics_frame = ttk.LabelFrame(scrollable_frame, text="Speed Metrics")
        metrics_frame.pack(fill="x", padx=10, pady=5)
        self.detail_metric_vars = {
            "actual": tk.StringVar(value="Actual       : 0.0 km/h"),
            "permitted": tk.StringVar(value="Permitted    : 0.0 km/h"),
            "warning": tk.StringVar(value="Warning      : 0.0 km/h"),
            "intervention": tk.StringVar(value="Intervention : 0.0 km/h"),
        }
        for var in self.detail_metric_vars.values():
            ttk.Label(metrics_frame, textvariable=var).pack(anchor="w")

        # Train Location
        location_frame = ttk.LabelFrame(scrollable_frame, text="Train Location")
        location_frame.pack(fill="x", padx=10, pady=5)
        self.detail_location_var = tk.StringVar(value="")
        ttk.Label(location_frame, textvariable=self.detail_location_var, justify="left").pack(anchor="w")
        
        # Target & Planning
        target_frame = ttk.LabelFrame(scrollable_frame, text="Target & Planning")
        target_frame.pack(fill="x", padx=10, pady=5)
        self.detail_target_var = tk.StringVar(value="")
        self.detail_plan_var = tk.StringVar(value="")
        ttk.Label(target_frame, textvariable=self.detail_target_var, justify="left").pack(anchor="w")
        ttk.Separator(target_frame, orient="horizontal").pack(fill="x", pady=5)
        ttk.Label(target_frame, textvariable=self.detail_plan_var, justify="left").pack(anchor="w")
        
        # Operational Status
        status_frame = ttk.LabelFrame(scrollable_frame, text="Operational Status")
        status_frame.pack(fill="x", padx=10, pady=5)
        self.detail_state_var = tk.StringVar(value="")
        self.detail_alarm_var = tk.StringVar(value="")
        ttk.Label(status_frame, textvariable=self.detail_state_var, justify="left").pack(anchor="w")
        ttk.Separator(status_frame, orient="horizontal").pack(fill="x", pady=5)
        ttk.Label(status_frame, textvariable=self.detail_alarm_var, justify="left").pack(anchor="w")
        
        # Update the details with current data
        if hasattr(self, 'last_status'):
            self._update_details_from_status(self.last_status, "FRESH")
        elif hasattr(self, 'last_train'):
            self._update_details_window(self.last_train)

    def _cleanup_details(self, window):
        # Clean up attributes when details window is closed
        if hasattr(self, 'detail_metric_vars'):
            delattr(self, 'detail_metric_vars')
        if hasattr(self, 'detail_location_var'):
            delattr(self, 'detail_location_var')
        if hasattr(self, 'detail_target_var'):
            delattr(self, 'detail_target_var')
        if hasattr(self, 'detail_plan_var'):
            delattr(self, 'detail_plan_var')
        if hasattr(self, 'detail_state_var'):
            delattr(self, 'detail_state_var')
        if hasattr(self, 'detail_alarm_var'):
            delattr(self, 'detail_alarm_var')
        window.destroy()

    def _update_details_window(self, train: Train):
        actual_kmh = ms_to_kmh(train.speed)
        permitted_kmh = min(ms_to_kmh(train.curves["P"]), train.psr_kmh) if train.curves["P"] > 0.0 else train.psr_kmh
        warning_kmh = ms_to_kmh(train.curves["W"])
        sbi_kmh = ms_to_kmh(train.hidden_curves["SBI"])
        ebi_kmh = ms_to_kmh(train.hidden_curves["EBI"])
        intervention_kmh = ebi_kmh
        current_uncertainty_m = max(train.effective_position_uncertainty_m(), abs(train.pos_error_m))
        target_distance_m = train.distance_to_eoa if train.constraint_type == "STOP" else train.distance_to_constraint_m
        target_speed_kmh = 0.0 if train.constraint_type == "STOP" else train.constraint_target_speed_kmh
        current_tsr_kmh = min(train.psr_kmh, train.limit_ahead_speed_kmh) if train.limit_ahead_dist <= 0.0 else train.psr_kmh
        head_pos_m = float(train.pos)
        tail_pos_m = head_pos_m - float(train.length)
        safe_front_m = float(train.safe_front_end_pos)
        safe_rear_m = float(train.safe_rear_end_pos())

        self.detail_metric_vars["actual"].set(f"Actual       : {actual_kmh:.1f} km/h")
        self.detail_metric_vars["permitted"].set(f"Permitted    : {permitted_kmh:.1f} km/h")
        self.detail_metric_vars["warning"].set(f"Warning      : {warning_kmh:.1f} km/h")
        self.detail_metric_vars["intervention"].set(f"Intervention : {intervention_kmh:.1f} km/h")
        self.detail_location_var.set(
            "\n".join(
                [
                    f"Head / front  : {head_pos_m:,.1f} m",
                    f"Tail / rear   : {tail_pos_m:,.1f} m",
                    f"Train length  : {float(train.length):,.1f} m",
                    f"Safe front    : {safe_front_m:,.1f} m",
                    f"Safe rear     : {safe_rear_m:,.1f} m",
                    f"Reported pos  : {float(train.reported_pos):,.1f} m",
                ]
            )
        )

        self.detail_target_var.set(
            "\n".join(
                [
                    f"EOA / SvL      : {train.eoa:,.1f} m  /  {train.stop_target_pos:,.1f} m",
                    f"Distance target: {target_distance_m:,.1f} m",
                    f"Target speed   : {target_speed_kmh:.1f} km/h",
                    f"Constraint     : {train.constraint_type}  curve={train.curve_mode}",
                    f"ATO target     : {ms_to_kmh(train.ato_target_speed):.1f} km/h",
                    f"Door stop state: {train.precise_stop_state}",
                ]
            )
        )
        self.detail_plan_var.set(
            "\n".join(
                [
                    f"Current SSP    : {train.psr_kmh:.1f} km/h",
                    f"Next SSP / TSR : {train.limit_ahead_speed_kmh:.1f} km/h @ {train.limit_ahead_dist:,.0f} m",
                    f"Gradient       : {train.gradient:+.3f}  ({train.gradient * 1000.0:+.1f} permille)",
                    f"Balise / Beacon: last {train.last_balise_pos:,.0f} m  next {train.next_balise_pos:,.0f} m",
                    f"Odo CI         : +/-{current_uncertainty_m:.1f} m  rep={train.reported_pos:,.1f} m",
                    f"TSR active     : {current_tsr_kmh:.1f} km/h  release={'ON' if train.release_active else 'OFF'}",
                ]
            )
        )
        integrity_text = "Confirmed" if train.train_integrity_ok() else "Lost"
        controller_status = train.controller_status_payload()
        self.detail_state_var.set(
            "\n".join(
                [
                    f"Drive mode     : {train.drive_mode}  ({train.ato_state})  /  {train.atp_state}",
                    f"Mode source    : requested={train.requested_drive_mode}  {'AUTO DEGRADED' if train.dcs_degraded_requested else train.mode_transition_reason or 'NORMAL'}",
                    f"Brake channel  : {train.atp_brake}  cutoff={'YES' if train.traction_cutoff else 'NO'}",
                    f"Door interlock : {train.ato_door_mode}  authorized={'YES' if train.door_authorized else 'NO'}",
                    f"Hold brake     : {'ACTIVE' if train.ato_hold_active else 'FREE'}",
                    f"Station state  : {train.station_state}  line={train.station_lane if train.station_lane is not None else '--'}",
                    f"Train integrity: {integrity_text}",
                    f"Zero speed mon : {'ON' if train.zero_speed_detected else 'OFF'}  standstill={'ON' if train.standstill_monitoring else 'OFF'}",
                    f"DCS / ZC link   : {'WITHHELD (INTEGRITY)' if not train.train_integrity_ok() else 'VALID' if train.safe_packet_valid else 'TIMEOUT'}  age={train.safe_packet_age_s:.1f}s",
                    f"Active CC      : {controller_status['active_controller']}  standby={controller_status['standby_controller']}",
                    f"CC_A status    : {controller_status['cc_a_status']}  reason={controller_status['cc_a_fault_reason'] or '--'}",
                    f"CC_B status    : {controller_status['cc_b_status']}  reason={controller_status['cc_b_fault_reason'] or '--'}",
                    f"Switch counter : {controller_status['switch_counter']}  last={controller_status['last_switch_time']:.1f}s",
                ]
            )
        )
        self.detail_alarm_var.set(
            "\n".join(
                [
                    f"Alert          : {train.atp_alert}",
                    f"ATO prepare    : {'YES' if train.ato_brake_prepare else 'NO'}  jog={train.jog_state}",
                    f"SBI / EBI      : {'ARMED' if train.service_brake_latch else 'IDLE'}  /  {'ARMED' if train.emg_latch else 'IDLE'}",
                    f"DWELL          : {train.dwell_remaining_s:.1f}s",
                    f"Beacon align   : {'LOCKED' if train.beacon_position_locked else 'TRACK'}  seen={'YES' if train.stop_beacon_seen else 'NO'}",
                    f"Headway        : {'--' if train.headway_time_s is None else f'{train.headway_time_s:,.1f}s'}",
                    f"Scheduled stop : {'--' if train.active_scheduled_stop is None else train.active_scheduled_stop['name']}",
                ]
            )
        )

    def update_from_train(self, train: Train, append_history: bool = True):
        self.last_train = train
        actual_kmh = ms_to_kmh(train.speed)
        table_curves = getattr(train, "raw_curves", train.curves)
        table_hidden_curves = getattr(train, "raw_hidden_curves", train.hidden_curves)
        permitted_ms = table_curves.get("P", train.curves["P"])
        permitted_kmh = min(ms_to_kmh(permitted_ms), train.psr_kmh) if permitted_ms > 0.0 else train.psr_kmh
        warning_kmh = ms_to_kmh(table_curves.get("W", train.curves["W"]))
        sbi_kmh = ms_to_kmh(table_hidden_curves.get("SBI", train.hidden_curves["SBI"]))
        ebi_kmh = ms_to_kmh(table_hidden_curves.get("EBI", train.hidden_curves["EBI"]))
        current_uncertainty_m = max(train.effective_position_uncertainty_m(), abs(train.pos_error_m))
        target_distance_m = train.distance_to_eoa if train.constraint_type == "STOP" else train.distance_to_constraint_m
        target_speed_kmh = 0.0 if train.constraint_type == "STOP" else train.constraint_target_speed_kmh
        current_tsr_kmh = min(train.psr_kmh, train.limit_ahead_speed_kmh) if train.limit_ahead_dist <= 0.0 else train.psr_kmh

        # Update target distance and speed
        self.target_distance_var.set(f"{target_distance_m:,.0f} m")
        self.target_speed_var.set(f"{target_speed_kmh:.1f} km/h")

        # Update mode and status
        if train.drive_mode == "ATO" and train.atp_state not in ("ATP_EMERGENCY", "ATP_TRIP"):
            self.mode_var.set("A")
            self.mode_label.configure(foreground="#1f77b4")
        elif train.drive_mode == "CMD25":
            self.mode_var.set("25")
            self.mode_label.configure(foreground="#d19c1d")
        else:
            self.mode_var.set("M")
            self.mode_label.configure(foreground="#ffc107")

        # Update badge
        alert_text = f"ATP {train.atp_alert}"
        badge_bg = "#d7f7dc"
        badge_fg = "#16351f"
        if not train.train_integrity_ok():
            alert_text = "CONSIST BREAK"
            badge_bg = "#7a1028"
            badge_fg = "white"
        elif train.atp_action == "OFF":
            badge_bg = "#ffefc4"
            badge_fg = "#704d00"
        elif train.atp_action == "WARN":
            badge_bg = "#ffd9d2"
            badge_fg = "#7f2319"
        elif train.atp_action == "SBI":
            badge_bg = "#d7f5de"
            badge_fg = "#12552a"
        elif train.atp_action == "EBI":
            badge_bg = "#ffd0d0"
            badge_fg = "#7f0d0d"
        self.badge_var.set(alert_text)
        self.badge_label.configure(bg=badge_bg, fg=badge_fg)

        # Update door status
        if train.door_authorized or train.ato_door_mode == "ENABLE":
            door_text = "Enabled"
        elif train.ato_door_mode == "READY":
            door_text = "Ready"
        else:
            door_text = "Closed"
        self.door_var.set(f"Door: {door_text}")

        # Update docking status
        if train.door_authorized:
            docking_text = "Door Ready"
        elif train.precise_stop_state == "JOG":
            docking_text = "Nhich"
        elif train.precise_stop_state == "JOG_FAILED":
            docking_text = "Jog Locked"
        elif train.precise_stop_state == "ALIGNED":
            docking_text = "Aligned"
        elif train.precise_stop_state == "WAIT_JOG":
            docking_text = "Need Nhich"
        else:
            docking_text = "Not Docked"
        self.docking_var.set(docking_text)

        # Update headway
        headway_text = f"{train.headway_time_s:.1f} s" if train.headway_time_s is not None else "--"
        self.headway_var.set(headway_text)
        controller_status = train.controller_status_payload()
        self.controller_var.set(f"Active CC: {controller_status['active_controller']}  switches={controller_status['switch_counter']}")
        self.cc_a_var.set(f"CC_A: {controller_status['cc_a_status']}")
        self.cc_b_var.set(f"CC_B: {controller_status['cc_b_status']}")

        # Update message area
        message = ""
        if not train.train_integrity_ok():
            message = "TRAIN INTEGRITY LOST: virtual consist line open"
        elif train.ato_fault_active:
            message = "ATO Fault - Manual Driving Required"
        elif train.ato_recovery_state == "READY":
            message = "ATO_READY: CONFIRM required"
        elif train.ato_recovery_state == "CONFIRMED":
            message = "ATO_READY confirmed: START required"
        elif train.atp_action in ["WARN", "SBI", "EBI"]:
            message = f"ATP {train.atp_action}: {train.atp_alert}"
        elif train.dwell_remaining_s > 0.0:
            message = f"DWELL: {train.dwell_remaining_s:.1f}s"
        self.message_var.set(message)

        # Hide release speed indicator
        self.release_label.grid_remove()

        # Jogging status remains an internal flag; keep the small label hidden for the new layout
        self.jog_label.grid_remove()

        # Update curves table
        self.curve_vars["Actual"].set(f"{actual_kmh:.1f} km/h")
        self.curve_vars["Permitted"].set(f"{permitted_kmh:.1f} km/h")
        self.curve_vars["Warning"].set(f"{warning_kmh:.1f} km/h")
        self.curve_vars["SBI"].set(f"{sbi_kmh:.1f} km/h")
        self.curve_vars["EBI"].set(f"{ebi_kmh:.1f} km/h")

        # Keep charts frozen while simulation time is paused.
        if append_history:
            self._push_history(train)
        self._draw_chart()
        if hasattr(self, 'detail_metric_vars'):
            self._update_details_window(train)

        # Update emergency button state
        if train.emergency_recovery_hold:
            self.emg_state = 2
        elif train.emergency_stop or train.emg_latch or train.trip_mode:
            self.emg_state = max(self.emg_state, 1)
        elif train.ato_recovery_state == "READY":
            self.emg_state = 3
        elif train.ato_recovery_state == "CONFIRMED":
            self.emg_state = 4
        elif self.emg_state != 2:
            self.emg_state = 0
        self._set_recovery_button_state(self.emg_state)

    def update_from_status(self, status: dict, freshness: str = "FRESH", append_history: bool = True):
        self.last_status = dict(status)
        actual_kmh = ms_to_kmh(float(status.get("speed_mps", 0.0)))
        speed_curves = dict(status.get("speed_curves_kmh", {}) or {})
        permitted_kmh = float(speed_curves.get("P", status.get("constraint_target_speed_kmh", 0.0)) or 0.0)
        warning_kmh = float(speed_curves.get("W", 0.0) or 0.0)
        sbi_kmh = float(speed_curves.get("SBI", 0.0) or 0.0)
        ebi_kmh = float(speed_curves.get("EBI", 0.0) or 0.0)
        constraint_type = str(status.get("constraint_type", "NONE"))
        if constraint_type == "STOP":
            target_distance_m = float(status.get("distance_to_eoa_m", 0.0) or 0.0)
            target_speed_kmh = 0.0
        else:
            target_distance_m = float(status.get("distance_to_constraint_m", status.get("distance_to_eoa_m", 0.0)) or 0.0)
            target_speed_kmh = permitted_kmh
        if math.isinf(target_distance_m):
            target_distance_text = "--"
        else:
            target_distance_text = f"{target_distance_m:,.0f} m"
        self.target_distance_var.set(target_distance_text)
        self.target_speed_var.set(f"{target_speed_kmh:.1f} km/h")

        mode = str(status.get("mode", "--"))
        if mode == "ATO":
            self.mode_var.set("A")
            self.mode_label.configure(foreground="#1f77b4")
        elif mode.startswith("CMD"):
            self.mode_var.set("M")
            self.mode_label.configure(foreground="#9a6b00")
        else:
            self.mode_var.set(mode[:1] if mode else "--")
            self.mode_label.configure(foreground="#5a2630")

        fault_flags = dict(status.get("fault_flags", {}) or {})
        atp_state = str(status.get("atp_state", "UNKNOWN"))
        active_controller = str(status.get("active_controller", "CC_A"))
        switch_counter = int(status.get("controller_switch_count", status.get("switch_counter", 0)) or 0)
        self.controller_var.set(f"Active CC: {active_controller}  switches={switch_counter}")
        self.cc_a_var.set(f"CC_A: {status.get('cc_a_status', 'HEALTHY')}")
        self.cc_b_var.set(f"CC_B: {status.get('cc_b_status', 'HEALTHY')}")
        if freshness != "FRESH":
            alert_text = f"ATS {freshness}"
            badge_bg = "#f0d68a"
            badge_fg = "#5a2630"
        elif fault_flags.get("INTEGRITY"):
            alert_text = "CONSIST BREAK"
            badge_bg = "#7a1028"
            badge_fg = "white"
        elif fault_flags.get("EMERGENCY") or atp_state in ("ATP_EMERGENCY", "ATP_TRIP"):
            alert_text = "EBI/TRIP"
            badge_bg = "#d84b3c"
            badge_fg = "white"
        elif any(fault_flags.values()):
            alert_text = "FAULT"
            badge_bg = "#f0a132"
            badge_fg = "#5a2630"
        else:
            alert_text = "ATS OK"
            badge_bg = "#74b65d"
            badge_fg = "white"
        self.badge_var.set(alert_text)
        self.badge_label.configure(bg=badge_bg, fg=badge_fg)

        door_state = str(status.get("door_state", "CLOSED")).replace("_", " ").title()
        self.door_var.set(f"Door: {door_state}")
        stop = status.get("active_scheduled_stop") or {}
        if status.get("departure_hold"):
            docking_text = "Departure Hold"
        elif stop:
            docking_text = str(stop.get("station_name", stop.get("name", "Scheduled Stop")))
        elif status.get("station_lane") is not None:
            docking_text = f"Lane {status.get('station_lane')}"
        else:
            docking_text = "Monitoring"
        self.docking_var.set(docking_text)
        self.headway_var.set("--")

        self.curve_vars["Actual"].set(f"{actual_kmh:.1f} km/h")
        self.curve_vars["Permitted"].set(f"{permitted_kmh:.1f} km/h")
        self.curve_vars["Warning"].set(f"{warning_kmh:.1f} km/h")
        self.curve_vars["SBI"].set(f"{sbi_kmh:.1f} km/h")
        self.curve_vars["EBI"].set(f"{ebi_kmh:.1f} km/h")

        if freshness != "FRESH":
            message = f"TRAIN_STATUS {freshness}: displaying last received packet"
        elif fault_flags.get("INTEGRITY"):
            message = "TRAIN_STATUS reports TRAIN INTEGRITY LOST / possible broken consist"
        elif fault_flags.get("ATO"):
            message = "ATO Fault - Manual Driving Required"
        elif str(status.get("ato_recovery_state", "NORMAL")) == "READY":
            message = "ATO_READY: CONFIRM required"
        elif str(status.get("ato_recovery_state", "NORMAL")) == "CONFIRMED":
            message = "ATO_READY confirmed: START required"
        elif fault_flags.get("DCS"):
            message = "TRAIN_STATUS reports DCS fault"
        elif atp_state in ("ATP_EMERGENCY", "ATP_TRIP"):
            message = f"ATP state from TRAIN_STATUS: {atp_state}"
        else:
            message = "TRAIN_STATUS supervision packet received"
        self.message_var.set(message)
        self.release_label.grid_remove()
        self.jog_label.grid_remove()

        if append_history:
            self._push_status_history(actual_kmh, speed_curves)
        self._draw_chart()
        if hasattr(self, "detail_metric_vars"):
            self._update_details_from_status(status, freshness)

        if atp_state == "ATP_RECOVERY_HOLD":
            self.emg_state = 2
        elif fault_flags.get("EMERGENCY") or atp_state in ("ATP_EMERGENCY", "ATP_TRIP"):
            self.emg_state = max(self.emg_state, 1)
        elif str(status.get("ato_recovery_state", "NORMAL")) == "READY":
            self.emg_state = 3
        elif str(status.get("ato_recovery_state", "NORMAL")) == "CONFIRMED":
            self.emg_state = 4
        elif self.emg_state != 2:
            self.emg_state = 0
        self._set_recovery_button_state(self.emg_state)

    def _update_details_from_status(self, status: dict, freshness: str):
        if not hasattr(self, "detail_metric_vars"):
            return
        pos_m = float(status.get("position_m", 0.0) or 0.0)
        speed_kmh = ms_to_kmh(float(status.get("speed_mps", 0.0) or 0.0))
        eoa_m = float(status.get("eoa_m", 0.0) or 0.0)
        target_distance_m = float(status.get("distance_to_eoa_m", 0.0) or 0.0)
        length_m = float(status.get("length_m", 0.0) or 0.0)
        tail_m = pos_m - length_m if length_m > 0.0 else None
        safe_front_m = status.get("safe_front_m")
        safe_rear_m = status.get("safe_rear_m")
        curves = dict(status.get("speed_curves_kmh", {}) or {})
        self.detail_metric_vars["actual"].set(f"Actual       : {speed_kmh:.1f} km/h")
        self.detail_metric_vars["permitted"].set(f"Permitted    : {float(curves.get('P', 0.0) or 0.0):.1f} km/h")
        self.detail_metric_vars["warning"].set(f"Warning      : {float(curves.get('W', 0.0) or 0.0):.1f} km/h")
        self.detail_metric_vars["intervention"].set(f"Intervention : {float(curves.get('EBI', 0.0) or 0.0):.1f} km/h")
        location_lines = [
            f"Head / front  : {pos_m:,.1f} m",
            f"Tail / rear   : {'--' if tail_m is None else f'{tail_m:,.1f} m'}",
            f"Train length  : {'--' if length_m <= 0.0 else f'{length_m:,.1f} m'}",
            f"Safe front    : {'--' if safe_front_m is None else f'{float(safe_front_m):,.1f} m'}",
            f"Safe rear     : {'--' if safe_rear_m is None else f'{float(safe_rear_m):,.1f} m'}",
        ]
        if hasattr(self, "detail_location_var"):
            self.detail_location_var.set("\n".join(location_lines))
        self.detail_target_var.set(
            "\n".join(
                [
                    f"EOA             : {eoa_m:,.1f} m",
                    f"Distance target : {target_distance_m:,.1f} m",
                    f"Constraint      : {status.get('constraint_type', '--')}",
                    f"ATO target      : {float(status.get('ato_target_speed_kmh', 0.0) or 0.0):.1f} km/h",
                    f"Freshness       : {freshness}",
                ]
            )
        )
        fault_flags = dict(status.get("fault_flags", {}) or {})
        controller_lines = [
            f"Active CC       : {status.get('active_controller', 'CC_A')}",
            f"Standby CC      : {status.get('standby_controller', 'CC_B')}",
            f"CC_A status     : {status.get('cc_a_status', 'HEALTHY')}",
            f"CC_B status     : {status.get('cc_b_status', 'HEALTHY')}",
            f"CC_A reason     : {status.get('cc_a_fault_reason', '') or '--'}",
            f"CC_B reason     : {status.get('cc_b_fault_reason', '') or '--'}",
            f"Last switch     : {float(status.get('last_switch_time', 0.0) or 0.0):.1f}s",
            f"Switch counter  : {int(status.get('controller_switch_count', status.get('switch_counter', 0)) or 0)}",
        ]
        self.detail_state_var.set(
            "\n".join(
                [
                    f"Door state      : {status.get('door_state', '--')}",
                    f"Station lane    : {status.get('station_lane', '--')}",
                    f"Departure hold  : {bool(status.get('departure_hold', False))}",
                    f"Direction       : {status.get('direction', '--')}",
                    f"Odo uncertainty : {float(status.get('odometry_uncertainty_m', 0.0) or 0.0):.2f} m",
                ]
                + controller_lines
            )
        )
        self.detail_alarm_var.set(
            "\n".join(
                [
                    f"Fault flags     : {fault_flags}",
                    f"ATP alert       : {status.get('atp_alert', '--')}",
                    f"Speed curves    : P={float(curves.get('P', 0.0)):.1f} W={float(curves.get('W', 0.0)):.1f} "
                    f"SBI={float(curves.get('SBI', 0.0)):.1f} EBI={float(curves.get('EBI', 0.0)):.1f}",
                    "Source          : ATS TRAIN_STATUS packet",
                ]
            )
        )

    def _push_status_history(self, actual_kmh: float, speed_curves: dict):
        self.hist_actual.append(actual_kmh)
        for key in self.hist_curves:
            self.hist_curves[key].append(float(speed_curves.get(key, 0.0) or 0.0))

    def _set_recovery_button_state(self, state: int):
        self.emg_state = state
        if state == 1:
            self.toggle_btn.config(text="Safe Confirmed")
            self.toggle_btn.grid(row=0, column=0, sticky="ew", padx=(0, int(4 * self.scale_factor)))
            self.more_btn.grid_configure(column=1, columnspan=1, padx=(int(4 * self.scale_factor), 0))
        elif state == 2:
            self.toggle_btn.config(text="Resume Train")
            self.toggle_btn.grid(row=0, column=0, sticky="ew", padx=(0, int(4 * self.scale_factor)))
            self.more_btn.grid_configure(column=1, columnspan=1, padx=(int(4 * self.scale_factor), 0))
        elif state == 3:
            self.toggle_btn.config(text="CONFIRM")
            self.toggle_btn.grid(row=0, column=0, sticky="ew", padx=(0, int(4 * self.scale_factor)))
            self.more_btn.grid_configure(column=1, columnspan=1, padx=(int(4 * self.scale_factor), 0))
        elif state == 4:
            self.toggle_btn.config(text="START")
            self.toggle_btn.grid(row=0, column=0, sticky="ew", padx=(0, int(4 * self.scale_factor)))
            self.more_btn.grid_configure(column=1, columnspan=1, padx=(int(4 * self.scale_factor), 0))
        else:
            self.toggle_btn.grid_remove()
            self.more_btn.grid_configure(column=0, columnspan=2, padx=0)

    def _push_history(self, train: Train):
        chart_curves = getattr(train, "raw_curves", train.curves)
        chart_hidden_curves = getattr(train, "raw_hidden_curves", train.hidden_curves)
        self.hist_actual.append(ms_to_kmh(train.speed))
        self.hist_curves["P"].append(ms_to_kmh(chart_curves.get("P", train.curves["P"])))
        self.hist_curves["I"].append(ms_to_kmh(chart_hidden_curves.get("I", train.hidden_curves["I"])))
        self.hist_curves["W"].append(ms_to_kmh(chart_curves.get("W", train.curves["W"])))
        self.hist_curves["SBI"].append(ms_to_kmh(chart_hidden_curves.get("SBI", train.hidden_curves["SBI"])))
        self.hist_curves["SBD"].append(ms_to_kmh(chart_curves.get("SBD", train.curves["SBD"])))
        self.hist_curves["EBI"].append(ms_to_kmh(chart_hidden_curves.get("EBI", train.hidden_curves["EBI"])))
        self.hist_curves["EBD"].append(ms_to_kmh(chart_curves.get("EBD", train.curves["EBD"])))

    def _draw_chart(self):
        canvas = self.chart_canvas
        w = max(320, int(canvas.winfo_width() or canvas["width"]))
        h = max(120, int(canvas.winfo_height() or canvas["height"]))
        canvas.delete("all")
        canvas.create_rectangle(1, 1, w - 1, h - 1, outline=APP_THEME["border"], fill=APP_THEME["card_alt"])

        if len(self.hist_actual) < 2:
            return

        max_v = max(max(self.hist_actual), 1.0)
        for key in self.hist_curves:
            max_v = max(max_v, max(self.hist_curves[key]) if self.hist_curves[key] else 1.0)
        max_v = max(20.0, math.ceil(max_v / 10.0) * 10.0)

        left = 34
        right = w - 10
        top = 62
        bottom = h - 22
        plot_w = max(1.0, right - left)
        plot_h = max(1.0, bottom - top)
        scale_y = plot_h / max_v

        def to_points(values):
            pts = []
            count = max(1, len(values) - 1)
            for i, v in enumerate(values):
                x = left + (i / count) * plot_w
                y = bottom - min(max_v, max(0.0, v)) * scale_y
                pts.extend([x, y])
            return pts

        # Title
        canvas.create_text(16, 18, anchor="w", text="Braking Curves & Speed", fill=APP_THEME["accent"], font=("Consolas", 11, "bold"))

        # Legend
        legend_x = 20
        legend_y = 30
        legend_items = [
            ("Actual", CURVE_COLORS["actual"]),
            ("P", CURVE_COLORS["P"]),
            ("W", CURVE_COLORS["W"]),
            ("I", CURVE_COLORS["I"]),
            ("SBI", ACTION_COLORS["SBI"]),
            ("SBD", CURVE_COLORS["SBD"]),
            ("EBI", ACTION_COLORS["EBI"]),
            ("EBD", CURVE_COLORS["EBD"]),
        ]
        for i, (label, color) in enumerate(legend_items):
            x = legend_x + (i % 4) * 78
            y = legend_y + (i // 4) * 18
            canvas.create_line(x, y, x + 16, y, fill=color, width=2)
            canvas.create_text(x + 20, y, anchor="w", text=label, fill=APP_THEME["text"], font=("Consolas", 8))

        canvas.create_line(left, top, left, bottom, fill=APP_THEME["canvas_grid"])
        canvas.create_line(left, bottom, right, bottom, fill=APP_THEME["canvas_grid"])
        tick_step = 10.0 if max_v <= 80.0 else 20.0
        tick = 0.0
        while tick <= max_v + 1e-6:
            y = bottom - tick * scale_y
            canvas.create_line(left, y, right, y, fill=APP_THEME["canvas_grid"], dash=(1, 3))
            canvas.create_text(left - 4, y, anchor="e", text=f"{tick:.0f}", fill=APP_THEME["muted"], font=("Consolas", 8))
            tick += tick_step
        canvas.create_text(left, top - 8, anchor="w", text="km/h", fill=APP_THEME["muted"], font=("Consolas", 8))

        # Draw curves
        canvas.create_line(*to_points(self.hist_curves["EBD"]), fill=CURVE_COLORS["EBD"], dash=(2, 2), width=2)
        canvas.create_line(*to_points(self.hist_curves["EBI"]), fill=ACTION_COLORS["EBI"], dash=(4, 2), width=2)
        canvas.create_line(*to_points(self.hist_curves["SBD"]), fill=CURVE_COLORS["SBD"], dash=(2, 2), width=2)
        canvas.create_line(*to_points(self.hist_curves["SBI"]), fill=ACTION_COLORS["SBI"], dash=(4, 2), width=2)
        canvas.create_line(*to_points(self.hist_curves["W"]), fill=CURVE_COLORS["W"], dash=(2, 4), width=2)
        canvas.create_line(*to_points(self.hist_curves["I"]), fill=CURVE_COLORS["I"], dash=(1, 3), width=2)
        canvas.create_line(*to_points(self.hist_curves["P"]), fill=CURVE_COLORS["P"], dash=(2, 2), width=2)
        canvas.create_line(*to_points(self.hist_actual), fill=CURVE_COLORS["actual"], width=3)

    def _toggle(self):
        if self.emg_state == 1:
            self.on_toggle(self.train_id, emergency=True)
        elif self.emg_state == 2:
            self.on_resume(self.train_id)
        elif self.emg_state in (3, 4):
            self.on_toggle(self.train_id, emergency=False)

    def _instant_stop(self):
        self.on_instant_stop(self.train_id)

    def _precise_jog(self):
        self.on_precise_jog(self.train_id)

    def set_track_range(self, track_max_m: float):
        self.track_max_m = track_max_m


