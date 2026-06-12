"""Public package exports for the CBTC/ATC simulator."""

import sys
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from GUI.main_gui import App
from GUI.panels.ats_overview_panel import ATSOverviewPanel
from GUI.panels.train_panel import TrainPanel
from GUI.panels.infrastructure_panel import InfrastructurePanel
from GUI.panels.engineering_panel import DataFlowPanel, EngineeringPanel, TimeDistancePanel
from GUI.panels.analytics_panel import AnalyticsPanel
from GUI.panels.control_panel import ControlPanel, SpeedLimitsPanel
from GUI.widgets.status_card import StatusCard
from GUI.widgets.curve_plot import CurvePlot
from GUI.widgets.table_view import TableView
from GUI.main_gui import Simulation, Train, ZoneController
from OPERATION.headway_manager import HeadwayDecision, HeadwayManager, HeadwayStats

__all__ = [
    "AnalyticsPanel",
    "App",
    "ATSOverviewPanel",
    "ControlPanel",
    "CurvePlot",
    "DataFlowPanel",
    "EngineeringPanel",
    "HeadwayDecision",
    "HeadwayManager",
    "HeadwayStats",
    "InfrastructurePanel",
    "Simulation",
    "SpeedLimitsPanel",
    "StatusCard",
    "TableView",
    "TimeDistancePanel",
    "Train",
    "TrainPanel",
    "ZoneController",
]
