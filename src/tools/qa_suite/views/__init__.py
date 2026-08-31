"""Module chứa các view (tab giao diện) cho ứng dụng Streamlit của VTX QA Suite."""

from __future__ import annotations

from tools.qa_suite.views.tab_batch import render_tab_batch
from tools.qa_suite.views.tab_inspector import render_tab_inspector
from tools.qa_suite.views.tab_stress import render_tab_stress


__all__ = [
    "render_tab_batch",
    "render_tab_inspector",
    "render_tab_stress",
]
