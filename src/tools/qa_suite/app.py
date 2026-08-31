"""Ứng dụng Web tương tác VTX Path Planning QA Suite (Streamlit App).

Cung cấp 3 phân hệ kiểm thử:
1. 🔍 Visual Scenario Inspector: Khảo sát kịch bản đơn lẻ và trực quan hóa 2D.
2. 📊 Batch Regression Runner: Kiểm thử hồi quy hàng loạt và xuất báo cáo.
3. ⚡ NATS Stress Tester: Đo tải đồng thời và độ trễ NATS microservice.
"""

from __future__ import annotations

import streamlit as st

from tools.qa_suite.views.tab_batch import render_tab_batch
from tools.qa_suite.views.tab_inspector import render_tab_inspector
from tools.qa_suite.views.tab_stress import render_tab_stress


def main() -> None:
    """Hàm khởi chạy chính của Streamlit Web App."""
    st.set_page_config(
        page_title="VTX Path Planning QA Suite",
        page_icon="✈️",
        layout="wide",
        initial_sidebar_state="expanded",
    )

    st.title("✈️ VTX Path Planning — QA & Verification Suite")
    st.caption(
        "Bộ công cụ kiểm định chất lượng, khảo sát quỹ đạo 2D, "
        "kiểm thử hồi quy hàng loạt và đo tải NATS Microservice."
    )

    tab_inspector, tab_batch, tab_stress = st.tabs(
        [
            "🔍 Visual Scenario Inspector",
            "📊 Batch Regression Runner",
            "⚡ NATS Stress Tester",
        ]
    )

    with tab_inspector:
        render_tab_inspector()

    with tab_batch:
        render_tab_batch()

    with tab_stress:
        render_tab_stress()


if __name__ == "__main__":
    main()
