"""Module xuất báo cáo kiểm thử tự động (Report Generator).

Hỗ trợ xuất kết quả kiểm thử hàng loạt (BatchSummary) ra các định dạng:
1. JSON: Cấu trúc dữ liệu đầy đủ cho tích hợp CI/CD và phân tích tự động.
2. CSV: Bảng dữ liệu chuẩn cho bảng tính Excel / Google Sheets.
3. HTML: Báo cáo giao diện web hiện đại, độc lập (standalone), có CSS và lọc tương tác.
"""

from __future__ import annotations

import csv
import html
import json
from pathlib import Path
from typing import TYPE_CHECKING


if TYPE_CHECKING:
    from tools.qa_suite.core.batch_runner import BatchSummary


class ReportGenerator:
    """Bộ tạo và xuất báo cáo kiểm thử đa định dạng (JSON, CSV, HTML)."""

    @staticmethod
    def export_json(summary: BatchSummary, file_path: Path | str) -> None:
        """Xuất tổng kết kiểm thử ra file định dạng JSON có cấu trúc.

        Args:
            summary: Đối tượng BatchSummary chứa kết quả toàn bộ đợt test.
            file_path: Đường dẫn file đích (Path hoặc str).
        """
        path = Path(file_path)
        path.parent.mkdir(parents=True, exist_ok=True)

        data = {
            "total_tests": summary.total_tests,
            "success_count": summary.success_count,
            "fail_count": summary.fail_count,
            "success_rate": summary.success_rate,
            "oracle_violation_count": summary.oracle_violation_count,
            "wall_time_stats": summary.wall_time_stats,
            "path_length_stats": summary.path_length_stats,
            "timestamp": summary.timestamp,
            "results": [
                {
                    "scenario_name": r.scenario_name,
                    "status": r.status,
                    "is_success": r.is_success,
                    "waypoints": [
                        {"position": list(wp[0]), "heading_rad": float(wp[1])}
                        for wp in r.waypoints
                    ],
                    "path_length_m": r.path_length_m,
                    "wall_time_s": r.wall_time_s,
                    "applied_time_budget_s": r.applied_time_budget_s,
                    "iterations": r.iterations,
                    "oracle_verdict": {
                        "is_ok": r.oracle_verdict.is_ok,
                        "detail": r.oracle_verdict.detail,
                    },
                    "error_detail": r.error_detail,
                }
                for r in summary.results
            ],
        }

        path.write_text(
            json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8"
        )

    @staticmethod
    def export_csv(summary: BatchSummary, file_path: Path | str) -> None:
        """Xuất tổng kết kiểm thử ra file bảng CSV chuẩn.

        Cột gồm: Scenario, Status, Success, WallTime_s, PathLength_m, Iterations,
                 OracleValid, FailureReason.

        Args:
            summary: Đối tượng BatchSummary chứa kết quả toàn bộ đợt test.
            file_path: Đường dẫn file đích (Path hoặc str).
        """
        path = Path(file_path)
        path.parent.mkdir(parents=True, exist_ok=True)

        headers = [
            "Scenario",
            "Status",
            "Success",
            "WallTime_s",
            "PathLength_m",
            "Iterations",
            "OracleValid",
            "FailureReason",
        ]

        with open(path, mode="w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            for r in summary.results:
                writer.writerow(
                    [
                        r.scenario_name,
                        r.status,
                        r.is_success,
                        f"{r.wall_time_s:.4f}",
                        f"{r.path_length_m:.2f}",
                        r.iterations,
                        r.oracle_verdict.is_ok,
                        r.error_detail or "",
                    ]
                )

    @staticmethod
    def export_html(summary: BatchSummary, file_path: Path | str) -> None:
        """Xuất báo cáo web HTML hiện đại, trực quan, có bộ lọc và độc lập.

        Args:
            summary: Đối tượng BatchSummary chứa kết quả toàn bộ đợt test.
            file_path: Đường dẫn file đích (Path hoặc str).
        """
        path = Path(file_path)
        path.parent.mkdir(parents=True, exist_ok=True)

        success_rate_color = (
            "#10b981"
            if summary.success_rate >= 90.0
            else "#f59e0b"
            if summary.success_rate >= 70.0
            else "#ef4444"
        )
        oracle_color = "#10b981" if summary.oracle_violation_count == 0 else "#ef4444"

        rows_html: list[str] = []
        for idx, r in enumerate(summary.results, start=1):
            is_success_badge = (
                '<span class="badge badge-success">PASSED</span>'
                if r.is_success
                else '<span class="badge badge-danger">FAILED</span>'
            )
            if r.oracle_verdict.is_ok:
                oracle_badge = '<span class="badge badge-success">VALID</span>'
            else:
                escaped_detail = html.escape(r.oracle_verdict.detail)
                oracle_badge = (
                    '<span class="badge badge-danger" '
                    f'title="{escaped_detail}">VIOLATION</span>'
                )

            status_class = (
                "badge-success"
                if r.status == "OK"
                else "badge-warning"
                if r.status in ("NO_PATH", "START_LEG_BLOCKED", "GOAL_LEG_BLOCKED")
                else "badge-danger"
            )
            status_badge = (
                f'<span class="badge {status_class}">{html.escape(r.status)}</span>'
            )

            error_text = html.escape(r.error_detail or "-")
            row_class = "row-pass" if r.is_success else "row-fail"
            if r.is_success and not r.oracle_verdict.is_ok:
                row_class = "row-violation"

            rows_html.append(
                f'<tr class="{row_class}" '
                f'data-status="{html.escape(r.status)}" '
                f'data-success="{str(r.is_success).lower()}" '
                f'data-oracle="{str(r.oracle_verdict.is_ok).lower()}">\n'
                f'  <td class="text-center text-muted">{idx}</td>\n'
                f'  <td class="font-mono font-bold">'
                f"{html.escape(r.scenario_name)}</td>\n"
                f"  <td>{status_badge}</td>\n"
                f"  <td>{is_success_badge}</td>\n"
                f'  <td class="font-mono text-right">{r.wall_time_s:.4f}s</td>\n'
                f'  <td class="font-mono text-right">'
                f"{r.path_length_m:,.1f}m</td>\n"
                f'  <td class="font-mono text-right">{r.iterations:,}</td>\n'
                f"  <td>{oracle_badge}</td>\n"
                f'  <td class="text-sm text-detail">{error_text}</td>\n'
                f"</tr>\n"
            )

        table_rows = "".join(rows_html)

        mean_wall = summary.wall_time_stats.get("mean", 0.0)
        p95_wall = summary.wall_time_stats.get("p95", 0.0)
        mean_len = summary.path_length_stats.get("mean", 0.0)
        min_len = summary.path_length_stats.get("min", 0.0)
        max_len = summary.path_length_stats.get("max", 0.0)
        num_results = len(summary.results)

        html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>VTX Path Planning - Batch QA Regression Report</title>
  <style>
    :root {{
      --bg-body: #0f172a;
      --bg-card: #1e293b;
      --bg-card-hover: #334155;
      --border: #334155;
      --text-main: #f8fafc;
      --text-muted: #94a3b8;
      --primary: #3b82f6;
      --success: #10b981;
      --warning: #f59e0b;
      --danger: #ef4444;
    }}
    * {{ box-sizing: border-box; margin: 0; padding: 0; }}
    body {{
      background-color: var(--bg-body);
      color: var(--text-main);
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      line-height: 1.5;
      padding: 2rem 1rem;
    }}
    .container {{
      max-width: 1280px;
      margin: 0 auto;
    }}
    header {{
      margin-bottom: 2rem;
      border-bottom: 1px solid var(--border);
      padding-bottom: 1.5rem;
      display: flex;
      justify-content: space-between;
      align-items: flex-end;
      flex-wrap: wrap;
      gap: 1rem;
    }}
    h1 {{
      font-size: 1.75rem;
      font-weight: 700;
      color: #ffffff;
      letter-spacing: -0.025em;
    }}
    .subtitle {{
      color: var(--text-muted);
      font-size: 0.875rem;
      margin-top: 0.25rem;
    }}
    .timestamp {{
      font-family: ui-monospace, monospace;
      font-size: 0.85rem;
      color: var(--text-muted);
      background: var(--bg-card);
      padding: 0.35rem 0.75rem;
      border-radius: 6px;
      border: 1px solid var(--border);
    }}
    .metrics-grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
      gap: 1rem;
      margin-bottom: 2rem;
    }}
    .metric-card {{
      background: var(--bg-card);
      border: 1px solid var(--border);
      border-radius: 10px;
      padding: 1.25rem;
      box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.1);
    }}
    .metric-label {{
      font-size: 0.75rem;
      font-weight: 600;
      text-transform: uppercase;
      letter-spacing: 0.05em;
      color: var(--text-muted);
      margin-bottom: 0.5rem;
    }}
    .metric-value {{
      font-size: 1.75rem;
      font-weight: 700;
      font-family: ui-monospace, monospace;
    }}
    .metric-subtext {{
      font-size: 0.75rem;
      color: var(--text-muted);
      margin-top: 0.25rem;
    }}
    .filter-section {{
      background: var(--bg-card);
      border: 1px solid var(--border);
      border-radius: 10px;
      padding: 1rem 1.25rem;
      margin-bottom: 1.5rem;
      display: flex;
      flex-wrap: wrap;
      gap: 1rem;
      align-items: center;
      justify-content: space-between;
    }}
    .filter-controls {{
      display: flex;
      gap: 0.75rem;
      flex-wrap: wrap;
      flex: 1;
    }}
    input[type="text"], select {{
      background: var(--bg-body);
      border: 1px solid var(--border);
      color: var(--text-main);
      padding: 0.5rem 0.85rem;
      border-radius: 6px;
      font-size: 0.875rem;
      outline: none;
    }}
    input[type="text"]:focus, select:focus {{
      border-color: var(--primary);
    }}
    .table-container {{
      background: var(--bg-card);
      border: 1px solid var(--border);
      border-radius: 10px;
      overflow-x: auto;
      box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.1);
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      text-align: left;
      font-size: 0.875rem;
    }}
    th {{
      background: rgba(15, 23, 42, 0.75);
      color: var(--text-muted);
      font-weight: 600;
      text-transform: uppercase;
      font-size: 0.7rem;
      letter-spacing: 0.05em;
      padding: 0.75rem 1rem;
      border-bottom: 1px solid var(--border);
    }}
    td {{
      padding: 0.75rem 1rem;
      border-bottom: 1px solid var(--border);
      vertical-align: middle;
    }}
    tbody tr:hover {{
      background: var(--bg-card-hover);
    }}
    .badge {{
      display: inline-block;
      padding: 0.2rem 0.5rem;
      border-radius: 4px;
      font-size: 0.7rem;
      font-weight: 700;
      text-transform: uppercase;
      letter-spacing: 0.03em;
    }}
    .badge-success {{
      background: rgba(16, 185, 129, 0.15);
      color: #34d399;
      border: 1px solid rgba(16, 185, 129, 0.3);
    }}
    .badge-warning {{
      background: rgba(245, 158, 11, 0.15);
      color: #fbbf24;
      border: 1px solid rgba(245, 158, 11, 0.3);
    }}
    .badge-danger {{
      background: rgba(239, 68, 68, 0.15);
      color: #f87171;
      border: 1px solid rgba(239, 68, 68, 0.3);
    }}
    .font-mono {{ font-family: ui-monospace, monospace; }}
    .font-bold {{ font-weight: 600; }}
    .text-center {{ text-align: center; }}
    .text-right {{ text-align: right; }}
    .text-muted {{ color: var(--text-muted); }}
    .text-sm {{ font-size: 0.8rem; }}
    .text-detail {{ max-width: 280px; word-break: break-word; color: #cbd5e1; }}
    footer {{
      margin-top: 2rem;
      text-align: center;
      font-size: 0.8rem;
      color: var(--text-muted);
    }}
  </style>
</head>
<body>
  <div class="container">
    <header>
      <div>
        <h1>VTX Path Planning - Regression Report</h1>
        <div class="subtitle">Automated batch testing and path validation oracle</div>
      </div>
      <div class="timestamp">
        Run Time: {summary.timestamp}
      </div>
    </header>

    <div class="metrics-grid">
      <div class="metric-card">
        <div class="metric-label">Total Tests</div>
        <div class="metric-value">{summary.total_tests}</div>
        <div class="metric-subtext">Scenarios executed</div>
      </div>
      <div class="metric-card">
        <div class="metric-label">Success Rate</div>
        <div class="metric-value" style="color: {success_rate_color}">
          {summary.success_rate:.1f}%
        </div>
        <div class="metric-subtext">
          {summary.success_count} Passed / {summary.fail_count} Failed
        </div>
      </div>
      <div class="metric-card">
        <div class="metric-label">Oracle Violations</div>
        <div class="metric-value" style="color: {oracle_color}">
          {summary.oracle_violation_count}
        </div>
        <div class="metric-subtext">Kinodynamic / obstacle violations</div>
      </div>
      <div class="metric-card">
        <div class="metric-label">Mean Wall Time</div>
        <div class="metric-value">{mean_wall:.3f}s</div>
        <div class="metric-subtext">P95: {p95_wall:.3f}s</div>
      </div>
      <div class="metric-card">
        <div class="metric-label">Mean Path Length</div>
        <div class="metric-value">{mean_len:,.0f}m</div>
        <div class="metric-subtext">
          Min: {min_len:,.0f}m | Max: {max_len:,.0f}m
        </div>
      </div>
    </div>

    <div class="filter-section">
      <div class="filter-controls">
        <input type="text" id="searchInput"
               placeholder="Search scenario..." onkeyup="filterTable()">
        <select id="statusFilter" onchange="filterTable()">
          <option value="all">All Statuses</option>
          <option value="success">Passed Only</option>
          <option value="failed">Failed Only</option>
          <option value="violation">Oracle Violations</option>
        </select>
      </div>
      <div id="counter" class="text-sm text-muted">
        Showing {num_results} of {num_results} results
      </div>
    </div>

    <div class="table-container">
      <table id="resultsTable">
        <thead>
          <tr>
            <th class="text-center">#</th>
            <th>Scenario Name</th>
            <th>Status</th>
            <th>Result</th>
            <th class="text-right">Wall Time</th>
            <th class="text-right">Path Length</th>
            <th class="text-right">Iterations</th>
            <th>Oracle Verdict</th>
            <th>Details</th>
          </tr>
        </thead>
        <tbody>
          {table_rows}
        </tbody>
      </table>
    </div>

    <footer>
      Generated by VTX QA Suite &bull; High-Performance Path Planning Toolchain
    </footer>
  </div>

  <script>
    function filterTable() {{
      const searchVal = document.getElementById('searchInput').value.toLowerCase();
      const statusFilter = document.getElementById('statusFilter').value;
      const table = document.getElementById('resultsTable');
      const rows = table.getElementsByTagName('tbody')[0].getElementsByTagName('tr');
      let visibleCount = 0;

      for (let i = 0; i < rows.length; i++) {{
        const row = rows[i];
        const text = row.textContent.toLowerCase();
        const isSuccess = row.getAttribute('data-success') === 'true';
        const isOracleValid = row.getAttribute('data-oracle') === 'true';

        let matchesSearch = text.includes(searchVal);
        let matchesStatus = true;

        if (statusFilter === 'success') {{
          matchesStatus = isSuccess;
        }} else if (statusFilter === 'failed') {{
          matchesStatus = !isSuccess;
        }} else if (statusFilter === 'violation') {{
          matchesStatus = (isSuccess && !isOracleValid);
        }}

        if (matchesSearch && matchesStatus) {{
          row.style.display = '';
          visibleCount++;
        }} else {{
          row.style.display = 'none';
        }}
      }}

      const cnt = document.getElementById('counter');
      cnt.innerText = `Showing ${{visibleCount}} of ${{rows.length}} results`;
    }}
  </script>
</body>
</html>
"""
        path.write_text(html_content, encoding="utf-8")
