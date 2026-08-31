# Thiết Kế Kiến Trúc Hệ Thống Kiểm Thử Chuẩn Mực (Test Architecture Specification)

## 1. Tổng quan & Mục tiêu

Tài liệu này quy định kiến trúc và tiêu chuẩn thiết kế cho toàn bộ hệ thống kiểm thử tự động (`tests/`) của dự án, tuân thủ nghiêm ngặt theo **Tài liệu quy chuẩn kỹ thuật lập trình Python** (`docs/coding_standards_extracted.txt` - Mục 11: *Kiểm thử & Testing Standards*).

Hệ thống kiểm thử mới được thiết kế nhằm đáp ứng 3 mục tiêu cốt lõi:
1. **Kiến trúc mở rộng đa thuật toán (Multi-Algorithm Extensible Architecture)**: Hỗ trợ linh hoạt việc bổ sung nhiều thuật toán độc lập trong tương lai (`path_planning`, `task_allocation`, `formation_control`...) được điều phối tập trung qua tầng `service`.
2. **Phân tầng kiểm thử chuẩn mực (Agile Test Pyramid)**: Phân tách rõ ràng giữa Unit Tests (cô lập logic nghiệp vụ), Integration Tests (luồng liên module) và Service Tests (giao tiếp & vận hành dịch vụ).
3. **Tính ổn định & quyết định tuyệt đối (100% Deterministic, Zero Flakiness)**: Loại bỏ hoàn toàn các test `xfail` lỗi thời, loại bỏ phụ thuộc ngẫu nhiên không kiểm soát, áp dụng mẫu AAA (Arrange - Act - Assert) và chuẩn định danh hàm test thống nhất.

---

## 2. Cấu trúc thư mục kiểm thử phân tầng (Test Directory Layout)

```text
tests/
├── conftest.py                                  # Fixtures toàn cục (môi trường test, logging, seed chung)
│
├── path_planning/                               # === 1. TOÀN BỘ KIỂM THỬ THUẬT TOÁN PATH PLANNING ===
│   ├── conftest.py                              # Fixtures riêng của Path Planning (obstacles, map, points)
│   ├── unit/                                    # Unit tests cô lập từng submodule của path_planning
│   │   ├── geometry/
│   │   │   ├── test_spatial.py                  # Toán học 2D: khoảng cách, góc, tiếp điểm
│   │   │   ├── test_arc.py                      # Hình học cung tròn, bám biên, bitangent
│   │   │   └── test_goal_shot.py                # Nghiệm giải tích 2 góc rẽ về đích
│   │   ├── collision/
│   │   │   └── test_detector.py                 # Kiểm tra va chạm đoạn thẳng, cung lượn, safezone
│   │   ├── search/
│   │   │   ├── test_state.py                    # Trạng thái ô lưới, hash, so sánh
│   │   │   ├── test_heuristic.py                # Tính khả nạp và nhất quán của heuristic
│   │   │   ├── test_successors.py               # Sinh láng giềng: Strategy A, B, pivot slide
│   │   │   └── test_astar.py                    # Động cơ A* và kiểm soát time budget
│   │   ├── trajectory/
│   │   │   ├── test_mission_path.py             # Ghép nối đường bay O -> W -> T
│   │   │   └── test_smoothing.py                # Thuật toán làm mượt DP chính xác
│   │   ├── scenario/
│   │   │   ├── test_preprocessing.py            # Giãn nở vật cản, tính W_1, W_{n-1}
│   │   │   ├── test_generator.py                # Sinh ngẫu nhiên đảo, vật cản tròn
│   │   │   └── test_presets.py                  # Kiểm tra 16 kịch bản chuẩn benchmark
│   │   ├── validation/
│   │   │   └── test_oracle.py                   # Kiểm định độc lập 4 tiêu chuẩn an toàn
│   │   └── render/
│   │       ├── test_sampling.py                 # Lấy mẫu đường bay straight & dubins
│   │       └── test_visualizer.py               # Vẽ đồ thị và bounding box
│   └── integration/                             # Integration tests riêng cho pipeline path_planning
│       ├── test_planner_pipeline.py             # Chạy end-to-end fixed-goal & free-goal
│       ├── test_preset_benchmarks.py            # Chạy toàn bộ 16 benchmark scenarios qua Oracle
│       └── test_time_budget.py                  # Kiểm tra giới hạn thời gian & deadline
│
├── <future_algorithm>/                          # === 2. THUẬT TOÁN TƯƠNG LAI (task_allocation, formation...) ===
│   ├── conftest.py                              # Fixtures riêng của thuật toán mới
│   ├── unit/                                    # Unit tests nội bộ thuật toán mới
│   └── integration/                             # Integration tests nội bộ thuật toán mới
│
└── service/                                     # === 3. KIỂM THỬ TẦNG DỊCH VỤ & ĐIỀU PHỐI (SERVICE LAYER) ===
    ├── conftest.py                              # Fixtures dịch vụ: Mock transport, Test messages, Payload
    ├── unit/                                    # Unit tests cho các thành phần hạ tầng dịch vụ
    │   ├── test_angles.py                       # Chuyển đổi góc / tọa độ tầng service
    │   ├── test_messages.py                     # Schema serialization/deserialization
    │   ├── test_map_file.py                     # Đọc/ghi và parse file bản đồ
    │   ├── test_runtime.py                      # Phát hiện cấu hình, version git, hash snapshot
    │   └── test_scenario_builder.py             # Khởi tạo kịch bản từ payload dịch vụ
    └── integration/                             # Integration tests: Gọi các thuật toán qua Service
        ├── test_boundary.py                     # Kiểm tra ranh giới kiến trúc và đóng gói module
        ├── test_equivalence.py                  # Kiểm tra tính tương đương kết quả giữa Service và Core
        ├── test_planner_service.py              # Kiểm thử gọi Path Planning thông qua Service
        ├── test_runner.py                       # Khởi chạy worker service và xử lý tiến trình
        └── test_transport.py                    # Giao tiếp truyền nhận dữ liệu DDS / Network
```

---

## 3. Ma trận phân công & Tiêu chuẩn bao phủ kiểm thử

| Nhóm Kiểm Thử | Tệp Test | Đối Tượng Kiểm Thử Cốt Lõi |
| :--- | :--- | :--- |
| **Geometry** | `tests/path_planning/unit/geometry/test_spatial.py` | `distance`, `angle_to_heading`, `angle_diff`, `point_to_line_distance`, `inflate_polygon`, `circle_tangent_points` |
| | `tests/path_planning/unit/geometry/test_arc.py` | `riding_sense`, `tangent_heading`, `arc_angle`, `arc_waypoints`, `departure_point`, `bitangent_departures`, `is_point_on_circle_boundary`, `is_point_on_any_circle_boundary`, `sector_polygon`, `has_angular_overlap` |
| | `tests/path_planning/unit/geometry/test_goal_shot.py` | `two_corner_candidates`, `TwoCornerCandidate` giải tích |
| **Collision** | `tests/path_planning/unit/collision/test_detector.py` | `is_collision_free`, `is_corner_arc_clear`, `is_sector_clear`, `on_circle_boundary`, `is_in_bounds`, `ray_chord_clear` |
| **Search** | `tests/path_planning/unit/search/test_state.py` | `State`, `state_to_tuple`, băm ô lưới, so sánh bằng nhau |
| | `tests/path_planning/unit/search/test_heuristic.py` | `euclidean_heuristic` (admissible & consistent) |
| | `tests/path_planning/unit/search/test_successors.py` | `seed_start_corners`, `get_next_states`, `pivot_candidate`, `slide_pivot`, `try_goal_shot` |
| | `tests/path_planning/unit/search/test_astar.py` | `AstarSearchEngine`, priority queue loop, `is_goal_reached`, `reconstruct_path`, `time_budget_s` |
| **Trajectory** | `tests/path_planning/unit/trajectory/test_mission_path.py` | `full_mission_path` (thêm O, T, xử lý free-goal, chống trùng lặp điểm mút) |
| | `tests/path_planning/unit/trajectory/test_smoothing.py` | `smooth_path` DP shortcuts, kiểm tra góc quay và đoản trình |
| **Scenario** | `tests/path_planning/unit/scenario/test_preprocessing.py` | `prepare_scenario`, `inflate_obstacles`, `calculate_start_state`, `calculate_end_state` |
| | `tests/path_planning/unit/scenario/test_generator.py` | `generate_random_islands`, `generate_dynamic_obstacles`, `create_scenario`, tính quyết định seed |
| | `tests/path_planning/unit/scenario/test_presets.py` | `get_all_scenarios`, tính hợp lệ của 16 benchmark presets |
| **Validation** | `tests/path_planning/unit/validation/test_oracle.py` | `ValidationResult.ok()`, `segments_clear`, `turn_angles_ok`, `straight_segments_ok`, `arcs_clear`, `path_is_valid` |
| **Render** | `tests/path_planning/unit/render/test_sampling.py` | `sample_trajectory` (straight/dubins), `turn_markers`, `build_full_path` |
| | `tests/path_planning/unit/render/test_visualizer.py` | `plot_scenario`, `_content_extents`, `_plot_extents` |
| **Pipeline Integration** | `tests/path_planning/integration/test_planner_pipeline.py` | `plan_trajectory` end-to-end cho cả fixed-goal và free-goal |
| | `tests/path_planning/integration/test_preset_benchmarks.py` | Chạy toàn bộ 16 preset scenarios, xác thực bằng Oracle |
| | `tests/path_planning/integration/test_time_budget.py` | Kiểm tra cutoff thời gian, `resolve_time_budget_s`, fallback budget |
| **Service Gateway** | `tests/service/unit/` & `tests/service/integration/` | Đóng gói thông điệp, DDS transport, runner worker, map file, runtime detection, tính tương đương kết quả |

---

## 4. Quy chuẩn kỹ thuật triển khai mã nguồn Test

### 4.1. Quy ước đặt tên (Naming Convention)
Tên hàm test bắt buộc tuân theo cấu trúc:
`test_<tên_chức_năng>_<điều_kiện_đầu_vào>_<kết_quả_mong_đợi>`
* Đảm bảo đọc tên hàm là hiểu ngay kịch bản kiểm thử mà không cần đọc nội dung code.

### 4.2. Mẫu thiết kế AAA (Arrange - Act - Assert)
Bên trong mỗi hàm test phải phân tách rõ ràng 3 khối lệnh bằng comment:
```python
def test_distance_with_identical_points_returns_zero() -> None:
    """Kiểm tra khoảng cách giữa hai điểm trùng nhau bằng 0."""
    # Arrange (Chuẩn bị dữ liệu)
    point = (100.0, 200.0)

    # Act (Thực thi)
    result = distance(point, point)

    # Assert (Kiểm chứng)
    assert result == 0.0
```

### 4.3. Type Annotations & Google Style Docstrings
* Tất cả các hàm test đều có chú thích kiểu trả về: `def test_...(...) -> None:`.
* Docstring tiếng Việt ngắn gọn, súc tích giải thích mục đích kiểm tra.

### 4.4. Tính quyết định (Determinism) & Ranh giới Mocking
* Cố định `seed` cho mọi thuật toán sinh ngẫu nhiên.
* Không sử dụng thời gian thực hoặc trạng thái phụ thuộc môi trường.
* Chỉ mock các ranh giới ngoại vi (network DDS, socket, file system ở tầng service). Tuyệt đối không mock domain logic nội bộ của các thuật toán.

---

## 5. Tiêu chuẩn nghiệm thu (Acceptance Criteria)

1. Toàn bộ test suite mới chạy thành công 100%: **300+ passed, 0 failed, 0 errors, 0 xfailed**.
2. Kiểm tra linter và định dạng mã nguồn: `ruff check` và `ruff format` đạt 0 lỗi/cảnh báo.
3. Kiểm tra kiểu dữ liệu tĩnh: `pyright` đạt 0 lỗi.
4. Xóa bỏ hoàn toàn cấu trúc cũ `tests/core/` sau khi đã chuyển đổi toàn diện.
