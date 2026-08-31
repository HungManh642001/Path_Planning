# Thiết kế: Chuẩn hóa Import Alias, Trả về Full Path trong plan_trajectory, và Tính chiều dài Quỹ đạo theo Cung lượn Dubins

**Ngày:** 31-08-2026  
**Nhánh:** `refactor/full-path-dubins-length-and-import-cleanup`  
**Mục tiêu:**
1. Chuẩn hóa triệt để cách import module, loại bỏ các alias viết tắt tối nghĩa cũ (`su`, `ag`, `pv`, `tr`, `mg`, `prep`) theo tiêu chuẩn kỹ thuật `docs/coding_standards_extracted.txt`.
2. Cập nhật `plan_trajectory` (và `KinodynamicAstar.plan`) trả về đường bay đầy đủ (Full Path bao gồm cả điểm cất cánh $O$ và điểm đích $T$).
3. Nghiên cứu, chuẩn hóa và áp dụng công thức tính chiều dài đường bay thực tế theo cung lượn tròn Dubins (Fillet Dubins Arc).

---

## 1. Yêu cầu 1: Chuẩn hóa Import Alias

### 1.1 Hiện trạng
Các file mã nguồn còn sử dụng các alias viết tắt 2 ký tự kế thừa từ tên module cũ:
- `spatial as su` (từ `spatial_utils`)
- `arc as ag` (từ `arc_geometry`)
- `oracle as pv` (từ `path_validation`)
- `sampling as tr` (từ `trajectory`)
- `generator as mg` (từ `map_generator`)
- `preprocessing as prep`

### 1.2 Giải pháp
- Thay thế toàn bộ bằng import tường minh:
  ```python
  from path_planning.geometry import arc, spatial
  from path_planning.render import sampling, visualizer
  from path_planning.scenario import generator, preprocessing, presets
  from path_planning.validation import oracle
  ```
- Gọi trực tiếp tên module: `spatial.distance(...)`, `oracle.path_is_valid(...)`, v.v.

---

## 2. Yêu cầu 2: Full Path trong `plan_trajectory`

### 2.1 Hiện trạng
- `KinodynamicAstar.plan()` và `plan_trajectory()` trả về `result["path"]` chỉ gồm các điểm nội suy trung gian (interior path), bỏ qua điểm cất cánh $O$ và điểm đích $T$.
- Các caller (service, render, tests) phải gọi thêm `build_full_path(result["path"], prep)`.

### 2.2 Giải pháp
- `KinodynamicAstar.plan()` gọi `full = full_mission_path(path, self.scenario)` và đóng gói `full` vào `result["path"]`.
- `result["path"][0]` là `(start_pos, start_heading)`.
- `result["path"][-1]` là `(goal_pos, final_heading)`.
- Cập nhật `src/service/vtx_service/planner.py`, `src/path_planning/render/`, `tests/` để sử dụng trực tiếp `result["path"]`.

---

## 3. Yêu cầu 3: Tính chiều dài Quỹ đạo theo Cung lượn Dubins

### 3.1 Cơ sở toán học
Đường bay thực tế của máy bay là đường cong liên tục gồm các đoạn thẳng nối với các cung lượn tròn bán kính $R$ tại mỗi góc rẽ $W_i$:
- Góc lệch hướng tại $W_i$: $\alpha_i = \text{angle\_diff}(h_{\text{out}}, h_{\text{in}})$.
- Khoảng cách tiếp tuyến: $t_i = R \cdot \tan\left(\frac{|\alpha_i|}{2}\right)$.
- Chiều dài cung tròn thay thế: $L_{\text{arc}, i} = R \cdot |\alpha_i|$.
- Độ rút ngắn so với đa giác đỉnh: $\Delta L_i = 2 t_i - L_{\text{arc}, i} = 2 R \tan\left(\frac{|\alpha_i|}{2}\right) - R |\alpha_i| \ge 0$.

### 3.2 Công thức tổng quát
$$L_{\text{actual}} = \sum_{i=0}^{n-1} \|W_{i+1} - W_i\|_2 - \sum_{i=1}^{n-1} \left( 2 R \tan\left(\frac{|\alpha_i|}{2}\right) - R |\alpha_i| \right)$$

### 3.3 Hàm triển khai
Thêm vào `src/path_planning/geometry/spatial.py` (hoặc `src/path_planning/trajectory/mission_path.py`):
```python
def calculate_dubins_path_length(
    path: Sequence[PlannerState], turn_radius: float
) -> float:
    """Tính tổng chiều dài quỹ đạo bay thực tế gồm các đoạn thẳng và cung lượn Dubins."""
```
và hàm phụ trợ:
```python
def calculate_polyline_length(path: Sequence[PlannerState]) -> float:
    """Tính tổng chiều dài đoạn thẳng đa giác qua các waypoints."""
```
Cập nhật `_planar_length` trong `src/service/vtx_service/planner.py` sang `calculate_dubins_path_length`.
