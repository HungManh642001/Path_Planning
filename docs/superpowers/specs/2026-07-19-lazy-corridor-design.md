# Thiết kế: Lazy focal search + AI corridor (bound-preserving) — Phương án A

Ngày: 2026-07-19 · Trạng thái: thiết kế đã duyệt miệng, chờ review spec
Kế thừa: spec `2026-07-19-gnn-guidance-design.md` (GNN tangent-graph, đã có model
`ml_planner/models/graph_guidance.npz` train xong, Spearman ~0.9)

## 1. Bối cảnh & động cơ

Prototype GNN (và CNN trước đó) chứng minh: learned cost-to-go đặt ở khe
**secondary của focal search chỉ xếp lại thứ tự expand** — không giảm branching,
không né được collision check (profiling: `_check_collision` ≈ 70% thời gian
search) → thua hand-crafted về tốc độ dù thắng chất lượng. Kết luận đã ghi
trong `ml_planner/EVAL.md` + memory: muốn tốc độ phải **cắt công việc**, không
phải xếp hạng nó.

Ràng buộc mới do user chốt: **PHẢI giữ bound hình thức ε = 5%** (`cost ≤
(1+ε)·C*` so với thuật toán search hiện tại, cùng ngữ nghĩa bảo đảm với focal
Phase 1). Điều này loại phương án cắt cứng ngoài corridor (Phương án B — bị
loại). Thiết kế được chọn = **lazy evaluation + suspended frontier** (họ
LazySP / lazy weighted A*), với AI corridor làm bộ điều phối.

**Nguyên lý làm bound sống sót:** collision check / đoản trình chỉ có thể LOẠI
hoặc làm ĐẮT một cạnh, không bao giờ làm rẻ. Vậy một node "treo" (chưa check)
mang chi phí lạc quan `f_opt = g(parent) + cost_hình_học(parent→node) +
h_euclid(node)` là cận dưới hợp lệ của mọi đường đi qua nó. Gộp heap treo vào
`f_min` thì `g_nghiệm ≤ w·f_min ≤ (1+ε)·C*` giữ nguyên chứng minh — dù đa số
node treo không bao giờ bị check.

## 2. Mục tiêu & tiêu chí nghiệm thu

- **Tốc độ (quyết định):** trên hard held-out, `lazy+corridor` thắng
  hand-focal về wall-time. Báo cáo tách lớp: `lazy vs hand` (phần cơ chế
  lazy thuần) và `lazy+corridor vs lazy` (phần đóng góp riêng của AI).
- **Đúng đắn (tuyệt đối, không thương lượng):** 0 vi phạm `mission_cost ≤
  1.05 × base` trên toàn bộ benchmark; oracle `core/path_validation` pass
  100% nghiệm trả về. Corridor sai chỉ được phép gây CHẬM (activation), không
  bao giờ được đổi đường/bound.
- **Cổng dừng sớm:** nếu `lazy` thuần (corridor=None) không thắng hand-focal
  về wall-time trên hard → dừng, không xây lớp corridor.

## 3. Ngoài phạm vi

- Không đụng `core/` (như mọi phase trước).
- Không train model mới — tái dùng nguyên `graph_guidance.npz` + `graph.py`.
- Safezone biến thiên: chưa (kiến trúc corridor kế thừa crop/affine dùng chung
  nên sẵn sàng về sau).
- Secondary của FOCAL quay về **hand-crafted** trong mọi mode lazy (đã chứng
  minh nhanh nhất cho vai trò xếp hạng); GNN chỉ đứng ở corridor.

## 4. Kiến trúc

```
ml_planner/
  corridor.py     # V̂ GNN → lưới boolean 128²; Corridor.contains ~0.2µs/call
  lazy_focal.py   # LazyFocalKinodynamicAstar(FocalKinodynamicAstar)
  benchmark.py    # +2 cột: lazy (C thuần) và lcor (lazy+corridor) — đo tách lớp
  config.py       # +CORRIDOR_DELTA=0.15, +CORRIDOR_GRID_RES=128
```

### 4.1 `corridor.py`

- `build_corridor(preprocessed, graph_guidance, delta=CORRIDOR_DELTA,
  grid_res=CORRIDOR_GRID_RES) -> Corridor | None`.
- Affine = `raster.compute_crop(preprocessed, grid_res)` (chung convention với
  CNN — train inputs == inference inputs == corridor inputs).
- V̂ trên lưới: MỘT truy vấn cKDTree vector hóa (k=3, cell centers → node đồ
  thị GNN), `V̂(cell) = min(d_i + values_i)` (~10–30 ms cho 128²).
- `mask[cell] = (‖cell − O‖ + V̂(cell)) ≤ (1+δ)·Ĉ` với `Ĉ =
  graph_guidance.lookup(start_pos)`; ô chứa start và goal ÉP = True.
- `Corridor.contains(x, y)`: affine → chỉ số nguyên → index mảng boolean;
  ngoài crop → False (ngoài crop = treo, bound machinery bao phủ).
- Model thiếu / build lỗi → trả None (mode lazy thuần).

### 4.2 `lazy_focal.py` — `LazyFocalKinodynamicAstar(FocalKinodynamicAstar)`

**Bẫy check theo ngữ cảnh (không nhân bản code core):**

- `get_next_states(current)`: arm `self._lazy_ctx = current`, gọi `super()`,
  finally disarm.
- `_check_collision(p1, p2)` override: khi `_lazy_ctx` đang arm, `p1 ==
  ctx.waypoint`, corridor active và `not corridor.contains(p2)` → KHÔNG check;
  ghi `(f_opt, counter, ctx, p2)` vào `self._suspended` heap; trả False (core
  tự bỏ candidate). Mọi call khác → `super()._check_collision`.
  - `f_opt = g(ctx) + hypot + TURN_PENALTY·turn + h_euclid(p2)` — turn tính từ
    `ctx.heading`; đoản trình CHƯA kiểm (chỉ có thể loại → optimism hợp lệ).
  - Kiểm LOS của valve (`_check_collision(P, goal_wp)`) không bị bẫy vì goal
    luôn trong corridor (ép True). `smooth_path`/`_check_fixed_legs` chạy khi
    cờ đã hạ → check thật.

**`f_min` toàn cục + activation (sửa tối thiểu `focal_astar.py`):**

- Hook mới trong `FocalKinodynamicAstar`: `_frontier_f_min(open_top)` mặc
  định trả `open_top` — hành vi base KHÔNG đổi (test bound hiện có xác nhận).
  `LazyFocal` override → `min(open_top, susp_top)`.
- Search loop của LazyFocal (override `search()` với phần lõi kế thừa qua
  hook, phần thêm):
  1. *Chấp nhận goal*: khi `_goal_reached(current) != None`, chỉ trả về nếu
     `g(current) ≤ w·f_min_global + config.EPS`. Chưa đạt → activation loop:
     while `susp_top_f < g(current)/w`: pop node treo; tính lại
     heading/turn/`_doan_trinh(parent,…)` + `_check_collision` THẬT; hợp lệ →
     tạo State, cập nhật `g_scores`, đẩy OPEN (+FOCAL nếu lọt band); không →
     bỏ. Xét lại điều kiện; goal tạm bị từ chối → đẩy lại OPEN (lazy-deletion
     sẵn có xử lý).
  2. *OPEN+FOCAL cạn nhưng heap treo còn* → activation bắt buộc trước khi
     được phép kết luận "no path" (completeness).
- Bất biến (ghi docstring + test): mọi đường khả thi chưa bị loại có đại diện
  trong OPEN ∪ SUSPENDED với f ≤ chi phí thật → `f_min_global ≤ C*` →
  `g_nghiệm ≤ w·f_min_global ≤ (1+ε)·C*`.
- Định nghĩa CHÍNH THỨC ba mode:
  - `eager` (mặc định cũ) = FocalKinodynamicAstar nguyên trạng.
  - `LazyFocal(corridor=None)` = **lazy thuần (Phương án C)**: bẫy treo MỌI
    candidate chưa check (không phân biệt corridor), validate-on-demand theo
    đúng loop activation trên; đây là baseline cơ chế.
  - `LazyFocal(corridor=C)` = chỉ treo candidate NGOÀI corridor; candidate
    trong corridor check ngay lúc sinh (như eager) để expand không trễ.
- Plan-time API: `plan_trajectory_lazy(preprocessed, corridor=None,
  focal_eps=None)` mirror `plan_trajectory_focal` (secondary=hand-crafted cố
  định), fallback `_safe`-style về focal thường nếu lỗi bất ngờ.

### 4.3 Benchmark (đo tách lớp)

- `compare_one` thêm 2 planner: `lazy` (`LazyFocal`, corridor=None) và `lcor`
  (`LazyFocal` + corridor từ GNN). Cột `lazy_*`, `lcor_*` (success/iters/
  time/mission/flight/cost_ratio/bound_ok) + số collision check thật đã trả
  (`lazy_checks`, `lcor_checks`, `hand_checks`) — bằng chứng trực tiếp cơ chế.
- Verdict mới: PASS khi `lcor` thắng hand về wall-time trên hard VÀ 0 vi phạm
  bound toàn benchmark; báo cáo attribution `lazy vs hand` và `lcor vs lazy`.

## 5. Xử lý lỗi — bậc thang suy giảm

1. Model GNN thiếu/hỏng, `build_corridor` exception → corridor=None → lazy
   thuần (vẫn bound, thường vẫn nhanh hơn eager).
2. `plan_trajectory_lazy` lỗi bất ngờ → fallback `plan_trajectory_focal`
   (hand) — pattern `_safe` của benchmark.
3. Corridor sai bất kỳ mức nào → chỉ thêm activation (chậm), không bao giờ
   đổi đường/bound — do cấu trúc, không do may mắn.

## 6. Kiểm thử (`*_test.py`, committed)

- `corridor_test.py`: membership math trên case tay; start/goal ép True;
  model thiếu → None; ngoài crop → False; determinism.
- `lazy_focal_test.py`:
  1. *Tương đương*: corridor all-True → path + iterations GIỐNG HỆT focal
     eager trên scenarios chuẩn (2/4/12/13).
  2. *Bound lazy thuần*: tham số hóa `mission_cost ≤ 1.05×base` trên
     scenarios 4/12/13/16 với corridor=None (tái dùng pattern test Phase 1).
  3. **Money test**: corridor all-False (trừ ô start/goal) → vẫn trả đường
     hợp lệ, `≤ 1.05×base` (activation cứu toàn bộ) — bằng chứng thực nghiệm
     của bất biến bound.
  4. *Cơ chế thật*: đếm call `super()._check_collision` — lazy < eager trên
     scenario khó; completeness: map không có đường → lazy cũng kết luận
     no-path (sau khi heap treo cạn), không treo vô hạn.
- `benchmark_test.py` mở rộng: cột mới xuất hiện; fallback sạch khi thiếu
  model.

## 7. Rủi ro & giảm thiểu

| Rủi ro | Giảm thiểu |
|---|---|
| Lazy thuần không thắng hand (cơ chế không lợi trên đồ thị thưa này) | Cổng dừng sớm §2 — đo `lazy vs hand` TRƯỚC khi xây corridor |
| Bẫy `_check_collision` bắt nhầm call ngoài ngữ cảnh sinh candidate | Cờ ngữ cảnh + điều kiện `p1 == ctx.waypoint`; goal ép trong corridor; test tương đương all-True |
| Activation churn khi model sai nhiều (đuôi chậm hơn cả eager) | Bound vẫn giữ; benchmark đo đuôi; nếu tệ → tăng δ hoặc chỉ dùng lazy thuần |
| Trùng lặp một phần search loop trong LazyFocal.search | Hook `_frontier_f_min` + kế thừa tối đa; test tương đương pin hành vi |

## 8. Hướng tiếp theo (ngoài phạm vi)

Nếu PASS: gộp `lcor` thành mode mặc định của ml_planner; thêm safezone vào
corridor (kiến trúc crop dùng chung đã sẵn); cân nhắc corridor cho arc-hop
(`_sector_clear`) — hiện chỉ phủ Strategy A + fan qua `_check_collision`.
