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

*(Amended trong bước lập plan: hiện thực hóa TƯƠNG ĐƯƠNG nhưng đơn giản hơn
bản duyệt miệng — không cần heap treo riêng hay gate chấp nhận goal; mọi tính
chất đã duyệt (bound, ba mode, tiêu chí, money test) giữ nguyên.)*

**Nguyên lý hiện thực:** node "treo" = node LẠC QUAN nằm ngay TRONG OPEN với
cạnh-vào chưa check (`edge_validated=False`). Vì f lạc quan ≤ f thật, `f_min
= OPEN top` tự động là cận dưới hợp lệ — không phải gộp heap nào. Corridor
chỉ gate **cửa vào FOCAL**; "activation" chính là cơ chế refill sẵn có.

- **Bẫy defer (không nhân bản code core):** `get_next_states` arm
  `self._lazy_ctx = current` rồi gọi `super()`; override `_check_collision`:
  khi cờ arm, `p1 == ctx.waypoint` và `p2 != goal_wp` → KHÔNG check, ghi `p2`
  vào tập deferred của lượt arm, trả **True** (candidate được tạo lạc quan);
  sau `super()` trả về, đánh dấu `edge_validated=False` cho các successor có
  waypoint trong tập deferred. Chord tới goal KHÔNG BAO GIỜ defer (giữ đúng
  hành vi valve LOS ở cùng call-site và bảo đảm goal luôn đã-validate khi
  được chấp nhận). Đoản trình vẫn kiểm eager (đứng trước collision trong core
  loop). Arc-hop không dùng `_check_collision` (đã xác minh) → luôn eager.
- **Validate-on-pop (hook trong `FocalKinodynamicAstar`, base = no-op):**
  vòng chọn node từ FOCAL thêm điều kiện `self._validate_on_pop(cand)`; lazy
  override: nếu `edge_validated` False → chạy `_check_collision` THẬT trên
  `(parent.waypoint, waypoint)`; fail → xóa entry `g_scores` (cho phép
  tái khám phá lattice cell qua cạnh khác) và trả False (node bị bỏ, KHÔNG
  vào closed). Node không bao giờ expand khi cạnh-vào chưa validate.
- **Corridor gate cửa FOCAL (hook `_focal_admissible`, base = True):** refill
  + inline-push chỉ nhận state có `corridor.contains(waypoint)`. Node ngoài
  corridor ở lại OPEN — giữ `f_min` (bound) mà không bao giờ bị check/expand
  chừng nào band còn node trong corridor. Nhánh drain có **fallback admit-all**
  (`self._admit_all`): band không còn node trong corridor → refill bỏ qua
  filter — corridor sai chỉ gây chậm, không bao giờ starve/livelock/no-path
  giả (đây là "activation" ở dạng đơn giản nhất).
- Bất biến (docstring + test): mọi đường khả thi chưa bị loại có đại diện
  trong OPEN với f lạc quan ≤ chi phí thật → `f_min ≤ C*` → nghiệm được
  chấp nhận (luôn đã-validate) có `g ≤ w·f_min + EPS ≤ (1+ε)·C* + EPS`.
- Định nghĩa CHÍNH THỨC ba mode:
  - `eager` (mặc định cũ) = FocalKinodynamicAstar nguyên trạng (hook no-op —
    hành vi không đổi byte nào, suite hiện có pin điều này).
  - `LazyFocal(corridor=None)` = **lazy thuần (Phương án C)**: defer MỌI
    candidate, validate-on-pop, FOCAL admission không đổi (hand secondary vẫn
    điều khiển thứ tự — tách bạch "lazy" khỏi "corridor").
  - `LazyFocal(corridor=C)` = lazy thuần + gate cửa FOCAL theo corridor.
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
  1. *Tương đương*: (a) trên scenario KHÔNG chướng ngại (mọi cạnh hợp lệ,
     defer không đổi gì) lazy ≡ eager (path + iterations GIỐNG HỆT); (b)
     corridor all-True ≡ lazy thuần (corridor không đổi hành vi khi không
     loại gì). Trên map CÓ chướng ngại lazy không buộc giống hệt eager từng
     iteration (node lạc quan có thể hạ f_min → band khác nhẹ) — hợp đồng là
     BOUND, được nhóm test 2–3 pin.
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
