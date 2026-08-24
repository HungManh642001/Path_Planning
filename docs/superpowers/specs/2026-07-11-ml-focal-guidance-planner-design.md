# Thiết kế: Learned-heuristic Focal Search cho Kinodynamic A* (`ml_planner`)

- **Ngày:** 2026-07-11
- **Trạng thái:** Design (chờ duyệt để chuyển sang implementation plan)
- **Phạm vi:** Thêm một biến thể planner tăng tốc bằng AI, **cô lập hoàn toàn** trong folder mới `ml_planner/`, **không sửa** `core/`, `config.py`, `tests/` hiện tại.

---

## 1. Bối cảnh & mục tiêu

Planner hiện tại (`core/kinodynamic_astar.py`) là A* trên đồ thị hình học động (tangent/bitangent + arc-hop) với heuristic **Euclid thuần** (`heuristic()`, dòng 224). Euclid admissible ⇒ A* cho đường **tối ưu**, nhưng rất lỏng khi phải đi vòng qua vật cản hoặc khi heading ban đầu bất lợi ⇒ nở ra nhiều state, phải vá bằng các cơ chế thủ công (Strategy B, re-arm).

**Mục tiêu:**
1. **Giảm thời gian chạy** thuật toán.
2. **Đưa AI/ML** vào planner một cách thực chất.
3. Giữ chất lượng đường: đường trả về **≤ (1+ε)× tối ưu** với **ε = 5%**, là **trần được chứng minh** (không phải trung bình).

**Quyết định nền tảng (từ brainstorming):**
- Bỏ hướng "learned heuristic thuần trong A* thường": mạng hồi quy không thể chứng minh admissible ⇒ phá tối ưu tuyệt đối.
- Chọn **B2 — Focal search (A\*ε)**: learned heuristic đóng vai **heuristic thứ hai (secondary)** trong FOCAL; một cận **admissible (Euclid)** giữ bound ε. Mô hình sai cỡ nào cũng **không phá bound ε, không phá an toàn**.
- Mô hình học là **CNN guidance-map kiểu Neural A\*** (một trường cost-to-go/bài toán), không phải CNN per-state.
- **Hoãn** hướng A (visibility-aware admissible heuristic) làm dự phòng nếu B2 không đạt.

## 2. Ràng buộc & non-goals

**Ràng buộc:**
- **Không đụng code hiện tại.** Tất cả nằm trong `ml_planner/`. Được **import và dùng lại** `core.*`, `config` ở chế độ **read-only**; không sửa file gốc.
- Bound ε = 5% là **hard-verify** trên toàn bộ benchmark.
- **Bản đồ vô hạn:** thực tế không có `map_bounds`, chỉ có `safezones`. Thiết kế không được giả định hộp 500 km.
- **Train off-machine** (Colab/GPU khác) bằng torch; **inference on-machine** (máy này, nhiều khả năng CPU; `requirements.txt` hiện chưa có torch).
- An toàn (tránh va chạm) **luôn** do `_check_collision` exact của bản gốc đảm nhiệm; ML không nằm trong vòng an toàn.

**Non-goals:**
- Không thay mô hình động lực học (vẫn `(waypoint, heading)`, R cố định, cost = độ dài gấp khúc).
- Không xử lý vật cản động theo thời gian.
- Không tối ưu đường bay filleted thực (search vẫn tối ưu độ dài gấp khúc, như bản gốc).
- Không đụng GUI, render.

## 3. Kiến trúc tổng thể & luồng dữ liệu

Ba nơi tách biệt:

```
[MÁY NÀY]  dataset_gen (oracle A* tối ưu + Dijkstra ngược) ──export──► dataset .npz
                                                                          │ upload
[COLAB/GPU]                                       train CNN (torch) ◄─────┘
                                                        │ export ONNX
[MÁY NÀY]  FocalKinodynamicAstar  ◄──────────────load──┘
             ├─ OPEN: f = g + h_euclid (admissible) → giữ bound ε
             └─ FOCAL: mở node có secondary_h nhỏ nhất (CNN cost-to-go)
```

**Bất biến an toàn/chất lượng:** CNN chỉ là secondary trong FOCAL. Euclid admissible luôn chặn `nghiệm ≤ (1+ε)·C*`. Va chạm luôn bị `_check_collision` exact loại. Rủi ro ML cô lập tuyệt đối.

### 3.1 Cấu trúc folder

```
ml_planner/
  __init__.py
  config.py            # FOCAL_EPS=0.05, GRID_RES=256, MODEL_PATH, ... (KHÔNG sửa config.py gốc)
  focal_astar.py       # class FocalKinodynamicAstar(KinodynamicAstar): override search()
  secondary.py         # secondary heuristic: bản thủ công (Pha 1) + interface guidance (Pha 2)
  guidance.py          # nạp ONNX, 1 forward/bài toán, tra cứu h_secondary(waypoint)
  dataset_gen.py       # dùng core/* (read-only) + Dijkstra ngược → .npz
  plan.py              # plan_trajectory_focal(): bản mỏng của plan_trajectory dùng subclass
  run_ml.py            # entrypoint benchmark A/B (giống batch_random_test, gọi focal)
  train/
    train_guidance.ipynb   # chạy Colab/GPU, ngoài repo-run
  tests/
    focal_astar_test.py    # test cô lập, chạy: python -m pytest ml_planner/tests
  models/
    guidance.onnx      # artifact từ Colab (gitignore; lưu/đồng bộ riêng)
```

## 4. Focal search (`focal_astar.py`)

`FocalKinodynamicAstar(KinodynamicAstar)` **kế thừa** bản gốc, **chỉ override `search()`** và thêm `secondary_h()`. Tái dùng nguyên `__init__` (gọi `super().__init__`), `get_next_states`, `_check_collision`, arc-hop, `smooth_path`, seeded corners, goal-acceptance qua kế thừa.

Thuật toán A\*ε:
- **OPEN** xếp theo `f = g + h_euclid` (`HEURISTIC_WEIGHT=1`, admissible, consistent ⇒ không reopening). Theo dõi `f_min = min f` trong OPEN.
- **FOCAL** = `{ n ∈ OPEN : f(n) ≤ w·f_min }`, `w = 1 + FOCAL_EPS = 1.05`.
- Mỗi vòng lặp: **mở node trong FOCAL có `secondary_h(n)` nhỏ nhất** (thay vì `f` nhỏ nhất).
- **Đảm bảo:** nghiệm ≤ `w·C*` (định lý A\*ε). ⇒ đúng ε = 5%.
- Cài đặt: heap thứ hai cho FOCAL, đồng bộ khi `f_min` tăng (đẩy các node mới vào dải). Giữ nguyên `closed_set`, per-object `State.parent`, dedup lattice, `MAX_ITERATIONS`, `TIME_BUDGET_S`.
- **Cờ invariance:** `FOCAL_EPS=0` + `secondary_h=h_euclid` ⇒ hành vi **đúng bằng** A* gốc (dùng để test tương đương, không cần đụng bản gốc).

`plan.py::plan_trajectory_focal()` là bản sao mỏng của `core.kinodynamic_astar.plan_trajectory` nhưng khởi tạo `FocalKinodynamicAstar`. Không sửa hàm gốc.

## 5. Secondary heuristic (`secondary.py`)

Interface: `secondary_h(state) -> float` — ước lượng cost-to-go (mét). Thấp = ưu tiên mở trước.

- **Pha 1 (thủ công, không ML):** `distance-to-go` = Euclid tới goal + phạt khi đoạn `P→goal` bị vật cản chắn (số/độ sâu vật cản cắt tia). Rẻ, O(N). Mục tiêu: kiểm chứng khung focal tăng tốc + làm **baseline** đo phần ML.
- **Pha 2 (ML):** `secondary_h(state) = guidance.lookup(state.waypoint)` từ trường CNN.
- **Fallback:** nếu không có model ONNX ⇒ tự dùng Pha 1. Planner không bao giờ crash vì thiếu model.

## 6. CNN guidance-map (`guidance.py`, Neural A* style)

- **Một forward/bài toán** lúc khởi tạo planner → **trường cost-to-go trên lưới cố định** `GRID_RES×GRID_RES` (mặc định 256). `lookup(waypoint) = nội suy song tuyến` trường tại vị trí. Per-state O(1).
- **Crop + chuẩn hoá theo bài toán** (xử lý bản đồ vô hạn): hộp bao `{start, goal, vật cản liên quan}` + lề, resize về lưới cố định. Lưu affine transform tọa-độ ↔ ô-lưới để map hai chiều.
- **Kênh đầu vào:**
  1. occupancy vật cản đã inflate (circle + polygon)
  2. mặt nạ safezone (trong/ngoài operating area)
  3. trường khoảng cách tới goal (encode goal)
  4. (tuỳ chọn) encode start / heading
- **Đầu ra:** 1 kênh cost-to-go (mét, cùng đơn vị `g`).
- **Kiến trúc:** U-Net nhỏ / VIN-style (định rõ ở notebook train).
- **Lưu ý:** trường inadmissible là **chấp nhận được** — nó chỉ xếp hạng trong FOCAL; Euclid giữ bound ε.

## 7. Pipeline dữ liệu (`dataset_gen.py`, chạy trên máy này)

- Tái dùng `batch_random_test.generate_random_scenario` (tất định theo seed). **Mở rộng phân bố để phủ cả mission thật (`y≈1.15e6`, safezone-only), không chỉ hộp 500 km** — nếu không CNN không tổng quát.
- Mỗi cảnh: chạy A* **tối ưu** (`FOCAL_EPS=0`, tắt time-budget) → **Dijkstra ngược từ goal trên đồ thị successor** → **nhãn cost-to-go dày cho mọi state đã đóng** (không chỉ trên đường tối ưu). Một cảnh cho hàng nghìn cặp gần như miễn phí.
- Rasterize theo đúng crop/chuẩn hoá mục 6, ghi `(channels, cost_to_go_field)` ra `.npz`. Nhãn field = cost-to-go nội suy lên lưới; ô chưa có nhãn → mask loại khỏi loss.
- Notebook `train/train_guidance.ipynb`: train trên Colab/GPU, export **ONNX**.

## 8. Inference runtime (máy này)

- Load ONNX bằng **`onnxruntime`** (CPU, nhẹ, nhanh). **Không cần torch** trong pipeline chạy chính; torch chỉ ở notebook train.
- Thêm `onnxruntime` vào một `ml_planner/requirements-ml.txt` **riêng** (không sửa `requirements.txt` gốc).
- Forward 1-lần/bài toán ~ms (đo trong benchmark). Nếu thiếu file model ⇒ fallback Pha 1.

## 9. Phân pha

- **Pha 1 (không ML):** focal search + Euclid + secondary thủ công. Kiểm chứng khung focal + baseline. *Chắc chắn hoàn thành và đã có giá trị.*
- **Pha 2 (ML):** dataset → train Colab → ONNX → cắm CNN làm secondary. Đo phần ML cộng thêm.

## 10. Kiểm thử & tiêu chí thành công

Benchmark A/B trên `batch_random_test` (1000 seed), so **cùng seed**:
- `planning_time`, `iterations` (kỳ vọng giảm).
- **Hard-verify: `path_cost ≤ 1.05 × cost A* tối ưu` trên MỌI seed.** Vi phạm = lỗi thiết kế.
- Tỉ lệ success không giảm so với bản gốc.

Test (`ml_planner/tests/`):
1. **Invariance:** `FOCAL_EPS=0`, `secondary=Euclid` ⇒ đường **đúng bằng** A* gốc trên tập seed mẫu.
2. **Bound ε:** trên tập seed, cost focal ≤ 1.05× cost oracle.
3. **Fallback:** thiếu model ONNX ⇒ dùng Pha 1, không crash.
4. **Guidance lookup:** crop/affine map hai chiều đúng; nội suy trong biên.

**Tiêu chí "đáng giữ":** Pha 1 giảm `planning_time` rõ rệt mà cost trong 5%; Pha 2 giảm thêm đáng kể trên case khó (heading bất lợi, nhiều đảo).

## 11. Rủi ro & giảm thiểu

| Rủi ro | Giảm thiểu |
|---|---|
| Độ phân giải lưới không đủ (safezone hành lang hẹp) | Thử nhiều `GRID_RES`; vỡ thì fallback secondary thủ công (luôn có) |
| Phân bố dữ liệu lệch (mission thật) | Đưa mission thật vào tập train từ đầu |
| CNN kém ⇒ FOCAL chọn dở | Vô hại: chỉ chậm hơn, **không** phá bound ε/an toàn |
| Latency forward 1-lần/bài toán | ~ms ONNX-CPU; đo trong benchmark; giảm `GRID_RES` nếu cần |
| Subclass lệch khi bản gốc đổi `search()` | Chấp nhận (chủ ý cô lập, ít đụng gốc); test invariance bắt lệch sớm |
| Phình phụ thuộc | torch chỉ ở notebook; planner chỉ `onnxruntime`, khai báo ở file requirements riêng |

## 12. Hoãn lại / tương lai

- **Hướng A (visibility-aware admissible heuristic):** dự phòng nếu B2 không đạt; giữ tối ưu tuyệt đối, tăng tốc có điều kiện. Có thể kết hợp làm `h_admissible` chặt hơn cho FOCAL sau này.
- Secondary heuristic học "search-effort-to-go" (số bước) thay vì cost-to-go — có thể giảm expansion tốt hơn; thử ở Pha 2 nếu cần.
