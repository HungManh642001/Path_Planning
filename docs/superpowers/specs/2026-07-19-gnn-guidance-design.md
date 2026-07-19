# Thiết kế: GNN guidance trên tangent graph cho focal A* (Prototype — Phương án A)

Ngày: 2026-07-19 · Trạng thái: đã duyệt thiết kế miệng, chờ review spec

## 1. Bối cảnh & động cơ

CNN raster guidance hiện tại (U-Net 384², `ml_planner/guidance.py`) là secondary
heuristic cho focal A* (ε = 5%). Benchmark 2026-07-19 (50 kịch bản held-out)
cho kết quả:

- **Chất lượng đường tốt hơn** hand-crafted trên hard maps (mean cost-ratio vs
  base: 1.0034 so với 1.0090), 0 vi phạm ε-bound.
- **Không thắng về tốc độ**: hard maps +58% tổng iterations, +47% wall-time so
  với hand-crafted; forward pass ONNX ~0.1–0.15 s/bài không khấu hao được.

Ba điểm yếu cấu trúc của hướng raster: (1) chi phí inference cố định quá lớn so
với search chỉ ~600 expansions median; (2) field theo vị trí, không phân giải
heading; (3) raster có giới hạn độ phân giải khi bản đồ lớn / hành lang hẹp —
điểm chí mạng khi triển khai thực tế với safezone biến thiên theo kịch bản.

GNN trên tangent graph sửa cả ba: inference ~ms trên đồ thị vài chục đến
~1 000 node,
biểu diễn khớp bản chất tổ hợp của bài toán (đi vòng bên nào của obstacle),
không phụ thuộc độ phân giải/tỷ lệ bản đồ.

## 2. Mục tiêu & tiêu chí thành công

- **Prototype kiểm chứng**: trả lời "GNN có thắng hand-crafted không" với ít
  code nhất; tái dùng tối đa oracle/label pipeline và benchmark hiện có.
- **Tiêu chí nghiệm thu** (hard held-out, so với hand-crafted secondary): thắng
  ít nhất một trong ① tổng iterations & wall-time, ② mean cost-ratio; trục còn
  lại không kém hơn (wall-time chênh ≤ ~5%, cost-ratio chênh ≤ 0.002).
- **Cổng dừng sớm (offline)**: Spearman rank correlation per-node ≥ 0.8 trên
  seed held-out; thấp hơn → dừng trước khi tốn công tích hợp benchmark.

## 3. Ngoài phạm vi (phase này)

- Safezone biến thiên: dữ liệu train/eval phase này KHÔNG có safezone; kiến
  trúc chừa sẵn feature slot (xem §6) để phase sau chỉ đổ dữ liệu, không đổi
  model.
- Prune/xếp thứ tự successor trong `get_next_states` (Phương án B — Phase 2 tự
  nhiên nếu prototype thắng; nhãn Q(u→v) = c(u,v) + V(v) suy ra miễn phí từ
  nhãn node của phase này).
- Không sửa search core (`core/`), không sửa contract secondary hiện có.

## 4. Kiến trúc

Bốn file mới trong `ml_planner/`, không sửa search core:

```
ml_planner/
  graph.py             # dựng tangent graph tường minh từ preprocessed scenario
  graph_dataset.py     # build dataset: oracle solve song song → nhãn per-node → shard .npz
  graph_guidance.py    # inference numpy + lookup; make_graph_secondary() cùng contract CNN
  train/train_graph.py # trainer GPU standalone (PyTorch, MPNN tự viết, không cần PyG)
  models/graph_guidance.npz   # trọng số model (thiếu file → fallback hand-crafted)
```

### 4.1 Đồ thị (`graph.py`)

Dựng trên đúng hình học planner đang dùng (bán kính đã inflate +
`config.CONSTRUCTION_CLEARANCE_M`):

- **Node** = điểm tiếp tuyến của các bitangent giữa từng cặp hình tròn
  + tiếp tuyến từ start/goal tới từng hình tròn + đỉnh polygon hull
  + start + goal. Với N ≈ 6–16 hình tròn: vài chục đến ~1 000 node.
- **Cạnh** = chord bitangent không va chạm (kiểm bằng đúng collision check của
  planner) + cạnh cung nối các node kề nhau theo góc trên cùng một biên tròn
  + cạnh thẳng start/goal ↔ tangent points (khi không va chạm).
- Kèm `scipy.spatial.cKDTree` trên tọa độ node phục vụ lookup.

API: `build_graph(preprocessed) -> Graph` (dataclass: `nodes (M,2)`,
`node_feat (M,F)`, `edges (E,2)`, `edge_feat (E,2)`, `kdtree`). Deterministic
theo scenario.

## 5. Luồng dữ liệu

**Huấn luyện:** scenario (`hard_scenario`, phân bố hiện tại) → oracle solve
không budget (tái dùng labeler backward-Dijkstra của `dataset_gen` — lấy
`costs` per-waypoint TRƯỚC bước rasterize) → snap nhãn lên node:
`V_label(v) = min_w [cost(w) + dist(v, w)]` trên các waypoint oracle trong bán
kính lân cận (cỡ `STATE_POS_QUANTUM`); node không có waypoint gần → mask khỏi
loss → shard `.npz` (mảng nối + offsets vì đồ thị biến kích thước) → train GPU
→ xuất `graph_guidance.npz`.

**Planning:** `prepare_scenario` → `build_graph` (~ms) → forward MPNN numpy
(~ms) → V̂ (mét) cho từng node → secondary của focal search:
`V̂(state) = min trên k=3 node gần nhất [dist(state, v) + V̂(v)]`.
Cắm qua `make_graph_secondary(preprocessed)` — cùng chữ ký `(callback,
available)` với `make_guidance_secondary`; fallback sạch khi thiếu model.

**Benchmark:** thêm cột thứ 4 (`gnn_*`) vào `ml_planner/benchmark.py` — một
lần chạy so base / hand / CNN / GNN trên cùng seed; verdict mở rộng theo tiêu
chí §2.

## 6. Feature & model

Chuẩn hóa theo `D` = khoảng cách start→goal (bất biến tỷ lệ bản đồ).

**Node (7 chiều):**
1. `dist(v, goal)/D`
2. `sin` góc (phương v→goal so với phương start→goal)
3. `cos` góc trên
4. bán kính hình tròn chứa node `/D` (0 nếu đỉnh polygon/start/goal)
5. cờ `is_goal`
6. cờ `is_start`
7. **slot safezone (chừa sẵn)**: `dist(v, biên safezone)/D`; phase này luôn
   = 1.0 ("xa biên") — phase sau đổ dữ liệu thật vào cùng chỗ, không đổi model.

**Cạnh (2 chiều):** độ dài `/D`; loại cạnh (chord = 0, cung = 1).

**Model:** MPNN tự viết ~4 vòng, hidden 64 (~50–80k tham số). Mỗi vòng:
message trên cạnh = MLP(h_u, h_v, edge_feat) → scatter-add về node → cập nhật
kiểu GRU. Đầu ra mỗi node một vô hướng `r(v) ≥ 0` (softplus):

```
V̂(v) = dist(v, goal) + softplus(r(v)) · D
```

Ràng buộc **V̂ ≥ Euclid theo cấu trúc** (cost-to-go không thể ngắn hơn đường
chim bay) — model chỉ học "phần vòng tránh", cùng triết lý residual-over-Euclid
của CNN hiện tại.

**Loss:** Huber trên `(V̂ − V_label)/D`, chỉ trên node có mask.
**Train:** Adam, batch = ghép nhiều đồ thị (offset edge index), ~100–150
epoch — mirror `train/train_guidance.py`. Inference production bằng numpy
thuần từ `.npz` (không thêm dependency runtime; PyTorch chỉ cần lúc train).

## 7. Xử lý lỗi — nguyên tắc "không bao giờ tệ hơn hiện trạng"

- Thiếu `graph_guidance.npz` / import lỗi → `make_graph_secondary` trả
  `(None, False)` → hand-crafted (đúng pattern CNN).
- `build_graph` hoặc forward ném exception trên một scenario → bắt tại chỗ,
  fallback hand-crafted cho scenario đó (pattern `_guided_plan` hiện có).
- Đồ thị thoái hóa (0 node trung gian — map trống) → bỏ GNN, secondary =
  Euclid thuần (trường hợp này hand-crafted cũng đã tối ưu sẵn).
- Dataset gen: seed oracle fail/timeout → skip (pattern `build_dataset`).
- ε-bound 5% không bao giờ bị đe dọa dù model sai hoàn toàn — cấu trúc focal
  đảm bảo (đã kiểm chứng với CNN: 0 vi phạm).

## 8. Kiểm thử (tên `*_test.py` để commit, theo convention repo)

- `graph_test.py`: scenario dựng tay 2–3 hình tròn — số tangent point đúng
  công thức; không cạnh nào cắt phần trong obstacle (trọng tài:
  `core/path_validation`); cạnh cung nối đúng node kề góc; build deterministic.
- `graph_guidance_test.py`: fallback sạch khi thiếu model; tính chất
  `V̂ ≥ Euclid` với trọng số ngẫu nhiên; forward numpy khớp forward PyTorch
  trên trọng số cố định nhỏ (golden test).
- `graph_dataset_test.py`: snap-label đúng trên case tay (waypoint oracle đặt
  sẵn); round-trip shard `.npz`.
- Mở rộng `benchmark_test.py`: cột `gnn_*` xuất hiện; smoke với model giả.

## 9. Rủi ro & giảm thiểu

| Rủi ro | Giảm thiểu |
|---|---|
| GNN không học tốt hơn hand-crafted (rủi ro nghiên cứu thật) | Cổng dừng sớm offline (Spearman ≥ 0.8) trước khi tốn công benchmark |
| Nhãn snap sai lệch (oracle waypoint không trùng node bitangent) | Snap dạng relaxation `min[cost + dist]` + mask node không có nhãn gần; test tay |
| Đồ thị quá lớn ở N=16 hình tròn (~1 000 node) làm chậm build | Vẫn ~ms với numpy dense; nếu đo thấy chậm, prune bitangent bị chặn ngay lúc dựng |
| Lookup k-NN cho state xa mọi node (fan Strategy B ngoài khơi) | Công thức `min[dist + V̂]` tự suy giảm về Euclid-dominated khi ở xa — an toàn |

## 10. Hướng tiếp theo (ngoài phạm vi, ghi để định hướng)

- **Phase 2 — Phương án B**: edge-scoring Q(u→v) xếp thứ tự successor trong
  `get_next_states` (nhãn suy từ nhãn node phase này), đòn bẩy tốc độ lớn hơn.
- **Safezone**: generator safezone ngẫu nhiên cho cả dataset lẫn benchmark; đổ
  feature slot §6; thêm node/cạnh biên safezone vào đồ thị.
