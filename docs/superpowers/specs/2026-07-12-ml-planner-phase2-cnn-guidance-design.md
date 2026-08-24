# Thiết kế Phase 2: CNN Guidance-map cho Focal Search (`ml_planner`)

- **Ngày:** 2026-07-12
- **Trạng thái:** Design (chờ duyệt để chuyển sang implementation plan)
- **Tiền đề:** Phase 1 xong (branch `feature/ml-planner`, focal A*ε + secondary thủ công, 17 test pass, benchmark ~46% nhanh hơn, 0 vi phạm bound ε=5%). Phase 2 thay secondary thủ công bằng **CNN guidance-map học được**, vẫn cô lập trong `ml_planner/`, **không sửa code Phase 1/base**.

---

## 1. Mục tiêu & phạm vi

Thêm một **secondary heuristic học được** (CNN guidance-map kiểu Neural A*) cho focal search, để dẫn tìm kiếm tốt hơn secondary thủ công — đặc biệt các case heading bất lợi / nhiều đảo. **Bound ε=5% vẫn do Euclid admissible OPEN giữ**, nên CNN sai cỡ nào cũng không phá bound/an toàn.

**Ranh giới in-session** (train chạy off-machine trên Colab/GPU, không trong session này):

| Phần | Trong session |
|---|---|
| `dataset_gen.py` — sinh dữ liệu `.npz` | ✅ xây + test (chạy oracle trên máy này) |
| `guidance.py` — inference ONNX + crop/lookup | ✅ xây + test bằng **model giả (stub)** |
| Wire CNN làm secondary + fallback | ✅ xây + test |
| `train/train_guidance.ipynb` | ✅ **scaffold** (không chạy trong session) |
| Lần train thật → `guidance.onnx` | ❌ off-machine (user chạy Colab, thả model vào `ml_planner/models/`) |

Toàn bộ đấu-nối được kiểm chứng in-session bằng **model giả**; khi user train xong chỉ việc thả ONNX vào là chạy.

## 2. Ràng buộc & non-goals

**Ràng buộc:**
- **Không sửa code Phase 1/base.** Chỉ THÊM file mới trong `ml_planner/` (dataset_gen.py, guidance.py, train/, models/, test mới). Được sửa NHẸ để wire secondary='guidance' vào `plan_trajectory_focal` — nhưng phải **giữ nguyên hành vi Phase 1 khi không có model** (mặc định vẫn hand-crafted).
- **Train off-machine** (torch, Colab/GPU). **Inference on-machine** qua `onnxruntime`. `onnxruntime` chỉ khai báo ở `ml_planner/requirements-ml.txt` (đã có); **không thêm dependency BẮT BUỘC cho Phase 1** — test ONNX-path thật `pytest.skip` khi onnxruntime chưa cài.
- **Bound ε=5%** do Euclid OPEN giữ; CNN chỉ là secondary trong FOCAL. An toàn (va chạm) vẫn do base `_check_collision`.
- **Bản đồ vô hạn:** crop + chuẩn hoá theo từng bài toán; không giả định `map_bounds`.

**Non-goals:**
- Không train trong session. Không tự động tải/đồng bộ model.
- Không đổi thuật toán focal Phase 1, không đổi mô hình động lực học.
- Trường guidance **position-only** (không điều kiện theo heading) ở Phase 2; heading-conditioning là mở rộng tương lai.

## 3. Kiến trúc & luồng dữ liệu

```
[MÁY NÀY] dataset_gen: oracle search (record edges) + backward-Dijkstra
          → nhãn cost-to-go dày → rasterize (4 kênh + label + mask) → .npz
                                                      │ upload
[COLAB]                          train U-Net (torch, masked-MSE) ──► guidance.onnx
                                                      │ đặt vào ml_planner/models/
[MÁY NÀY] Guidance(onnxruntime): build_field 1 lần/bài toán → lookup O(1)
          → secondary trong FOCAL của FocalKinodynamicAstar (Phase 1)
```

**Bất biến:** không có `guidance.onnx` ⇒ `Guidance.available=False` ⇒ secondary tự về **hand-crafted (Phase 1)**; hành vi Phase 1 không đổi.

Cấu trúc file mới:
```
ml_planner/
  dataset_gen.py      # sinh .npz từ oracle + backward-Dijkstra + rasterize
  raster.py           # crop/affine world<->grid + build 4 channels (dùng chung dataset_gen & guidance)
  guidance.py         # Guidance: load ONNX, build_field 1-forward, lookup bilinear, available flag
  train/train_guidance.ipynb   # scaffold train (off-machine)
  models/             # guidance.onnx (gitignore; đồng bộ riêng)
  tests/
    raster_test.py
    dataset_gen_test.py
    guidance_test.py
    guidance_integration_test.py
```

## 4. `raster.py` — crop/affine + channels (dùng chung)

Tách riêng để `dataset_gen` (nhãn) và `guidance` (inference) **dùng CHUNG một định nghĩa crop/channels** (nếu lệch nhau thì model vô dụng).

- `compute_crop(preprocessed, margin_frac=0.1) -> Affine`: bbox của `{start_pos, goal_pos, mọi vật cản (circle center±r, polygon verts)}`, cộng lề `margin_frac`, ép **vuông**, map về `[0,GRID_RES)²`. Trả affine 2 chiều `world_to_grid(x,y)` / `grid_to_world(i,j)`.
- `build_channels(preprocessed, affine, grid_res) -> ndarray (4,H,W)`:
  - kênh 0: occupancy vật cản đã inflate (circle + polygon) — 1 nếu ô bị chiếm.
  - kênh 1: mask safezone (1 trong operating area; toàn 1 nếu không có safezone).
  - kênh 2: khoảng cách chuẩn hoá tới goal (encode goal).
  - kênh 3: marker start (Gaussian/one-hot quanh start cell).
- `GRID_RES=256` (đọc từ `ml_planner/config.py`, đã có placeholder).

## 5. `dataset_gen.py` — nhãn cost-to-go dày

- `_RecordingAstar(FocalKinodynamicAstar)`: `focal_eps=0`, bỏ time/iteration budget; override `search()` để GHI mọi cạnh `(u_key → v_key, cost)` đã relax, `key = spatial_utils.state_to_tuple(wp, heading)` (đúng lattice dedup của search). Cũng lưu `key -> waypoint` đại diện.
- `backward_costs(edges, goal_key) -> {key: cost_to_go}`: Dijkstra trên **đồ thị đảo cạnh** từ `goal_key`.
- `rasterize_labels(costs, key2wp, affine, grid_res) -> (label(H,W), mask(H,W))`: mỗi key → ô; `label[cell]=min` (cost-to-go tốt nhất tại vị trí, vì trường position-only); `mask` = ô có nhãn.
- `generate_sample(scenario) -> dict`: chạy oracle, dựng channels (raster.py) + label + mask + affine. `None` nếu oracle không giải được.
- `export_dataset(seeds, out_path)`: lặp `batch_random_test.generate_random_scenario(seed)` (mở rộng phân bố phủ cả mission thật nếu có), ghi `.npz` nén nhiều mẫu.
- **Sanity bất biến (test):** cost-to-go tại key của start-corner ≈ chi phí mission tối ưu của base (cùng thước đo mission-cost Phase 1).

## 6. Hợp đồng CNN (cứng, cả 3 nơi tuân theo)

- **Input:** `(1, 4, GRID_RES, GRID_RES)` float32 — 4 kênh mục 4.
- **Output:** `(1, 1, GRID_RES, GRID_RES)` float32 — trường cost-to-go (mét).
- **ONNX:** input name `channels`, output name `cost_to_go`; opset ≥ 11.
- `guidance.py`, notebook train, và stub test đều theo đúng hợp đồng này.

## 7. `guidance.py` — inference

- `Guidance(model_path=config.MODEL_PATH, grid_res=config.GRID_RES)`:
  - `available`: True nếu file model tồn tại VÀ `import onnxruntime` thành công; else False (không raise).
  - `build_field(preprocessed)`: `affine=compute_crop(...)`; `channels=build_channels(...)`; **1 forward** ONNX → `field(H,W)`; lưu `field`, `affine`.
  - `lookup(waypoint) -> float`: world→grid qua affine; **nội suy song tuyến** trên `field`; ngoài lưới ⇒ trả `LARGE` (search không ưu tiên node ngoài crop).
- `make_guidance_secondary(preprocessed, model_path=None)`: trả `(secondary_callable_or_None, available_bool)` — build field 1 lần, trả closure `lambda st: guidance.lookup(st.waypoint)`; nếu không available trả `(None, False)`.

## 8. Tích hợp (sửa nhẹ, giữ Phase 1)

- `plan_trajectory_focal(preprocessed, focal_eps=None, secondary=None, verbose=False)` (Phase 1) thêm nhánh: nếu `secondary == 'guidance'` (chuỗi cờ) ⇒ gọi `make_guidance_secondary(preprocessed)`; nếu `available` ⇒ dùng closure đó; **nếu không ⇒ về hand-crafted (None → FocalKinodynamicAstar tự dùng hand-crafted)**.
- `secondary=None`/callable ⇒ **hành vi Phase 1 y nguyên**.
- Không có model ⇒ `secondary='guidance'` **degrade êm** về hand-crafted, không crash.

## 9. Kiểm thử (in-session, không cần model thật)

- `raster_test.py`: `compute_crop` bao đúng start/goal/obstacles + lề; affine world↔grid roundtrip; `build_channels` shape `(4,H,W)`, occupancy đánh dấu đúng 1 vật cản đã biết, safezone toàn-1 khi vắng.
- `dataset_gen_test.py`: trên scenario nhỏ có nghiệm, `generate_sample` trả channels/label/mask đúng shape; **cost-to-go tại start ≈ mission-cost tối ưu base** (sai số nhỏ); mask không rỗng.
- `guidance_test.py`: **StubGuidance** (Python, cùng interface, field = distance-to-goal) → `lookup` nội suy đúng, ngoài-crop trả LARGE. Test load ONNX thật: `pytest.importorskip("onnxruntime")` + một ONNX hằng nhỏ (dựng bằng `onnx` helper nếu có, else skip).
- `guidance_integration_test.py`: cắm StubGuidance làm secondary vào focal → planner giải được VÀ **giữ bound ε=5%** trên scenario có vật cản; **không có model ⇒ `plan_trajectory_focal(..., secondary='guidance')` cho kết quả hệt Phase 1 hand-crafted** (degrade êm).

## 10. `train/train_guidance.ipynb` (scaffold, off-machine)

Cell tài liệu hoá: nạp `.npz` → `Dataset`/`DataLoader` → U-Net nhỏ (torch) → train **masked-MSE** (chỉ ô `mask=1`) → validate → **export ONNX đúng hợp đồng mục 6** (4→1 kênh, `GRID_RES²`, tên `channels`/`cost_to_go`). Không chạy trong session; là hướng dẫn để user chạy Colab.

## 11. Rủi ro & giảm thiểu

| Rủi ro | Giảm thiểu |
|---|---|
| Crop/channels lệch giữa dataset_gen và guidance | Dùng CHUNG `raster.py`; test roundtrip |
| Độ phân giải lưới không đủ (hành lang hẹp) | `GRID_RES` cấu hình; fallback hand-crafted luôn có |
| CNN kém ⇒ FOCAL chọn dở | Vô hại: chỉ chậm hơn, **không** phá bound ε/an toàn |
| onnxruntime chưa cài | `available=False` ⇒ fallback; test ONNX-path skip |
| Nhãn chỉ phủ vùng oracle khám phá | Chấp nhận (đó là vùng search cần dẫn); nhiều scenario để phủ rộng |
| Phân bố dữ liệu lệch mission thật | Đưa mission thật vào tập seed sinh dữ liệu |

## 12. Hoãn lại / tương lai

- Heading-conditioning cho trường guidance (kênh heading hoặc field 3D).
- Nhãn "search-effort-to-go" (số bước) thay cost-to-go nếu cần giảm expansion mạnh hơn.
- DAgger (train lại trên state do chính guidance dẫn) nếu lệch phân bố.
- Hướng A (visibility-aware admissible heuristic) vẫn là dự phòng độc lập.
