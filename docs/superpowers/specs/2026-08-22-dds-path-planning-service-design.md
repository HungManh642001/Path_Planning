# Thiết kế: service path planning độc lập, giao tiếp qua DDS

Ngày: 2026-08-22 (sửa lần 2)
Nhánh: `feature/dds-service`

## 0. Bản sửa này thay đổi gì

Bản đầu đề xuất một node C++ Fast DDS cầu nối sang worker Python. Chủ sở hữu
chất vấn, và **chất vấn đúng**: mục tiêu là một service thuật toán **bằng
Python, độc lập** với hệ thống đang triển khai, tự khởi động cùng Ubuntu.

Lập luận cũ chọn C++ vì "bên gọi là C++, đã có IDL/toolchain" - một lợi ích tổ
chức, không phải bắt buộc kỹ thuật. Lý do *kỹ thuật* duy nhất là planner Python
không hủy được từ bên ngoài nên cần một tiến trình khác giữ quyền `SIGKILL`.
Điều đó giải quyết trọn vẹn trong Python thuần bằng `multiprocessing`, nên C++
bị loại, và cùng với nó là toàn bộ lớp giao thức nội bộ (Unix socket, msgpack,
đóng khung).

Bốn thay đổi khác do chủ sở hữu đặt ra:

1. **Chỉ hệ toạ độ Oxy phẳng, mét.** WGS84 để sau. Xoá phép chiếu và `pyproj`.
2. **Bản đồ nạp sẵn là file XML**, mang `safezones` + obstacles, **không**
   `map_bounds`.
3. **`time_budget_s` có trên dây nhưng chưa được tôn trọng**: service dùng
   `config.TIME_BUDGET_S`. Sau này thuật toán sẽ nhận nó như một tham số thật.
4. **Stack DDS chưa chốt**: một spike đo cả Fast DDS Python binding lẫn Cyclone
   DDS rồi mới quyết. Lớp transport được cô lập để kết quả spike chỉ ảnh hưởng
   một module.

## 1. Mục tiêu và phạm vi

Một hệ thống ngoài gửi một mission và nhận về danh sách waypoint, qua DDS.
Thuật toán hiện tại được bọc thành một service Python độc lập.

Trong phạm vi: hợp đồng dữ liệu, lớp transport DDS, bọc thuật toán, bản đồ nền
XML, thời hạn cứng, và một unit systemd tự khởi động.

Ngoài phạm vi: WGS84; replan liên tục theo tick; nhiều mission song song; mọi
thay đổi trong `core/`, `render/`, `config.py`; đóng gói phân phối (sẽ phân tích
riêng).

### Ràng buộc do chủ sở hữu đặt ra

1. Không sửa thuật toán hiện tại. Toàn bộ mã mới nằm trong `service/`.
2. Làm trên nhánh riêng `feature/dds-service`.
3. Khi thuật toán thay đổi, service phải tự động đi theo, không cần sửa tay.

Ràng buộc 3 được cưỡng chế bằng một nguyên tắc và ba cơ chế: **adapter chỉ gọi,
tuyệt đối không sao chép** (mục 7).

## 2. Bối cảnh từ codebase

**Điểm vào rất gọn.** `prepare_scenario(scenario)` -> `plan_trajectory(pre)` ->
`result['path']`, một list `((x, y), heading)`. Hai dict shape đã có TypedDict
trong `core/types.py`.

**Planner đang ship là `core/kinodynamic_astar_v0.py`**, không phải
`kinodynamic_astar.py`.

**`success` nghĩa là oracle độc lập đã chấp nhận toàn bộ đường bay**, không phải
"search trả về cái gì đó". Service kế thừa hợp đồng đó nguyên vẹn.

**Planner không tất định giữa các máy.** `config.TIME_BUDGET_S = 15` cắt theo
đồng hồ. Phơi ra bằng `budget_bound` trong reply.

Số đo tham chiếu, 18 preset trên v0 (2026-08-22, máy này): median 16 ms,
`scenario_18_reversed_approach_cluttered` 3,9 s, trần cứng 15 s.

**35 hằng số global.** Chỉ 5 giá trị là tham số hàm của `prepare_scenario`
(`turn_radius`, `l0`, `dss`, `safe_margin`, `alpha_max_rad`). Đã xác minh trên
code: `prepare_scenario` ghi `l0` vào `start_state['straight_length']` và `dss`
vào `goal_state['engagement_distance']` (`preprocessing.py:118,154,219`), planner
đọc lại đúng hai khoá đó và chỉ rơi về config khi chúng vắng
(`kinodynamic_astar_v0.py:212,267`). Ngược lại `max_iterations` và
`TIME_BUDGET_S` chỉ tồn tại như global (`:283`, `:1017`).

**Thuật toán chỉ phụ thuộc vào shapely.** `core/` không import numpy ở đâu cả
(0 lần), cũng không scipy, cũng không matplotlib. Đã kiểm chứng bằng venv sạch
chỉ có `shapely 2.1.2`, kéo theo **numpy 2.4.6** - đúng phiên bản mà `CLAUDE.md`
cảnh báo làm vỡ stack - và cả **18/18 preset đều giải được**. Cái pin
`numpy==1.26.4` là ràng buộc của matplotlib/pandas trong stack test-GUI và
**không áp dụng cho service**.

## 3. Kiến trúc

Một tiến trình Python, cộng một tiến trình con dùng-một-lần cho mỗi lần lập kế
hoạch.

```
Hệ thống gọi (DDS)
   |  publish VtxPathPlanRequest / subscribe VtxPathPlanReply
   v
vtx_planner_service  (Python 3.11, một tiến trình)
   |
   +-- transport/           lớp DDS cô lập (Cyclone hoặc Fast DDS binding)
   +-- service loop         tuần tự, một request tại một thời điểm
   +-- PreloadedMap         XML nạp một lần lúc khởi động
   +-- PlanRunner  --fork--> tiến trình con: vtx_planner.plan(request)
   |                         thời hạn cứng -> kill -> PLAN_TIMEOUT
   v
core/preprocessing + core/kinodynamic_astar_v0 + core/mission   (không sửa)
```

### Vì sao vẫn có một tiến trình con

Planner là Python thuần, CPU-bound, và chỉ kiểm tra ngân sách tại các điểm trong
vòng lặp search - nó **không hủy được từ bên ngoài một cách lịch sự**. Một tiến
trình con là cách duy nhất để có thời hạn cứng thật, và nó cũng cách ly hoàn
toàn 35 hằng số global: tiến trình con sửa `config` thoải mái rồi chết.

### Vì sao `forkserver`, và vì sao nó phải lên trước DDS

Đây là chỗ đo đạc đổi quyết định.

DDS chạy thread nền ở tầng C. `fork()` từ một tiến trình có thread là công thức
kinh điển của deadlock trong tiến trình con: fork chỉ mang theo thread đang gọi,
nên một mutex do thread khác đang giữ sẽ bị giữ vĩnh viễn trong bản sao.

Đo được, cùng máy, plan `scenario_01`:

| cách tạo tiến trình con | median | ghi chú |
| --- | --- | --- |
| `fork`, cha chưa import `core` | 1012 ms | trả giá import trong mỗi con |
| `fork`, cha đã import `core` | 37,7 ms | kế thừa module đã nạp |
| `fork`, có participant DDS sống, dưới lưu lượng | 16,6 ms | 15/15 lần chạy được |
| `spawn` | 907-989 ms | interpreter mới, import lại |
| `forkserver` không preload | 1132-1247 ms | |
| **`forkserver` + preload `core`, khởi động TRƯỚC DDS** | **56,4 ms** | min 38,5 / max 84,7 |

**Chọn dòng cuối.** `fork` trần nhanh hơn 40 ms, nhưng 15 lần chạy thành công
không phải bằng chứng an toàn cho một deadlock xác suất - đó đúng là loại lỗi
chỉ hiện ra dưới tải, không hiện ra trong một phép thử. Và 40 ms là vô nghĩa so
với 16 ms - 4 s thời gian lập kế hoạch thật.

`forkserver` an toàn về **cấu trúc**, không phải nhờ may mắn: tiến trình
forkserver được khởi động **trước khi DDS tồn tại**, nên không tiến trình con
nào từng fork từ một tiến trình có thread DDS. `set_forkserver_preload` nạp sẵn
`core.*` để trả giá import một lần thay vì mỗi request.

### Cái cố tình không có

Không hàng đợi, không worker pool: 1 request/lúc. Bận thì trả `PLAN_BUSY`.
Không C++, không giao thức nội bộ, không msgpack, không Unix socket.

## 4. Hợp đồng dữ liệu

### Topic và QoS

Hai topic, `VtxPathPlanRequest` và `VtxPathPlanReply`, tương quan bằng
`request_id` do client sinh và service trả nguyên.

| Topic | Reliability | History | Durability |
| --- | --- | --- | --- |
| Request | RELIABLE | KEEP_ALL | **VOLATILE** |
| Reply | RELIABLE | KEEP_LAST(8) | **VOLATILE** |

`VOLATILE` là bắt buộc. `TRANSIENT_LOCAL` trên topic request nghĩa là service
khởi động lại sẽ nhận và lập kế hoạch lại một mission cũ đã hết hiệu lực. Một
lệnh bay không được phép phát lại.

### Kiểu dữ liệu

Khai báo một lần trong `service/vtx_service/messages.py` bằng dataclass, và
lớp transport dịch sang kiểu của stack DDS được chọn. IDL `.idl` tương đương
được sinh ra để bên gọi dùng.

```
PlanStatus:  OK=0, NO_PATH=1, START_LEG_BLOCKED=2, GOAL_LEG_BLOCKED=3,
             ORACLE_REJECTED=4, INVALID_REQUEST=5, TIMEOUT=6,
             INTERNAL_ERROR=7, BUSY=8

Point2D            x, y                                    (mét)
Polygon            vertices: sequence<Point2D>             (vành mở)
Circle             center: Point2D, radius_m
VehicleLimits      turn_radius_m, l0_m, dss_m, safe_margin_m, alpha_max_deg
SearchBudget       time_budget_s, max_iterations           (xem mục 4.3)

VtxPathPlanRequest
  @key request_id[16], idl_version
  start: Point2D, start_heading_deg
  goal:  Point2D, goal_heading_deg, goal_heading_free
  islands:           sequence<Polygon>
  dynamic_obstacles: sequence<Circle>
  safezones:         sequence<Polygon>
  use_preloaded_map: boolean
  limits: VehicleLimits
  budget: SearchBudget

Waypoint     position: Point2D, heading_deg
SearchStats  iterations, max_iterations, open_set_size, search_failed, budget_bound

VtxPathPlanReply
  @key request_id[16], idl_version
  status: PlanStatus, detail: string
  waypoints: sequence<Waypoint>
  path_length_m, plan_wall_time_s
  applied_time_budget_s                    (xem mục 4.3)
  stats: SearchStats
  planner_version: string, config_hash: string
```

### 4.1 Đơn vị và quy ước

**Khoảng cách: mét, trên mặt phẳng Oxy. Góc: độ, phương vị thật, thuận chiều kim
đồng hồ từ chính bắc.** Quy ước `+y` là bắc, `+x` là đông.

Thuật toán bên trong dùng radian ngược chiều kim đồng hồ từ `+x`, nên quy đổi là
`theta = deg_to_rad(90 - bearing_deg)`, thực hiện tại đúng **một** module.

Một quy ước duy nhất trên dây. Loại lỗi này - đường bay lệch 90 độ hoặc bị gương
- vẫn là đường bay hợp lệ về hình học, nên mọi test hình học đều bỏ lọt.

Không có trường `frame`: chỉ có một hệ toạ độ. Thêm WGS84 sau này là một lần
tăng `idl_version`.

### 4.2 `map_bounds` cố tình không có

`core.types.Scenario` có khoá này, nhưng nó là hình chữ nhật `(w, h)` neo tại
gốc toạ độ và test là `0 < x < w` (`kinodynamic_astar_v0.py:863`), tức phụ thuộc
vào việc gốc toạ độ nằm ở đâu. `safezones` biểu diễn được đúng vùng đó, mạnh hơn
hẳn, và bất biến với tịnh tiến. Khi thiếu cả hai, `_in_bounds` trả `True`
(`:860`), nên bỏ trường này không mất tính năng nào.

Hệ quả cần biết trước: 18 preset trong `map_generator` **có** `map_bounds`, nên
đường bay qua service có thể khác đường bay khi chạy preset trực tiếp. Test
tương đương xử lý điều này bằng hai khẳng định tách bạch (mục 7).

### 4.3 `time_budget_s` chưa được tôn trọng, và reply nói thật về điều đó

Trường có mặt trên dây để sau này không phải tăng `idl_version`, nhưng service
hiện dùng `config.TIME_BUDGET_S`. Chủ sở hữu sẽ sửa thuật toán để nhận nó như
một tham số thật.

Đã đo: override `config.TIME_BUDGET_S` lúc chạy **có** hiệu lực (v0 đọc nó trong
vòng lặp search, không phải lúc import), và với tiến trình con thì hoàn toàn
cách ly. Nên đây là lựa chọn thiết kế, không phải giới hạn kỹ thuật.

Reply mang `applied_time_budget_s` = giá trị service **thực sự** đã dùng. Nhận
một trường rồi lặng lẽ bỏ qua là cách chắc chắn để client tin vào một điều không
đúng; báo cáo ngược giá trị thật thì client tự đối chiếu được.

`max_iterations` được đối xử y hệt, và `stats.max_iterations` trong reply là giá
trị thật đã dùng. Hai trường cạnh nhau mà một cái được tôn trọng, một cái không,
là một cái bẫy không cần thiết.

### 4.4 Các quyết định còn lại

**`goal_heading_free` là cờ riêng, không phải giá trị canh gác.** Python dùng
`goal_heading = None` cho chế độ free-goal; IDL không có optional và mã hoá bằng
NaN là mời gọi tai nạn.

**Trả về đường bay đầy đủ `O..T`.** Điểm cất cánh và mục tiêu do
`core.mission.full_mission_path` ghép vào; service gọi đúng hàm đó chứ không tự
ghép.

**`config_hash` báo cáo ngược cấu hình đã dùng.** Băm SHA-256 rút gọn của các
hằng số mà `kinodynamic_astar_v0` thực sự đọc, phát hiện bằng cách quét mã nguồn
planner chứ không hardcode. Trên một codebase nghiên cứu nơi hằng số được A/B
liên tục, đây là điều kiện để reply có nghĩa.

**`detail` giữ nguyên văn chuỗi từ oracle**, kể cả những chuỗi mang tham số như
`first W1..W2 l=7421.3 < L0=8000`. Enum để máy rẽ nhánh, `detail` để người đọc.
Một `failure_reason` mới trong `core/` rơi vào `PLAN_ORACLE_REJECTED` thay vì
làm sập adapter - suy giảm êm, không vỡ.

## 5. Bản đồ nền XML

Nạp một lần lúc khởi động. Mặc định triển khai là **không có** bản đồ nào:
request tự chứa thì replay được và chẩn đoán được, còn state ẩn trong service thì
không.

Schema tối giản, toạ độ mét trong hệ Oxy:

```xml
<vtx-map version="1">
  <safezones>
    <polygon>
      <point x="0" y="0"/>
      <point x="500000" y="0"/>
      <point x="500000" y="500000"/>
      <point x="0" y="500000"/>
    </polygon>
  </safezones>
  <obstacles>
    <polygon>
      <point x="150000" y="120000"/>
      <point x="200000" y="120000"/>
      <point x="175000" y="200000"/>
    </polygon>
    <circle cx="220000" cy="180000" r="15000"/>
  </obstacles>
</vtx-map>
```

Đọc bằng `xml.etree.ElementTree` của thư viện chuẩn - không thêm phụ thuộc.

Ba quy tắc:

- **Đa giác là vành MỞ**: không lặp lại đỉnh đóng. Nếu file lặp, parser cắt bỏ,
  vì `core/` giả định vành mở và một đỉnh trùng lặp tạo ra cạnh dài 0.
- **Gộp là NỐI THÊM, không thay thế.** `safezones` và obstacles của request đứng
  trước, của bản đồ nền đứng sau. Với `safezones` thì planner lấy HỢP của chúng,
  nên thêm một safezone là **nới rộng** vùng bay chứ không thu hẹp - điều này
  phải nói rõ trong tài liệu vận hành, vì trực giác thường ngược lại.
- **`version` không khớp là lỗi**, không phải cảnh báo.

## 6. Vòng đời request

```
DDS request --> service loop
  |- idl_version sai / hình học vô lý     --> PLAN_INVALID_REQUEST
  |- đang bận                             --> PLAN_BUSY
  |- use_preloaded_map nhưng không có map --> PLAN_INVALID_REQUEST
  `- PlanRunner.submit()
        |- forkserver tạo con (median 56 ms)
        |- con: build_scenario -> prepare_scenario -> plan_trajectory
        |        -> full_mission_path -> gửi PlanReply qua Pipe -> thoát
        `- quá hạn --> kill(SIGKILL) --> PLAN_TIMEOUT
DDS reply  <---'
```

### Ba tầng thời hạn

1. `config.TIME_BUDGET_S` - planner tự dừng, êm, đặt `budget_bound = true`.
2. Thời hạn của `PlanRunner`, `= config.TIME_BUDGET_S + 2 s` - cứng, `SIGKILL`.
3. Thời hạn của client trên DDS - ngoài phạm vi service, phải lớn hơn (2).

Tầng 2 tồn tại vì planner không hủy được từ bên ngoài. Chi phí của nó bằng 0 ở
đường thường: tiến trình con vẫn được tạo cho mọi request, giết chỉ là bỏ chờ.

## 7. Ba cơ chế cưỡng chế "tự động cập nhật"

Nguyên tắc: adapter chỉ gọi, không sao chép.

**Cơ chế 1 - test hợp đồng khoá.** Đọc `core.types.Scenario` qua
`typing.get_type_hints` và so với tập khoá adapter điền. Thêm một khoá bắt buộc
bên đó thì test đỏ ngay, thay vì `KeyError` lúc chạy thật.

**Cơ chế 2 - test tương đương.** Hai khẳng định tách bạch, và việc tách là có lý
do:

- *Adapter trong suốt*: đường bay qua `vtx_service.plan()` phải **bit-identical**
  với việc gọi thẳng thuật toán TRÊN CÙNG dict `Scenario`. Cả hai vế gọi thuật
  toán hiện hành, nên test không bao giờ lỗi thời.
- *Không mất mission*: 18 preset qua service phải giải được hết và không dài hơn
  0,5%. KHÔNG đòi bit-identical, vì preset mang `map_bounds` mà IDL cố tình bỏ
  (mục 4.2). Ép hai thứ khác nhau phải giống nhau là một test nói dối.

**Cơ chế 3 - test ranh giới.** `git diff --stat main -- core/ render/ config.py`
phải rỗng.

## 8. Stack DDS: chưa chốt, và lớp transport được cô lập vì thế

Đã đo:

| | Fast DDS Python | Cyclone DDS Python |
| --- | --- | --- |
| Trên PyPI | **Không** (thử `fastdds`, `fastdds-python`, `eprosima-fastdds`) | **Có**, wheel binary 7,7 MB gói sẵn core |
| Cài đặt | colcon: Fast-CDR + Fast-DDS + Fast-DDS-python (SWIG) + fastddsgen (cần Java) | `pip install cyclonedds` |
| Sinh mã từ IDL | `fastddsgen -python`, compile lại mỗi lần đổi IDL | **Không cần**: kiểu khai báo bằng dataclass |
| Cùng bản với hệ thống gọi | Chắc chắn | RTPS 2.x, **phải chứng minh** |
| Đã chạy thử ở đây | Chưa cài được gì | **Rồi**: participant, sequence lồng, `@key` 16 byte, double bit-identical |

Máy này chưa có Fast DDS, fastddsgen hay ROS 2.

Rủi ro của Cyclone là **interop giữa hai bản cài DDS khác nhau**. Cả hai đều nói
RTPS 2.x nên trên nguyên tắc chạy được, nhưng type consistency của XTypes, tên
kiểu và cấu hình discovery là những chỗ hay vênh. Đây là thứ phải chứng minh
bằng spike với hệ thống thật, không phải thứ suy ra từ lý thuyết.

**Vì thế lớp transport là một module cô lập sau một interface hẹp**
(`Transport.serve(handler)`), và spike là task đầu tiên. Kết quả spike chỉ ảnh
hưởng một file; mọi thứ khác - hợp đồng dữ liệu, adapter, PlanRunner, bản đồ,
toàn bộ test - độc lập với nó và làm được song song.

## 9. Kiểm thử

| Tầng | Chạy gì | Bắt được gì |
| --- | --- | --- |
| 1. Hợp đồng | `get_type_hints` vs khoá adapter điền | khoá bắt buộc mới trong `Scenario` |
| 2. Tương đương | 18 preset, bit-identical + không mất mission | sai lệch adapter, đơn vị, tên tham số |
| 3. Góc | phương vị <-> heading, dải giá trị, chiều quay | lỗi quy ước hướng |
| 4. Bản đồ XML | parse, vành mở, gộp nối thêm, version sai | lỗi định dạng, lỗi ngữ nghĩa gộp |
| 5. PlanRunner | con treo -> `PLAN_TIMEOUT` và service vẫn phục vụ tiếp | lỗi tầng thời hạn 2 |
| 6. Transport | round-trip qua DDS thật, so với gọi trong tiến trình | lỗi dịch kiểu, tương quan, QoS |
| 7. Ranh giới | `git diff --stat main -- core/ render/ config.py` rỗng | vi phạm ràng buộc 1 |

Tầng 1-5 và 7 chạy được không cần DDS.

**Baseline hiện có phải giữ nguyên:** `pytest -q tests/` = 188 passed, 6 failed
(2026-08-21). Sáu ca đỏ có từ trước và không liên quan.

## 10. Rủi ro đã biết

**Interop Cyclone <-> Fast DDS.** Rủi ro lớn nhất, và spike ở Task 1 tồn tại
chính vì nó. Nếu spike đỏ thì quay về Fast DDS Python binding - mất một buổi,
không mất cả kế hoạch.

**Payload lớn.** Sequence không giới hạn: bản đồ nhiều đa giác có thể vượt
ngưỡng phân mảnh UDP mặc định. Phải đo với bản đồ thật trước khi chốt cấu hình
transport.

**Tính không tất định.** Đã phơi ra bằng `budget_bound`, nhưng tài liệu phải nói
rõ với client rằng service *không* phải một hàm thuần.

**`fork` và thread DDS.** Đã tránh bằng forkserver-trước-DDS (mục 3). Nếu ai đó
sau này đổi sang `fork` trần vì nó nhanh hơn 40 ms, đây là chỗ ghi lại vì sao
không nên.

**Ngữ nghĩa gộp safezone.** Thêm safezone là NỚI RỘNG vùng bay, không thu hẹp.
Trực giác thường ngược, nên nó phải nằm trong tài liệu vận hành chứ không chỉ
trong docstring.
