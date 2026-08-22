# Thiết kế: đóng gói thuật toán path planning thành service Fast DDS

Ngày: 2026-08-22
Nhánh: `feature/dds-service`

## 1. Mục tiêu và phạm vi

Một hệ thống ngoài (C++, đã chạy Fast DDS) gửi một mission và nhận về danh sách
waypoint. Thuật toán hiện tại được bọc thành một service độc lập, giao tiếp qua
Fast DDS.

Trong phạm vi:

- Định nghĩa IDL cho cặp topic request/reply.
- Một node C++ cầu nối Fast DDS.
- Một worker Python bọc `core/` và trả về đường bay đầy đủ `O..T`.
- Quy đổi hệ toạ độ WGS84 <-> mặt phẳng mét.
- Đóng gói, triển khai bằng systemd, và bộ kiểm thử.

Ngoài phạm vi:

- Replan liên tục theo tick, streaming, hoặc nhiều mission song song.
- Bất kỳ thay đổi nào trong `core/`, `render/`, `config.py`.
- Port thuật toán sang C++.

### Ràng buộc do chủ sở hữu đặt ra

1. Không sửa thuật toán hiện tại. Toàn bộ mã mới nằm trong `service/`.
2. Làm trên nhánh riêng `feature/dds-service`.
3. Khi thuật toán thay đổi, service phải tự động đi theo, không cần sửa tay.

Ràng buộc 3 được cưỡng chế bằng một nguyên tắc và ba cơ chế: **adapter chỉ gọi,
tuyệt đối không sao chép**. Không copy công thức hình học, không copy TypedDict,
không hardcode danh sách hằng số. Ba cơ chế nằm ở mục 7.

## 2. Bối cảnh từ codebase

Bốn sự thật định hình thiết kế này.

**Điểm vào rất gọn.** Toàn bộ pipeline là
`prepare_scenario(scenario)` -> `plan_trajectory(preprocessed)` ->
`result['path']`, một list các `((x, y), heading)`. Hai dict shape đã có
TypedDict trong `core/types.py`, nên hợp đồng dữ liệu không phải phát minh lại.

**Planner đang ship là `core/kinodynamic_astar_v0.py`**, không phải
`kinodynamic_astar.py`. Service gọi v0.

**`success` nghĩa là oracle độc lập đã chấp nhận toàn bộ đường bay**, không phải
"search trả về cái gì đó". Hợp đồng "trả về nghĩa là bay được" đã có sẵn và
service kế thừa nguyên vẹn.

**Planner không tất định giữa các máy.** `config.TIME_BUDGET_S = 15` cắt theo
đồng hồ, nên cùng một input trên máy tải nặng có thể ra đường bay khác. Đây là
vấn đề hợp đồng của service, không chỉ là vấn đề đo đạc, và được phơi ra bằng
trường `budget_bound`.

Số đo tham chiếu, 18 preset trên v0 (2026-08-22, máy này):
median 16 ms, `scenario_18_reversed_approach_cluttered` 3,9 s, trần cứng 15 s.

**35 hằng số global.** `kinodynamic_astar_v0.py` đọc 35 hằng số từ `config.py`.
Chỉ 5 giá trị là tham số hàm của `prepare_scenario`
(`turn_radius`, `l0`, `dss`, `safe_margin`, `alpha_max_rad`). Đây là lý do chỉ
5+2 tham số được override theo request (mục 4).

## 3. Kiến trúc

```
Hệ thống gọi (C++, Fast DDS có sẵn)
  publish VtxPathPlanRequest / subscribe VtxPathPlanReply
        |
        |  DDS domain, 2 topic, tương quan bằng request_id
        v
vtx_planner_dds_node  (C++)
  - type support sinh từ IDL bằng fastddsgen
  - QoS, discovery, correlation, thời hạn cứng
  - không chứa logic hình học
        |
        |  Unix domain socket, khung length-prefixed, payload MessagePack
        v
vtx_planner_worker  (Python 3.11)
  - adapter: msgpack <-> dataclass, quy đổi đơn vị và hệ toạ độ
  - vtx_planner: API thuần  plan(PlanRequest) -> PlanReply
        |
        v
  core/preprocessing + core/kinodynamic_astar_v0 + core/mission   (không sửa)
```

### Vì sao tách hai tiến trình

Bên gọi đã có Fast DDS C++ và toolchain IDL, nên node C++ tái dùng được hạ tầng
đó và tránh phải build Fast DDS Python binding (không có trên PyPI, phải dựng
bằng colcon + SWIG).

Quan trọng hơn: planner là Python thuần, CPU-bound, và **không hủy được từ bên
ngoài một cách lịch sự** - nó chỉ kiểm tra ngân sách tại các điểm trong vòng lặp
search. Tiến trình tách rời cho node quyền `SIGKILL` và trả về một reply lỗi tử
tế thay vì để client treo.

### Vì sao Unix socket, không phải TCP

Hai tiến trình luôn ở cùng máy: chúng dùng chung cấu hình bản đồ và cùng một bản
thuật toán. Unix socket loại bỏ toàn bộ câu hỏi về cổng, tường lửa và bảo mật
mạng ở lớp này. Bề mặt mạng duy nhất của hệ thống là DDS.

### Cái cố tình không có

Không hàng đợi, không worker pool. Yêu cầu đã chốt là 1 request/lúc. Node xử lý
tuần tự và trả `PLAN_BUSY` khi đang bận: rõ ràng hơn một hàng đợi ẩn với độ trễ
không đoán được.

## 4. Hợp đồng dữ liệu

### Topic và QoS

Hai topic, `VtxPathPlanRequest` và `VtxPathPlanReply`, tương quan bằng
`request_id` do client sinh và service trả nguyên. Không dùng RPC-over-DDS vì
Fast DDS chưa ổn định phần này.

| Topic | Reliability | History | Durability |
| --- | --- | --- | --- |
| Request | RELIABLE | KEEP_ALL | **VOLATILE** |
| Reply | RELIABLE | KEEP_LAST(8) | **VOLATILE** |

`VOLATILE` là bắt buộc, không phải mặc định tuỳ tiện. `TRANSIENT_LOCAL` trên
topic request nghĩa là node service khởi động lại sẽ nhận và lập kế hoạch lại
một mission cũ đã hết hiệu lực. Một lệnh bay không được phép phát lại.

### IDL

```idl
module vtx { module planning {

enum Frame      { FRAME_LOCAL_METERS, FRAME_WGS84 };
enum PlanStatus { PLAN_OK, PLAN_NO_PATH, PLAN_START_LEG_BLOCKED,
                  PLAN_GOAL_LEG_BLOCKED, PLAN_ORACLE_REJECTED,
                  PLAN_INVALID_REQUEST, PLAN_TIMEOUT, PLAN_INTERNAL_ERROR,
                  PLAN_BUSY };

struct Point2D  { double x; double y; };
struct Polygon  { sequence<Point2D> vertices; };
struct Circle   { Point2D center; double radius_m; };

struct VehicleLimits {
  double turn_radius_m;  double l0_m;  double dss_m;
  double safe_margin_m;  double alpha_max_deg;
};

struct SearchBudget { double time_budget_s; unsigned long max_iterations; };

struct VtxPathPlanRequest {
  @key octet          request_id[16];
  unsigned long       idl_version;
  Frame               frame;
  Point2D             start;   double start_heading_deg;
  Point2D             goal;    double goal_heading_deg;
  boolean             goal_heading_free;
  sequence<Polygon>   islands;
  sequence<Circle>    dynamic_obstacles;
  sequence<Polygon>   safezones;
  boolean             use_preloaded_map;
  VehicleLimits       limits;
  SearchBudget        budget;
};

struct Waypoint { Point2D position; double heading_deg; };

struct SearchStats {
  unsigned long iterations;  unsigned long max_iterations;
  unsigned long open_set_size;  boolean search_failed;
  boolean budget_bound;
};

struct VtxPathPlanReply {
  @key octet          request_id[16];
  unsigned long       idl_version;
  PlanStatus          status;
  string              detail;
  sequence<Waypoint>  waypoints;
  double              path_length_m;
  double              plan_wall_time_s;
  SearchStats         stats;
  string              planner_version;
  string              config_hash;
};
}; };
```

`idl_version` bắt đầu ở 1 và tăng khi bố cục struct đổi. Node từ chối request có
`idl_version` không khớp bằng `PLAN_INVALID_REQUEST`, thay vì diễn giải sai
những byte lệch pha.

### Đơn vị và quy ước

**Khoảng cách: mét. Góc: độ, và luôn là phương vị thật, thuận chiều kim đồng hồ
từ chính bắc**, ở cả hai frame. `FRAME_LOCAL_METERS` quy ước `+y` là bắc, `+x`
là đông.

Thuật toán bên trong dùng radian, ngược chiều kim đồng hồ từ `+x`, nên quy đổi
là `theta = deg_to_rad(90 - bearing_deg)`, thực hiện tại đúng **một** hàm trong
adapter.

Một quy ước duy nhất trên dây, không phải mỗi frame một kiểu. Hai quy ước trên
cùng một trường sinh ra đúng loại lỗi mà mọi test hình học đều bỏ lọt: đường bay
lệch 90 độ hoặc bị gương vẫn là một đường bay hợp lệ.

`config.ALPHA_MAX` và `START_ANGLE_*` vốn đã lưu bằng độ và chỉ đổi sang radian
ở biên, nên giao diện đối ngoại theo cùng quy ước đó. Mọi trường góc mang hậu tố
`_deg`.

### Các quyết định trong IDL, và lý do

**`goal_heading_free` là cờ riêng, không phải giá trị canh gác.** Python dùng
`goal_heading = None` để chọn chế độ free-goal; IDL không có optional, và mã hoá
bằng NaN là mời gọi tai nạn. Khi cờ bật, `goal_heading_deg` bị bỏ qua hoàn toàn.

**Trả về đường bay đầy đủ `O..T`, không phải waypoint nội bộ.** Planner trả về
các waypoint đã tìm kiếm; điểm cất cánh `O` và mục tiêu `T` do
`core.mission.full_mission_path` ghép vào. Service gọi đúng hàm đó chứ không tự
ghép.

**Chỉ 5+2 tham số override được theo request.** Năm tham số trong
`VehicleLimits` là tham số hàm của `prepare_scenario`, an toàn tuyệt đối. Hai
tham số trong `SearchBudget` là hằng số global, xử lý ở mục 5. Ba mươi ba hằng
số còn lại cố định lúc triển khai.

Đã xác minh trên code chứ không phải suy đoán, vì đây là chỗ override dễ âm
thầm không có tác dụng: `prepare_scenario` ghi `l0` vào
`start_state['straight_length']` và `dss` vào `goal_state['engagement_distance']`
(`core/preprocessing.py:118`, `:154`, `:219`), còn planner đọc lại đúng hai khoá
đó và chỉ rơi về `config.L0`/`config.DSS` khi chúng vắng mặt
(`kinodynamic_astar_v0.py:212`, `:267`). `turn_radius` và `alpha_max_rad` nằm
thẳng trong dict preprocessed; `safe_margin` chỉ dùng lúc inflate. Cả năm đều
tới được planner mà không đụng tới global nào.

Ngược lại, hai tham số trong `SearchBudget` **không** có đường nào khác ngoài
global: `self.max_iterations = config.MAX_ITERATIONS` đọc lúc khởi tạo
(`:283`) và `budget_s = config.TIME_BUDGET_S` đọc trong vòng lặp search
(`:1017`). Đây chính là lý do chúng cần context manager ở mục 5 thay vì đi kèm
`VehicleLimits`.

**`map_bounds` cố tình không có trong IDL.** `Scenario` có khoá này, nhưng nó là
một hình chữ nhật `(w, h)` neo tại gốc toạ độ và test là `0 < x < w`, tức phụ
thuộc vào việc gốc toạ độ nằm ở đâu - một khái niệm không có nghĩa ổn định sau
phép chiếu (mục 6). `safezones` biểu diễn được đúng vùng đó và mạnh hơn hẳn, vì
là đa giác nên bất biến với phép tịnh tiến. Khi thiếu cả hai, `_in_bounds` trả
`True` (`kinodynamic_astar_v0.py:860`), nên bỏ trường này không mất tính năng
nào.

**`config_hash` báo cáo ngược cấu hình đã dùng.** Băm SHA-256 rút gọn của 35
hằng số mà `kinodynamic_astar_v0` thật sự đọc, liệt kê bằng cách quét module
`config` lúc chạy chứ không hardcode. Client nhờ đó luôn phân biệt được hai
đường bay khác nhau là do input khác hay do cấu hình planner khác - điều mà một
service dựng trên codebase nghiên cứu bắt buộc phải trả lời được.

**`budget_bound` là trường hạng nhất.** Che giấu tính không tất định sẽ khiến
client tin vào một sự đảm bảo không tồn tại. Phơi ra thì client biết khi nào nên
hỏi lại.

**`detail` giữ nguyên văn chuỗi từ oracle.** Enum bắt được `no_path`,
`start_leg_blocked`, `goal_leg_blocked`, nhưng `path_validation` còn trả về
những chuỗi có tham số như `segment 3 blocked (...)` hay
`first W1..W2 l=7421.3 < L0=8000`. Ép chúng vào enum là làm mất đúng phần thông
tin giúp chẩn đoán. Enum để máy rẽ nhánh, `detail` để người đọc.

### Ánh xạ `PlanStatus`

| Nguồn | Status |
| --- | --- |
| `success = True` | `PLAN_OK` |
| `failure_reason = "no_path"` | `PLAN_NO_PATH` |
| `failure_reason = "start_leg_blocked"` | `PLAN_START_LEG_BLOCKED` |
| `failure_reason = "goal_leg_blocked"` | `PLAN_GOAL_LEG_BLOCKED` |
| chuỗi khác từ oracle | `PLAN_ORACLE_REJECTED`, nguyên văn vào `detail` |
| worker quá hạn, bị `SIGKILL` | `PLAN_TIMEOUT` |
| worker chết bất thường / lỗi adapter | `PLAN_INTERNAL_ERROR` |
| request sai `idl_version` hoặc hình học vô lý | `PLAN_INVALID_REQUEST` |
| đang xử lý request khác | `PLAN_BUSY` |

Ánh xạ này dựa trên tập hằng chuỗi hiện có; nếu `core/` thêm một
`failure_reason` mới, nó rơi vào `PLAN_ORACLE_REJECTED` với nguyên văn trong
`detail` thay vì làm sập adapter. Đây là hành vi mong muốn: service suy giảm êm,
không vỡ.

## 5. Vòng đời request

```
DDS request --> node C++
                 |- idl_version sai / hình học vô lý --> PLAN_INVALID_REQUEST
                 |- đang bận                          --> PLAN_BUSY
                 `- msgpack --> worker Python
                                 |- quy đổi hệ toạ độ (nếu WGS84)
                                 |- dựng Scenario --> prepare_scenario
                                 |                --> plan_trajectory (v0)
                                 `- full_mission_path --> kết quả
                 <-- kết quả, hoặc quá hạn --> SIGKILL, respawn, PLAN_TIMEOUT
DDS reply   <----'
```

### Ba tầng thời hạn

Lồng nhau và có chủ đích:

1. `config.TIME_BUDGET_S` - planner tự dừng, êm, đặt `budget_bound = true`.
2. Thời hạn của node dành cho worker, `= time_budget_s + 2 s` - cứng, `SIGKILL`.
3. Thời hạn của client trên DDS - ngoài phạm vi service, nhưng phải lớn hơn (2).

Tầng 2 tồn tại vì planner không hủy được từ bên ngoài. Giết rồi spawn lại là
cách trung thực duy nhất; chi phí là ~1-2 s import, chấp nhận được vì đây là
đường hiếm.

### Worker là tiến trình sống lâu, không fork theo request

Hai knob duy nhất override được (`time_budget_s`, `max_iterations`) được đặt
rồi khôi phục trong `try/finally` bằng một context manager có test riêng. Với 1
request/lúc và một luồng, cách này an toàn; tầng `SIGKILL` đã lo phần hủy.

**Phương án đã cân nhắc và loại: fork-per-request.** Nó cách ly hoàn hảo 35
hằng số global và cho phép hủy tức thì, nhưng thêm một tầng cho một yêu cầu chưa
tồn tại. Đây là chỗ quay lại nếu sau này cần chạy song song - lúc đó `config`
global mới thật sự thành vấn đề.

### Bản đồ nạp sẵn

Khi `use_preloaded_map = true`, worker gộp bản đồ nền tĩnh (đọc từ một file
GeoJSON lúc khởi động, đường dẫn nằm trong cấu hình worker) với chướng ngại vật
trong request. Khi `false`, request là tự chứa hoàn toàn.

Mặc định triển khai là **không** nạp sẵn bản đồ nào: request tự chứa thì dễ
replay và dễ chẩn đoán hơn nhiều.

## 6. Quy đổi hệ toạ độ

`FRAME_LOCAL_METERS` đi thẳng, chỉ quy đổi góc.

`FRAME_WGS84` dùng phép chiếu phương vị cách đều (AEQD, qua `pyproj`) **neo tại
trung điểm start-goal**. Phép chiếu này bảo toàn chính xác khoảng cách theo
phương xuyên tâm từ tâm chiếu; sai số theo phương tiếp tuyến xấp xỉ
`(c/R)^2 / 6`, tức khoảng 0,03% (~65 m) ở mép một mission 500 km.

Với bán kính lượn 8 km và biên an toàn tính bằng km, sai số này chấp nhận được -
nhưng nó là con số phải ghi vào tài liệu và phải có test cận trên, không phải
giấu đi.

UTM bị loại: múi UTM rộng 6 độ (~670 km ở xích đạo), nên một mission 500 km có
thể cắt qua hai múi.

### Tịnh tiến về góc phần tư dương

AEQD neo tại trung điểm sinh ra toạ độ **quanh gốc 0, tức khoảng một nửa là số
âm**. Thuật toán chấp nhận được điều đó trong cấu hình mặc định, vì `_in_bounds`
trả `True` khi không có `safezones` lẫn `map_bounds`. Nhưng phép kiểm tra hình
chữ nhật của nó là `0 < x < w and 0 < y < h`
(`core/kinodynamic_astar_v0.py:863`) - neo tại gốc và đòi hỏi dương ngặt - nên
toạ độ âm là một cái bẫy đang chờ: chỉ cần sau này có ai đó thêm bound là toàn
bộ mission bị loại sạch, và triệu chứng sẽ là `no_path` chứ không phải một lỗi
nói thật.

Vì vậy adapter, sau khi chiếu, **tịnh tiến toàn bộ hình học sao cho hộp bao nằm
trọn trong góc phần tư dương** với một khoảng đệm, rồi tịnh tiến ngược waypoint
trên đường ra. Phép tịnh tiến không đổi khoảng cách hay phương vị nên không ảnh
hưởng gì tới đường bay; nó chỉ đưa mission về đúng quy ước toạ độ mà thuật toán
ngầm giả định.

Ở `FRAME_LOCAL_METERS` adapter không tịnh tiến - client đang nói bằng chính hệ
toạ độ của thuật toán - nhưng tài liệu phải nêu rõ quy ước gốc toạ độ ở góc tây
nam và toạ độ dương.

Phép chiếu là một component riêng, thuần hàm, test được độc lập.

## 7. Ba cơ chế cưỡng chế "tự động cập nhật"

Nguyên tắc: adapter chỉ gọi, không sao chép. Ba cơ chế biến nguyên tắc thành
thứ kiểm tra được.

**Cơ chế 1 - test hợp đồng khoá.** Đọc `core.types.Scenario` và
`PreprocessedScenario` qua `typing.get_type_hints` và so với tập khoá adapter
điền. Thêm một khoá bắt buộc vào `Scenario` mà adapter chưa biết thì test đỏ
ngay, thay vì `KeyError` lúc chạy thật.

**Cơ chế 2 - test tương đương.** Chạy 18 preset qua `vtx_planner.plan()` và so
với việc gọi trực tiếp `prepare_scenario` + `plan_trajectory`. Waypoint phải
**bit-identical**.

Đây là cơ chế cốt lõi. Cả hai vế của phép so đều gọi thuật toán *hiện hành*, nên
test không bao giờ lỗi thời: thuật toán đổi thì cả hai vế đổi cùng nhau và test
vẫn xanh; adapter lệch khỏi thuật toán thì đỏ ngay. Nó bắt được đúng loại thay
đổi từng xảy ra trong repo này - `prepare_scenario` đã có lần đổi tham số từ
`R=/L0=/DSS=` sang `turn_radius=/l0=/dss=`.

**Cơ chế 3 - test ranh giới.** `git diff --stat core/ render/ config.py` phải
rỗng trên mọi commit của nhánh này.

## 8. Đóng gói và triển khai

**Không Docker.** Lý do chính là lý do đã chọn kiến trúc này: node C++ phải build
với đúng bản Fast DDS mà hệ thống gọi đang dùng. Đóng nó vào container là mời
lại đúng vấn đề lệch phiên bản mà thiết kế sinh ra để tránh.

```
service/
  idl/vtx_path_planning.idl        nguồn duy nhất của hợp đồng
  dds_node/    CMakeLists.txt, src/
  worker/      vtx_planner/        package Python, không biết DDS là gì
  deploy/      vtx-planner.service, worker-requirements.txt, README.md
  tests/
```

Worker chạy trong venv riêng, ghim `numpy==1.26.4` theo đúng ghi chú trong
`requirements.txt` gốc, và import `core.*` từ chính repo này qua `PYTHONPATH`.

Node C++ **tự spawn worker** như tiến trình con, nên vòng đời gắn liền và node
giữ quyền giết/dựng lại. systemd chỉ quản một unit.

Việc import thẳng từ repo, không đóng gói `core/` thành wheel, **chính là cơ chế
cập nhật tự động ở mức triển khai**: nâng cấp thuật toán là
`git pull && systemctl restart vtx-planner`. `planner_version` trong reply
(`git describe --always --dirty`) nói đúng bản nào đang chạy, và `config_hash`
nói đúng cấu hình nào đã sinh ra đường bay.

## 9. Kiểm thử

| Tầng | Chạy gì | Bắt được gì |
| --- | --- | --- |
| 1. Hợp đồng | `get_type_hints` vs khoá adapter điền | khoá bắt buộc mới trong `Scenario` |
| 2. Tương đương | 18 preset, `vtx_planner.plan()` vs gọi trực tiếp, bit-identical | sai lệch adapter, đơn vị, tên tham số |
| 3. Round-trip DDS | node + worker thật, request qua loopback | lỗi IDL, tương quan `request_id`, QoS |
| 4. Phép chiếu | xuôi-ngược < 1e-6 m; cận biến dạng mission 500 km; mọi toạ độ sau chiếu đều dương | lỗi hệ toạ độ, lỗi quy ước phương vị, toạ độ âm |
| 5. Ranh giới | `git diff --stat core/ render/ config.py` rỗng | vi phạm ràng buộc 1 |
| 6. Thời hạn | worker giả treo -> node trả `PLAN_TIMEOUT` và respawn | lỗi tầng thời hạn 2 |

Tầng 1, 2, 4 chạy được không cần DDS. Chỉ tầng 3 và 6 cần node đã build.

Bộ test của service nằm ở `service/tests/`, tách khỏi `tests/` ở gốc (vốn nằm
trong `.gitignore` và chỉ được track bằng force-add).

**Baseline hiện có phải giữ nguyên:** `pytest -q tests/` = 188 passed, 6 failed
(2026-08-21). Sáu ca đỏ đó có từ trước và không liên quan; công việc này không
được thêm ca đỏ nào.

## 10. Rủi ro đã biết

**Payload lớn.** Sequence không giới hạn trong IDL: một bản đồ nhiều đa giác có
thể vượt ngưỡng phân mảnh UDP mặc định. Cần bật publish bất đồng bộ và flow
controller trên writer, hoặc nâng `max_message_size`. Phải đo với bản đồ thật
trước khi chốt cấu hình transport.

**Tính không tất định.** Đã phơi ra bằng `budget_bound`, nhưng client phải được
tài liệu hoá là *không* nên coi service như một hàm thuần.

**Sai lệch phiên bản Fast DDS.** Node phải build với cùng bản Fast DDS như bên
gọi. Ghi lại phiên bản trong `service/deploy/README.md`, và in nó lúc node khởi
động.

**Chi phí import khi respawn.** ~1-2 s sau mỗi `PLAN_TIMEOUT`. Nếu timeout trở
nên thường xuyên chứ không hiếm, đó là tín hiệu phải xem lại ngân sách chứ không
phải tối ưu đường respawn.
