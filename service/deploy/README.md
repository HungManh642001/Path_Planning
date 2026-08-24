# Triển khai VTX path planning service

Service Python độc lập, tự khởi động cùng máy. Đóng gói phân phối sẽ phân tích
riêng; phần này là bản cài trực tiếp.

## Cài đặt

```bash
sudo useradd --system --home /opt/vtx --shell /usr/sbin/nologin vtx

sudo git clone <repo> /opt/vtx/path_planning
cd /opt/vtx/path_planning && sudo git checkout <tag>

sudo python3.11 -m venv /opt/vtx/venv
sudo /opt/vtx/venv/bin/pip install -r service/deploy/requirements.txt

sudo chown -R vtx:vtx /opt/vtx
sudo cp service/deploy/vtx-planner.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now vtx-planner
```

`enable` là phần "tự khởi động cùng Ubuntu". Kiểm tra:
`journalctl -u vtx-planner -f`.

## Phụ thuộc

**Hai gói**: `shapely` và một binding DDS. Không numpy trực tiếp, không scipy,
không matplotlib, không pyproj. Xem `requirements.txt` để biết vì sao cái pin
`numpy==1.26.4` ở gốc repo không áp dụng ở đây.

## `PYTHONPATH`

`vtx-planner.service` đặt `PYTHONPATH=/opt/vtx/path_planning:.../service` và
NÊN tiếp tục làm vậy - tiến trình top-level (`python -m vtx_service.main`) vẫn
cần nó để tìm ra package `vtx_service`/`core` khi khởi động.

Nhưng service KHÔNG còn PHỤ THUỘC vào biến này để tiến trình con lập kế hoạch
import đúng: `PlanRunner.start()` tự đảm bảo `PYTHONPATH` chứa gốc repo và
`service/` (nối thêm, không thay thế) trước khi tạo tiến trình `forkserver` -
xem `runner.py::_ensure_pythonpath_for_forkserver`. Trước bản sửa này, ai đó
chẩn đoán bằng cách chạy module tay (vd. `python -m vtx_service.main` từ một
checkout không đặt `PYTHONPATH`, hoặc gọi qua một harness sửa `sys.path` lúc
chạy như pytest hay làm) sẽ đo được một service **chậm hơn ~50x** so với bản
triển khai thật - KHÔNG lỗi, KHÔNG cảnh báo, chỉ chậm âm thầm: `forkserver` là
fork+exec một interpreter mới, đọc `PYTHONPATH` từ biến môi trường chứ không
thấy `sys.path` cha thêm lúc chạy, và khi import `_PRELOAD` thất bại,
`multiprocessing` nuốt lỗi im lặng rồi để mỗi tiến trình con tự import lại mọi
thứ - kể cả `git describe` mà `runtime.py` đáng lẽ chỉ trả một lần. Đo được:
`sys.path` sửa lúc chạy 3.52-4.05 s/request; `PYTHONPATH` đặt qua biến môi
trường 0.07-0.08 s/request.

## Nâng cấp thuật toán

```bash
cd /opt/vtx/path_planning && sudo git pull && sudo systemctl restart vtx-planner
```

Không build lại gì: service import `core.*` thẳng từ cây mã nguồn. Mỗi reply
mang `planner_version` (`git describe --always --dirty`) và `config_hash`, nên
client luôn phân biệt được hai đường bay khác nhau là do input khác hay do
phiên bản/cấu hình planner khác.

## Bản đồ nền

Tuỳ chọn. Thêm `--preloaded-map /opt/vtx/basemap.xml` vào `ExecStart`. Định
dạng ở `basemap.example.xml`.

**Ngữ nghĩa dễ hiểu ngược:** safezone của bản đồ nền được NỐI THÊM vào safezone
của request, và planner lấy HỢP của chúng. Thêm một safezone là **nới rộng**
vùng bay, không phải thu hẹp.

## Ngân sách thời gian

`budget.time_budget_s` trong request **được tôn trọng** (từ `idl_version` 2):
nó đi thẳng vào thuật toán làm điều kiện dừng **duy nhất** - không còn trần
theo số vòng lặp nào nữa.

| client gửi | service dùng |
| --- | --- |
| `<= 0`, hoặc không phải số hữu hạn | `config.TIME_BUDGET_S` (mặc định 15 s) |
| một giá trị hợp lệ | đúng giá trị đó |
| lớn hơn `runtime.MAX_REQUEST_TIME_BUDGET_S` (300 s) | bị kẹp xuống 300 s |

Reply luôn mang `applied_time_budget_s` là giá trị **thật** đã dùng, nên client
biết đề nghị của mình được nhận nguyên vẹn hay đã bị thay. Thời hạn cứng của
`PlanRunner` (SIGKILL cho tiến trình con) là ngân sách đó cộng `--grace-seconds`.

Trần 300 s không phải để bảo vệ một mission - nó bảo vệ **hàng đợi**: service
phục vụ tuần tự, nên ngân sách một client xin cũng là thời gian mọi client khác
phải chờ.

## Những gì service CHƯA làm

- **Chỉ hệ toạ độ Oxy phẳng, mét.** Không WGS84.
- **Một request tại một thời điểm.** Không có phát hiện "bận": `PLAN_BUSY` là giá trị RESERVED, không đường mã nào sinh ra nó. Vòng phục vụ tuần tự trên một reader `KEEP_ALL`, nên một request đến khi service đang bận được DDS (RELIABLE + KEEP_ALL) **xếp hàng** và trả lời sau, theo đúng thứ tự - không bị từ chối.

## Chẩn đoán

| triệu chứng | nguyên nhân thường gặp |
| --- | --- |
| Client không nhận reply nào | Sai `--domain-id`, hoặc discovery bị chặn. Kiểm tra log "sẵn sàng trên domain". |
| Mọi reply là `PLAN_INVALID_REQUEST` | `idl_version` lệch: client và service build từ hai bản IDL khác nhau. |
| `PLAN_INVALID_REQUEST` kèm "preloaded map" | Client đặt `use_preloaded_map` nhưng service khởi động không có `--preloaded-map`. |
| `PLAN_TIMEOUT` lặp lại | Bản đồ quá khó cho ngân sách đang áp dụng, hoặc máy quá tải. Xem `applied_time_budget_s` (ngân sách thật đã dùng) và `stats.budget_bound` trên các reply thành công. |
| Đường bay đúng độ dài nhưng sai hướng 90 độ | Quy ước phương vị. Trên dây LUÔN là phương vị thật, thuận kim đồng hồ từ bắc, `+y` bắc. |
| Service treo cứng sau một thời gian chạy | Nghi ngờ đầu tiên: có ai đó đổi `PlanRunner` sang `fork` trần, hoặc đảo thứ tự `runner.start()` và khởi tạo DDS. Xem mục 3 của spec. |
| Reply thiếu mẫu tin với bản đồ lớn | Phân mảnh UDP; cần chỉnh cấu hình transport của binding DDS. |
| `PLAN_INTERNAL_ERROR` | Tiến trình con ném lỗi, hoặc lỗi khi dịch/ghi reply. `detail` mang traceback rút gọn - được log lại ở mức WARNING trên chính service (`journalctl -u vtx-planner`), không chỉ gửi cho client. |
