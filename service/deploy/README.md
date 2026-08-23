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

## Những gì service CHƯA làm

- **`time_budget_s` và `max_iterations` trong request bị bỏ qua.** Service dùng
  `config.TIME_BUDGET_S` / `config.MAX_ITERATIONS`. Reply mang
  `applied_time_budget_s` và `stats.max_iterations` là giá trị thật đã dùng.
- **Chỉ hệ toạ độ Oxy phẳng, mét.** Không WGS84.
- **Một request tại một thời điểm.** Bận thì trả `PLAN_BUSY`.

## Chẩn đoán

| triệu chứng | nguyên nhân thường gặp |
| --- | --- |
| Client không nhận reply nào | Sai `--domain-id`, hoặc discovery bị chặn. Kiểm tra log "sẵn sàng trên domain". |
| Mọi reply là `PLAN_INVALID_REQUEST` | `idl_version` lệch: client và service build từ hai bản IDL khác nhau. |
| `PLAN_INVALID_REQUEST` kèm "preloaded map" | Client đặt `use_preloaded_map` nhưng service khởi động không có `--preloaded-map`. |
| `PLAN_TIMEOUT` lặp lại | Bản đồ quá khó cho `config.TIME_BUDGET_S`, hoặc máy quá tải. Xem `stats.budget_bound` trên các reply thành công. |
| Đường bay đúng độ dài nhưng sai hướng 90 độ | Quy ước phương vị. Trên dây LUÔN là phương vị thật, thuận kim đồng hồ từ bắc, `+y` bắc. |
| Service treo cứng sau một thời gian chạy | Nghi ngờ đầu tiên: có ai đó đổi `PlanRunner` sang `fork` trần, hoặc đảo thứ tự `runner.start()` và khởi tạo DDS. Xem mục 3 của spec. |
| Reply thiếu mẫu tin với bản đồ lớn | Phân mảnh UDP; cần chỉnh cấu hình transport của binding DDS. |
