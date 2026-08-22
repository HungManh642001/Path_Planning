# Quyết định: stack DDS cho service path-planning

Ngày: 2026-08-22
Nhánh: `feature/dds-service`
Trạng thái: **TẠM THỜI (PROVISIONAL)** — chờ đo interop với hệ thống Fast DDS
thật của chủ sở hữu. Task 10 (transport DDS) vẫn triển khai được ngay trên
quyết định này vì lớp transport được cô lập sau một interface hẹp
(`DdsTransport(domain_id)` — xem spec thiết kế); nếu interop hỏng, chi phí
đổi lại là một file, không phải viết lại service.

## Quyết định

**Dùng Cyclone DDS (`cyclonedds` trên PyPI) cho service, tạm thời.**

Lý do KHÔNG phải "Fast DDS Python không dựng được" — spike này chứng minh
điều ngược lại (xem bên dưới). Lý do là **chi phí vận hành lâu dài**: Cyclone
là một dòng `pip install`, một wheel manylinux 7,7 MB gói sẵn core library,
không cần compiler hay toolchain C++ ở máy đích, và tự động đi theo mọi lần
`pip install -r requirements.txt` trong tương lai. Fast DDS Python không có
wheel trên PyPI; mọi máy chạy service (dev, CI, production) phải tự dựng lại
đúng chuỗi 5 gói C++ theo thứ tự phụ thuộc (Fast-CDR → foonathan_memory_vendor
→ Fast-DDS → Fast-DDS-python), điều mà spike đo được là làm được nhưng đòi hỏi
người vận hành biết chẩn đoán lỗi CMake/linker, không phải chỉ gõ lệnh cài đặt.

## Số đo cả hai phương án

### Cyclone DDS

| | |
| --- | --- |
| Cài đặt | `pip install cyclonedds` — MỘT lệnh |
| Kích thước | wheel 7,7 MB, gói sẵn core library, không cần biên dịch |
| Toolchain cần | không — thuần Python + wheel binary |
| Chạy thử ở đây | `service/spike/cyclone_probe.py`, hai vai `listen`/`send`, xem log thật bên dưới |

Log thật (venv có sẵn `cyclonedds==11.0.1`, domain 91,
`python service/spike/cyclone_probe.py listen` chạy nền, sau đó `... send`):

```
đã gửi 123456.78901234567

request_id  : True
detail      : 'first W1..W2 l=7421.3 < L0=8000'
đỉnh đảo    : 3
double khớp : True
```

Đúng cả bốn kết quả kỳ vọng: `@key` 16 byte round-trip đúng, chuỗi UTF-8 có
dấu tiếng Việt nguyên vẹn, `sequence<Polygon>` lồng đúng số đỉnh, `double`
khớp bit-for-bit.

**Một trục trặc khi transcribe (không phải lỗi Cyclone, lỗi tương tác giữa
`from __future__ import annotations` và cách Cyclone resolve kiểu generic):**
brief gốc có `from __future__ import annotations` ở đầu file. Với dòng đó,
tạo `Topic` ném:

```
TypeError: Type array[uint8, 16] as used in __main__ cannot be resolved.
```

vì với postponed evaluation, annotation của `request_id` trở thành CHUỖI
`"array[uint8, 16]"`, và cơ chế resolve kiểu của cyclonedds cố
`getattr(module, "array[uint8, 16]")` — không phải một tên hợp lệ. Đã bỏ dòng
`from __future__ import annotations` khỏi `cyclone_probe.py` đã ship (ghi chú
trong docstring của file) — sau đó chạy đúng như log ở trên. Ghi lại ở đây vì
đây là gotcha thật, đo được, không phải giả thuyết.

### Fast DDS Python (build từ nguồn)

| | |
| --- | --- |
| Cài đặt | KHÔNG có trên PyPI — phải build từ nguồn, 5 repo |
| Kích thước | 263 MB cây nguồn + build, 28 MB prefix cài đặt cục bộ |
| Toolchain cần | cmake ≥ 3.22 (hệ thống chỉ có 3.20, phải vá), SWIG < 4.2, g++, JRE (chỉ nếu cần `fastddsgen`) |
| Chạy thử ở đây | build thành công, `import fastdds` + tạo/huỷ `DomainParticipant` thành công (xem `service/spike/fastdds_probe.md` để có log đầy đủ) |

Tổng thời gian đo được: **~18 phút** (không chạm ngân sách 2 giờ của brief),
qua ba vướng mắc, cả ba vá được KHÔNG CẦN sudo:

1. cmake hệ thống 3.20.0 < yêu cầu 3.22 → `pip install cmake==3.26.4` vào venv.
2. Không có SWIG đúng version (< 4.2) → `conda create -n fastdds_tmp swig=4.0.2`
   (env cô lập, không đụng `base`).
3. `.so` build xong nhưng `import fastdds` báo
   `GLIBCXX_3.4.30' not found` — libstdc++ đi kèm Anaconda cũ hơn bản hệ
   thống mà Fast DDS được biên dịch bằng → vá bằng
   `LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6`.

Không cần `sudo apt install libasio-dev libtinyxml2-dev` như hướng dẫn chính
thức — cờ `-DTHIRDPARTY=ON` khiến Fast-DDS tự kéo `asio`/`tinyxml2` qua
submodule sẵn có trong repo.

**Đây KHÔNG phải kết luận "Fast DDS Python khả thi để ship như hiện trạng".**
Đây là build TỪ NGUỒN, một lần, trên một máy cụ thể — không phải một artefact
`pip install` được và tái lập trên máy khác. Không viết full
publisher/subscriber tương đương `cyclone_probe.py` (không tạo TypeSupport từ
IDL) — xem `fastdds_probe.md` mục "Việc đã KHÔNG thử" để biết vì sao dừng ở
đó có chủ đích (câu hỏi trung tâm là interop, không phải "dựng có nhanh
không" — điều đó đã có câu trả lời).

## Khoảng trống interop — CHƯA đo được, đây là rủi ro thật của quyết định

**Đây là câu hỏi trung tâm của spike (Step 4 trong brief) và KHÔNG đo được
trong phiên này** — cần hệ thống Fast DDS thật của chủ sở hữu, không truy cập
được từ máy này. Không giả lập, không đoán kết quả.

Để đóng khoảng trống này, chủ sở hữu (hoặc ai có quyền truy cập hệ thống Fast
DDS thật) cần chạy:

```bash
# Trên máy CÓ quyền truy cập hệ thống Fast DDS thật, đúng domain_id thật:
python service/spike/cyclone_probe.py listen   # sửa DOMAIN = <domain_id thật>
                                                # và tên Topic/kiểu khớp phía
                                                # Fast DDS đang publish, nếu
                                                # khác VtxProbe/Probe
```

rồi để hệ thống Fast DDS publish một sample lên đúng topic/domain đó, và quan
sát MỘT trong ba khả năng (brief đã liệt kê, nhắc lại để không bỏ sót khi đo):

1. **Nhận được, dữ liệu đúng** → Cyclone khả thi cho service, quyết định này
   được xác nhận, gỡ trạng thái TẠM THỜI.
2. **Discovery khớp nhưng không có mẫu tin** → nhiều khả năng lệch tên kiểu
   (`typename=` phải khớp CHÍNH XÁC chuỗi IDL bên Fast DDS dùng) hoặc XTypes
   (Fast DDS mặc định bật Complete TypeObject, Cyclone có thể cần cấu hình
   tương ứng) → cần log discovery cả hai phía trước khi kết luận.
3. **Không khớp gì (không thấy nhau ở tầng discovery)** → kiểm cấu hình
   discovery cả hai phía (multicast có bị chặn, domain_id có đúng, initial
   peers nếu dùng unicast) trước khi kết luận bất cứ điều gì về khả năng
   tương thích của bản thân giao thức.

**Vì spike này đã chứng minh Fast DDS Python build được trong ~18 phút (không
phải một nỗ lực nhiều giờ), rủi ro của việc chọn Cyclone rồi phải quay đầu
THẤP hơn so với giả định ban đầu**: nếu bước đo interop ở trên cho kết quả (2)
hoặc (3) và không vá được nhanh, phương án dự phòng — build Fast DDS Python
tại chỗ và cho service dùng thẳng cùng implementation với hệ thống thật — vẫn
nằm trong tầm tay, đã có lộ trình đo sẵn trong `fastdds_probe.md`.

## Gotcha đã biết (bắt buộc đọc trước khi làm Task 10)

**Một DataWriter khớp (`matched`) với một DataReader thuộc CÙNG
DomainParticipant.** Đo được trên máy này: với một participant giữ cả writer
lẫn reader, `writer.get_publication_matched_status().current_count` trả về
`1` — nghĩa là một guard kiểu "chờ tới khi có peer" sẽ PASS ngay cả khi
KHÔNG có peer thật nào tồn tại. Đo lại lần nữa trong spike này để xác nhận
con số, script:

```python
p = DomainParticipant(92)
t = Topic(p, "R1TestTopic", Msg)
w = DataWriter(Publisher(p), t)
r = DataReader(Subscriber(p), t)
# ... discovery settle ...
w.get_publication_matched_status().current_count   # == 1, SAI nếu coi là "có peer thật"
```

Thêm `Policy.IgnoreLocal.Participant` vào QoS của cả writer và reader sửa
đúng:

```python
qos = Qos(Policy.IgnoreLocal.Participant)
w2 = DataWriter(Publisher(p), t2, qos=qos)
r2 = DataReader(Subscriber(p), t2, qos=qos)
# ...
w2.get_publication_matched_status().current_count  # == 0, ĐÚNG — không có peer khác participant
```

Task 10 phụ thuộc trực tiếp vào việc này: một `wait_for_service()` viết mà
không có `IgnoreLocal.Participant` sẽ PASS giả trong test/dev khi service và
client cùng process/participant, rồi treo thật ở request đầu tiên trên một
topology khác — đúng dạng lỗi tốn cả buổi để tìm ra.

## Điều gì sẽ khiến phải xét lại quyết định này

- **Bước đo interop ở trên cho kết quả (2) hoặc (3) và không vá được bằng
  cấu hình QoS/XTypes/discovery thông thường.** Đây là điều kiện chính để
  đảo quyết định — xem lộ trình dự phòng ở trên.
- Chủ sở hữu xác nhận hệ thống thật của họ dùng một tính năng Fast-DDS-only
  mà Cyclone không hỗ trợ (ví dụ: Fast DDS Discovery Server thay vì
  multicast SPDP thuần, Content-Filtered Topics phức tạp, hoặc Zero-Copy qua
  shared memory theo API riêng của Fast DDS).
- Yêu cầu vận hành đổi: nếu service PHẢI build từ nguồn dù sao (ví dụ đóng
  gói thành container riêng có kiểm soát toolchain), lợi thế "một dòng pip
  install" của Cyclone không còn quan trọng bằng việc dùng ĐÚNG implementation
  với hệ thống thật — khi đó nên đảo sang Fast DDS Python luôn, vì rủi ro
  interop bằng 0 theo định nghĩa (cùng một implementation).

## Lệnh cài đặt của stack thắng cuộc (Cyclone DDS)

```bash
pip install cyclonedds
```

Không cần thêm bước nào khác — không system package, không compiler, không
biến môi trường đặc biệt (khác với Fast DDS Python, cần `LD_PRELOAD` nếu chạy
qua một Python phân phối kèm libstdc++ riêng như Anaconda — xem gotcha ở
trên nếu sau này đảo sang Fast DDS).

QoS trio đã xác nhận chạy được ở đây, dùng làm điểm khởi đầu cho Task 10:

```python
Qos(
    Policy.Reliability.Reliable(duration(seconds=10)),
    Policy.History.KeepAll,
    Policy.Durability.Volatile,
    Policy.IgnoreLocal.Participant,   # xem gotcha ở trên — bắt buộc
)
```
