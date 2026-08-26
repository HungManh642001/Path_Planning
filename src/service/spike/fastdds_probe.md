# Spike: dựng Fast DDS Python binding — công sức đo được

Máy: WSL2 Ubuntu 22.04 (container/dev box này), 2026-08-22. Không có sudo
(`sudo -n true` → "a password is required"). Có mạng ra ngoài (github.com,
pypi.org đều truy cập được). Có sẵn: cmake 3.20.0 (hệ thống, quá cũ), g++
11.4.0, git, OpenJDK 11 (`java`/`javac`), Python 3.11.7 (anaconda), conda,
pip. KHÔNG có sẵn: Fast DDS C++, `fastddsgen`, ROS 2, SWIG, cmake ≥ 3.22
(đã biết trước khi bắt đầu — không đo lại các mục "không có sẵn").

**KẾT QUẢ: build THÀNH CÔNG và CHẠY ĐƯỢC**, trái với kỳ vọng ban đầu rằng
đây sẽ là nơi Ruling R7 cho dừng sớm. Không gặp blocker cứng nào (không cần
sudo, không thiếu mạng, không có lỗi build không vá được trong một dòng).
Mọi trở ngại đều là lệch phiên bản vá được cục bộ. Tổng thời gian thực đo
từ `git clone` đầu tiên đến khi `import fastdds; DomainParticipant()` chạy
OK: **~18 phút** (21:45:20 → 22:03:26), so với ngân sách 2 giờ của brief.

Quan trọng: **KHÔNG chạy `sudo apt install`** — máy này không có quyền
sudo, nên đã đi vòng mọi bước mà hướng dẫn chính thức của eProsima giả định
apt. Mọi gói thay thế đều cài vào prefix cục bộ (venv riêng / conda env
riêng / `CMAKE_INSTALL_PREFIX` cục bộ), không đụng vào hệ thống hay vào
Anaconda `base` (mọi `conda create -n <env-mới>` đều chạy `--dry-run`
trước để xác nhận không có gì bị "UPDATE" trong `base`).

Thư mục làm việc:
`/tmp/claude-1000/.../scratchpad/fastdds_build/` (mã nguồn 4 repo clone),
`/tmp/claude-1000/.../scratchpad/fastdds_install/` (prefix cài đặt cục bộ,
28 MB sau khi cài xong).

## Dòng thời gian thực đo

| bước | lệnh chính | thời gian | kết quả |
| --- | --- | --- | --- |
| clone Fast-CDR | `git clone --depth 1` | 5.0s | OK |
| clone Fast-DDS | `git clone --depth 1` | 10.7s | OK, 87 MB |
| clone foonathan_memory_vendor | `git clone --depth 1` | 1.5s | OK |
| clone Fast-DDS-python | `git clone --depth 1` | 1.7s | OK |
| clone Fast-DDS-Gen | `git clone --depth 1` | 2.7s | OK (KHÔNG được dựng — xem "Không thử" bên dưới) |
| Fast-CDR configure | `cmake ..` | 12.5s | OK |
| Fast-CDR build+install | `cmake --build . --target install -j8` | 3.1s | OK |
| foonathan_memory_vendor configure | `cmake ..` | 20.4s | OK ("foonathan_memory not found" là bình thường — vendor tự fetch qua ExternalProject) |
| foonathan_memory_vendor build+install | `cmake --build . --target install -j8` | 39.6s | OK |
| pip install cmake mới vào venv | `pip install cmake==3.26.4` | ~17s | OK — **vướng #1, vá 1 dòng** |
| Fast-DDS configure (`-DTHIRDPARTY=ON`) | `cmake ..` | 43.9s | OK — tự kéo `asio` 1.34.2 + `tinyxml2` qua submodule, không cần apt |
| Fast-DDS build+install | `cmake --build . --target install -j8` | **7m05s** (21:49:48 → 21:56:53) | OK, `libfastdds.so.3.6.2.0` |
| conda env riêng cho SWIG 4.0.2 | `conda create -n fastdds_tmp -y swig=4.0.2` | 11.6s | OK — **vướng #2, vá bằng env riêng** |
| Fast-DDS-python configure | `cmake ..` | 6.1s | OK, thấy SWIG 4.0.2 + Python 3.11.7 dev components |
| Fast-DDS-python build (SWIG codegen + compile `_fastdds_python.so`) | `cmake --build . --target install -j8` | **2m01s** (22:01:09 → 22:03:10) | OK — build đúng theo cách chính thức, KHÔNG hand-hack |
| `import fastdds` từ Python 3.11 (anaconda) | `python3.11 -c "import fastdds"` | tức thời | **LỖI lần đầu — vướng #3, GLIBCXX** |
| vá bằng `LD_PRELOAD` | `LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 python3.11 -c "..."` | tức thời | **OK — `DomainParticipant` tạo và huỷ thành công** |

## Ba vướng mắc thật, cả ba đều vá được KHÔNG CẦN sudo

### Vướng #1 — cmake hệ thống quá cũ

Fast-DDS yêu cầu `cmake_minimum_required(VERSION 3.22)` trở lên
(`CMakeLists.txt:18`), máy này có cmake hệ thống 3.20.0:

```
CMake Error at CMakeLists.txt:18 (cmake_minimum_required):
  CMake 3.22 or higher is required.  You are running version 3.20.0
```

Vá: `pip install cmake==3.26.4` vào venv riêng, dùng `$VENV/bin/cmake` thay
`cmake` hệ thống trong mọi bước sau. Không sudo, không build cmake từ
nguồn.

### Vướng #2 — không có SWIG, và cần đúng dải version

Fast-DDS-python cần SWIG **< 4.2** (khuyến nghị 4.1). Máy không có `swig`
nào (`which swig` → không thấy). Hướng dẫn chính thức gọi
`sudo apt install swig4.1` — không dùng được ở đây.

Vá: `conda create -n fastdds_tmp -y swig=4.0.2` (env MỚI, tách biệt hoàn
toàn khỏi `base` — xác nhận bằng `--dry-run`: chỉ 8 gói nhỏ, 1.4 MB, không
"UPDATE" gì trong `base`), trỏ `PATH` vào
`.../envs/fastdds_tmp/bin` lúc cấu hình CMake cho `fastdds_python`.
Không có đúng bản 4.1.x trên kênh `pkgs/main`, dùng 4.0.2 (vẫn < 4.2) —
CMake tìm thấy (`Found SWIG: ... found version "4.0.2"`) và chấp nhận.

### Vướng #3 — `.so` build xong nhưng KHÔNG import được: lệch `GLIBCXX`

Đây là vướng nghiêm trọng nhất, và nó xuất hiện SAU khi build báo "thành
công" — dạng lỗi dễ bị bỏ sót nếu spike dừng lại ở "build xong" mà không
thử `import`:

```
ImportError: /home/hungmanh/anaconda3/bin/../lib/libstdc++.so.6:
version `GLIBCXX_3.4.30' not found
(required by .../fastdds_install/lib/libfastdds.so.3.6)
```

Nguyên nhân: `libfastdds.so` được biên dịch bằng g++ 11.4.0 hệ thống, đòi
`GLIBCXX_3.4.30`; nhưng Python 3.11 của Anaconda tự mang theo một bản
`libstdc++.so.6` RIÊNG, cũ hơn (dừng ở `GLIBCXX_3.4.3x` thấp hơn), và trình
liên kết động ưu tiên thư viện này vì nó nằm cạnh interpreter Python
(`$CONDA_PREFIX/lib`), trước khi tìm trong `LD_LIBRARY_PATH`.

Vá: `LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6` — ép nạp bản
libstdc++ mới của HỆ THỐNG trước bản cũ của Anaconda. Sau đó:

```
fastdds module imported OK: .../fastdds_install/lib/python3.11/site-packages/fastdds/__init__.py
DomainParticipant created: <fastdds.DomainParticipant; proxy of ...>
cleanup OK
```

Đây là vướng đáng nhớ nhất cho vận hành thật: bất cứ ai chạy Python qua
Anaconda/conda (rất phổ biến) và build Fast DDS bằng compiler hệ thống sẽ
dính lỗi y hệt. Không phải lỗi của Fast DDS — là xung đột toolchain hai
runtime C++ khác nhau trên cùng máy.

## Việc đã KHÔNG thử, và vì sao

- **`fastddsgen` (JRE + Gradle, sinh code từ IDL)**: đã clone repo
  (`Fast-DDS-Gen`, 1.5 MB) nhưng KHÔNG build. Máy có sẵn OpenJDK 11
  (`java`/`javac`) nên JRE không phải là vướng — nhưng dựng `fastddsgen`
  cần Gradle wrapper tải thêm dependency, và giá trị đo được của bước này
  (tốc độ codegen IDL) không phải câu hỏi trung tâm của spike (đó là
  interop, Step 4). Dừng ở đây có chủ đích, không phải vì bị chặn.
- **Một chương trình publish/subscribe Python đầy đủ tương đương
  `cyclone_probe.py`** (tạo Topic với TypeSupport, DataWriter/DataReader
  thật, gửi–nhận một sample): KHÔNG viết. `DomainParticipant` tạo/huỷ được
  đã đủ để trả lời câu hỏi của Step 3 ("dựng được không, tốn bao lâu");
  viết full publisher cần thêm sinh TypeSupport (qua `fastddsgen` hoặc API
  dynamic-types), là một lượng việc riêng, và không đổi câu trả lời cho
  Step 4 (interop với hệ thống thật ở xa vẫn không đo được từ đây bất kể
  probe Fast DDS phía này hoàn thiện tới đâu).

## Dung lượng đĩa (đo lúc build xong, `du -sh`)

```
Mã nguồn 4 repo (fastdds_build/):        263M
  Fast-CDR                                12M
  Fast-DDS (gồm cả asio+tinyxml2 vendor)  214M
  Fast-DDS-Gen (chỉ clone, không build)   1.5M
  Fast-DDS-python                         28M
  foonathan_memory_vendor                7.6M
Prefix cài đặt cục bộ (fastdds_install/): 28M
conda env fastdds_tmp (chỉ để có SWIG):   25M
```
So với Cyclone DDS: một wheel PyPI 7.7 MB, `pip install cyclonedds`, không
cần compiler, không cần build tree.

## Kết luận công sức

**Build được, chạy được, trong ~18 phút — không cần dừng ở mốc 2 giờ của
brief, và không gặp blocker cứng theo định nghĩa của Ruling R7** (không có
gói hệ thống nào đòi sudo mà không vá được cục bộ; không có bước nào thiếu
mạng; không có lỗi build phải sửa nhiều hơn một dòng). Ba vướng mắc — cmake
cũ, thiếu SWIG đúng version, và xung đột `GLIBCXX` giữa libstdc++ hệ thống
với libstdc++ đi kèm Anaconda — đều vá được mà không cần quyền quản trị,
nhưng đòi hỏi biết chẩn đoán lỗi linker C++, không phải việc gõ một lệnh cài
đặt. Hướng dẫn "chuẩn" của eProsima (`sudo apt install ...`, quy trình
colcon workspace) không chạy thẳng một bước nào trên máy không sudo — mọi
bước ở trên là đường vòng tự dựng.

Điều này KHÔNG đổi kết luận cho service: đây là build TỪ NGUỒN trên một máy
cụ thể, không phải một artefact có thể `pip install` và tái lập trên máy
CI/production khác. Không có wheel; không có cách "chỉ cần khai báo trong
requirements.txt". Bất kỳ máy vận hành nào chạy service PHẢI lặp lại đúng
chuỗi build này (hoặc đóng gói kết quả build thành image/container riêng),
kèm vướng `GLIBCXX`/`LD_PRELOAD` nếu Python ở đó cũng là Anaconda. So với
Cyclone: MỘT dòng `pip install cyclonedds`, một wheel manylinux gói sẵn core
library, không cần compiler ở máy đích. Chênh lệch vận hành (không phải khả
thi-hay-không, mà là chi phí lặp lại và bảo trì lâu dài) vẫn rất lớn, đúng
hướng đã biết trước khi bắt đầu — chỉ khác là "khó dựng" hoá ra không có
nghĩa "không dựng được trong ngân sách hợp lý".
