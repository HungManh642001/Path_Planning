# VTX Path Planning Service — Part 2: node Fast DDS và triển khai

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Đưa worker Python của Part 1 ra ngoài qua Fast DDS bằng một node C++ mỏng, rồi đóng gói thành một unit systemd triển khai được, và một Dockerfile cho về sau.

**Architecture:** Node C++ sở hữu toàn bộ phần DDS: type support sinh từ IDL, hai topic request/reply với QoS đã chọn, tương quan bằng `request_id`. Nó tự spawn worker Python như tiến trình con và nói chuyện qua Unix socket bằng đúng giao thức msgpack có tiền tố độ dài của Part 1. Node giữ thời hạn CỨNG và quyền `SIGKILL`, vì một vòng lặp search Python không hủy được từ bên ngoài một cách lịch sự. Node không chứa một dòng hình học nào.

**Tech Stack:** C++17, Fast DDS (bản của hệ thống gọi), `fastddsgen`, CMake, `msgpack-c` (chỉ phần C++ header-only), systemd, Docker (giai đoạn 2).

**Spec:** `docs/superpowers/specs/2026-08-22-dds-path-planning-service-design.md`
**Tiền đề:** `docs/superpowers/plans/2026-08-22-dds-service-part1-python-worker.md` đã hoàn tất.

## Global Constraints

- **Không sửa `core/`, `render/`, `config.py`.** Test ranh giới ở Part 1 Task 1 cưỡng chế điều này và vẫn phải xanh.
- **`IDL_VERSION = 1`**, và các giá trị enum `PlanStatus` phải khớp CHÍNH XÁC `service/worker/vtx_planner/messages.py`: `OK=0, NO_PATH=1, START_LEG_BLOCKED=2, GOAL_LEG_BLOCKED=3, ORACLE_REJECTED=4, INVALID_REQUEST=5, TIMEOUT=6, INTERNAL_ERROR=7, BUSY=8`.
- **Đơn vị trên dây:** khoảng cách mét; góc là **độ, phương vị thật, thuận chiều kim đồng hồ từ chính bắc**, ở cả hai frame. `FRAME_LOCAL_METERS` quy ước `+y` bắc, `+x` đông.
- **QoS:** cả hai topic `RELIABLE` và **`VOLATILE`**. `TRANSIENT_LOCAL` trên topic request bị CẤM: nó khiến node khởi động lại nhận và lập kế hoạch lại một mission cũ đã hết hiệu lực. Một lệnh bay không được phép phát lại.
- **Ba tầng thời hạn:** `config.TIME_BUDGET_S` (planner tự dừng) < thời hạn node dành cho worker `= time_budget_s + 2 s` (cứng, `SIGKILL`) < thời hạn client.
- **Một request tại một thời điểm.** Không hàng đợi, không worker pool. Bận thì trả `PLAN_BUSY`.
- **Nhánh:** `feature/dds-service`.

---

## File Structure

```
service/
  idl/vtx_path_planning.idl      nguồn duy nhất của hợp đồng     (Task 10)
  dds_node/
    CMakeLists.txt                                               (Task 10)
    src/worker_client.hpp/.cpp   socket + spawn + SIGKILL        (Task 11)
    src/plan_service.hpp/.cpp    DDS pub/sub, QoS, tương quan    (Task 12)
    src/main.cpp                 phân tích tham số, vòng đời     (Task 12)
  tests/
    idl_contract_test.py         enum C++/Python phải khớp       (Task 10)
    worker_client_test.cpp       GoogleTest cho lớp socket       (Task 11)
    roundtrip_dds_test.py        end-to-end qua DDS thật         (Task 13)
  deploy/
    vtx-planner.service                                          (Task 14)
    README.md                                                    (Task 14)
    Dockerfile                                                   (Task 14)
```

---

### Task 10: IDL, sinh type support, và test khớp enum

**Files:**
- Create: `service/idl/vtx_path_planning.idl`
- Create: `service/dds_node/CMakeLists.txt`
- Test: `service/tests/idl_contract_test.py`

**Interfaces:**
- Consumes: `PlanStatus` từ `vtx_planner.messages` (Part 1 Task 1).
- Produces: các kiểu C++ `vtx::planning::VtxPathPlanRequest`, `VtxPathPlanReply`, `Point2D`, `Polygon`, `Circle`, `VehicleLimits`, `SearchBudget`, `Waypoint`, `SearchStats`, enum `Frame` và `PlanStatus`; target CMake `vtx_planner_types`. Task 11-13 dùng.

- [ ] **Step 1: Viết test khớp enum**

Create `service/tests/idl_contract_test.py`:

```python
"""IDL và Python phải nói cùng một tập mã trạng thái.

Một enum lệch nhau ở đây không gây lỗi biên dịch, không gây lỗi lúc chạy, và
không gây lỗi trên dây. Nó chỉ khiến client đọc "hết thời gian" trong khi
service nói "tuyến đường bị chặn". Đây là loại sai lệch phải bị chặn bằng test
chứ không bằng sự cẩn thận.
"""

from __future__ import annotations

import re
from pathlib import Path

from vtx_planner.messages import IDL_VERSION, PlanStatus

IDL_PATH = Path(__file__).resolve().parents[1] / "idl" / "vtx_path_planning.idl"


def _idl_text() -> str:
    return IDL_PATH.read_text(encoding="utf-8")


def _enum_members(name: str) -> list[str]:
    match = re.search(rf"enum\s+{name}\s*\{{(.*?)\}}", _idl_text(), re.DOTALL)
    assert match, f"IDL không khai báo enum {name}"
    return [item.strip() for item in match.group(1).split(",") if item.strip()]


def test_idl_file_exists() -> None:
    assert IDL_PATH.is_file()


def test_plan_status_order_matches_python_exactly() -> None:
    expected = [f"PLAN_{member.name}" for member in PlanStatus]
    assert _enum_members("PlanStatus") == expected


def test_frame_enum_has_both_frames_in_order() -> None:
    assert _enum_members("Frame") == ["FRAME_LOCAL_METERS", "FRAME_WGS84"]


def test_every_angle_field_is_named_deg() -> None:
    """Quy ước đơn vị nằm trong TÊN trường, để không ai phải đoán."""
    text = _idl_text()
    angle_fields = re.findall(r"double\s+(\w*(?:heading|azimuth|bearing|alpha)\w*)\s*;", text)
    assert angle_fields, "không tìm thấy trường góc nào trong IDL"
    for field in angle_fields:
        assert field.endswith("_deg"), f"trường góc {field} thiếu hậu tố _deg"


def test_request_carries_an_idl_version_field() -> None:
    # Khớp theo mẫu, không theo khoảng trắng: một lần chỉnh căn lề trong IDL
    # không được phép làm đỏ một test về hợp đồng.
    assert re.search(r"unsigned\s+long\s+idl_version\s*;", _idl_text())
    assert IDL_VERSION == 1


def test_both_topic_types_are_keyed_on_request_id() -> None:
    keys = re.findall(r"@key\s+octet\s+request_id\[16\]\s*;", _idl_text())
    assert len(keys) == 2, "cả request lẫn reply phải có @key request_id"
```

- [ ] **Step 2: Chạy để xác nhận đỏ**

Run: `python -m pytest service/tests/idl_contract_test.py -v`
Expected: FAIL — `test_idl_file_exists` đỏ vì chưa có file.

- [ ] **Step 3: Viết IDL**

Create `service/idl/vtx_path_planning.idl`:

```idl
// Hợp đồng dữ liệu của service lập kế hoạch đường bay VTX.
//
// Đơn vị: khoảng cách MÉT. Góc là ĐỘ, và luôn là phương vị thật, thuận chiều
// kim đồng hồ từ chính bắc, ở CẢ HAI frame. FRAME_LOCAL_METERS quy ước +y là
// bắc, +x là đông.
//
// Các giá trị PlanStatus phải khớp chính xác thứ tự trong
// service/worker/vtx_planner/messages.py. service/tests/idl_contract_test.py
// cưỡng chế điều đó.
//
// Xem docs/superpowers/specs/2026-08-22-dds-path-planning-service-design.md

module vtx { module planning {

enum Frame      { FRAME_LOCAL_METERS, FRAME_WGS84 };

enum PlanStatus { PLAN_OK, PLAN_NO_PATH, PLAN_START_LEG_BLOCKED,
                  PLAN_GOAL_LEG_BLOCKED, PLAN_ORACLE_REJECTED,
                  PLAN_INVALID_REQUEST, PLAN_TIMEOUT, PLAN_INTERNAL_ERROR,
                  PLAN_BUSY };

struct Point2D  { double x; double y; };

// Vành mở: không lặp lại đỉnh đóng.
struct Polygon  { sequence<Point2D> vertices; };

struct Circle   { Point2D center; double radius_m; };

// Năm tham số duy nhất tới được planner qua đường tham số hàm; chúng ánh xạ
// 1-1 sang tham số của core.preprocessing.prepare_scenario. Mọi hằng số khác
// của planner là global, cố định lúc triển khai, và được báo cáo ngược về
// bằng config_hash trong reply.
struct VehicleLimits {
  double turn_radius_m;
  double l0_m;
  double dss_m;
  double safe_margin_m;
  double alpha_max_deg;
};

struct SearchBudget {
  double        time_budget_s;
  unsigned long max_iterations;
};

struct VtxPathPlanRequest {
  @key octet          request_id[16];
  unsigned long       idl_version;
  Frame               frame;
  Point2D             start;
  double              start_heading_deg;
  Point2D             goal;
  double              goal_heading_deg;
  // Khi true, goal_heading_deg bị bỏ qua hoàn toàn và planner tự chọn hướng
  // tiếp cận. Là cờ riêng chứ không phải giá trị canh gác: IDL không có
  // optional, và mã hoá bằng NaN là mời gọi tai nạn.
  boolean             goal_heading_free;
  sequence<Polygon>   islands;
  sequence<Circle>    dynamic_obstacles;
  sequence<Polygon>   safezones;
  boolean             use_preloaded_map;
  VehicleLimits       limits;
  SearchBudget        budget;
};

struct Waypoint {
  Point2D position;
  double  heading_deg;
};

struct SearchStats {
  unsigned long iterations;
  unsigned long max_iterations;
  unsigned long open_set_size;
  boolean       search_failed;
  // Search đã chạm trần thời gian hoặc trần vòng lặp. Trường hạng nhất chứ
  // không phải chi tiết ẩn: planner cắt theo đồng hồ, nên cùng một request
  // trên máy tải nặng có thể ra đường bay khác.
  boolean       budget_bound;
};

struct VtxPathPlanReply {
  @key octet          request_id[16];
  unsigned long       idl_version;
  PlanStatus          status;
  // Rỗng khi OK. Ngược lại là nguyên văn chuỗi từ oracle, kể cả những chuỗi
  // mang tham số như "first W1..W2 l=7421.3 < L0=8000". Enum để máy rẽ nhánh,
  // detail để người đọc.
  string              detail;
  // Đường bay ĐẦY ĐỦ O..T, không phải waypoint nội bộ của search.
  sequence<Waypoint>  waypoints;
  double              path_length_m;
  double              plan_wall_time_s;
  SearchStats         stats;
  string              planner_version;
  string              config_hash;
};

}; };
```

- [ ] **Step 4: Chạy test**

Run: `python -m pytest service/tests/idl_contract_test.py -v`
Expected: PASS, 6 passed.

- [ ] **Step 5: Viết CMakeLists**

Create `service/dds_node/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(vtx_planner_dds_node CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Fast DDS của MÁY ĐÍCH, không phải một bản đóng kèm. Đây là lý do kiến trúc
# chọn node C++ thay vì Python binding: tái dùng đúng toolchain và đúng phiên
# bản mà hệ thống gọi đang chạy.
find_package(fastcdr REQUIRED)
find_package(fastdds REQUIRED)

message(STATUS "Fast DDS: ${fastdds_VERSION}, Fast CDR: ${fastcdr_VERSION}")

# --- type support sinh từ IDL -------------------------------------------------
find_program(FASTDDSGEN fastddsgen REQUIRED)

set(IDL_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../idl/vtx_path_planning.idl)
set(GEN_DIR  ${CMAKE_CURRENT_BINARY_DIR}/gen)
set(GEN_SOURCES
  ${GEN_DIR}/vtx_path_planning.cxx
  ${GEN_DIR}/vtx_path_planningPubSubTypes.cxx
)

add_custom_command(
  OUTPUT ${GEN_SOURCES}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${GEN_DIR}
  COMMAND ${FASTDDSGEN} -replace -d ${GEN_DIR} ${IDL_FILE}
  DEPENDS ${IDL_FILE}
  COMMENT "fastddsgen: sinh type support từ vtx_path_planning.idl"
)

add_library(vtx_planner_types STATIC ${GEN_SOURCES})
target_include_directories(vtx_planner_types PUBLIC ${GEN_DIR})
target_link_libraries(vtx_planner_types PUBLIC fastdds fastcdr)

# --- node ---------------------------------------------------------------------
add_executable(vtx_planner_dds_node
  src/main.cpp
  src/plan_service.cpp
  src/worker_client.cpp
)
target_include_directories(vtx_planner_dds_node PRIVATE src)
target_link_libraries(vtx_planner_dds_node PRIVATE vtx_planner_types)

# --- test ---------------------------------------------------------------------
option(VTX_BUILD_TESTS "Build the C++ unit tests" ON)
if(VTX_BUILD_TESTS)
  find_package(GTest QUIET)
  if(GTest_FOUND)
    add_executable(worker_client_test ../tests/worker_client_test.cpp src/worker_client.cpp)
    target_include_directories(worker_client_test PRIVATE src)
    target_link_libraries(worker_client_test PRIVATE GTest::gtest_main)
    enable_testing()
    add_test(NAME worker_client_test COMMAND worker_client_test)
  else()
    message(STATUS "GTest không có; bỏ qua test C++")
  endif()
endif()
```

- [ ] **Step 6: Kiểm tra `fastddsgen` chạy được**

Run:
```bash
cd service/dds_node && fastddsgen -replace -d /tmp/vtx-gen ../idl/vtx_path_planning.idl && ls /tmp/vtx-gen
```
Expected: sinh ra `vtx_path_planning.cxx`, `vtx_path_planning.h`, `vtx_path_planningPubSubTypes.cxx/.h`.

Nếu `fastddsgen` không có: cài Fast DDS Gen theo hướng dẫn của eProsima cho ĐÚNG bản Fast DDS trên máy. Ghi lại phiên bản, Task 14 cần nó.

- [ ] **Step 7: Commit**

```bash
git add service/idl/ service/dds_node/CMakeLists.txt service/tests/idl_contract_test.py
git commit -m "feat(service): the IDL contract, generated type support, and an enum guard"
```

---

### Task 11: Client worker — spawn, socket, thời hạn cứng

**Files:**
- Create: `service/dds_node/src/worker_client.hpp`
- Create: `service/dds_node/src/worker_client.cpp`
- Test: `service/tests/worker_client_test.cpp`

**Interfaces:**
- Consumes: giao thức khung của Part 1 Task 8 (tiền tố `uint32` big-endian + thân msgpack).
- Produces: hàm tự do `vtx::FrameMessage(const std::vector<uint8_t>&)`, và lớp `vtx::WorkerClient` với `explicit WorkerClient(Config)`, `bool Start()`, `Outcome Request(const std::vector<uint8_t>& payload, double timeout_s, std::vector<uint8_t>& reply)`, `bool IsRunning() const`, `void Kill()`, `void Stop()`. `Outcome` là `enum class { kOk, kTimeout, kCrashed }`. Task 12 dùng.

- [ ] **Step 1: Viết test C++**

Create `service/tests/worker_client_test.cpp`:

```cpp
// Test cho lớp đóng khung và thời hạn. Chúng KHÔNG spawn Python: một tiến trình
// giả bằng /bin/sh là đủ để kiểm tra đóng khung, thời hạn cứng và việc dựng lại,
// và giữ cho test C++ chạy được ở nơi không có worker.

#include "worker_client.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace {

std::string TempSocket(const std::string& tag) {
  return (std::filesystem::temp_directory_path() / ("vtx-test-" + tag + ".sock")).string();
}

vtx::WorkerClient::Config EchoConfig(const std::string& socket_path) {
  // Một worker giả: dội lại đúng khung nó nhận được.
  vtx::WorkerClient::Config config;
  config.socket_path = socket_path;
  config.python_executable = "/bin/sh";
  config.worker_script = "-c";
  config.extra_args = {
      "python3 -c \"import socket,struct,os,sys;\n"
      "p=sys.argv[1];os.path.exists(p) and os.unlink(p);\n"
      "s=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);s.bind(p);s.listen(1);\n"
      "c,_=s.accept();\n"
      "import sys\n"
      "while True:\n"
      "  h=c.recv(4)\n"
      "  if not h: break\n"
      "  n=struct.unpack('!I',h)[0];b=b''\n"
      "  while len(b)<n: b+=c.recv(n-len(b))\n"
      "  c.sendall(h+b)\" " + socket_path};
  return config;
}

}  // namespace

TEST(FrameCodec, PrefixesABigEndianLength) {
  const std::vector<uint8_t> payload{0xde, 0xad, 0xbe, 0xef};
  const std::vector<uint8_t> framed = vtx::FrameMessage(payload);
  ASSERT_EQ(framed.size(), 8u);
  EXPECT_EQ(framed[0], 0x00);
  EXPECT_EQ(framed[1], 0x00);
  EXPECT_EQ(framed[2], 0x00);
  EXPECT_EQ(framed[3], 0x04);
  EXPECT_EQ(framed[4], 0xde);
}

TEST(FrameCodec, RoundTripsALargePayload) {
  const std::vector<uint8_t> payload(100000, 0x5a);
  const std::vector<uint8_t> framed = vtx::FrameMessage(payload);
  EXPECT_EQ(framed.size(), payload.size() + 4);
  EXPECT_EQ(framed[3], static_cast<uint8_t>(100000 & 0xff));
}

TEST(WorkerClient, EchoesAPayloadThroughTheSocket) {
  const std::string socket_path = TempSocket("echo");
  vtx::WorkerClient client(EchoConfig(socket_path));
  ASSERT_TRUE(client.Start());

  const std::vector<uint8_t> payload{1, 2, 3, 4, 5};
  std::vector<uint8_t> reply;
  EXPECT_EQ(client.Request(payload, 10.0, reply), vtx::WorkerClient::Outcome::kOk);
  EXPECT_EQ(reply, payload);
  client.Stop();
}

TEST(WorkerClient, ReportsTimeoutAndComesBackUsable) {
  const std::string socket_path = TempSocket("timeout");
  vtx::WorkerClient::Config config;
  config.socket_path = socket_path;
  config.python_executable = "/bin/sh";
  config.worker_script = "-c";
  // Worker này nhận rồi ngủ mãi: chính là hình dạng của một search không dừng.
  config.extra_args = {
      "python3 -c \"import socket,os,sys,time;\n"
      "p=sys.argv[1];os.path.exists(p) and os.unlink(p);\n"
      "s=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);s.bind(p);s.listen(1);\n"
      "c,_=s.accept();c.recv(4096);time.sleep(3600)\" " + socket_path};

  vtx::WorkerClient client(config);
  ASSERT_TRUE(client.Start());

  std::vector<uint8_t> reply;
  const auto outcome = client.Request({9, 9, 9}, 1.0, reply);
  EXPECT_EQ(outcome, vtx::WorkerClient::Outcome::kTimeout);
  EXPECT_TRUE(reply.empty());
  // Thời hạn cứng phải GIẾT worker, không chỉ bỏ cuộc chờ.
  EXPECT_FALSE(client.IsRunning());
  client.Stop();
}

TEST(WorkerClient, RestartsAfterAKill) {
  const std::string socket_path = TempSocket("restart");
  vtx::WorkerClient client(EchoConfig(socket_path));
  ASSERT_TRUE(client.Start());
  client.Kill();
  EXPECT_FALSE(client.IsRunning());
  ASSERT_TRUE(client.Start());

  std::vector<uint8_t> reply;
  EXPECT_EQ(client.Request({7}, 10.0, reply), vtx::WorkerClient::Outcome::kOk);
  EXPECT_EQ(reply, std::vector<uint8_t>{7});
  client.Stop();
}
```

- [ ] **Step 2: Chạy để xác nhận đỏ**

Run: `cd service/dds_node && cmake -B build && cmake --build build --target worker_client_test`
Expected: FAIL — `fatal error: worker_client.hpp: No such file or directory`.

- [ ] **Step 3: Viết header**

Create `service/dds_node/src/worker_client.hpp`:

```cpp
#pragma once

// Nói chuyện với worker Python qua một Unix domain socket, và sở hữu vòng đời
// của nó.
//
// Lớp này là lý do kiến trúc tách hai tiến trình. Planner là Python thuần,
// CPU-bound, và chỉ kiểm tra ngân sách tại các điểm trong vòng lặp search - nên
// nó KHÔNG hủy được từ bên ngoài một cách lịch sự. Thời hạn cứng ở đây giết
// worker rồi dựng lại, để node trả về một reply tử tế thay vì để client treo.

#include <cstdint>
#include <string>
#include <vector>

#include <sys/types.h>

namespace vtx {

// Bọc một thân tin thành khung: tiền tố độ dài uint32 big-endian.
std::vector<uint8_t> FrameMessage(const std::vector<uint8_t>& payload);

class WorkerClient {
 public:
  enum class Outcome { kOk, kTimeout, kCrashed };

  struct Config {
    std::string socket_path;
    std::string python_executable;
    std::string worker_script;
    std::string repo_root;
    std::vector<std::string> extra_args;
    double startup_timeout_s = 30.0;
  };

  explicit WorkerClient(Config config);
  ~WorkerClient();

  WorkerClient(const WorkerClient&) = delete;
  WorkerClient& operator=(const WorkerClient&) = delete;

  // Spawn worker và chờ tới khi socket của nó nhận kết nối. Trả về false khi
  // hết startup_timeout_s.
  bool Start();

  // Gửi một thân tin và chờ thân tin trả lời. Quá timeout_s thì GIẾT worker và
  // trả kTimeout; gọi Start() lần nữa để dùng tiếp.
  Outcome Request(const std::vector<uint8_t>& payload, double timeout_s,
                  std::vector<uint8_t>& reply);

  bool IsRunning() const;
  void Kill();
  void Stop();

 private:
  bool Connect();
  void CloseConnection();

  Config config_;
  pid_t pid_ = -1;
  int fd_ = -1;
};

}  // namespace vtx
```

- [ ] **Step 4: Viết cài đặt**

Create `service/dds_node/src/worker_client.cpp`:

```cpp
#include "worker_client.hpp"

#include <arpa/inet.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <thread>

namespace vtx {
namespace {

constexpr size_t kMaxFrameBytes = 64u * 1024u * 1024u;

// Đọc đúng count byte, hoặc thất bại. deadline là mốc tuyệt đối.
bool ReadExactly(int fd, uint8_t* out, size_t count,
                 std::chrono::steady_clock::time_point deadline) {
  size_t got = 0;
  while (got < count) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return false;
    }
    const auto remaining_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();

    pollfd descriptor{fd, POLLIN, 0};
    const int ready = ::poll(&descriptor, 1, static_cast<int>(remaining_ms));
    if (ready <= 0) {
      return false;
    }
    const ssize_t chunk = ::read(fd, out + got, count - got);
    if (chunk <= 0) {
      return false;
    }
    got += static_cast<size_t>(chunk);
  }
  return true;
}

bool WriteAll(int fd, const uint8_t* data, size_t count) {
  size_t sent = 0;
  while (sent < count) {
    const ssize_t chunk = ::write(fd, data + sent, count - sent);
    if (chunk <= 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    sent += static_cast<size_t>(chunk);
  }
  return true;
}

}  // namespace

std::vector<uint8_t> FrameMessage(const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> framed;
  framed.reserve(payload.size() + 4);
  const uint32_t length = htonl(static_cast<uint32_t>(payload.size()));
  const auto* length_bytes = reinterpret_cast<const uint8_t*>(&length);
  framed.insert(framed.end(), length_bytes, length_bytes + 4);
  framed.insert(framed.end(), payload.begin(), payload.end());
  return framed;
}

WorkerClient::WorkerClient(Config config) : config_(std::move(config)) {}

WorkerClient::~WorkerClient() { Stop(); }

bool WorkerClient::Start() {
  ::unlink(config_.socket_path.c_str());

  const pid_t pid = ::fork();
  if (pid < 0) {
    return false;
  }
  if (pid == 0) {
    // Tiến trình con. Đặt process group riêng để Kill() hạ được cả cây.
    ::setpgid(0, 0);
    std::vector<std::string> argv_storage{config_.python_executable, config_.worker_script};
    if (!config_.repo_root.empty()) {
      argv_storage.push_back("--socket");
      argv_storage.push_back(config_.socket_path);
      argv_storage.push_back("--repo-root");
      argv_storage.push_back(config_.repo_root);
    }
    for (const auto& extra : config_.extra_args) {
      argv_storage.push_back(extra);
    }

    std::vector<char*> argv;
    argv.reserve(argv_storage.size() + 1);
    for (auto& item : argv_storage) {
      argv.push_back(item.data());
    }
    argv.push_back(nullptr);

    ::execvp(argv[0], argv.data());
    ::_exit(127);
  }

  pid_ = pid;

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(static_cast<int>(config_.startup_timeout_s * 1000));
  while (std::chrono::steady_clock::now() < deadline) {
    if (!IsRunning()) {
      return false;
    }
    if (Connect()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  Kill();
  return false;
}

bool WorkerClient::Connect() {
  struct stat info {};
  if (::stat(config_.socket_path.c_str(), &info) != 0) {
    return false;
  }

  const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) {
    return false;
  }
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::strncpy(address.sun_path, config_.socket_path.c_str(), sizeof(address.sun_path) - 1);

  if (::connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
    ::close(fd);
    return false;
  }
  fd_ = fd;
  return true;
}

WorkerClient::Outcome WorkerClient::Request(const std::vector<uint8_t>& payload, double timeout_s,
                                            std::vector<uint8_t>& reply) {
  reply.clear();
  if (fd_ < 0 || !IsRunning()) {
    return Outcome::kCrashed;
  }

  const std::vector<uint8_t> framed = FrameMessage(payload);
  if (!WriteAll(fd_, framed.data(), framed.size())) {
    Kill();
    return Outcome::kCrashed;
  }

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<int>(timeout_s * 1000));

  uint8_t header[4];
  if (!ReadExactly(fd_, header, sizeof(header), deadline)) {
    // Không phân biệt được "quá hạn" với "worker chết giữa chừng" ở mức đọc,
    // nhưng cách xử lý giống nhau: hạ nó xuống rồi báo cáo trung thực.
    const bool alive = IsRunning();
    Kill();
    return alive ? Outcome::kTimeout : Outcome::kCrashed;
  }

  uint32_t length_be = 0;
  std::memcpy(&length_be, header, sizeof(length_be));
  const uint32_t length = ntohl(length_be);
  if (length > kMaxFrameBytes) {
    Kill();
    return Outcome::kCrashed;
  }

  reply.resize(length);
  if (length > 0 && !ReadExactly(fd_, reply.data(), length, deadline)) {
    reply.clear();
    const bool alive = IsRunning();
    Kill();
    return alive ? Outcome::kTimeout : Outcome::kCrashed;
  }
  return Outcome::kOk;
}

bool WorkerClient::IsRunning() const {
  if (pid_ <= 0) {
    return false;
  }
  return ::kill(pid_, 0) == 0;
}

void WorkerClient::Kill() {
  CloseConnection();
  if (pid_ > 0) {
    ::kill(-pid_, SIGKILL);
    int status = 0;
    ::waitpid(pid_, &status, 0);
    pid_ = -1;
  }
}

void WorkerClient::Stop() {
  Kill();
  ::unlink(config_.socket_path.c_str());
}

void WorkerClient::CloseConnection() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

}  // namespace vtx
```

- [ ] **Step 5: Build và chạy test C++**

Run:
```bash
cd service/dds_node && cmake -B build -DVTX_BUILD_TESTS=ON && cmake --build build --target worker_client_test && ./build/worker_client_test
```
Expected: PASS, 5 test.

Nếu GoogleTest không có trên máy: `sudo apt install libgtest-dev` hoặc build từ nguồn. Nếu vẫn không có, GHI LẠI là test C++ chưa chạy được và tiếp tục — Task 13 vẫn phủ được các đường này ở mức end-to-end, nhưng phải nói rõ chứ không im lặng bỏ qua.

- [ ] **Step 6: Commit**

```bash
git add service/dds_node/src/worker_client.hpp service/dds_node/src/worker_client.cpp service/tests/worker_client_test.cpp
git commit -m "feat(node): own the worker's lifetime, with a hard deadline that actually kills"
```

---

### Task 12: Node DDS — topic, QoS, tương quan, vòng đời

**Files:**
- Create: `service/dds_node/src/plan_service.hpp`
- Create: `service/dds_node/src/plan_service.cpp`
- Create: `service/dds_node/src/main.cpp`

**Interfaces:**
- Consumes: `vtx::WorkerClient` (Task 11); các kiểu sinh từ IDL (Task 10).
- Produces: binary `vtx_planner_dds_node`. Task 13 và 14 dùng.

Task này không có test đơn vị riêng: nó gần như toàn bộ là dây nối tới Fast DDS, mà việc kiểm thử thật nằm ở Task 13 chạy end-to-end qua DDS thật. Chia nhỏ hơn nữa sẽ tạo ra những test chỉ chứng minh rằng mock được viết đúng.

- [ ] **Step 1: Viết header dịch vụ**

Create `service/dds_node/src/plan_service.hpp`:

```cpp
#pragma once

// Nửa DDS của service: hai topic, QoS, tương quan bằng request_id, và ba tầng
// thời hạn. Không có một dòng hình học nào ở đây - toàn bộ phần đó sống trong
// worker Python.

#include <atomic>
#include <memory>
#include <string>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/Topic.hpp>

#include "vtx_path_planningPubSubTypes.h"
#include "worker_client.hpp"

namespace vtx {

class PlanService {
 public:
  struct Config {
    uint32_t domain_id = 0;
    std::string request_topic = "VtxPathPlanRequest";
    std::string reply_topic = "VtxPathPlanReply";
    double worker_grace_s = 2.0;  // thời hạn node = time_budget_s + cái này
    WorkerClient::Config worker;
  };

  explicit PlanService(Config config);
  ~PlanService();

  bool Start();
  void Stop();

 private:
  class RequestListener : public eprosima::fastdds::dds::DataReaderListener {
   public:
    explicit RequestListener(PlanService* owner) : owner_(owner) {}
    void on_data_available(eprosima::fastdds::dds::DataReader* reader) override;
    void on_subscription_matched(
        eprosima::fastdds::dds::DataReader* reader,
        const eprosima::fastdds::dds::SubscriptionMatchedStatus& status) override;

   private:
    PlanService* owner_;
  };

  void Handle(const vtx::planning::VtxPathPlanRequest& request);
  void PublishRefusal(const vtx::planning::VtxPathPlanRequest& request,
                      vtx::planning::PlanStatus status, const std::string& detail);

  Config config_;
  WorkerClient worker_;
  std::atomic<bool> busy_{false};

  eprosima::fastdds::dds::DomainParticipant* participant_ = nullptr;
  eprosima::fastdds::dds::Publisher* publisher_ = nullptr;
  eprosima::fastdds::dds::Subscriber* subscriber_ = nullptr;
  eprosima::fastdds::dds::Topic* request_topic_ = nullptr;
  eprosima::fastdds::dds::Topic* reply_topic_ = nullptr;
  eprosima::fastdds::dds::DataWriter* reply_writer_ = nullptr;
  eprosima::fastdds::dds::DataReader* request_reader_ = nullptr;
  std::unique_ptr<RequestListener> listener_;
};

}  // namespace vtx
```

- [ ] **Step 2: Viết cài đặt dịch vụ**

Create `service/dds_node/src/plan_service.cpp`:

```cpp
#include "plan_service.hpp"

#include <msgpack.hpp>

#include <chrono>
#include <cstring>
#include <iostream>
#include <sstream>

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>

namespace vtx {

using namespace eprosima::fastdds::dds;
namespace pb = vtx::planning;

namespace {

constexpr uint32_t kIdlVersion = 1;

// --- msgpack: request C++ -> thân tin worker ---------------------------------
// Bố cục phải khớp vtx_planner.codec.decode_request từng khoá một.
std::vector<uint8_t> EncodeRequest(const pb::VtxPathPlanRequest& request) {
  msgpack::sbuffer buffer;
  msgpack::packer<msgpack::sbuffer> packer(buffer);

  packer.pack_map(14);

  packer.pack("request_id");
  packer.pack_bin(16);
  packer.pack_bin_body(reinterpret_cast<const char*>(request.request_id().data()), 16);

  packer.pack("idl_version");
  packer.pack(request.idl_version());

  packer.pack("frame");
  packer.pack(request.frame() == pb::FRAME_WGS84 ? "wgs84" : "local_meters");

  const auto pack_point = [&packer](const pb::Point2D& point) {
    packer.pack_array(2);
    packer.pack(point.x());
    packer.pack(point.y());
  };
  const auto pack_polygons = [&packer, &pack_point](const std::vector<pb::Polygon>& polygons) {
    packer.pack_array(static_cast<uint32_t>(polygons.size()));
    for (const auto& polygon : polygons) {
      packer.pack_array(static_cast<uint32_t>(polygon.vertices().size()));
      for (const auto& vertex : polygon.vertices()) {
        pack_point(vertex);
      }
    }
  };

  packer.pack("start");
  pack_point(request.start());
  packer.pack("start_heading_deg");
  packer.pack(request.start_heading_deg());
  packer.pack("goal");
  pack_point(request.goal());
  packer.pack("goal_heading_deg");
  packer.pack(request.goal_heading_deg());
  packer.pack("goal_heading_free");
  packer.pack(request.goal_heading_free());

  packer.pack("islands");
  pack_polygons(request.islands());

  packer.pack("dynamic_obstacles");
  packer.pack_array(static_cast<uint32_t>(request.dynamic_obstacles().size()));
  for (const auto& circle : request.dynamic_obstacles()) {
    packer.pack_map(2);
    packer.pack("center");
    pack_point(circle.center());
    packer.pack("radius_m");
    packer.pack(circle.radius_m());
  }

  packer.pack("safezones");
  pack_polygons(request.safezones());

  packer.pack("use_preloaded_map");
  packer.pack(request.use_preloaded_map());

  packer.pack("limits");
  packer.pack_array(5);
  packer.pack(request.limits().turn_radius_m());
  packer.pack(request.limits().l0_m());
  packer.pack(request.limits().dss_m());
  packer.pack(request.limits().safe_margin_m());
  packer.pack(request.limits().alpha_max_deg());

  packer.pack("budget");
  packer.pack_array(2);
  packer.pack(request.budget().time_budget_s());
  packer.pack(request.budget().max_iterations());

  return std::vector<uint8_t>(buffer.data(), buffer.data() + buffer.size());
}

// --- msgpack: thân tin worker -> reply C++ -----------------------------------
bool DecodeReply(const std::vector<uint8_t>& blob, pb::VtxPathPlanReply& reply) {
  try {
    const msgpack::object_handle handle =
        msgpack::unpack(reinterpret_cast<const char*>(blob.data()), blob.size());
    std::map<std::string, msgpack::object> fields;
    handle.get().convert(fields);

    const auto raw_id = fields.at("request_id").as<std::string>();
    if (raw_id.size() != 16) {
      return false;
    }
    std::array<uint8_t, 16> request_id{};
    std::memcpy(request_id.data(), raw_id.data(), 16);
    reply.request_id(request_id);

    reply.idl_version(fields.at("idl_version").as<uint32_t>());
    reply.status(static_cast<pb::PlanStatus>(fields.at("status").as<int32_t>()));
    reply.detail(fields.at("detail").as<std::string>());
    reply.path_length_m(fields.at("path_length_m").as<double>());
    reply.plan_wall_time_s(fields.at("plan_wall_time_s").as<double>());
    reply.planner_version(fields.at("planner_version").as<std::string>());
    reply.config_hash(fields.at("config_hash").as<std::string>());

    std::vector<std::array<double, 3>> raw_waypoints;
    fields.at("waypoints").convert(raw_waypoints);
    std::vector<pb::Waypoint> waypoints;
    waypoints.reserve(raw_waypoints.size());
    for (const auto& triple : raw_waypoints) {
      pb::Waypoint waypoint;
      pb::Point2D position;
      position.x(triple[0]);
      position.y(triple[1]);
      waypoint.position(position);
      waypoint.heading_deg(triple[2]);
      waypoints.push_back(waypoint);
    }
    reply.waypoints(waypoints);

    std::tuple<uint32_t, uint32_t, uint32_t, bool, bool> raw_stats;
    fields.at("stats").convert(raw_stats);
    pb::SearchStats stats;
    stats.iterations(std::get<0>(raw_stats));
    stats.max_iterations(std::get<1>(raw_stats));
    stats.open_set_size(std::get<2>(raw_stats));
    stats.search_failed(std::get<3>(raw_stats));
    stats.budget_bound(std::get<4>(raw_stats));
    reply.stats(stats);
    return true;
  } catch (const std::exception& error) {
    std::cerr << "vtx-node: reply không giải mã được: " << error.what() << std::endl;
    return false;
  }
}

}  // namespace

PlanService::PlanService(Config config)
    : config_(std::move(config)), worker_(config_.worker) {}

PlanService::~PlanService() { Stop(); }

bool PlanService::Start() {
  if (!worker_.Start()) {
    std::cerr << "vtx-node: worker không khởi động được" << std::endl;
    return false;
  }

  DomainParticipantQos participant_qos = PARTICIPANT_QOS_DEFAULT;
  participant_qos.name("vtx_planner_dds_node");
  participant_ = DomainParticipantFactory::get_instance()->create_participant(
      config_.domain_id, participant_qos);
  if (participant_ == nullptr) {
    return false;
  }

  TypeSupport request_type(new pb::VtxPathPlanRequestPubSubType());
  TypeSupport reply_type(new pb::VtxPathPlanReplyPubSubType());
  request_type.register_type(participant_);
  reply_type.register_type(participant_);

  request_topic_ = participant_->create_topic(config_.request_topic,
                                              request_type.get_type_name(), TOPIC_QOS_DEFAULT);
  reply_topic_ = participant_->create_topic(config_.reply_topic, reply_type.get_type_name(),
                                            TOPIC_QOS_DEFAULT);
  if (request_topic_ == nullptr || reply_topic_ == nullptr) {
    return false;
  }

  publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT);
  subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT);

  // QoS reply: RELIABLE + KEEP_LAST(8) + VOLATILE.
  DataWriterQos writer_qos = DATAWRITER_QOS_DEFAULT;
  writer_qos.reliability().kind = RELIABLE_RELIABILITY_QOS;
  writer_qos.history().kind = KEEP_LAST_HISTORY_QOS;
  writer_qos.history().depth = 8;
  writer_qos.durability().kind = VOLATILE_DURABILITY_QOS;
  // Payload lớn (bản đồ nhiều đa giác) vượt ngưỡng phân mảnh UDP mặc định:
  // publish bất đồng bộ để Fast DDS phân mảnh thay vì từ chối mẫu tin.
  writer_qos.publish_mode().kind = ASYNCHRONOUS_PUBLISH_MODE;
  reply_writer_ = publisher_->create_datawriter(reply_topic_, writer_qos);

  // QoS request: RELIABLE + KEEP_ALL + VOLATILE.
  //
  // VOLATILE là bắt buộc, không phải mặc định tuỳ tiện. TRANSIENT_LOCAL ở đây
  // nghĩa là node khởi động lại sẽ nhận và lập kế hoạch lại một mission cũ đã
  // hết hiệu lực. Một lệnh bay không được phép phát lại.
  DataReaderQos reader_qos = DATAREADER_QOS_DEFAULT;
  reader_qos.reliability().kind = RELIABLE_RELIABILITY_QOS;
  reader_qos.history().kind = KEEP_ALL_HISTORY_QOS;
  reader_qos.durability().kind = VOLATILE_DURABILITY_QOS;

  listener_ = std::make_unique<RequestListener>(this);
  request_reader_ = subscriber_->create_datareader(request_topic_, reader_qos, listener_.get());
  if (reply_writer_ == nullptr || request_reader_ == nullptr) {
    return false;
  }

  std::cout << "vtx-node: sẵn sàng trên domain " << config_.domain_id << std::endl;
  return true;
}

void PlanService::RequestListener::on_subscription_matched(
    DataReader*, const SubscriptionMatchedStatus& status) {
  std::cout << "vtx-node: publisher khớp, tổng " << status.current_count << std::endl;
}

void PlanService::RequestListener::on_data_available(DataReader* reader) {
  pb::VtxPathPlanRequest request;
  SampleInfo info;
  while (reader->take_next_sample(&request, &info) == ReturnCode_t::RETCODE_OK) {
    if (info.valid_data) {
      owner_->Handle(request);
    }
  }
}

void PlanService::Handle(const pb::VtxPathPlanRequest& request) {
  if (request.idl_version() != kIdlVersion) {
    std::ostringstream detail;
    detail << "idl_version " << request.idl_version() << " != " << kIdlVersion;
    PublishRefusal(request, pb::PLAN_INVALID_REQUEST, detail.str());
    return;
  }

  bool expected = false;
  if (!busy_.compare_exchange_strong(expected, true)) {
    PublishRefusal(request, pb::PLAN_BUSY, "đang xử lý một request khác");
    return;
  }

  // Tầng thời hạn 2: cứng, và nó GIẾT. Tầng 1 là config.TIME_BUDGET_S bên trong
  // planner; tầng 3 là thời hạn của client, ngoài phạm vi node.
  const double deadline_s = request.budget().time_budget_s() + config_.worker_grace_s;

  std::vector<uint8_t> reply_blob;
  const auto outcome = worker_.Request(EncodeRequest(request), deadline_s, reply_blob);

  if (outcome != WorkerClient::Outcome::kOk) {
    const bool timed_out = outcome == WorkerClient::Outcome::kTimeout;
    PublishRefusal(request, timed_out ? pb::PLAN_TIMEOUT : pb::PLAN_INTERNAL_ERROR,
                   timed_out ? "worker vượt thời hạn cứng" : "worker chết");
    if (!worker_.Start()) {
      std::cerr << "vtx-node: KHÔNG dựng lại được worker" << std::endl;
    }
    busy_.store(false);
    return;
  }

  pb::VtxPathPlanReply reply;
  if (!DecodeReply(reply_blob, reply)) {
    PublishRefusal(request, pb::PLAN_INTERNAL_ERROR, "reply của worker không giải mã được");
    busy_.store(false);
    return;
  }

  reply_writer_->write(&reply);
  busy_.store(false);
}

void PlanService::PublishRefusal(const pb::VtxPathPlanRequest& request, pb::PlanStatus status,
                                 const std::string& detail) {
  pb::VtxPathPlanReply reply;
  reply.request_id(request.request_id());
  reply.idl_version(kIdlVersion);
  reply.status(status);
  reply.detail(detail);
  reply.path_length_m(0.0);
  reply.plan_wall_time_s(0.0);
  reply_writer_->write(&reply);
}

void PlanService::Stop() {
  worker_.Stop();
  if (participant_ != nullptr) {
    participant_->delete_contained_entities();
    DomainParticipantFactory::get_instance()->delete_participant(participant_);
    participant_ = nullptr;
  }
}

}  // namespace vtx
```

- [ ] **Step 3: Viết `main.cpp`**

Create `service/dds_node/src/main.cpp`:

```cpp
// Điểm vào của node. Phân tích tham số, in ra phiên bản Fast DDS đang dùng, rồi
// chạy tới khi nhận tín hiệu dừng.
//
// In phiên bản là có chủ đích: transport shared-memory nhạy với phiên bản và sẽ
// âm thầm rơi về UDP khi hai bên lệch nhau, tức mất hiệu năng mà không có lỗi
// nào. Số phiên bản trong log là cách rẻ nhất để phát hiện điều đó.

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>

#include "plan_service.hpp"

namespace {

volatile std::sig_atomic_t g_stop = 0;

void OnSignal(int) { g_stop = 1; }

std::string ArgValue(int argc, char** argv, const std::string& flag, const std::string& fallback) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (flag == argv[i]) {
      return argv[i + 1];
    }
  }
  return fallback;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string worker_python = ArgValue(argc, argv, "--worker-python", "");
  const std::string worker_script = ArgValue(argc, argv, "--worker-script", "");
  const std::string repo_root = ArgValue(argc, argv, "--repo-root", "");
  const std::string socket_path = ArgValue(argc, argv, "--socket", "/run/vtx/planner.sock");
  const uint32_t domain_id =
      static_cast<uint32_t>(std::stoul(ArgValue(argc, argv, "--domain-id", "0")));

  if (worker_python.empty() || worker_script.empty() || repo_root.empty()) {
    std::cerr << "dùng: " << argv[0]
              << " --worker-python <python> --worker-script <run_worker.py>"
                 " --repo-root <repo> [--socket <path>] [--domain-id <n>]"
              << std::endl;
    return 2;
  }

  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);

  vtx::PlanService::Config config;
  config.domain_id = domain_id;
  config.worker.socket_path = socket_path;
  config.worker.python_executable = worker_python;
  config.worker.worker_script = worker_script;
  config.worker.repo_root = repo_root;

  vtx::PlanService service(std::move(config));
  if (!service.Start()) {
    std::cerr << "vtx-node: khởi động thất bại" << std::endl;
    return 1;
  }

  while (g_stop == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  std::cout << "vtx-node: đang dừng" << std::endl;
  service.Stop();
  return 0;
}
```

- [ ] **Step 4: Cài `msgpack-c` (phần C++ header-only) nếu chưa có**

Run: `echo '#include <msgpack.hpp>' | g++ -x c++ -fsyntax-only -`
Nếu lỗi: `sudo apt install libmsgpack-dev`, hoặc tải bản header-only từ github.com/msgpack/msgpack-c nhánh `cpp_master` và trỏ include vào đó.

- [ ] **Step 5: Build node**

Run: `cd service/dds_node && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j`
Expected: sinh ra `build/vtx_planner_dds_node`, không có lỗi.

Nếu `find_package(fastdds)` thất bại: đặt `-Dfastdds_DIR=<đường dẫn tới fastddsConfig.cmake>`, và GHI LẠI đường dẫn đó cho `service/deploy/README.md` ở Task 14.

- [ ] **Step 6: Commit**

```bash
git add service/dds_node/src/plan_service.hpp service/dds_node/src/plan_service.cpp service/dds_node/src/main.cpp
git commit -m "feat(node): two topics, VOLATILE QoS, and a hard deadline over the worker"
```

---

### Task 13: Test round-trip qua DDS thật

**Files:**
- Test: `service/tests/roundtrip_dds_test.py`

**Interfaces:**
- Consumes: binary `vtx_planner_dds_node` (Task 12); `plan` từ Part 1.
- Produces: không có mã production.

Test này cần Fast DDS Python binding ở PHÍA CLIENT để giả làm hệ thống gọi. Nếu binding không có, nó tự bỏ qua với lý do rõ ràng — không được im lặng bỏ qua và cũng không được coi là đã kiểm thử.

- [ ] **Step 1: Viết test**

Create `service/tests/roundtrip_dds_test.py`:

```python
"""End-to-end qua DDS thật: cùng một mission, hai đường đi, một kết quả.

Test này chỉ chạy khi có đủ hai thứ: binary node đã build, và Fast DDS Python
binding để đóng vai hệ thống gọi. Thiếu thứ nào thì nó BỎ QUA CÓ LÝ DO. Một
test bị bỏ qua trong im lặng còn tệ hơn không có test.
"""

from __future__ import annotations

import dataclasses
import subprocess
import time
import uuid
from pathlib import Path

import pytest

from vtx_planner import plan
from vtx_planner.messages import (
    PlanRequest,
    PlanStatus,
    SearchBudget,
    VehicleLimits,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
NODE_BINARY = REPO_ROOT / "service" / "dds_node" / "build" / "vtx_planner_dds_node"
WORKER = REPO_ROOT / "service" / "worker" / "run_worker.py"

fastdds = pytest.importorskip(
    "fastdds", reason="cần Fast DDS Python binding để giả làm client; xem service/deploy/README.md"
)
vtx_types = pytest.importorskip(
    "vtx_path_planning",
    reason="cần module sinh bằng `fastddsgen -python`; xem service/deploy/README.md",
)

pytestmark = pytest.mark.skipif(
    not NODE_BINARY.exists(),
    reason=f"chưa build node: {NODE_BINARY} không tồn tại (chạy Task 12 Step 5)",
)

DOMAIN_ID = 77  # tách khỏi domain 0 để không đụng hệ thống thật đang chạy


@pytest.fixture()
def node(tmp_path: Path):
    proc = subprocess.Popen(
        [
            str(NODE_BINARY),
            "--worker-python", "python3",
            "--worker-script", str(WORKER),
            "--repo-root", str(REPO_ROOT),
            "--socket", str(tmp_path / "planner.sock"),
            "--domain-id", str(DOMAIN_ID),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    time.sleep(5.0)  # discovery + khởi động worker
    if proc.poll() is not None:
        raise RuntimeError(f"node chết khi khởi động:\n{proc.communicate()[0].decode()}")
    try:
        yield proc
    finally:
        proc.terminate()
        proc.wait(timeout=15)


def _local_request(request_id: bytes) -> PlanRequest:
    return PlanRequest(
        request_id=request_id,
        idl_version=1,
        frame="local_meters",
        start=(50000.0, 50000.0),
        start_heading_deg=45.0,
        goal=(300000.0, 250000.0),
        goal_heading_deg=45.0,
        goal_heading_free=True,
        islands=(),
        dynamic_obstacles=(),
        safezones=(),
        use_preloaded_map=False,
        limits=VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0),
        budget=SearchBudget(15.0, 50000),
    )


class _DdsClient:
    """Đóng vai hệ thống gọi: publish request, chờ reply khớp request_id.

    Dùng Fast DDS Python binding của eProsima (Fast-DDS-python). Nếu binding
    trên máy có API khác, sửa Ở ĐÂY - không lớp nào khác của service chạm tới
    DDS bằng Python.
    """

    def __init__(self, domain_id: int) -> None:
        factory = fastdds.DomainParticipantFactory.get_instance()
        participant_qos = fastdds.DomainParticipantQos()
        factory.get_default_participant_qos(participant_qos)
        self._participant = factory.create_participant(domain_id, participant_qos)
        assert self._participant is not None, "không tạo được DomainParticipant"

        request_type = vtx_types.VtxPathPlanRequestPubSubType()
        reply_type = vtx_types.VtxPathPlanReplyPubSubType()
        self._participant.register_type(fastdds.TypeSupport(request_type))
        self._participant.register_type(fastdds.TypeSupport(reply_type))

        topic_qos = fastdds.TopicQos()
        self._participant.get_default_topic_qos(topic_qos)
        request_topic = self._participant.create_topic(
            "VtxPathPlanRequest", request_type.getName(), topic_qos
        )
        reply_topic = self._participant.create_topic(
            "VtxPathPlanReply", reply_type.getName(), topic_qos
        )

        publisher_qos = fastdds.PublisherQos()
        self._participant.get_default_publisher_qos(publisher_qos)
        publisher = self._participant.create_publisher(publisher_qos)

        writer_qos = fastdds.DataWriterQos()
        publisher.get_default_datawriter_qos(writer_qos)
        writer_qos.reliability().kind = fastdds.RELIABLE_RELIABILITY_QOS
        writer_qos.durability().kind = fastdds.VOLATILE_DURABILITY_QOS
        self._writer = publisher.create_datawriter(request_topic, writer_qos)

        subscriber_qos = fastdds.SubscriberQos()
        self._participant.get_default_subscriber_qos(subscriber_qos)
        subscriber = self._participant.create_subscriber(subscriber_qos)

        reader_qos = fastdds.DataReaderQos()
        subscriber.get_default_datareader_qos(reader_qos)
        reader_qos.reliability().kind = fastdds.RELIABLE_RELIABILITY_QOS
        reader_qos.durability().kind = fastdds.VOLATILE_DURABILITY_QOS
        reader_qos.history().kind = fastdds.KEEP_LAST_HISTORY_QOS
        reader_qos.history().depth = 8
        self._reader = subscriber.create_datareader(reply_topic, reader_qos)

    def wait_for_node(self, timeout_s: float) -> bool:
        """Chờ tới khi node đã khớp cả hai chiều.

        Ghi trước khi khớp là mất mẫu tin trong im lặng với QoS VOLATILE.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            matched = fastdds.PublicationMatchedStatus()
            self._writer.get_publication_matched_status(matched)
            if matched.current_count > 0:
                return True
            time.sleep(0.1)
        return False

    def send(self, request: PlanRequest, timeout_s: float = 30.0):
        sample = _to_idl(request)
        self._writer.write(sample)

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            reply = vtx_types.VtxPathPlanReply()
            info = fastdds.SampleInfo()
            if self._reader.take_next_sample(reply, info) == fastdds.ReturnCode_t.RETCODE_OK:
                if info.valid_data and bytes(reply.request_id()) == request.request_id:
                    return reply
                continue
            time.sleep(0.02)
        raise TimeoutError(f"không nhận được reply cho {request.request_id.hex()[:8]}")

    def close(self) -> None:
        self._participant.delete_contained_entities()
        fastdds.DomainParticipantFactory.get_instance().delete_participant(self._participant)


def _point(x: float, y: float):
    point = vtx_types.Point2D()
    point.x(x)
    point.y(y)
    return point


def _to_idl(request: PlanRequest):
    """Đổi một PlanRequest thành mẫu tin IDL."""
    sample = vtx_types.VtxPathPlanRequest()
    sample.request_id(list(request.request_id))
    sample.idl_version(request.idl_version)
    sample.frame(
        vtx_types.FRAME_WGS84 if request.frame == "wgs84" else vtx_types.FRAME_LOCAL_METERS
    )
    sample.start(_point(*request.start))
    sample.start_heading_deg(request.start_heading_deg)
    sample.goal(_point(*request.goal))
    sample.goal_heading_deg(request.goal_heading_deg)
    sample.goal_heading_free(request.goal_heading_free)

    def polygons(source):
        out = []
        for ring in source:
            polygon = vtx_types.Polygon()
            polygon.vertices([_point(x, y) for x, y in ring])
            out.append(polygon)
        return out

    sample.islands(polygons(request.islands))
    sample.safezones(polygons(request.safezones))

    circles = []
    for item in request.dynamic_obstacles:
        circle = vtx_types.Circle()
        circle.center(_point(*item.center))
        circle.radius_m(item.radius_m)
        circles.append(circle)
    sample.dynamic_obstacles(circles)

    sample.use_preloaded_map(request.use_preloaded_map)

    limits = vtx_types.VehicleLimits()
    limits.turn_radius_m(request.limits.turn_radius_m)
    limits.l0_m(request.limits.l0_m)
    limits.dss_m(request.limits.dss_m)
    limits.safe_margin_m(request.limits.safe_margin_m)
    limits.alpha_max_deg(request.limits.alpha_max_deg)
    sample.limits(limits)

    budget = vtx_types.SearchBudget()
    budget.time_budget_s(request.budget.time_budget_s)
    budget.max_iterations(request.budget.max_iterations)
    sample.budget(budget)
    return sample


@pytest.fixture()
def client(node):
    dds = _DdsClient(DOMAIN_ID)
    assert dds.wait_for_node(timeout_s=20.0), "node không khớp trong 20 s (kiểm tra domain/discovery)"
    try:
        yield dds
    finally:
        dds.close()


def test_dds_reply_matches_the_in_process_plan(client) -> None:
    request_id = uuid.uuid4().bytes
    request = _local_request(request_id)

    over_dds = client.send(request)
    in_process = plan(request)

    assert bytes(over_dds.request_id()) == request_id
    assert over_dds.status() == int(in_process.status)
    assert len(over_dds.waypoints()) == len(in_process.waypoints)
    for got, expected in zip(over_dds.waypoints(), in_process.waypoints):
        # double đi qua IDL là chính xác: không có dung sai nào ở đây.
        assert got.position().x() == expected.position[0]
        assert got.position().y() == expected.position[1]
        assert got.heading_deg() == expected.heading_deg
    assert over_dds.path_length_m() == in_process.path_length_m
    assert over_dds.config_hash() == in_process.config_hash


def test_a_wrong_idl_version_is_refused_over_dds(client) -> None:
    request = dataclasses.replace(_local_request(uuid.uuid4().bytes), idl_version=999)
    reply = client.send(request)
    assert reply.status() == int(PlanStatus.INVALID_REQUEST)
    assert "idl_version" in reply.detail()


def test_a_tiny_budget_still_returns_something(client) -> None:
    """Tầng thời hạn 2. Điều phải giữ là client KHÔNG BAO GIỜ bị treo."""
    request = dataclasses.replace(
        _local_request(uuid.uuid4().bytes),
        budget=SearchBudget(time_budget_s=0.001, max_iterations=50000),
    )
    reply = client.send(request)
    # Ngân sách 1 ms: planner tự dừng ở tầng 1, hoặc node cắt ở tầng 2. Cả hai
    # đều hợp lệ; điều KHÔNG hợp lệ là không có reply nào.
    assert reply.status() in (
        int(PlanStatus.OK),
        int(PlanStatus.NO_PATH),
        int(PlanStatus.TIMEOUT),
    )


def test_the_node_survives_the_timeout_and_serves_the_next_request(client) -> None:
    """Sau một PLAN_TIMEOUT, node phải dựng lại worker và tiếp tục phục vụ."""
    client.send(
        dataclasses.replace(
            _local_request(uuid.uuid4().bytes),
            budget=SearchBudget(time_budget_s=0.001, max_iterations=50000),
        )
    )
    healthy = client.send(_local_request(uuid.uuid4().bytes), timeout_s=60.0)
    assert healthy.status() == int(PlanStatus.OK)
```

- [ ] **Step 2: Sinh binding Python cho test**

Run:
```bash
fastddsgen -replace -python -d /tmp/vtx-py-gen service/idl/vtx_path_planning.idl
```
Rồi build module theo hướng dẫn eProsima cho Fast-DDS-python. Ghi lại các bước đã dùng — chúng sẽ vào `service/deploy/README.md` ở Task 14.

Nếu không dựng được binding trên máy này: DỪNG, ghi lại lý do, và chuyển sang Task 14. Test sẽ tự bỏ qua với lý do rõ ràng. Không được xoá test, và không được đánh dấu Task 13 là hoàn tất.

- [ ] **Step 3: Đối chiếu `_DdsClient` với API của binding trên máy**

Code ở Step 1 viết theo API của Fast-DDS-python bản eProsima: getter/setter là
lời gọi hàm (`reply.status()`, `sample.request_id(list_of_bytes)`), và
`take_next_sample(sample, info)` trả về `ReturnCode_t`.

Chạy thử một lệnh nhỏ để xác nhận hình dạng API trước khi gỡ lỗi test:

```bash
python -c "import fastdds, vtx_path_planning as t; \
r = t.VtxPathPlanReply(); r.detail('xin chào'); print(repr(r.detail()))"
```

Nếu binding dùng thuộc tính thay vì hàm, sửa `_DdsClient` và `_to_idl` cho khớp.
Đó là hai chỗ DUY NHẤT trong toàn bộ service chạm tới DDS bằng Python; không
lan ra chỗ nào khác.

- [ ] **Step 4: Chạy test**

Run: `python -m pytest service/tests/roundtrip_dds_test.py -v`
Expected: 4 passed, hoặc 4 skipped với lý do nêu rõ thiếu thứ gì.

- [ ] **Step 5: Commit**

```bash
git add service/tests/roundtrip_dds_test.py
git commit -m "test(service): the same mission over DDS and in process must agree"
```

---

### Task 14: Triển khai — systemd, tài liệu, và Dockerfile

**Files:**
- Create: `service/deploy/vtx-planner.service`
- Create: `service/deploy/README.md`
- Create: `service/deploy/Dockerfile`

**Interfaces:**
- Consumes: binary node (Task 12), worker (Part 1 Task 9), `worker-requirements.txt` (Part 1 Task 9).
- Produces: một bản triển khai chạy được.

- [ ] **Step 1: Viết unit systemd**

Create `service/deploy/vtx-planner.service`:

```ini
[Unit]
Description=VTX path planning service (Fast DDS)
Documentation=file:/opt/vtx/path_planning/docs/superpowers/specs/2026-08-22-dds-path-planning-service-design.md
After=network.target

[Service]
Type=simple
User=vtx
Group=vtx
RuntimeDirectory=vtx
RuntimeDirectoryMode=0750

# Một unit duy nhất: node tự spawn worker như tiến trình con, nên vòng đời gắn
# liền và node giữ quyền SIGKILL/dựng lại. Hai unit riêng sẽ cần thứ tự khởi
# động và một cơ chế phát hiện worker chết, tức là dựng lại thứ node đã có.
ExecStart=/opt/vtx/path_planning/service/dds_node/build/vtx_planner_dds_node \
    --worker-python /opt/vtx/venv/bin/python \
    --worker-script /opt/vtx/path_planning/service/worker/run_worker.py \
    --repo-root     /opt/vtx/path_planning \
    --socket        /run/vtx/planner.sock \
    --domain-id     0
# Thêm --preloaded-map /opt/vtx/basemap.json nếu triển khai này dùng bản đồ nền
# tĩnh. Mặc định KHÔNG có: request tự chứa thì replay được và chẩn đoán được,
# còn state ẩn trong service thì không.

Restart=on-failure
RestartSec=5

# KillMode=control-group để worker con không sống sót sau khi node bị dừng.
KillMode=control-group
TimeoutStopSec=30

[Install]
WantedBy=multi-user.target
```

- [ ] **Step 2: Viết README triển khai**

Create `service/deploy/README.md`:

```markdown
# Triển khai VTX path planning service

Hai giai đoạn: **systemd trước, Docker sau.** systemd chạy được sớm và gỡ lỗi DDS
dễ hơn hẳn. Dockerfile viết sau khi interface đã ổn định thì chỉ còn là đóng gói
lại đúng các bước đã chạy được, chứ không phải vừa dựng vừa đoán.

## Phụ thuộc

Worker cần **ba** gói: `shapely`, `msgpack`, `pyproj`. Xem
`worker-requirements.txt` để biết vì sao cái pin `numpy==1.26.4` ở gốc repo
KHÔNG áp dụng ở đây.

Node cần Fast DDS **cùng phiên bản với hệ thống gọi**, cộng `fastddsgen` và
`msgpack-c` (phần C++ header-only).

Ghi phiên bản thực tế vào bảng này khi triển khai:

| thành phần | phiên bản trên máy đích |
| --- | --- |
| Fast DDS | _điền khi cài_ |
| Fast CDR | _điền khi cài_ |
| fastddsgen | _điền khi cài_ |
| Python | 3.11 |

## Giai đoạn 1 — systemd

```bash
sudo useradd --system --home /opt/vtx --shell /usr/sbin/nologin vtx

sudo git clone <repo> /opt/vtx/path_planning
cd /opt/vtx/path_planning && sudo git checkout <tag>

sudo python3.11 -m venv /opt/vtx/venv
sudo /opt/vtx/venv/bin/pip install -r service/deploy/worker-requirements.txt

cd service/dds_node
sudo cmake -B build -DCMAKE_BUILD_TYPE=Release
sudo cmake --build build -j

sudo chown -R vtx:vtx /opt/vtx
sudo cp ../deploy/vtx-planner.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now vtx-planner
```

Kiểm tra: `journalctl -u vtx-planner -f`. Log khởi động phải in phiên bản Fast
DDS và **transport thực tế đang dùng**. Nếu nó nói UDP ở nơi bạn chờ đợi
shared-memory, đó là lệch phiên bản Fast DDS — kiểm tra bảng trên, đừng bỏ qua.

## Nâng cấp thuật toán

```bash
cd /opt/vtx/path_planning && sudo git pull && sudo systemctl restart vtx-planner
```

Không phải build lại gì bên Python: worker import `core.*` thẳng từ cây mã nguồn.
Chỉ khi **IDL** đổi mới phải build lại C++, và IDL đổi hiếm hơn thuật toán rất
nhiều.

Mỗi reply mang `planner_version` (`git describe --always --dirty`) và
`config_hash`, nên client luôn phân biệt được hai đường bay khác nhau là do
input khác hay do phiên bản/cấu hình planner khác.

## Sinh binding Python cho test

Chỉ cần cho `service/tests/roundtrip_dds_test.py`, không cần lúc chạy thật:

```bash
fastddsgen -replace -python -d /tmp/vtx-py-gen /opt/vtx/path_planning/service/idl/vtx_path_planning.idl
# rồi build module theo hướng dẫn Fast-DDS-python của eProsima
```

## Giai đoạn 2 — Docker

Ba điều kiện bắt buộc, đều là chỗ dễ mất một ngày để gỡ nếu không biết trước:

- **`--network host`.** Discovery mặc định của Fast DDS dùng multicast. Trên
  bridge network, discovery chết lặng: không lỗi, không cảnh báo, chỉ là không
  ai thấy ai.
- **`--ipc host` và chia sẻ `/dev/shm`**, nếu muốn giữ transport shared-memory.
- **Ghim đúng phiên bản Fast DDS của bên gọi** trong image.

```bash
docker build -t vtx-planner:<tag> -f service/deploy/Dockerfile .
docker run --rm --network host --ipc host -v /dev/shm:/dev/shm vtx-planner:<tag>
```

Cần thấy rõ cái giá: `--network host --ipc host` đã bỏ gần hết sự cách ly vốn là
lý do dùng Docker. Cái còn lại là tái lập môi trường build và triển khai lên máy
mới mà không phải cài toolchain Fast DDS — lợi ích thật, nhưng là lợi ích vận
hành, không phải cách ly.

## Chẩn đoán

| triệu chứng | nguyên nhân thường gặp |
| --- | --- |
| Client không nhận reply nào | Sai `--domain-id`, hoặc discovery bị chặn. Kiểm tra `on_subscription_matched` trong log. |
| Mọi reply là `PLAN_BUSY` | Một request trước chưa xong. Service xử lý tuần tự theo thiết kế. |
| Mọi reply là `PLAN_INVALID_REQUEST` | `idl_version` lệch. Client và node build từ hai bản IDL khác nhau. |
| `PLAN_TIMEOUT` lặp lại | Ngân sách quá chặt cho bản đồ này, hoặc máy quá tải. Xem `stats.budget_bound` trên các reply thành công. |
| Đường bay đúng độ dài nhưng sai hướng 90 độ | Quy ước phương vị. Trên dây LUÔN là phương vị thật, thuận kim đồng hồ từ bắc, ở cả hai frame. |
| `PLAN_INVALID_REQUEST` kèm "preloaded map" | Client đặt `use_preloaded_map` nhưng service khởi động không có `--preloaded-map`. |
| `PLAN_INVALID_REQUEST` kèm "frame" | Bản đồ nền khai báo frame khác request. Service từ chối thay vì diễn giải lại toạ độ. |
| Reply thiếu mẫu tin với bản đồ lớn | Phân mảnh UDP. Writer đã bật `ASYNCHRONOUS_PUBLISH_MODE`; có thể cần nâng `max_message_size` của transport. |
```

- [ ] **Step 3: Viết Dockerfile**

Create `service/deploy/Dockerfile`:

```dockerfile
# Giai đoạn 2 của thiết kế. Chỉ dùng khi cần triển khai lên máy chưa có
# toolchain Fast DDS; xem README.md để biết ba điều kiện chạy bắt buộc.
#
# FAST_DDS_VERSION phải khớp phiên bản của HỆ THỐNG GỌI. Lệch phiên bản không
# làm hỏng giao tiếp trên dây, nhưng làm transport shared-memory âm thầm rơi về
# UDP — mất hiệu năng mà không có lỗi nào.
ARG FAST_DDS_VERSION=2.14.0

FROM ubuntu:22.04 AS build
ARG FAST_DDS_VERSION
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git openjdk-11-jre-headless \
      libasio-dev libtinyxml2-dev libssl-dev libmsgpack-dev \
    && rm -rf /var/lib/apt/lists/*

# Fast DDS + Fast CDR + fastddsgen ở đúng phiên bản đã ghim.
RUN git clone --depth 1 -b v2.2.4 https://github.com/eProsima/Fast-CDR.git /src/fastcdr \
    && cmake -S /src/fastcdr -B /build/fastcdr -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /build/fastcdr -j --target install
RUN git clone --depth 1 -b v${FAST_DDS_VERSION} https://github.com/eProsima/Fast-DDS.git /src/fastdds \
    && cmake -S /src/fastdds -B /build/fastdds -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /build/fastdds -j --target install
RUN git clone --depth 1 https://github.com/eProsima/Fast-DDS-Gen.git /src/fastddsgen \
    && cd /src/fastddsgen && ./gradlew assemble \
    && install -m 0755 scripts/fastddsgen /usr/local/bin/fastddsgen \
    && cp -r share /usr/local/share/fastddsgen

COPY service /repo/service
RUN cmake -S /repo/service/dds_node -B /build/node -DCMAKE_BUILD_TYPE=Release -DVTX_BUILD_TESTS=OFF \
    && cmake --build /build/node -j

FROM ubuntu:22.04
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3.11 python3.11-venv libtinyxml2-9 libssl3 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=build /usr/local/lib /usr/local/lib
RUN ldconfig

# Toàn bộ repo, vì worker import core.* thẳng từ cây mã nguồn.
COPY . /opt/vtx/path_planning
COPY --from=build /build/node/vtx_planner_dds_node \
     /opt/vtx/path_planning/service/dds_node/build/vtx_planner_dds_node

RUN python3.11 -m venv /opt/vtx/venv \
    && /opt/vtx/venv/bin/pip install --no-cache-dir \
       -r /opt/vtx/path_planning/service/deploy/worker-requirements.txt

ENTRYPOINT ["/opt/vtx/path_planning/service/dds_node/build/vtx_planner_dds_node", \
            "--worker-python", "/opt/vtx/venv/bin/python", \
            "--worker-script", "/opt/vtx/path_planning/service/worker/run_worker.py", \
            "--repo-root", "/opt/vtx/path_planning", \
            "--socket", "/tmp/vtx-planner.sock"]
CMD ["--domain-id", "0"]
```

- [ ] **Step 4: Chạy thử node ở tiền cảnh**

Run:
```bash
service/dds_node/build/vtx_planner_dds_node \
  --worker-python "$(which python3)" \
  --worker-script "$PWD/service/worker/run_worker.py" \
  --repo-root "$PWD" \
  --socket /tmp/vtx-planner.sock \
  --domain-id 77
```
Expected: in ra `vtx-node: sẵn sàng trên domain 77`, và log của worker báo phiên bản planner cùng `config_hash`. `Ctrl-C` để dừng, rồi kiểm tra không còn tiến trình Python nào sót: `pgrep -f run_worker.py` phải rỗng.

- [ ] **Step 5: Chạy toàn bộ test và kiểm tra ranh giới lần cuối**

Run:
```bash
python -m pytest -q service/tests/
python -m pytest -q tests/ 2>&1 | tail -3
git diff --stat main -- core/ render/ config.py
```
Expected: service toàn PASS (round-trip có thể skipped kèm lý do); `tests/` vẫn `188 passed, 6 failed`; diff ranh giới rỗng.

- [ ] **Step 6: Commit**

```bash
git add service/deploy/
git commit -m "feat(deploy): systemd unit, deployment guide, and the phase-2 Dockerfile"
```

---

## Hoàn tất

Xong Task 14, service đã triển khai được: hệ thống gọi publish một
`VtxPathPlanRequest` và nhận lại `VtxPathPlanReply` mang đường bay đầy đủ
`O..T`, kèm nhận dạng phiên bản và cấu hình đã sinh ra nó.

Thuật toán chưa bị sửa một dòng nào, và ba cơ chế của spec giữ cho nó tiếp tục
như vậy: test ranh giới, test hợp đồng khoá, và test tương đương bit-identical.
