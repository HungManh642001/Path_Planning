# Spec: NATS Microservice Redesign for Algorithm Gateway

## 1. Context & Objectives
The VTX Path Planning service previously communicated using a DDS transport (CycloneDDS).
To simplify system topology, improve scalability across microservices, and enable seamless integration with an ecosystem of multiple UAV algorithms, the service is being redesigned to communicate via a **NATS Server** message broker.

### Key Decisions:
- **Transport Pattern**: Core NATS Request-Reply with Queue Groups.
- **Subject Hierarchy**: `vtx.algorithms.<algorithm_name>.<action>`
  - Path Planning Primary Subject: `vtx.algorithms.path_planning.plan`
  - Path Planning Queue Group: `vtx.algorithms.path_planning`
- **Serialization Format**: Protocol Buffers (`proto3`).
- **Execution Engine**: `nats-py` on `asyncio` event loop in main process, dispatching CPU-bound planning calculations (budget $\le 5.0$s) to the forkserver `PlanRunner`.

---

## 2. System Architecture

```mermaid
graph TD
    Client[Client / Autopilot / Mission Control]
    NATS[NATS Server Broker]
    
    subgraph VTX Algorithm Service (Queue Group: vtx.algorithms.path_planning)
        NatsTransport[NatsTransport - Asyncio Engine]
        ProtoCodec[Protobuf Codec - Request/Reply]
        Runner[PlanRunner - ForkServer Multiprocessing]
        CorePlanner[Kinodynamic A* Core]
    end
    
    Client -->|1. nc.request('vtx.algorithms.path_planning.plan', proto_bytes)| NATS
    NATS -->|2. Load-balanced delivery to worker| NatsTransport
    NatsTransport -->|3. Decode proto -> PlanRequest| ProtoCodec
    ProtoCodec -->|4. runner.submit(req)| Runner
    Runner -->|5. IPC to isolated child process| CorePlanner
    CorePlanner -->|6. Result| Runner
    Runner -->|7. PlanReply| ProtoCodec
    ProtoCodec -->|8. Encode PlanReply -> proto_bytes| NatsTransport
    NatsTransport -->|9. Publish to msg.reply inbox| NATS
    NATS -->|10. Deliver reply| Client
```

---

## 3. Protocol Buffers Specification (`vtx_path_planning.proto`)

```protobuf
syntax = "proto3";

package vtx.algorithms.path_planning;

enum PlanStatus {
  PLAN_STATUS_UNSPECIFIED = 0;
  PLAN_STATUS_OK = 1;
  PLAN_STATUS_TIMEOUT_NO_PATH = 2;
  PLAN_STATUS_INVALID_REQUEST = 3;
  PLAN_STATUS_GOAL_INSIDE_OBSTACLE = 4;
  PLAN_STATUS_START_INSIDE_OBSTACLE = 5;
  PLAN_STATUS_SEARCH_EXHAUSTED = 6;
  PLAN_STATUS_INTERNAL_ERROR = 7;
}

message Point2D {
  double x = 1;
  double y = 2;
}

message Waypoint {
  Point2D position = 1;
  double heading_rad = 2;
}

message PolygonObstacle {
  repeated Point2D vertices = 1;
}

message CircleObstacle {
  Point2D center = 1;
  double radius_m = 2;
}

message VehicleLimits {
  double turn_radius_m = 1;
  double alpha_max_deg = 2;
  double l0_m = 3;
  double dss_m = 4;
  double safe_margin_m = 5;
}

message SearchBudget {
  double max_time_s = 1;
}

message PathPlanRequest {
  bytes request_id = 1;          // 16-byte UUID
  uint32 idl_version = 2;
  Point2D takeoff = 3;
  double takeoff_heading_deg = 4;
  Point2D target = 5;
  double target_heading_deg = 6;
  bool is_target_heading_free = 7;

  VehicleLimits vehicle_limits = 8;
  SearchBudget budget = 9;

  repeated PolygonObstacle safezones = 10;
  repeated PolygonObstacle islands = 11;
  repeated CircleObstacle dynamic_obstacles = 12;
  bool merge_with_preloaded_map = 13;
}

message PathPlanReply {
  bytes request_id = 1;
  uint32 idl_version = 2;
  PlanStatus status = 3;
  string detail = 4;

  repeated Waypoint waypoints = 5;
  double path_length_m = 6;
  double plan_wall_time_s = 7;
  uint32 iterations = 8;

  string planner_version = 9;
  string config_hash = 10;
  double applied_time_budget_s = 11;
}
```

---

## 4. Component Design

### 4.1 Proto Compilation & Codec
- Source file: `src/service/proto/vtx_path_planning.proto`
- Output compiled module: `src/service/vtx_service/proto/vtx_path_planning_pb2.py`
- Codec functions:
  - `encode_request(msg.PlanRequest) -> bytes`
  - `decode_request(bytes) -> msg.PlanRequest`
  - `encode_reply(msg.PlanReply) -> bytes`
  - `decode_reply(bytes) -> msg.PlanReply`

### 4.2 NatsTransport (`src/service/vtx_service/transport.py`)
- **`NatsTransport` Class**:
  - `__init__(self, server_url: str = "nats://localhost:4222", subject: str = "vtx.algorithms.path_planning.plan", queue: str = "vtx.algorithms.path_planning")`
  - `async start(self, handler: Callable[[PlanRequest], PlanReply]) -> None`: Connects to NATS, subscribes with queue group.
  - `async close(self) -> None`: Gracefully drains subscriptions and closes connection.
  - `async message_handler(self, msg: nats.aio.msg.Msg) -> None`: Deserializes protobuf payload, invokes handler in threadpool to keep asyncio loop responsive, serializes reply, and publishes back to `msg.reply`.
- **`NatsClient` Helper Class** (for clients, tests, and CLI tools):
  - `async request_plan(self, request: PlanRequest, timeout: float = 6.0) -> PlanReply`

### 4.3 Service Lifecycle (`src/service/vtx_service/main.py`)
- CLI Arguments:
  - `--nats-server`: NATS server URL (default: `nats://localhost:4222`).
  - `--subject`: Subject to listen on (default: `vtx.algorithms.path_planning.plan`).
  - `--queue`: Queue group name (default: `vtx.algorithms.path_planning`).
  - `--preloaded-map`: XML basemap path (optional).
  - `--grace-seconds`: Worker child timeout grace (default: 2.0s).
- Startup sequence:
  1. Parse arguments and configure logging.
  2. Load preloaded map.
  3. Start `PlanRunner` (forkserver initialized clean before networking threads).
  4. Run `NatsTransport` inside `asyncio.run()`.
  5. Handle `SIGINT`/`SIGTERM` gracefully.

---

## 5. Verification Plan & Test Strategy
1. **Unit Tests (`tests/service/unit/`)**:
   - `test_proto_codec.py`: Verify bit-exact round-trip conversion between canonical `PlanRequest`/`PlanReply` dataclasses and protobuf messages.
2. **Integration Tests (`tests/service/integration/`)**:
   - `test_nats_transport.py`: In-process NATS client-server round-trip test (using mock/embedded or running NATS server / asyncio mock).
   - `test_planner_service.py`: Verify end-to-end plan execution over NATS client and server.
   - `test_equivalence.py`: Verify output equivalence between direct core planner and NATS service reply.
3. **Full Suite Verification**:
   - 100% tests passing, zero linter/pyright issues.
