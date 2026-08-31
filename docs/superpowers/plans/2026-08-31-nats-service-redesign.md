# NATS Microservice Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Redesign the VTX Path Planning service to communicate over NATS Server with Protocol Buffers serialization under the subject `vtx.algorithms.path_planning.plan`.

**Architecture:** Core NATS Request-Reply pattern with Queue Groups (`vtx.algorithms.path_planning`), Protocol Buffers (`proto3`) wire encoding, non-blocking `asyncio` event loop in main process dispatching CPU-bound planning to `PlanRunner` forkserver subprocesses.

**Tech Stack:** Python 3.10+, `nats-py`, `protobuf`, `shapely`, `numpy`, `pytest`, `ruff`, `pyright`.

**Spec:** [`docs/superpowers/specs/2026-08-31-nats-service-redesign.md`](file:///mnt/d/Workspace/VTX/Path_Planning/docs/superpowers/specs/2026-08-31-nats-service-redesign.md)

## Global Constraints
- Primary Subject: `vtx.algorithms.path_planning.plan`
- Worker Queue Group: `vtx.algorithms.path_planning`
- Serialization Format: Protocol Buffers (`proto3`)
- Planning Time Budget: Configurable per request, capped at $\le 5.0$s
- Code Quality: Strict adherence to `docs/coding_standards_extracted.txt` (AAA unit tests, Google docstrings, explicit type annotations, 100% test pass rate).

---

### Task 1: Protocol Buffers Schema & Codec Implementation

**Files:**
- Create: `src/service/proto/vtx_path_planning.proto`
- Create: `src/service/vtx_service/proto/__init__.py`
- Generate: `src/service/vtx_service/proto/vtx_path_planning_pb2.py`
- Create: `src/service/vtx_service/codec.py`
- Test: `tests/service/unit/test_proto_codec.py`

**Interfaces:**
- Produces:
  - `encode_request(request: msg.PlanRequest) -> bytes`
  - `decode_request(data: bytes) -> msg.PlanRequest`
  - `encode_reply(reply: msg.PlanReply) -> bytes`
  - `decode_reply(data: bytes) -> msg.PlanReply`

- [ ] **Step 1: Create the `.proto` schema file**
  Write `src/service/proto/vtx_path_planning.proto` defining `PlanStatus`, `Point2D`, `Waypoint`, `PolygonObstacle`, `CircleObstacle`, `VehicleLimits`, `SearchBudget`, `PathPlanRequest`, and `PathPlanReply`.

- [ ] **Step 2: Generate python protobuf bindings**
  Run `protoc` to generate `src/service/vtx_service/proto/vtx_path_planning_pb2.py`.

- [ ] **Step 3: Write failing unit tests for the codec**
  Create `tests/service/unit/test_proto_codec.py` asserting bit-exact conversions for all message types and boundary cases.

- [ ] **Step 4: Implement `src/service/vtx_service/codec.py`**
  Implement the encode/decode functions mapping between canonical Python dataclasses (`service.vtx_service.messages`) and protobuf objects.

- [ ] **Step 5: Run pytest and verify codec tests pass**
  Run `pytest tests/service/unit/test_proto_codec.py -v`.

- [ ] **Step 6: Commit Task 1**
  `git add src/service/proto/ src/service/vtx_service/proto/ src/service/vtx_service/codec.py tests/service/unit/test_proto_codec.py && git commit -m "feat(service): add protobuf schema and codec for NATS payload serialization"`

---

### Task 2: NatsTransport Engine Implementation

**Files:**
- Modify/Rewrite: `src/service/vtx_service/transport.py`
- Test: `tests/service/unit/test_transport.py`
- Test: `tests/service/integration/test_nats_transport.py`

**Interfaces:**
- Produces:
  - `class NatsTransport`:
    - `__init__(self, server_url: str = ..., subject: str = ..., queue: str = ...)`
    - `async def start(self, handler: Callable[[PlanRequest], PlanReply]) -> None`
    - `async def close(self) -> None`
  - `class NatsClient`:
    - `__init__(self, server_url: str = ..., subject: str = ...)`
    - `async def connect(self) -> None`
    - `async def close(self) -> None`
    - `async def request_plan(self, request: PlanRequest, timeout: float = 6.0) -> PlanReply`

- [ ] **Step 1: Write failing unit & integration tests for `NatsTransport` and `NatsClient`**
  Write tests in `tests/service/unit/test_transport.py` and `tests/service/integration/test_nats_transport.py` checking message delivery, error handling, status enum mappings, and timeout behavior.

- [ ] **Step 2: Implement `src/service/vtx_service/transport.py`**
  Implement `NatsTransport` using `nats-py` with async message callback, threadpool runner execution, and reply publication; implement `NatsClient` for request-reply querying.

- [ ] **Step 3: Run pytest and verify transport tests pass**
  Run `pytest tests/service/unit/test_transport.py tests/service/integration/test_nats_transport.py -v`.

- [ ] **Step 4: Commit Task 2**
  `git add src/service/vtx_service/transport.py tests/service/unit/test_transport.py tests/service/integration/test_nats_transport.py && git commit -m "feat(service): implement NatsTransport and NatsClient with asyncio request-reply"`

---

### Task 3: Service Entrypoint CLI (`main.py`) & Dependencies Configuration

**Files:**
- Modify: `src/service/vtx_service/main.py`
- Modify: `pyproject.toml`
- Modify: `src/service/deploy/requirements.txt`
- Modify: `src/service/deploy/vtx-planner.service`

**Interfaces:**
- Produces: CLI interface supporting `--nats-server`, `--subject`, `--queue`, `--preloaded-map`, `--grace-seconds`.

- [ ] **Step 1: Update dependencies in `pyproject.toml` and `requirements.txt`**
  Add `nats-py>=2.12.0` and `protobuf>=5.0.0` to project dependencies; remove `cyclonedds` requirement.

- [ ] **Step 2: Update `src/service/vtx_service/main.py`**
  Refactor `main.py` to use `asyncio.run()` with `NatsTransport`, setting up signal handlers (`SIGINT`, `SIGTERM`), clean runner lifecycle, and logging.

- [ ] **Step 3: Update systemd service template in `src/service/deploy/vtx-planner.service`**
  Update ExecStart command line to use NATS arguments.

- [ ] **Step 4: Commit Task 3**
  `git add src/service/vtx_service/main.py pyproject.toml src/service/deploy/ && git commit -m "feat(service): update service entrypoint and deployment configurations for NATS"`

---

### Task 4: Integration Tests & Full Suite Verification

**Files:**
- Update: `tests/service/integration/test_planner_service.py`
- Update: `tests/service/integration/test_equivalence.py`
- Test: All unit and integration tests across the repository.

- [ ] **Step 1: Update `test_planner_service.py` and `test_equivalence.py`**
  Refactor tests to use `NatsTransport` / `NatsClient` instead of `DdsTransport` / `DdsClient`.

- [ ] **Step 2: Run Ruff and Pyright**
  Run `ruff format . && ruff check .` and `pyright` to ensure zero typing/lint issues.

- [ ] **Step 3: Run full pytest suite**
  Run `pytest tests/ -v` and confirm 100% pass rate.

- [ ] **Step 4: Commit Task 4**
  `git add tests/ && git commit -m "test(service): update integration tests and verify full test suite with NATS service"`
