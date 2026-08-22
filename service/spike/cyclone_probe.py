"""Spike: Cyclone DDS có mang được đúng hình dạng dữ liệu service cần không.

Dùng một lần. Không ship.

Chạy hai vai trong hai terminal:
    python service/spike/cyclone_probe.py listen
    python service/spike/cyclone_probe.py send

LƯU Ý (đo được khi transcribe từ brief, 2026-08-22): brief gốc có dòng
`from __future__ import annotations` ở đầu file. Với dòng đó, cyclonedds
11.0.1 KHÔNG resolve được annotation `array[uint8, 16]` trên trường
`request_id` -- nó cố `getattr(module, "array[uint8, 16]")` trên chuỗi
annotation đã bị hoãn và ném TypeError ngay lúc tạo Topic, ở CẢ HAI vai
(send và listen). Đã bỏ future-import này để probe chạy được; xem
fastdds_probe.md / decision doc để biết log traceback gốc.
"""

import sys
import time
from dataclasses import dataclass

from cyclonedds.core import Policy, Qos, ReadCondition, InstanceState, SampleState, ViewState, WaitSet
from cyclonedds.domain import DomainParticipant
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.annotations import key
from cyclonedds.idl.types import array, sequence, uint8
from cyclonedds.pub import DataWriter, Publisher
from cyclonedds.sub import DataReader, Subscriber
from cyclonedds.topic import Topic
from cyclonedds.util import duration

DOMAIN = 91


@dataclass
class Point2D(IdlStruct, typename="vtx.planning.Point2D"):
    x: float
    y: float


@dataclass
class Polygon(IdlStruct, typename="vtx.planning.Polygon"):
    vertices: sequence[Point2D]


@dataclass
class Probe(IdlStruct, typename="vtx.planning.Probe"):
    request_id: array[uint8, 16]
    key("request_id")
    idl_version: int
    detail: str
    islands: sequence[Polygon]
    length_m: float


REQ_QOS = Qos(
    Policy.Reliability.Reliable(duration(seconds=10)),
    Policy.History.KeepAll,
    Policy.Durability.Volatile,
)


def _endpoint():
    participant = DomainParticipant(DOMAIN)
    topic = Topic(participant, "VtxProbe", Probe, qos=REQ_QOS)
    return participant, topic


def send() -> None:
    participant, topic = _endpoint()
    writer = DataWriter(Publisher(participant), topic, qos=REQ_QOS)
    time.sleep(2.0)  # discovery
    sample = Probe(
        request_id=list(range(16)),
        idl_version=1,
        detail="first W1..W2 l=7421.3 < L0=8000",
        islands=[Polygon(vertices=[Point2D(0.0, 0.0), Point2D(1e5, 0.0), Point2D(5e4, 1e5)])],
        length_m=123456.78901234567,
    )
    writer.write(sample)
    print("đã gửi", sample.length_m)
    time.sleep(2.0)


def listen() -> None:
    participant, topic = _endpoint()
    reader = DataReader(Subscriber(participant), topic, qos=REQ_QOS)
    condition = ReadCondition(
        reader, ViewState.Any | InstanceState.Alive | SampleState.NotRead
    )
    waitset = WaitSet(participant)
    waitset.attach(condition)
    if waitset.wait(duration(seconds=30)) == 0:
        print("KHÔNG nhận được gì trong 30 s")
        return
    for sample in reader.take(N=10, condition=condition):
        print("request_id  :", list(sample.request_id) == list(range(16)))
        print("detail      :", repr(sample.detail))
        print("đỉnh đảo    :", len(sample.islands[0].vertices))
        print("double khớp :", sample.length_m == 123456.78901234567)


if __name__ == "__main__":
    {"send": send, "listen": listen}[sys.argv[1]]()
