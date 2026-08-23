# Sổ thực thi: service path planning qua DDS

Ghi lại 26 phán quyết đưa ra trong lúc thực thi kế hoạch
`docs/superpowers/plans/2026-08-22-dds-service-python.md`, mỗi phán quyết kèm
số đo đã dẫn tới nó và cái giá nếu nó sai. Giữ lại vì đây là những quyết định
được đưa ra thay chủ sở hữu, không phải cùng chủ sở hữu.

# SDD ledger — plan: docs/superpowers/plans/2026-08-22-dds-service-python.md

Spec: docs/superpowers/specs/2026-08-22-dds-path-planning-service-design.md (đọc rồi)
Nhánh: feature/dds-service (không phải main). Cây làm việc bẩn ở 4 file ngoài core/ — không đụng tới.

## Quét tiền-thực-thi

### Cặp task dùng chung file / interface

| A | B | A sinh ra | B tiêu thụ | kết quả |
| --- | --- | --- | --- | --- |
| T2 | T7 | `vtx_service/__init__.py` | T7 sửa để export `plan` | OK — T7 Step 4 nói rõ |
| T2 | T4,5,7,9,10,11 | messages: PlanStatus/Circle/VehicleLimits/SearchBudget/PlanRequest/Waypoint/SearchStats/PlanReply/IDL_VERSION/Point | tất cả | OK — không ai import `FRAMES` (đã bỏ cùng WGS84) |
| T3 | T5,7,8 | `bearing_deg_to_math_rad`, `math_rad_to_bearing_deg` | T5 dùng cái đầu, T7/T8 cái sau | OK |
| T4 | T7,9,11 | `PreloadedMap.load/.merged_into/.safezones/.islands/.dynamic_obstacles` | cả ba | OK |
| T5 | T7,8 | `build_scenario(request)` | cả hai | OK |
| T6 | T7,9 | `config_hash`, `planner_version`, `effective_time_budget_s`, `effective_max_iterations` | T7 cả 4; T9 cả 4 trong `_failed` + `effective_time_budget_s` trong `submit` | OK |
| T7 | T8,9 | `plan(request, preloaded=None)` | T8 gọi `plan(request)`; T9 `_child` gọi `plan(request, preloaded=...)` | OK |
| T9 | T11 | `PlanRunner(preloaded, grace_s)`, `start/submit/stop` | main.py | OK |
| T10 | T11 | `DdsTransport(domain_id)`, `serve/close` | main.py | OK |
| T4 | T11 | `service/deploy/` | T4 tạo basemap.example.xml, T11 tạo phần còn lại | OK — không chồng file |

### Tự nhất quán trong từng task

| Task | kết quả |
| --- | --- |
| T1 | OK — spike, không sinh mã ship |
| T2 | OK — test khẳng định `applied_time_budget_s` tồn tại, dataclass có |
| T3 | OK — code đã CHẠY THẬT, 16/16 xanh |
| T4 | OK — test file mẫu khớp file mẫu Step 4 |
| T5 | OK |
| T6 | OK |
| T7 | OK |
| T8 | OK — `zip(strict=True)` cần 3.10+, môi trường là 3.11 |
| T9 | **LỖI** — xem R2, R3 |
| T10 | **LỖI** — xem R1; R5 |
| T11 | Minor — xem R4 |

## Phán quyết trước khi chạy

Ruling R1 (T10, LỖI THẬT): QoS phải thêm `Policy.IgnoreLocal.Participant` cho cả hai profile.
  — ĐO ĐƯỢC: không có nó, một DataWriter khớp với DataReader của CHÍNH participant mình
    (`current_count = 1` khi chỉ có một participant; có IgnoreLocal thì = 0). Hệ quả:
    `wait_for_service` trả True ngay cả khi KHÔNG có service nào, nên
    `test_a_request_survives_the_wire_unchanged` sẽ treo ở `client.request` thay vì
    hỏng nhanh — đúng loại lỗi làm mất một buổi.
  — Giá nếu sai: bằng 0. IgnoreLocal.Participant chỉ chặn khớp trong CÙNG participant;
    service và client là hai participant riêng nên vẫn thấy nhau.

Ruling R2 (T9): `PlanRunner.start()` phải `.join()` tiến trình mồi forkserver.
  — Mã trong kế hoạch gọi `.start()` rồi vứt handle, để lại zombie tới khi GC.
  — Giá nếu sai: một tiến trình zombie mỗi lần start. Nhỏ nhưng vô cớ.

Ruling R3 (T9): test `test_a_hung_child_becomes_timeout_...` phải hạ `config.TIME_BUDGET_S`
  (try/finally) xuống ~0.5 s thay vì dùng giá trị thật.
  — Như viết trong kế hoạch, thời hạn = effective_time_budget_s() + grace = 15.5 s, nên
    test chạy 15,5 giây và TRÔNG NHƯ TREO. Chính là điều Step 4 dặn đừng nhầm.
  — Giá nếu sai: bộ test chậm 15 s và gây hiểu nhầm; không sai về logic.

Ruling R4 (T11 + rà chung): bỏ import không dùng (main.py import `PlanStatus` nhưng chỉ
  đọc `reply.status.name`). Ruff/pyflakes sẽ bắt.
  — Giá nếu sai: nhiễu linter, không ảnh hưởng hành vi.

Ruling R5 (T10): test dùng domain 92/95, KHÔNG dùng 93 — Task 11 Step 2 hướng dẫn chạy
  service tay trên domain 93.
  — Giá nếu sai: test chập chờn khi ai đó đang chạy service tay song song.

Ruling R6 (T9): giữ cửa hậu `force_hang_next` / `force_raise_next` như kế hoạch mô tả,
  có đánh dấu rõ là chỉ dùng cho test.
  — Đây là cách rẻ nhất để test được thời hạn cứng mà không cần inject target picklable.
  — Reviewer nhiều khả năng sẽ nêu; nếu nêu thì tôi phân xử lúc đó, không phán trước.
  — Giá nếu sai: mã production mang hai cờ chỉ test dùng.

Ruling R7 (T1, phạm vi spike): Step 3 (dựng Fast DDS Python binding) chạy có giới hạn —
  dừng ở blocker cứng đầu tiên và GHI LẠI, thay vì cố cho đủ 2 giờ.
  — Lý do: (a) "không có wheel trên PyPI" đã đo xong, (b) Cyclone đã chạy được ở đây,
    (c) câu hỏi thật là interop, mà interop cần hệ thống của chủ sở hữu — không truy cập
    được trong phiên này. Giá trị còn lại của một lần build đầy đủ là thấp.
  — Quyết định stack vì thế là TẠM THỜI, chờ đo interop. Ghi rõ trong văn bản quyết định.
  — Giá nếu sai: nếu sau này interop hỏng, phải làm lại Task 10 — đúng một file, vì
    transport được cô lập sau interface hẹp. Đó chính là lý do nó được cô lập.

## Tiến độ

Task 1: dispatched (spike, sonnet) tại BASE 6b25d38 — rulings R7/R1 mang theo trong dispatch

Ruling R8 (T10, LỖI THẬT trong kế hoạch — do subagent Task 1 phát hiện, tôi kiểm chứng lại độc lập):
  `service/vtx_service/transport.py` KHÔNG được dùng `from __future__ import annotations`.
  — ĐO ĐƯỢC: với dòng đó, `Topic(...)` ném `TypeError: Type array[uint8, 16] as used in
    __main__ cannot be resolved.`; bỏ dòng đó ra thì Topic tạo được. cyclonedds phân giải
    chú thích kiểu lúc chạy, mà PEP 563 biến chúng thành chuỗi.
  — Kế hoạch viết transport.py CÓ dòng đó VÀ có `array[uint8, 16]`, nên Task 10 sẽ hỏng.
  — Chỉ áp cho transport.py (module duy nhất khai báo IdlStruct). Mọi module khác giữ nguyên.
  — Giá nếu sai: bằng 0 — bỏ một import chỉ ảnh hưởng cách chú thích được lưu.

Task 1: complete pending review (commit 4b3e072). Bất ngờ đáng ghi: Fast DDS Python
  DỰNG ĐƯỢC từ nguồn trong ~18 phút (không blocker cứng), nên phương án dự phòng cho
  quyết định tạm thời RẺ HƠN tôi giả định ở R7. Có xung đột GLIBCXX/libstdc++ của Anaconda,
  né bằng LD_PRELOAD.
Ghi chú môi trường (ảnh hưởng Task 10/11): `cyclonedds` KHÔNG có trong env Python chính
  (Anaconda), chỉ có trong venv phụ ở scratchpad. Task 10 dùng `pytest.importorskip`, nên
  test transport sẽ SKIP im lặng nếu chạy bằng env chính. Khi tới Task 10 phải cài
  cyclonedds vào env chính HOẶC chạy test bằng venv — và phải xác nhận test THỰC SỰ chạy
  chứ không phải skipped. Task 11 Step 7 (venv sạch) là chỗ bắt được điều này.
Task 1: reviewer dispatched (sonnet) trên diff 6b25d38..4b3e072
Task 1: complete (commits 6b25d38..4b3e072, review clean — spec ✅, quality approved, 0 finding)
Task 2: dispatched (sonnet) tại BASE 4b3e072
Task 2: reviewer dispatched (sonnet) trên diff 4b3e072..92babf9; service tests 10/10, baseline tests/ 188+6 khớp
Task 2: complete (commits 4b3e072..92babf9, review clean — spec ✅, quality approved, 0 finding)
Quyết định điều phối: gộp Task 3 (angles) + Task 6 (runtime) vào MỘT dispatch — hai module
  nhỏ, không phụ thuộc nhau, code đầy đủ trong brief. Review như một đơn vị, ledger hai dòng.
Task 3+6: dispatched (sonnet, gộp) tại BASE 92babf9

Ruling R9 (T6, LỖI THẬT trong kế hoạch — subagent phát hiện, tôi kiểm chứng lại):
  `runtime.py::_REPO_ROOT` phải là `parents[2]`, không phải `parents[3]`.
  — ĐO ĐƯỢC: parents[2] = /mnt/d/Workspace/VTX/Path_Planning (có .git);
    parents[3] = /mnt/d/Workspace/VTX (KHÔNG phải repo). Với parents[3],
    `planner_version()` nuốt CalledProcessError và luôn trả "unknown" trong im lặng.
    Sau sửa: trả '550dabe-dirty'. config_hash = cde049c0152e5bbf, budget = 15.0.
  — NGUYÊN NHÂN GỐC (lỗi của tôi): bản kế hoạch trước để file ở
    service/worker/vtx_planner/runtime.py (sâu 3 mức); khi đổi sang service/vtx_service/
    (sâu 2 mức) tôi không sửa chỉ số. Cùng loại lỗi có thể còn ở chỗ khác — đã rà: chỉ
    runtime.py dùng parents[N] trong mã ship; conftest.py dùng parents[1] (đúng),
    test dùng parents[2] (đúng).
  — Giá nếu sai: bằng 0 — chỉ số đúng đã được xác minh bằng sự tồn tại của .git.
  — CÒN LẠI: test của brief chỉ khẳng định chuỗi khác rỗng, nên nó KHÔNG bắt được lỗi
    này. Để reviewer tự nêu, không phán trước.
Điều phối: vá THẲNG vào kế hoạch (R9 parents[2], R8 bỏ __future__ trong transport.py,
  R1 thêm IgnoreLocal.Participant vào cả hai QoS, R5 đổi domain test 93->95) và SINH LẠI
  task-10-brief.md. Lý do: brief sinh ra TỪ kế hoạch, nên vá kế hoạch an toàn hơn nhiều
  so với mang ghi chú trong dispatch — ghi chú có thể bị bỏ sót, brief thì không.
  Đã rà toàn bộ 5 chỗ dùng parents[N] trong kế hoạch: chỉ runtime.py sai, 4 chỗ kia đúng.

Ruling R10 (T6, finding do KẾ HOẠCH quy định — tôi phán quyết theo spec, không bác bỏ):
  Finding: `test_version_is_a_non_empty_string` không bắt được lỗi parents[3];
  reviewer đã ép lỗi quay lại và thấy test VẪN XANH ("unknown" cũng khác rỗng).
  — Test này chép nguyên văn từ brief, tức do KẾ HOẠCH của tôi quy định. Nhưng
    spec mục 4.4 nói rõ `planner_version`/`config_hash` tồn tại để client phân biệt
    được hai đường bay là do input khác hay phiên bản/cấu hình khác. Một dấu phiên bản
    im lặng nói "unknown" phá đúng mục đích đó. SPEC LÀ THẨM QUYỀN RÀNG BUỘC, nên
    finding thắng kế hoạch.
  — Quyết định: siết test — trong một checkout git, `planner_version()` phải KHÁC
    "unknown". Có guard: chỉ khẳng định khi `.git` tồn tại, để bản triển khai dạng
    tarball không có git vẫn hợp lệ trả "unknown".
  — Giá nếu sai: gần 0. Rủi ro duy nhất là test đỏ ở môi trường không có binary git
    dù có thư mục .git — guard `.git` không che được ca đó, nhưng đó là môi trường
    hỏng chứ không phải cấu hình hợp lệ.
Task 3+6: fix round 1/5 dispatched (resume implementer gốc) — 1 finding Important
Task 3+6: fix round 1/5 (1 addressed, 0 open — test ghim planner_version vào git describe
  thật, có bằng chứng đỏ→xanh; commit e22c059)
Task 3: complete (commit 4e134a2, review clean)
Task 6: complete (commits 550dabe..e22c059, review clean sau 1 vòng sửa)
Điều phối: đồng bộ R10 ngược vào kế hoạch (test mới + import + đếm 6->7).
Task 4: dispatched->DONE (commit 313cd0a), 10/10 + suite 43/43; concern: guard radius dư trong _circles

Ruling R11 (T4, finding Important — do KẾ HOẠCH của tôi gây ra, tôi kiểm chứng lại):
  `map_file.py` dùng `p.get("x", "nan")` nên thuộc tính thiếu thành `float("nan")`.
  — ĐO ĐƯỢC: XML thiếu `x` và `cx` NẠP THÀNH CÔNG, cho đỉnh (nan, 120000.0) và tâm
    (nan, 180000.0). NaN đi thẳng vào planner, không lỗi, không cảnh báo.
  — Spec mục 5 đặt quy tắc "version không khớp là LỖI, không phải cảnh báo"; tinh thần
    là bản đồ hỏng phải hỏng to. Toàn bộ codebase này cũng theo nguyên tắc "thất bại
    nhanh và trung thực". Một toạ độ NaN lọt vào planner là đúng thứ thiết kế chống lại.
  — Quyết định: thuộc tính toạ độ thiếu hoặc không phải số => ValueError, và thông báo
    phải NÊU TÊN FILE.
  — Gộp luôn finding Minor: reviewer chỉ ra guard radius trong `_circles` hiện KHÔNG
    thêm giá trị nào so với `Circle.__post_init__` (cùng hình dạng thông báo, không có
    đường dẫn), nên test không phân biệt được. Nêu tên file trong MỌI lỗi parse khiến
    guard đó có giá trị thật VÀ khiến test phân biệt được — một thay đổi giải quyết cả hai.
  — Giá nếu sai: một bản đồ trước đây "nạp được" nay bị từ chối. Đó chính là mục đích;
    và nó chỉ từ chối thứ vốn đã hỏng.
Task 4: fix round 1/5 dispatched (resume implementer gốc) — 1 Important + 1 Minor gộp chung
Task 4: fix round 1/5 (2 addressed, 0 open — _float_attr nêu tên file, 3 test mới, test bán
  kính giờ phân biệt được; commit 73d3d76). Re-review xác nhận không breakage: x="0", số âm,
  ký hiệu khoa học, mục rỗng đều còn parse đúng.
Task 4: complete (commits 4a57f58..73d3d76, review clean sau 1 vòng sửa)
Điều phối: đồng bộ R11 ngược vào kế hoạch (_float_attr + luồng `path` + đếm 10->13).
Task 5: DONE (commit 302b09b), 9/9 + suite 55/55; concern tự báo: goal_heading nhánh không-free KHÔNG có test — implementer tiêm lỗi (để nguyên độ) và mọi test vẫn xanh

Ruling R12 (T5, finding Important — thiếu sót do KẾ HOẠCH của tôi, không phải implementer):
  Không có khẳng định nào phủ phép quy đổi `goal_heading` ở nhánh KHÔNG free-goal.
  — ĐO ĐƯỢC (implementer tiêm lỗi, reviewer xác nhận độc lập): để nguyên đơn vị độ thì
    toàn bộ 55 test vẫn xanh, kể cả test chạy planner thật — mission trong fixture tình
    cờ vẫn giải được nhờ góc bị wrap, không phải nhờ đúng.
  — Spec mục 4.1 nói thẳng lý do có quy ước góc duy nhất: "đường bay lệch 90 độ hoặc bị
    gương vẫn là đường bay hợp lệ về hình học, nên MỌI test hình học đều bỏ lọt". Thiếu
    test đúng ở phép quy đổi đó là đi ngược chính lập luận của spec. Finding thắng.
  — Bất đối xứng đáng ngờ: `start_heading` CÓ test, `goal_heading` thì không. Cùng một
    phép quy đổi, cùng một hàm.
  — Quyết định: thêm khẳng định ngay bây giờ, không hoãn sang task sau. Chi phí một dòng.
  — Giá nếu sai: gần 0 — thêm một test cho hành vi đã đúng.
Task 5: fix round 1/5 dispatched (resume implementer gốc) — 1 Important
Task 5: fix round 1/5 (1 addressed — test goal_heading, đỏ tại 'assert 0.785 == 45.0'; commit 0396a5b)
Task 5: complete (commits e94d03a..0396a5b, review clean sau 1 vòng sửa)
Điều phối: đồng bộ R12 ngược vào kế hoạch; sinh lại brief 7/8/9 từ kế hoạch đã vá.

Ruling R13 (T7, KHÔI PHỤC sau khi agent chết giữa chừng):
  Agent Task 7 bị API ngắt ("session limit, resets 2am") ĐÚNG lúc đang tiêm lỗi có chủ
  đích, ngay trước bước khôi phục. Nguy cơ: cây làm việc còn code hỏng cố ý.
  — ĐÃ KIỂM: suite 67/67 xanh; và quan trọng hơn, tôi so TỪNG FILE với khối code trong
    kế hoạch: planner.py KHỚP TUYỆT ĐỐI, planner_test.py KHỚP, __init__.py đúng hai dòng
    export cần thêm. Không còn lỗi tiêm. (Chỉ tin test xanh là không đủ — đó đúng là bài
    học của nhánh này.)
  — Công việc ĐÚNG nhưng CHƯA COMMIT và KHÔNG CÓ BÁO CÁO. Bằng chứng tiêm lỗi đã mất;
    agent chỉ kịp nói "all three injections caught" mà không để lại chi tiết.
  — Quyết định: controller commit phần việc đã xác minh (đây là khôi phục, không phải
    tôi sửa finding), rồi giao phần kiểm tiêm lỗi cho REVIEWER tự làm lại — thay vì tin
    một câu khẳng định của agent đã chết.
  — Giá nếu sai: bằng 0 cho phần commit (đã so từng dòng với kế hoạch). Phần tiêm lỗi
    được làm lại nên không mất gì.
Task 7: minor (deferred): fixture dùng start_heading_deg=45.0, là ĐIỂM BẤT ĐỘNG của phép
  90-x, nên lỗi "mất phép lật phương vị" vô hình ở đúng góc đó. Reviewer phát hiện khi
  phép tiêm lỗi cho FALSE NEGATIVE (math.degrees() vẫn xanh; chỉ khi bỏ hẳn quy đổi mới đỏ).
  Cùng vấn đề ở test_goal_heading_... của Task 5 (cũng 45.0).
  PHÂN TÍCH GIẢM NHẸ (tôi tự đo): 11/18 preset dùng góc KHÔNG phải điểm bất động
  (30/60/90/270), và angles_test.py ghim quy ước tại 0/90/180/270 bằng so cos+sin, cộng
  test chiều quay và test dải — một cú đảo quy ước SẼ bị bắt ở chính angles.py, nơi phép
  quy đổi thực sự sống. Nên đây là điểm yếu của test đầu-cuối, không phải lỗ hổng thật.
  ĐỀ XUẤT CHO REVIEW CUỐI: đổi fixture sang góc không bất động (30 hoặc 120) trong hai
  test đó. Một dòng mỗi chỗ. Không chặn merge.
Task 7: complete (commits 50f8270..8a99246, review clean — spec ✅, 3/3 injection RED-then-GREEN
  sau khi reviewer làm lại, cây khôi phục sạch, 1 minor deferred)
Task 8: DONE (commit e778bc6), 37/37 xanh lượt đầu, suite 104. Bỏ map_bounds: 18/18 preset trong ngưỡng 0,5% — không tốn gì đo được.
Task 8: complete (commits 8a99246..e778bc6, review clean — spec ✅, 2/2 injection RED, cây sạch).
  Ghi nhận: đổi sang planner nghiên cứu chỉ làm 5/18 preset đỏ (13/18 hai planner trùng nhau),
  nhưng thế là đủ để lưới bắt được. Bỏ map_bounds: 18/18 trong ngưỡng 0,5%.
Điều phối: vá R2 (join tiến trình mồi) + R3 (test treo hạ TIME_BUDGET_S) vào kế hoạch, sinh lại brief 9.

Ruling R14 (T6/T7/T9, LỖI HIỆU NĂNG THẬT trong kế hoạch của tôi — implementer T9 nêu như
  "test chập chờn", tôi đo ra nó lớn hơn thế nhiều):
  `planner_version()` gọi `git describe` MỖI REPLY.
  — ĐO ĐƯỢC: planner_version median 3665 ms (min 3583 / max 3963); config_hash 11,7 ms;
    `git describe` trần 3538 ms. Mission trung vị lập kế hoạch hết 16 ms.
    => service dành ~99,6% thời gian chạy git, không phải lập kế hoạch.
  — Đây cũng là nguyên nhân thật của test chập chờn ở T9 (ngưỡng 5 s bị git ăn hết),
    nên implementer đã chẩn đoán ĐÚNG nguồn gốc, chỉ đánh giá thấp hệ quả.
  — Quyết định: nhớ đệm `planner_version()` (tính một lần rồi dùng lại). Phiên bản mã
    nguồn KHÔNG đổi trong vòng đời tiến trình — README đã yêu cầu restart sau git pull —
    nên đệm là đúng ngữ nghĩa, không phải mẹo.
  — KHÔNG đệm `config_hash()`: test `test_hash_is_stable_and_sensitive` sửa
    config.NUM_START_CORNERS và đòi hash phải ĐỔI. Đệm nó sẽ phá đúng thuộc tính đó,
    và 11,7 ms là chấp nhận được.
  — Lưu ý bối cảnh: 3,5 s một phần do repo nằm trên mount 9p của WSL2; máy đích Linux
    thật sẽ nhanh hơn nhiều. Nhưng đệm vẫn đúng bất kể, và số đo tệ nhất là số đo thật.
  — Giá nếu sai: nếu ai đó commit trong lúc service chạy, reply sẽ báo phiên bản cũ.
    Đó đúng là hành vi mong muốn — reply phải mô tả mã ĐANG chạy, không phải mã trên đĩa.

Ruling R15 (T6/T9, nối tiếp R14 — implementer BÁC LẠI bản sửa của tôi và nó đúng):
  Nhớ đệm ở mức HÀM vô ích: mỗi request fork một tiến trình con mới, cache chết theo con.
  Đo lại vẫn 3,79-5,12 s/request.
  — Tôi kiểm chứng cơ chế: `forkserver.ForkServer.ensure_running` dùng `spawnv_passfds`,
    tức fork+EXEC một interpreter mới => forkserver KHÔNG thừa hưởng module đã nạp của cha,
    nên làm ấm cache trong start() cũng vô ích nốt.
  — Quyết định: chốt giá trị LÚC IMPORT. `_PRELOAD` đã có `vtx_service.planner` (import
    `runtime`), nên forkserver trả giá git describe đúng một lần và mọi con thừa hưởng.
  — KẾT QUẢ ĐO: mỗi request 3,79-5,12 s -> 1,03-1,09 s. runtime_test 15,74 -> 2,37 s.
    runner_test 44-62 -> 18,48 s. Ngưỡng envelope siết 5,0 -> 2,0 s (một ngưỡng không bao
    giờ bắt được gì thì không phải test).
  — Giá nếu sai: import `runtime` giờ chạy git một lần. Ở service đó là lúc khởi động
    (đúng chỗ); ở test là một lần mỗi phiên.
Task 9: DONE (commits fb2f4de, a07da0f, bfb52f5); injection thời hạn RED-then-GREEN
  (poll(timeout=None) -> test treo, bị timeout 20 giết ở mã 143).
Task 9: minor (deferred): không test nào khẳng định `reply.planner_version` — giá trị THỰC SỰ
  đi qua ranh giới tiến trình trong PlanRunner.submit() — bằng git describe thật. Test cơ chế
  chỉ gọi planner_version() trong tiến trình test. Là seam chưa test, không phải lỗi.
Task 9: complete (commits 11a7e2e..bfb52f5, review clean — spec ✅, quality approved, 1 minor deferred)

Ruling R16 (T10, môi trường): cài `cyclonedds` vào env Python chính.
  — Test transport dùng `pytest.importorskip("cyclonedds")`, nên nếu không cài, cả bộ test
    DDS sẽ SKIP TRONG IM LẶNG và Task 10 trông như xanh mà chưa chạy dòng nào.
  — Nằm trong phạm vi đã duyệt: service/deploy/requirements.txt của chính kế hoạch ghi
    `cyclonedds==11.0.1`. Đây là cài phụ thuộc đã công bố, không phải thay đổi bất ngờ.
  — Bắt buộc implementer XÁC NHẬN test THỰC SỰ CHẠY (không phải skipped) và báo số đếm.
  — Giá nếu sai: thêm một gói vào env Anaconda. Gỡ bằng `pip uninstall cyclonedds`.

Ruling R17 (T10, MÂU THUẪN NỘI TẠI trong kế hoạch của tôi — implementer phát hiện):
  `test_idl_has_no_frame_field` khẳng định chuỗi "frame" không có trong file IDL, nhưng
  comment IDL của chính tôi viết "Không có trường frame:" => test hỏng vì tài liệu của nó.
  — Implementer đã né bằng cách đổi từ ("hệ quy chiếu"), và BÁO CÁO rằng đây là mâu thuẫn
    của brief chứ không phải lỗi nó gây ra. Đúng cách xử lý.
  — Nhưng bản né đó để lại một test giòn: từ nay không ai được viết chữ "frame" ở bất kỳ
    đâu trong IDL, kể cả trong comment giải thích vì sao không có trường đó. Một test đã
    cho một false positive thì sẽ còn cho nữa.
  — Quyết định: đổi test sang kiểm tra KHAI BÁO TRƯỜNG, không phải chuỗi con trong cả file.
    Ý định thật của nó là "IDL không khai báo trường frame", và đó là thứ nên được khẳng định.
  — Giá nếu sai: gần 0. Test chặt hơn về ý nghĩa và nới ra đúng chỗ đáng nới (comment).
Task 10: DONE (commits 198c61a, 5983247); 4 passed 0 skipped; injection request_id -> 2/4 RED; R17 fix có cặp đỏ/xanh

Ruling R18 (T10, 2 finding Important):
  (a) `serve()` không bọc lời gọi handler => một ngoại lệ giết vòng phục vụ VĨNH VIỄN.
      Ở Task 11 `serve()` chạy ở luồng chính nên ít nhất còn ồn ào, nhưng nguyên tắc của
      cả thiết kế là suy giảm êm: một request xấu không được hạ cả service.
      Thừa hưởng nguyên văn từ brief của tôi, không phải implementer gây ra.
  (b) Regex no-frame-field có khoảng mù THẬT: `\b\w+\s+frame...` không khớp
      `sequence<Point2D> frame;` vì `>` phá vỡ liền kề `\w+`+khoảng trắng. Reviewer đã thử
      trực tiếp. Mà IDL này DÙNG chính kiểu đó cho islands/dynamic_obstacles/safezones/
      waypoints — tức kiểu mà người ta có khả năng thêm nhất. Cặp đỏ/xanh vòng trước chỉ
      thử `string frame;` nên không chạm tới khoảng mù này.
  — Quyết định (b): bỏ comment ra khỏi văn bản trước, rồi khớp theo TÊN TRƯỜNG
    (`\bframe\s*(\[...\])?\s*;`) bất kể kiểu. Bắt mọi kiểu, miễn nhiễm với comment.
  — Bài học lặp lại: một cặp đỏ/xanh chỉ chứng minh test bắt được ĐÚNG CÁI ĐÃ THỬ.
  — Giá nếu sai: (a) thêm một lớp bắt lỗi, không đổi đường thành công. (b) test chặt hơn.
Task 10: fix round 2/5 dispatched — 2 Important
Task 10: fix round 2/5 (2 addressed, 0 open — serve() guard nhiều tầng, regex tổng quát;
  commit e1a6c3d). Re-review truy đường: reply lỗi dựng TRONG vùng bắt, có try/except lồng
  thoát sớm, lỗi ghi bắt riêng => vòng lặp sống trên mọi đường. 5 passed 0 skipped.
Task 10: complete (commits bfb52f5..e1a6c3d, review clean sau 2 vòng sửa)
Task 11: DONE (commit cef2c5a); round-trip 3 wp / 320.219 km / version e1a6c3d-dirty / hash cde049c0152e5bbf; venv sạch 2 gói 116/116; tests/ 188+6
Task 11: minor (deferred): main.py import PlanStatus nhưng không dùng — import chết, thừa
  hưởng từ brief của tôi; service/ không nằm trong phạm vi ruff nên chỉ là mỹ quan.
Task 11: complete (commits e1a6c3d..cef2c5a, review clean — spec ✅, thứ tự start-trước-DDS
  và docstring còn nguyên, hướng dẫn triển khai chạy được nếu làm đúng từng bước, 1 minor)
=== TẤT CẢ 11 TASK HOÀN TẤT ===

=== REVIEW TOÀN NHÁNH: 5 Important, 6 Minor. Phán quyết R19-R23 ===

Ruling R19 (F1, seam nguy hiểm nhất): runner.py và planner.py BẤT ĐỒNG về nghĩa của 0.0.
  runtime.py ghi 0.0 = KHÔNG GIỚI HẠN; planner.py tôn trọng (`budget_s > 0.0 and ...`);
  runner.py thì không => với config.TIME_BUDGET_S = None (kiểu hợp lệ), "không giới hạn"
  biến thành SIGKILL sau 2,0 s MỖI REQUEST.
  — Quyết định: runner phải hiểu 0.0 = không giới hạn, và khi đó dùng một trần TUYỆT ĐỐI
    riêng (`unlimited_deadline_s`, mặc định 300 s) thay vì `0 + grace`. Không bỏ hẳn thời
    hạn cứng — nó là lý do tồn tại của cả kiến trúc tiến trình con.
  — Thêm cảnh báo lúc khởi động khi ngân sách là không giới hạn: với một service, đó là
    cấu hình nguy hiểm chứ không phải mặc định vô hại.
  — Giá nếu sai: 300 s là con số tôi chọn, không phải đo. Nhưng nó chỉ áp dụng ở cấu hình
    mà hôm nay đang hỏng hoàn toàn, nên mọi giá trị hữu hạn đều tốt hơn hiện trạng.

Ruling R20 (F2): lỗi validate trong `_to_domain` rơi vào catch chung => VehicleLimits toàn 0
  (lỗi client dễ mắc nhất) trả INTERNAL_ERROR thay vì INVALID_REQUEST như spec mục 6 hứa,
  và thông báo thật ("turn_radius_m phải dương") bị vứt.
  — Quyết định: dịch-và-phân loại TRƯỚC catch chung; ValueError từ tầng dataclass => 
    INVALID_REQUEST, giữ nguyên văn thông báo.
  — Giá nếu sai: gần 0 — chỉ đổi mã trạng thái và giữ lại thông tin vốn đang bị vứt.

Ruling R21 (F3): `_internal_error_reply` để planner_version="" , config_hash="",
  applied_time_budget_s=0.0 — mà 0.0 lại NGHĨA LÀ không giới hạn. Reply nói dối trên đúng
  đường mà người ta cần nó nói thật nhất.
  — Quyết định: import `runtime` trong transport (nó không phụ thuộc DDS) và điền giá trị thật.

Ruling R22 (F4, sẵn sàng vận hành): `handle()` không bao giờ log `reply.detail`, nên
  traceback của tiến trình con tới được CLIENT mà KHÔNG tới journal. Thêm log detail khi
  status khác OK, và thêm dòng INTERNAL_ERROR vào bảng chẩn đoán trong README.

Ruling R23 (F5, PLAN_BUSY): trạng thái được hứa trong spec/IDL/README nhưng KHÔNG đường nào
  sinh ra nó — `serve()` tuần tự nên không bao giờ "thấy" request khi đang bận.
  — Quyết định: KHÔNG xây cơ chế phát hiện bận (cần thêm luồng đọc, mâu thuẫn với thiết kế
    một-request-một-lúc). Thay vào đó ĐÁNH DẤU LÀ DÀNH SẴN và nói thật hành vi hiện tại:
    request đến khi bận được DDS xếp hàng (RELIABLE + KEEP_ALL) và trả lời sau, không bị từ chối.
  — Sửa cả spec mục 3/6 và README cho khớp. Một trạng thái được hứa mà không thể xảy ra thì
    tệ hơn là không có.
  — Giá nếu sai: nếu sau này cần từ chối thật thay vì xếp hàng, phải xây lại phần đó.

Triage minor hoãn (theo review cuối, tôi đồng ý):
  - fixture 45 độ: SHIP — và reviewer chỉ ra bản sửa tôi đề xuất (đổi sang 30 độ) là VÔ ÍCH:
    test đó round-trip qua song ánh nên xanh với MỌI góc khi quy ước bị đảo theo cặp.
    Quy ước thực ra được ghim bởi angles_test (cos/sin tuyệt đối tại 4 hướng chính).
    Phân tích giảm nhẹ của tôi đúng, đề xuất sửa của tôi thì sai.
  - seam reply.planner_version: SHIP (một dòng nếu muốn, không có gì rủi ro).
  - import chết: FIX, kèm hai chỗ ledger bỏ sót (runner_test Circle, transport_test time).

Ruling R24 (VÁCH ĐÁ HIỆU NĂNG IM LẶNG 50x — tôi tìm ra khi không chấp nhận
  "test envelope chập chờn do wall-clock drift"):
  — ĐO ĐƯỢC, cùng máy, cùng commit:
      sys.path sửa lúc chạy (kiểu pytest):  3,52 / 3,69 / 4,05 s mỗi submit
      PYTHONPATH qua biến môi trường:       0,07 / 0,07 / 0,08 s mỗi submit
  — NGUYÊN NHÂN: forkserver tạo bằng fork+EXEC (interpreter mới), nên thừa hưởng
    PYTHONPATH từ MÔI TRƯỜNG nhưng KHÔNG thấy sys.path sửa lúc chạy. Preload khi đó
    thất bại, và `multiprocessing` NUỐT ImportError của preload TRONG IM LẶNG =>
    mỗi tiến trình con import lại tất cả, gồm cả git describe lúc import.
  — Vì sao trước đây không thấy: systemd unit CÓ đặt PYTHONPATH, nên production đúng.
    Nhưng phụ thuộc đó vô hình và im lặng: chạy service không có PYTHONPATH thì chậm
    50x mà không một dòng lỗi nào.
  — Đây cũng là lời giải thật cho "test envelope chập chờn": test TRUNG THỰC, môi
    trường mới là chỗ hỏng. Nhận nó là "wall-clock drift" sẽ chôn luôn phát hiện này.
  — Quyết định: `PlanRunner.start()` tự bảo đảm forkserver import được preload — đặt
    PYTHONPATH (suy ra từ __file__) trước khi tạo forkserver. Bỏ hẳn vách đá thay vì
    dựa vào việc người triển khai nhớ đặt biến.
  — Giá nếu sai: sửa os.environ trong tiến trình. Chỉ THÊM đường dẫn của chính repo,
    không xoá gì; systemd vẫn đặt PYTHONPATH và hai bên nhất quán.

Ruling R25 (REGRESSION do chính đợt sửa cuối gây ra — re-review bắt được):
  Bản sửa F2 thu hẹp `except Exception` thành `except ValueError` quanh `_to_domain`.
  `serve()` KHÔNG bọc `_handle_one`, nên một ngoại lệ không-phải-ValueError khi dịch
  request giờ thoát ra và giết vòng phục vụ VĨNH VIỄN — mở lại đúng lỗ R18(a) đã đóng.
  Hôm nay còn tiềm ẩn (mọi __post_init__ hiện chỉ ném ValueError) và KHÔNG test nào phủ.
  — Đây đúng là khiếm khuyết ĐỐI XỨNG GƯƠNG mà tôi đã nêu thành câu hỏi 3 cho reviewer:
    khi sửa một lỗi phân loại sai theo một chiều, rất dễ tạo lỗi phân loại sai chiều kia.
  — Quyết định: SỬA, dù quy trình nói "không có đợt sửa thứ hai". Lý do: đây không phải
    đợt mới mà là đóng một regression do chính đợt vừa rồi tạo ra, tốn một dòng, và phương
    án thay thế là ship một cách đã biết để giết service vĩnh viễn. Bất biến "không ngoại
    lệ nào giết được serve()" đáng giá hơn việc bám đúng số vòng.
  — Giá nếu sai: thêm một nhánh except. Không đổi đường thành công, không đổi F2.

Ruling R26 (do SUITE ĐẦY ĐỦ phát hiện — test siết ở R10 đã làm đúng việc của nó):
  `test_version_matches_git_describe_inside_a_checkout` XANH khi chạy riêng, ĐỎ trong suite
  đầy đủ (120 passed, 1 failed).
  — NGUYÊN NHÂN ĐO ĐƯỢC: `git describe` trên hệ tệp này mất 3,60 / 3,97 / 4,84 s, còn
    subprocess trong `planner_version()` đặt `timeout=5`. Biên an toàn 3%. Khi máy tải nặng
    nó vượt 5 s, `TimeoutExpired` rơi vào `except (OSError, SubprocessError)` và hàm ÂM THẦM
    trả "unknown".
  — Đây ĐÚNG là chế độ hỏng mà R9/R10 sinh ra để chặn, xuất hiện lại từ một hướng khác.
    Test siết ở R10 đã bắt được. Nếu giữ test yếu ban đầu ("chuỗi khác rỗng"), lỗi này sẽ
    đi thẳng vào production và mọi reply mang phiên bản vô nghĩa.
  — Quyết định: nâng timeout lên 30 s VÀ log cảnh báo khi rơi về "unknown". Chi phí trả một
    lần lúc import, không nằm trong đường request, nên ngân sách rộng không tốn gì vận hành.
    Im lặng mới là phần không chấp nhận được, không phải thời gian.
  — Giá nếu sai: khởi động chậm hơn tối đa 30 s trong trường hợp git thực sự treo — và
    lúc đó có cảnh báo trong log thay vì một dấu phiên bản vô nghĩa.
