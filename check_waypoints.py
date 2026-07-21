"""Kiểm tra một tập waypoint có thoả điều kiện đường bay không.

Đọc danh sách waypoint từ file (JSON hoặc python-literal), rồi chạy các
validator độc lập trong core/path_validation.py.

Mỗi waypoint là dict có 'pos' (x, y) và tuỳ chọn 'heading' (rad), 'type'.
Cũng chấp nhận list [x, y] hoặc [x, y, heading].

    python check_waypoints.py cases/my_waypoints.txt
    python check_waypoints.py cases/my_waypoints.txt --scenario cases/case_1.json

Không có --scenario thì bỏ qua kiểm tra va chạm (chỉ động học).
"""
import argparse
import ast
import json
import math
import sys

import config
import core.path_validation as pv
import core.preprocessing as prep
import core.spatial_utils as su
import gui.scenario_io as sio


def load_waypoints(path):
    """Đọc file waypoint. Chấp nhận JSON hoặc python-literal, có/không dấu [] bao ngoài."""
    text = open(path, encoding='utf-8').read().strip()
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        # python-literal: hỗ trợ cả chuỗi dict phân tách bằng dấu phẩy, không có []
        try:
            data = ast.literal_eval(text)
        except (ValueError, SyntaxError):
            data = ast.literal_eval('[' + text + ']')
    if isinstance(data, dict):          # một waypoint đơn lẻ
        data = [data]
    return [_as_state(item, i) for i, item in enumerate(data)]


def _as_state(item, i):
    """Chuẩn hoá về (waypoint, heading_khai_bao_hoac_None, label)."""
    if isinstance(item, dict):
        pos = tuple(item['pos'])
        return (pos, item.get('heading'), item.get('type', f'WP{i}'))
    seq = list(item)
    return (tuple(seq[:2]), seq[2] if len(seq) > 2 else None, f'WP{i}')


def _norm(d):
    return math.atan2(math.sin(d), math.cos(d))


def load_obstacles(scenario_path):
    """Trả về (inflated_circles, inflated_polys, raw_circles, raw_polys)."""
    text = open(scenario_path, encoding='utf-8').read()
    scenario = sio.scenario_from_json(text)   # khôi phục tuple cho (x, y)
    pre = prep.prepare_scenario(scenario)
    raw_circles = [(o['center'], o['radius'])
                   for o in scenario['obstacles'] if o['type'] == 'circle']
    raw_polys = [o['polygon'] for o in scenario['obstacles'] if o['type'] == 'polygon']
    return pre['circle_obstacles'], pre['polygon_obstacles'], raw_circles, raw_polys


def report(states, obstacles=None):
    """In báo cáo chi tiết, trả về True nếu toàn bộ ràng buộc đều đạt."""
    path = [(s[0], s[1] if s[1] is not None else 0.0) for s in states]
    R, a_max, L0, dss = config.R, config.ALPHA_MAX_RAD, config.L0, config.DSS
    ok_all = True

    print(f"Số waypoint: {len(path)}   R={R:.0f} m  alpha_max={config.ALPHA_MAX:.0f} deg  "
          f"L0={L0:.0f} m  DSS={dss:.0f} m\n")

    # --- 1. Góc lượn tại các điểm trong ---
    alphas = pv.turn_angles(path)
    print("--- Góc lượn tại waypoint trong (từ hình học) ---")
    for i, a in enumerate(alphas, start=1):
        print(f"  [{i:2d}] {states[i][2]:<14} alpha = {math.degrees(a):7.2f} deg  "
              f"{'OK' if a <= a_max + 1e-9 else 'FAIL (> alpha_max)'}")
    ok_all &= pv.turn_angles_ok(path, a_max)

    # --- 2. Heading khai báo vs heading hình học ---
    declared = [s[1] for s in states]
    if any(h is not None for h in declared):
        print("\n--- Heading khai báo vs hình học ---")
        print("  (quy ước quan sát được: heading tại góc = phân giác của heading vào/ra)")
        for i, s in enumerate(states):
            if s[1] is None:
                continue
            h_in = su.angle_to_heading(path[i - 1][0], path[i][0]) if i > 0 else None
            h_out = su.angle_to_heading(path[i][0], path[i + 1][0]) if i < len(path) - 1 else None
            ref = h_out if h_in is None else (h_in if h_out is None
                                              else h_in + _norm(h_out - h_in) / 2.0)
            dev = abs(_norm(s[1] - ref))
            print(f"  [{i:2d}] {s[2]:<14} khai báo {s[1]:8.5f}  phân giác {ref:8.5f}  "
                  f"lệch {math.degrees(dev):6.3f} deg")

    # --- 3. Ràng buộc đoản trình ---
    # pv.straight_segments_ok() là "quan toà" nhưng dừng ở lỗi ĐẦU TIÊN; vòng lặp
    # dưới đây lặp lại đúng công thức của nó để in được TOÀN BỘ đoạn. Verdict cuối
    # cùng vẫn lấy từ pv.
    print("\n--- Đoản trình (l = d - R*(tan(a_i/2) + tan(a_next/2))) ---")
    a_all = [0.0] + alphas + [0.0]
    for i in range(len(path) - 1):
        d = su.distance(path[i][0], path[i + 1][0])
        l = d - R * (math.tan(a_all[i] / 2) + math.tan(a_all[i + 1] / 2))
        if i == 0:
            ok, rule = l >= L0 - 1.0, f"l1 >= L0 ({L0:.0f})"
        elif i == len(path) - 2:
            ok, rule = l - dss >= -1.0, f"l - DSS >= 0 (DSS={dss:.0f})"
        else:
            ok, rule = l >= 1.0, "l > 0"
        print(f"  seg {i:2d}: d = {d:9.1f}  l = {l:10.1f}   {rule:<24} "
              f"{'OK' if ok else 'FAIL'}")
    straight_ok, detail = pv.straight_segments_ok(path, R, L0, dss)
    ok_all &= straight_ok
    print(f"  => pv.straight_segments_ok: {'OK' if straight_ok else 'FAIL - ' + detail}")

    # --- 4. Va chạm (chỉ khi có scenario) ---
    if obstacles is None:
        print("\n--- Va chạm: BỎ QUA (không cung cấp --scenario) ---")
    else:
        inf_c, inf_p, raw_c, raw_p = obstacles
        seg_ok = pv.segments_clear(path, inf_c, inf_p)
        arc_ok = pv.arcs_clear(path, R, raw_c, raw_p)
        print(f"\n--- Va chạm ---")
        print(f"  Đoạn thẳng vs chướng ngại đã inflate: {'OK' if seg_ok else 'FAIL'}")
        print(f"  Cung lượn vs chướng ngại gốc:         {'OK' if arc_ok else 'FAIL'}")
        # Cổng tổng hợp một-lời-gọi của dự án (giống gui/summary.py dùng).
        ok_all = pv.path_is_valid(path, inf_c, inf_p, R, a_max, L0, dss,
                                  raw_circle_obstacles=raw_c, raw_polygon_obstacles=raw_p)

    print("\n" + ("KẾT LUẬN: đường bay HỢP LỆ" if ok_all
                  else "KẾT LUẬN: đường bay KHÔNG hợp lệ"))
    return ok_all


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('waypoints', help='file chứa danh sách waypoint')
    ap.add_argument('--scenario', help='file scenario JSON để kiểm tra va chạm')
    args = ap.parse_args()

    states = load_waypoints(args.waypoints)
    obstacles = load_obstacles(args.scenario) if args.scenario else None
    return 0 if report(states, obstacles) else 1


if __name__ == '__main__':
    sys.exit(main())
