"""Module cấu hình tham số hệ thống lập kế hoạch đường bay tự hành.

Định nghĩa các tham số động học, giới hạn an toàn, thông số thuật toán A*
và các hằng số hình học dùng chung trên toàn bộ hệ thống.
"""

import math


# ====== RÀNG BUỘC ĐỘNG HỌC ======
# Bán kính quay vòng R (m) - cố định cho toàn bộ quỹ đạo bay
R = 8000.0

# Góc quay vòng tối đa cho phép (độ)
ALPHA_MAX = 90.0  # tính theo độ, sẽ được đổi sang radian

# Chiều dài đoạn bay thẳng tối thiểu để ổn định bay bằng sau khi cất cánh (m)
L0 = 4000.0

# Chiều dài đoạn thẳng tự dẫn để đầu dò camera khóa mục tiêu ở pha cuối (m)
DSS = 23000.0

# Chiều dài đoạn bay thẳng tối thiểu sử dụng được giữa 2 góc rẽ liên tiếp (m), tức là
# cận dưới đoản trình cho mọi phân đoạn BÊN TRONG (các chặng đầu và cuối dùng ngưỡng lớn
# hơn là L0 / DSS tương ứng). Được đọc bởi cả hai bộ lập kế hoạch.
MIN_STRAIGHT_M = 10.0

# Độ lệch góc tối đa cho phép của dây cung làm mịn đầu tiên so với start_heading (rad).
# Không có góc rẽ nào tại điểm xuất phát O, do đó waypoint đầu tiên giữ lại phải nằm
# trên tia cất cánh; đây là chốt chặn đảm bảo tính chính xác, không phải là nới lỏng
# dung sai.
TAKEOFF_RAY_TOL_RAD = 1e-9

# Tương tự như trên ở đầu mút bên kia (rad): trong chế độ đích CỐ ĐỊNH (fixed-goal),
# chặng tự dẫn vào T phải bay dọc theo goal_heading, do đó dây cung làm mịn cuối cùng
# phải nằm trên tia tiếp cận. Không có chốt này, bộ làm mịn đoạn con sẽ tùy tiện bỏ qua
# W_{n-1} và nối thẳng vào T từ bất kỳ đâu, làm ngắn đường bay nhưng lặng lẽ tiếp cận
# sai hướng (đo đạc lệch tới 68 độ).
APPROACH_RAY_TOL_RAD = 1e-9

# ====== AN TOÀN & XỬ LÝ CHƯỚNG NGẠI VẬT ======
# Vùng đệm an toàn (m) - khoảng cách giãn nở biên chướng ngại vật
SAFE_MARGIN = 0.0

# Kiểu nối khi giãn nở đa giác: 'mitre' giữ các góc nhọn để mỗi vật cản tạo ra một vài
# đỉnh góc thực (dùng làm waypoint dẫn đường) thay vì ~70 điểm cung tròn bo cong.
# mitre_limit giới hạn độ nhọn của chóp góc; đủ lớn để đa giác mitre luôn CHỨA vùng đệm
# Minkowski tròn chính xác (bảo toàn đảm bảo an toàn cung lượn) cho các đảo gần như lồi
# ở đây.
POLYGON_MITRE_LIMIT = 5.0

# Kiểm tra va chạm là CHÍNH XÁC TUYỆT ĐỐI: bất kỳ sự xâm phạm nào vào biên ĐÃ GIÃN NỞ
# của hình tròn (dist < radius) đều là va chạm — dung sai bằng 0. Tính khả thi của hình
# học bám biên đạt được từ phía KHÂU DỰNG HÌNH: toàn bộ hình học bám biên (tiếp điểm,
# nhánh rời tiếp tuyến kép bitangent, đỉnh cung ngoại tiếp) đều được dựng trên bán kính
# r + CONSTRUCTION_CLEARANCE_M, do đó mọi dây cung do bộ lập kế hoạch tạo ra luôn giữ ít
# nhất khoảng an toàn thực đó — sai số số thực trôi float (~mm) được triệt tiêu khi dựng
# hình, không bao giờ được tha thứ khi kiểm định. Khoảng hở dựng hình (m): bán kính bổ
# sung để DỰNG hình học bám biên (r_bám = bán kính giãn nở + giá trị này). Giữ các dây
# cung tiếp tuyến đã dựng nằm hoàn toàn ngoài biên giãn nở để kiểm tra va chạm chính xác
# (dung sai 0) chấp nhận chúng với biên vượt xa sai số float. Không đáng kể về mặt hình
# học: ~1 m so với bán kính giãn nở 23-63 km.
CONSTRUCTION_CLEARANCE_M = 1.0

# ====== HỆ TỌA ĐỘ ======
# Giới hạn bản đồ (mét) cho mô phỏng
MAP_WIDTH = 500000.0
MAP_HEIGHT = 500000.0

# ====== TÌM KIẾM A* ======
# Số lượng trạng thái góc xuất phát (start-corner) được gieo mầm trên tia start-heading.
# Góc thứ i (i = 1..K, chia đều theo tan: tan(a_i/2) = (i/K) * tan(alpha_max/2)) nằm ở
# d_i = L0 + R*tan(a_i/2) và đáp ứng các góc rẽ đầu tiên alpha <= a_i trong khi vẫn giữ
# đoạn thẳng cất cánh l1 >= L0 chính xác. Nhóm K tái lập điểm W1 xấu nhất trước đây, do
# đó NUM_START_CORNERS = 1 chính là hành vi kế thừa cũ.
NUM_START_CORNERS = 4

# Quỹ thời gian thực cho một lần tìm kiếm (giây). Đây là điều kiện dừng DUY NHẤT của tìm
# kiếm -- giới hạn MAX_ITERATIONS cũ đã bị loại bỏ: số lần lặp không phải là đại lượng
# mà người vận hành có thể ước lượng trực quan, và hai điều kiện dừng độc lập từng khiến
# quá trình tìm kiếm kết thúc vì lý do mà kết quả không nêu rõ.
#
# Phải là một số hữu hạn > 0; không còn khái niệm "không giới hạn". Một tìm kiếm không
# có giới hạn có thể treo GUI hoặc service worker vô thời hạn, và giá trị từng biểu diễn
# điều đó (None) cũng bị đọc thành "0.0 = không giới hạn" trên đường truyền service,
# khiến hạn mức cấp yêu cầu bằng 0 lặng lẽ trở thành vô hạn.
#
# Đây chỉ là GIÁ TRỊ MẶC ĐỊNH. Mọi điểm vào đều nhận tham số `time_budget_s=` và chỉ
# dùng giá trị này khi bên gọi không truyền gì.
#
# LƯU Ý điều này làm cho tìm kiếm phụ thuộc vào thời gian thực: cùng một nhiệm vụ trên
# máy chạy chậm hơn sẽ duyệt ít nút hơn và có thể trả về đường bay khác. Cần đo đạc kiểm
# thử với hạn mức đủ lớn để không bị giới hạn bởi thời gian.
TIME_BUDGET_S: float = 15.0


def resolve_time_budget_s(value: float | None = None) -> float:
    """Xác thực và chuyển đổi hạn mức thời gian tìm kiếm từ tham số đầu vào.

    Một định nghĩa duy nhất về "đây có phải là hạn mức dùng được không",
    dùng chung cho cả hai bộ lập kế hoạch và service microservice, đảm bảo
    giá trị bị từ chối ở một nơi thì cũng bị từ chối ở mọi nơi khác.

    Args:
        value: Số giây, hoặc ``None`` biểu thị "bên gọi không chỉ định". ``None``
            KHÔNG PHẢI là "không giới hạn": tìm kiếm không giới hạn có thể treo GUI
            hoặc service worker mãi mãi, nên không có cách nào yêu cầu điều đó.

    Returns:
        Hạn mức thời gian thực thi tính bằng giây.

    Raises:
        ValueError: Nếu hạn mức sau xử lý không phải là số hữu hạn > 0. NaN bị từ
            chối tường minh vì mọi phép so sánh với nó đều là False, khiến nó
            lặng lẽ vô hiệu hóa hạn chót thay vì kích hoạt nó.
    """
    budget = TIME_BUDGET_S if value is None else value
    if math.isnan(budget):
        raise ValueError(
            f"time budget must be a finite number of seconds, got {budget!r}"
        )
    budget = float(budget)
    if budget <= 0.0 or math.isinf(budget):
        raise ValueError(f"time budget must be finite and > 0 seconds, got {budget!r}")
    return budget


# Rời rạc hóa lưới trạng thái (state-lattice quantisation) để khử trùng lặp trong A*
STATE_POS_QUANTUM = 1000.0  # mét
STATE_HEADING_QUANTUM_DEG = 3.0  # độ

# Trọng số hàm Heuristic (1.0 = Dijkstra, > 1.0 = tham lam hơn)
HEURISTIC_WEIGHT = 1.0

# Ngưỡng khoảng cách xác định đã đến đích (mét)
GOAL_THRESHOLD = 1.0  # mét; khả thi với STATE_POS_QUANTUM

# Chi phí cộng thêm cho mỗi radian đổi hướng tại một bước chuyển tiếp (mét / radian)
TURN_PENALTY_WEIGHT = 0  # 4000.0

# Chiến lược dự phòng cho A* khi không tìm thấy trạng thái kế tiếp hợp lệ: quạt nan
# hướng tâm (radial fan)
RADIAL_FAN_DIRECTIONS = 3  # số hướng trong quạt nan

# Số lượng nấc KHOẢNG CÁCH phát ra cho mỗi hướng quạt nan. Một nhánh quạt phải bao phủ
# dự trữ gần + dự trữ xa + mức đoản trình tối thiểu:
#
# d_j = R*tan(theta/2) + R*tan(beta_j/2) + RADIAL_FAN_STEP_M
#
# trong đó theta là góc rẽ của chính quạt nan (đã biết) và beta_j là góc rẽ TIẾP THEO
# tại điểm xoay pivot (hoãn lại bởi _doan_trinh). Code cũ từng gán cứng beta =
# alpha_max, nên mỗi nhánh đều phải chịu khoản dự trữ xa xấu nhất ngay cả khi điểm xoay
# hầu như không đổi hướng — gây ra độ phồng vô điều kiện trên các đường bay qua quạt nan
# ở vùng biển thoáng.
#
# Nấc j thay vào đó là "đoạn ngắn nhất vẫn đáp ứng được góc rẽ tiếp theo beta <=
# beta_j", chia đều theo tan tương tự như NUM_START_CORNERS: tan(beta_j/2) =
# (j/M)*tan(alpha_max/2), do đó dự trữ xa đơn giản là R*(j/M)*tan(alpha_max/2) — tuyến
# tính theo j, không cần hàm lượng giác trong vòng lặp.
#
# Khoảng cách giữa các nấc là R*tan(alpha_max/2)/M, phải luôn lớn hơn STATE_POS_QUANTUM
# nếu không các nấc liền kề sẽ bị gộp chung vào cùng một ô khử trùng lặp
# => M <= 8 với các giá trị mặc định R / alpha_max / quantum. M = 2 cho 4 km.
#
# M = 2 là giá trị ĐO ĐẠC THỰC TẾ, không phải giả định. Trên các nhiệm vụ hướng ngược
# bất lợi không có vật cản (trường hợp mà bậc thang khoảng cách nhắm tới), bản cũ vs M,
# 22 cặp hướng start/goal:
#
# M=2  0 lỗi, 12 tốt hơn, giảm ròng -49.6 km   <- tốt nhất M=3  4 lỗi,  7 tốt hơn, tăng
# ròng  +2.1 km M=4  4 lỗi, 12 tốt hơn, giảm ròng -23.5 km
#
# M >= 3 đẩy các ca khó vượt quá quỹ thời gian tìm kiếm (phân nhánh x3-x4), làm mất các
# nhiệm vụ mà bản cũ giải được — và các ca bị mất lại chính là nơi bậc thang thắng lớn
# nhất (start 90 / goal 90: 458.4 -> 410.6 km tại M=2, THẤT BẠI tại M=3+). Mối quan hệ
# KHÔNG đơn điệu theo M: lưới khử trùng lặp thô làm cho M=3 tệ hơn cả 2 mức lân cận. Hãy
# đo đạc lại trước khi thay đổi giá trị này.
#
# Núm điều chỉnh A/B: NUM_FAN_DISTANCES = 1 cùng với RADIAL_FAN_STEP_M = 1000.0 tái lập
# chính xác một nhánh xấu nhất duy nhất của bản cũ.
NUM_FAN_DISTANCES = 2

# Đoạn thẳng đệm cộng thêm vào mỗi nấc quạt nan trên hai mức dự trữ góc rẽ. Mức tối
# thiểu trên lý thuyết là _MIN_STRAIGHT_M của bộ lập kế hoạch (10 m), nhưng _doan_trinh
# tính toán lại R*tan(turn/2) từ góc đã qua biến đổi vòng tròn _angle_diff, do đó một
# nấc được dựng sát sạt ngưỡng có thể bị loại bỏ bởi sai số float. 100 m vượt xa sai số
# đó ~10 bậc độ lớn trong khi vẫn cắt giảm được 90% đoạn đệm 1000 m cũ.
RADIAL_FAN_STEP_M = 100.0

# Trượt pivot dọc tia (Along-ray pivot slide): số vị trí thử lại khi một ứng viên Chiến
# lược A (Strategy-A) bị từ chối (thường bởi _is_corner_arc_clear tại một đỉnh bao lồi
# đa giác, nơi cung fillet lấn vào đa giác chứa đỉnh đó).
#
# Điểm pivot trượt TIẾN VỀ PHÍA TRƯỚC dọc theo hướng bay vào, P' = P + d*h_in, do đó
# đoạn bay vào giữ nguyên HƯỚNG và chỉ dài thêm: góc cua của nút cha, dự trữ góc rẽ và
# mọi tổ tiên đều giữ nguyên tính hợp lệ theo cấu trúc dựng hình, và quỹ đoạn thẳng chỉ
# có thể tăng lên. Việc trượt dọc theo phân giác ngoài sẽ làm xoay đoạn bay vào và buộc
# phải kiểm định lại các tổ tiên (phiên bản không dừng của ý tưởng này).
#
# Với h_in là trục x và V - P = (a, b), góc rẽ mới là |atan2(b, a - d)|, TĂNG DẦN theo d
# — do đó độ trượt bị chặn ở d_max = a - |b|/tan(alpha_max) và độ phồng fillet tăng lên
# khi được sửa lỗi. Các vị trí thử lại do đó được tham số hóa theo góc rẽ KẾT QUẢ, trong
# các nhóm năng lực chia đều theo tan: tan(alpha_i/2) = (i/K)*tan(alpha_max/2) — cùng
# quy tắc như NUM_START_CORNERS và NUM_FAN_DISTANCES — và nhóm đầu tiên thỏa mãn sẽ được
# chọn (độ trượt nhỏ nhất = đường vòng ngắn nhất).
#
# 0 sẽ tắt hoàn toàn cơ chế này (núm điều chỉnh A/B).
NUM_PIVOT_SLIDES = 4

# Độ trượt ngắn nhất đáng để phát sinh (m). Nhóm nào có d thấp hơn ngưỡng này chỉ khác
# góc chưa trượt bởi sai số float, do đó sẽ tốn toàn bộ chi phí kiểm tra va chạm + cung
# lượn để kiểm tra lại hình học vừa mới bị từ chối.
MIN_PIVOT_SLIDE_M = 1.0

# Bước tiếp tục bay thẳng rời khỏi biên hình tròn đã giãn nở (m), chỉ dùng bởi bản thử
# nghiệm ưu tiên tính dễ đọc core/kinodynamic_astar_v0.py; bộ lập kế hoạch chính đã thay
# thế bước bọc này bằng _arc_hop_successors, vốn không cần tham số bước nào cả.
WRAP_STEP_M = 10000.0

# Số lượng nút O..T lớn nhất mà smooth_path sẽ chạy quy hoạch động (DP) chính xác. Thuật
# toán DP có độ phức tạp O(m^3) bước chuyển tiếp với một lần kiểm tra cung rẽ mỗi bước,
# hoàn toàn không đáng kể ở quy mô mà bộ lập kế hoạch này tạo ra (đo đạc trên 114 đường
# bay: trung vị 9 nút, tối đa 21 nút, 2.4 ms mỗi đường bay) nhưng sẽ lãng phí trên đầu
# vào bất thường. Vượt quá ngưỡng này, đường bay sẽ được trả về dạng chưa làm mịn thay
# vì tiêu tốn quỹ thời gian tại đây.
SMOOTH_MAX_NODES = 64

# Phân định hòa (tie-break) cho DP của smooth_path: số mét chiều dài đường bay tương
# đương giá trị của một waypoint được giữ lại. DP chỉ tối thiểu hóa chiều dài đơn thuần,
# và một waypoint mà khí tài bay THẲNG qua tiêu tốn đúng 0 mét chiều dài -- đo đạc từng
# bit bằng nhau trên seed 34, nơi dây cung qua ba waypoint trượt-pivot/quạt-nan đều ra
# 28299.999971999972 m dù theo cách nào -- do đó nếu không có phân định hòa, DP sẽ giữ
# hoặc bỏ các waypoint đó một cách tùy tiện, và kế hoạch bàn giao sẽ mang theo các
# waypoint không đánh dấu thao tác đổi hướng nào. Phạt mỗi waypoint 1 mét giúp đường bay
# ngắn nhất cũng là đường bay có ít waypoint nhất. Nó cũng giới hạn chi phí mà sự ưu
# tiên đó phải trả: tối đa bấy nhiêu mét cho mỗi waypoint được loại bỏ, ~5 phần triệu
# của nhiệm vụ 200 km. Đây là TIÊU CHÍ PHÂN ĐỊNH HÒA, không phải mục tiêu chính -- tăng
# nó đủ lớn để đổi lấy chiều dài thực tế là một quyết định khác và cần đo đạc riêng.
SMOOTH_NODE_PENALTY_M = 1.0

# Độ thụt vào (m) của các bản sao đa giác thu nhỏ CHỈ dùng để ngắt sớm phép đo giao cắt
# phần ruột chính xác trong _is_collision_free. Dây cung có phần ruột lấn vào bản sao
# thu nhỏ sẽ chồng lấn đa giác thực tế nhiều hơn khoảng cách này và chắc chắn bị chặn,
# do đó không bao giờ cần đo đạc; phép đo tốn 63 us so với 12 us của vị từ, và 14.4% các
# lần gọi va chạm trúng vào đa giác (đo trên 60 kịch bản). Chỉ các dây cung sượt qua
# biên nông hơn mức này mới phải chuyển tiếp sang kiểm tra chính xác -- 8 trên 77333 lần
# chạm. Đây là chốt TỐI ƯU HIỆU NĂNG, không phải dung sai: nó chỉ có thể bỏ qua xử lý
# trên các dây cung vốn đã bị chặn, không bao giờ tha thứ cho dây cung nào. Hãy giữ nó
# cao hơn nhiều POLYGON_TOUCH_TOL_M (ngưỡng thực, 1e-6 m) và thấp hơn nhiều bất kỳ
# khoảng cách vận hành nào.
POLYGON_DEEP_HIT_INSET_M = 1e-3

# Dải bảo vệ (rad) cho bộ lọc trước góc rẽ giá rẻ trong _pivot_candidate. Cửa chặn chính
# xác là |turn| <= alpha_build với turn lấy từ atan2; dạng tích vô hướng tương đương
# (dot >= cos(alpha_build) * seg_len) giống nhau về mặt toán học nhưng không giống nhau
# từng bit ở gần giới hạn, và các góc rẽ rơi ĐÚNG trên giới hạn rất thường xuyên ở đây
# (0.31% quyết định góc rẽ nằm trong phạm vi 1e-12 rad của alpha_max). Vì vậy bộ lọc
# trước chỉ từ chối những gì vượt quá giới hạn NHIỀU HƠN MỨC NÀY, và bất kỳ giá trị nào
# nằm trong dải bảo vệ đều được chuyển sang kiểm tra chính xác -- dạng giá rẻ không bao
# giờ là bên đưa ra phán quyết ở các trường hợp ranh giới. 1e-6 rad xấp xỉ 1e10 lần sai
# số tương đối của chính tích vô hướng và vẫn đủ hẹp để 55% ứng viên bỏ qua được atan2.
TURN_PREFILTER_BAND_RAD = 1e-6

# Quỹ van xả (escape-valve): số lượt mở rộng CŨNG có thể nhận quạt nan hướng tâm khi
# đích bị che khuất tầm nhìn line-of-sight (các bước tái định hướng giá rẻ, ví dụ khôi
# phục sau khi góc hướng ban đầu ngược bất lợi). Quạt nan dự phòng/bám biên không bị
# tính quỹ.
#
# CẢNH BÁO — MỘT TÊN GỌI, HAI Ý NGHĨA. Đây là ngoại lệ đối với quy tắc "mỗi hằng số một
# ý nghĩa", và đây không phải là điều cố ý: v0 (bộ lập kế hoạch ĐÃ ĐÓNG GÓI) đọc nó như
# một bộ đếm TOÀN CỤC cho cả quá trình tìm kiếm, trong đó các góc xuất phát được MIỄN
# TRỪ, và được nạp lại khi biên mở rộng gần cạn kiệt. v0 hoàn toàn không đọc
# STRATEGY_B_CONSECUTIVE hay STRATEGY_B_GLOBAL_CAP, do đó đoạn dưới chỉ mô tả main. main
# đọc nó như giới hạn liên tiếp TRÊN MỖI ĐƯỜNG BAY (State.consec_b), vì
# STRATEGY_B_CONSECUTIVE được bật True. Do đó "3" là 3-trong-toàn-bộ-tìm-kiếm ở một bên
# và 3-liên-tiếp-trên-mỗi-đường-bay ở bên kia. Đừng suy luận hành vi của bộ lập kế hoạch
# này từ con số của bộ lập kế hoạch kia.
#
# Tác dụng thực sự của dạng toàn cục, đo đạc trên v0 qua 100 seed tự do: cửa chặn được
# chạm tới 8,967 lần và NGĂN CHẶN 8,747 lần trong số đó (97.5%) — quỹ cạn kiệt gần như
# ngay lập tức và sau đó luôn rỗng, do đó nó hoạt động như một công tắc tắt quạt nan khi
# đích bị che khuất hơn là một hạn mức cho phép. Chỉ có 220 lần kích hoạt thực sự tiêu
# thụ nó. Trong khi đó 89% của 2,016 lần kích hoạt thực tế của quạt nan không bao giờ
# tham vấn nó (góc xuất phát 348, đích đã thông thoáng, hoặc dự phòng khi không có trạng
# thái kế tiếp), và nó không hề giới hạn số bước liên tiếp: các chuỗi quạt nan vẫn đạt
# tới độ sâu 6, và 16% số lần kích hoạt nằm trên một trạng thái đã sâu 3+ bước quạt nan.
NUM_STRATEGY_B = 3

# CHỈ DÀNH CHO CHẾ ĐỘ TIẾP CẬN ĐÍCH TỰ DO (FREE-GOAL): bỏ qua quạt nan hướng tâm khi
# đích thông thoáng tầm nhìn line-of-sight và vấn đề duy nhất của đoạn bay thẳng là
# không đủ chiều dài đoạn tự dẫn DSS. Được đọc bởi core/kinodynamic_astar_v0.py; False
# là hành vi kế thừa cũ.
#
# Quạt nan không thể giải quyết vấn đề đó. Mọi nhánh quạt đều rẽ ở góc +-alpha_max hoặc
# bay thẳng về phía trước, ở một khoảng cách nấc cố định, và không nhánh nào nhắm về
# phía đích, do đó kích hoạt ở đây chỉ làm ngập lưới trạng thái gần mục tiêu. Đo đạc
# trên 300 seed tiếp cận tự do: 1,108 lần kích hoạt đóng góp KHÔNG một waypoint nào cho
# bất kỳ đường bay bàn giao nào.
#
# CỐ Ý GIỚI HẠN Ở CHẾ ĐỘ TIẾP CẬN TỰ DO, vì cùng một sự từ chối đó mang ý nghĩa khác
# trong chế độ đích cố định: ở đó vấn đề là "không thể rẽ vào goal_heading", bài toán
# tiếp cận pha cuối. Trường hợp đó chiếm 43.6% tổng số lần kích hoạt và THỰC SỰ đóng góp
# vào đường bay (143 waypoint trên 300 seed); việc bỏ quạt nan ở đó làm tăng chiều dài
# +0.426% với một seed tăng +40%. Quạt nan là công cụ thô sơ cho việc này -- công cụ
# đúng đắn là phát bắn analytic goal shot mà v0 chưa có -- nhưng công cụ thô sơ vẫn hơn
# là không có gì.
#
# Lưu ý điều này được thu hẹp một cách có chủ ý so với điều kiện "đích thông thoáng tầm
# nhìn". Các lần kích hoạt khác khi đích thông thoáng của quạt nan mang tính đa dạng hóa
# lưới trạng thái hơn là tiến triển: mở rộng việc bỏ qua cho tất cả chúng làm seed 51
# tăng +73.5%. Và lưu ý kết quả phủ định bên cạnh -- bỏ qua quạt nan khi đích ĐÃ LÀ một
# trạng thái kế tiếp được chấp nhận thoạt nhìn cũng tưởng là miễn phí (không đóng góp
# waypoint nào) nhưng đo đạc thực tế TỆ HƠN trên cả hai trục (+0.0376% chiều dài, +0.93%
# số lần lặp), do đó không có đóng góp trực tiếp không đồng nghĩa với vô giá trị ở đây.
FAN_SKIP_ON_SHORT_RUNIN = True

# Diễn giải của NUM_STRATEGY_B — CHỈ TRONG BỘ LẬP KẾ HOẠCH CHÍNH (MAIN). v0 không bao
# giờ đọc cờ này; nó được gán cứng theo nhánh "False" mô tả bên dưới. False (kế thừa) =
# quỹ TOÀN CỤC: tối đa NUM_STRATEGY_B lần mở rộng quạt nan tái định hướng khi bị che
# khuất trong TOÀN BỘ tìm kiếm (miễn trừ các góc xuất phát), được nạp lại khi biên mở
# rộng gần cạn. True (LAI / HYBRID) = giới hạn TRÊN MỖI ĐƯỜNG BAY tối đa NUM_STRATEGY_B
# waypoint quạt nan LIÊN TIẾP trên bất kỳ đường bay đơn lẻ nào (mỗi State mang bộ đếm B
# liên tiếp; một bước không phải quạt nan sẽ đặt lại nó — ý định ban đầu, Wi..Wi+k đều
# là Strategy-B) CỘNG VỚI một van an toàn toàn cục STRATEGY_B_GLOBAL_CAP TỔNG SỐ lần
# kích hoạt quạt nan tái định hướng khi bị che khuất. Giới hạn trên mỗi đường bay chi
# phối bản đồ thông thường (chất lượng hướng bất lợi tốt hơn); van toàn cục (KHÔNG nạp
# lại) ngăn chặn sự bùng nổ biên mở rộng mà quy tắc thuần túy trên mỗi đường bay gây ra
# trên các bản đồ bất thường (ví dụ một seed hợp lệ nếu không sẽ bị hết thời gian
# timeout). Các góc xuất phát KHÔNG được miễn trừ (consec_b bắt đầu từ 0).
#
# MẶC ĐỊNH True (lai) từ commit fd7584d, áp dụng như một phần của việc loại bỏ macro bay
# lượn (loiter). Nó từng được giới thiệu dạng tùy chọn (OPT-IN) trong commit 0a0eacf, và
# đoạn văn này từng giữ nguyên rất lâu sau khi giá trị đã đổi — hãy nhìn vào giá trị
# thực tế, không nhìn vào văn phong cũ. Đặt False sẽ trả về quỹ toàn cục cũ, cũng là
# chính xác những gì v0 làm.
#
# Được áp dụng trên 40 seed hướng bất lợi ngẫu nhiên (trên mỗi đường bay
# NUM_STRATEGY_B=3, GLOBAL_CAP=50) so với global5: +6 seed ngắn hơn (giảm ròng -33.9 km,
# không có seed nào dài hơn) và sửa được vòng lặp rộng của scenario_3 (307.8 -> 291.7
# km, 17 -> 7 wp). Đây KHÔNG PHẢI là một thắng lợi tuyệt đối vào thời điểm đó — một seed
# bị tụt lùi từ hợp lệ -> va chạm tự thân path_self_collision (việc mở rộng thêm quạt
# nan đưa ra một đường bay ngắn hơn nhưng kiểm định oracle CUỐI CÙNG lại đánh trượt; các
# kiểm tra trong lúc tìm kiếm khi đó chưa khớp với oracle). Cảnh báo đó được ghi lại như
# LỊCH SỬ: nó chưa được đo đạc lại kể từ đó, và phần lớn sự lệch pha giữa tìm
# kiếm/oracle bị đổ lỗi đã được khép lại trong thời gian qua (cửa kiểm tra cung fillet
# arc, đo đạc giao cắt ruột, cửa kiểm tra góc rẽ chính xác của bộ làm mịn). Cần đo đạc
# lại trước khi trích dẫn nó như một rủi ro hiện hữu.
STRATEGY_B_CONSECUTIVE = True

# Van an toàn toàn cục cho chế độ Strategy-B LAI (HYBRID): TỔNG số lần kích hoạt quạt
# nan tái định hướng tối đa khi bị che khuất trong một lần tìm kiếm trước khi quạt nan
# bị ngắt hoàn toàn (giới hạn cứng, không nạp lại). Đủ lớn để các nhiệm vụ hướng bất lợi
# (giải xong trong vài trăm lần mở rộng) giữ được chuỗi quạt nan trên mỗi đường bay; đủ
# nhỏ để chặn đứng sự bùng nổ trên mỗi đường bay trước khi hết quỹ thời gian. Chỉ được
# tham vấn khi STRATEGY_B_CONSECUTIVE là True, tức là CHỈ bởi bộ lập kế hoạch CHÍNH.
#
# 50 là giá trị đã được xác thực trong 0a0eacf (sửa seed32/seed18 nơi các giới hạn thấp
# hơn vẫn thất bại/dài hơn, không tệ hơn 150 ở phần còn lại). Hiện tại là 100, và đây
# KHÔNG PHẢI là con số qua đo đạc: fd7584d đưa nó lên 500 cùng với việc loại bỏ bay lượn
# loiter, và 60359b1 — bản sửa smooth_path mà thông điệp commit không hề nhắc tới quạt
# nan — đưa nó về 100. Hãy coi 100 là chưa được xác thực cho đến khi có ai đó chạy đo
# đạc lại.
#
# Mặc dù được diễn đạt ở trên là ("van an toàn"), giới hạn này thực chất là thứ chi phối
# bộ lập kế hoạch chính: trên 100 seed tự do nó ngăn chặn 16,806 lần kích hoạt so với
# 2,457 lần của giới hạn trên mỗi đường bay, do đó nó là bộ giới hạn sơ cấp và
# NUM_STRATEGY_B là thứ cấp — ngược lại với những gì hai tên gọi gợi ý.
STRATEGY_B_GLOBAL_CAP = 100

# ====== GOAL SHOT (bước nối giải tích vào đích) ======
# Mở rộng giải tích kiểu Hybrid-A*. Từ mỗi trạng thái lấy ra khỏi hàng đợi OPEN, bộ lập
# kế hoạch thử một thao tác cơ động 2 góc rẽ hợp lệ theo động học phương tiện thẳng tới
# đích, tiếp cận trong phạm vi alpha_max của goal_heading; đường bay hợp lệ không va
# chạm sẽ được BƠM vào OPEN với chi phí g thực tế của nó (h=0), không trả về ngay lập
# tức — khối chấp nhận đích thông thường chỉ tiếp nhận nó khi nó nổi lên thành nút có
# chi phí rẻ nhất trên biên mở rộng, nhờ đó phát bắn triệt tiêu được sự tràn ngập tiếp
# cận bất lợi (hàm heuristic Euclid mù hướng tiếp cận pha cuối, khiến các trạng thái
# lệch hướng tích tụ gần đích) MÀ KHÔNG làm giảm chất lượng đường bay so với A* thuần.
# Chỉ dành cho chế độ đích cố định (fixed-goal) — chế độ đích tự do vốn đã rất nhanh.
GOAL_SHOT_ENABLED = True

# Thử nghiệm phát bắn sau mỗi N trạng thái lấy ra khỏi OPEN. Việc kiểm tra rất rẻ (lọc
# góc, sau đó tối đa vài phép kiểm tra va chạm 2 đoạn thẳng), do đó 1 (mỗi lần lấy ra)
# là hoàn toàn ổn; chỉ tăng giá trị này để hạn chế chi phí mỗi lần pop trên các bản đồ
# cực dày đặc nơi phát bắn hiếm khi kết nối thành công.
GOAL_SHOT_EVERY_N = 1

# Độ phân giải quét ứng viên: các hướng rẽ tại P trên dải [h ± alpha_max] và các hướng
# tiếp cận trên dải [goal_heading ± alpha_max]. Lưới này bao quanh thao tác cơ động 2
# góc rẽ ngắn nhất; lưới mịn hơn tìm ra đường ngắn hơn (và đôi khi tìm ra đường HỢP LỆ
# trong khi lưới thô chỉ tìm ra đường vi phạm đoản trình). ĐO ĐẠC THỰC TẾ trên 40 seed
# hướng bất lợi vùng biển thoáng (được oracle kiểm định, do đó các lựa chọn lại không
# hợp lệ được tính là thất bại chứ không phải thành công ngầm): 9x9   -> 38/40 hợp lệ,
# khoảng cách chiều dài trung bình so với Dubins-LB là 6.8% 25x25 -> 40/40 hợp lệ, 5.3%
# (sửa các seed 15,35 mà 9x9 chỉ có thể chạm tới với đường bay không thể bay được; -1.5
# điểm % trên các seed hợp lệ ở cả hai) 15x15 không đơn điệu (sửa được 15,35 nhưng làm
# hỏng 2,10) và 19x19 thất bại ở trường hợp đảo ngược hoàn toàn — 25x25 là điểm tối ưu.
# Chi phí trên mỗi lần pop là không đổi (phát bắn trả về sớm qua cơ chế inject); chỉ
# tăng GOAL_SHOT_EVERY_N nếu bản đồ dày đặc vật cản làm cho lưới quét 25x25 mỗi lần pop
# tốn kém. Cần thử nghiệm A/B trước khi thay đổi — chất lượng không đơn điệu theo mật độ
# trạng thái kế tiếp ở đây.
GOAL_SHOT_DIRS = 25

# Hai trục KHÔNG đối xứng, và lưới 25x25 đã che giấu điều đó trong một thời gian dài.
#
# Chiều dài thao tác cơ động 2 góc rẽ có công thức giải tích đóng chính xác (đã xác minh
# với sai số tương đối 1.2e-10 trên 50,111 cấu hình hợp lệ):
#
# L = |PG| * cos((a - b)/2) / cos((a + b)/2)
#
# với a, b là độ lệch tại P và tại G so với phương vị P->G. Vì |a + b| chính là góc rẽ
# tại góc trung gian C, mẫu số là cos(turn_C/2): chiều dài bị chi phối bởi góc rẽ đó, và
# việc tiếp cận một mục tiêu ĐẢO HƯỚNG cần độ vung góc tối đa, đẩy góc tiếp cận ra BIÊN
# của hình nón. Đo đạc trên 80 nhiệm vụ đảo hướng, nơi mà ứng viên chiến thắng nằm:
#
# hình nón tiếp cận   90.5% nằm chính xác tại biên, 99.99% nằm trong phạm vi 2 mẫu của
# biên, 0.0% nằm ở giữa             -> 25 mẫu không phân giải thêm được gì góc rẽ tại P
# 0.13% nằm trong phạm vi 2 mẫu của biên, trải rộng khắp bên trong -> trục này thực sự
# cần 25 mẫu
#
# Vì vậy hình nón tiếp cận được cắt giảm xuống 3 điểm cực trị và giữ nguyên DIRS. Đo đạc
# thực tế, bộ kiểm thử hướng bất lợi / 40 bản đồ benchmark đảo hướng, so với 25x25:
#
# 25x3 (ở đây)  giải cùng các nhiệm vụ, +0.125% chiều dài, lưới từ 625 -> 75 điểm, thời
# gian hướng bất lợi giảm từ 27.6s -> 20.0s 13x3          cùng số bài giải được,
# +0.101%, nhưng tốn 113,951 lần lặp so với 81,174 9x3          cùng số bài giải được,
# +0.340%, 136,687 lần lặp 5x3          cùng số bài giải được, +0.493%, 227,659 lần lặp
# và CHẬM HƠN về tổng thể
#
# Lưu ý cái bẫy trong bảng trên: lưới nhỏ hơn không tự động đồng nghĩa với rẻ hơn. Phát
# bắn kết nối ít hơn, nên tìm kiếm phải làm việc nhiều hơn, và 5x3 kết thúc chậm hơn
# 25x25 dù lưới rẻ hơn gấp 8 lần. Cắt giảm trục không mang lại độ phân giải, chứ đừng
# cắt trục có tác dụng. (Ghi chú cũ "9x9 -> 38/40 hợp lệ" không hề mâu thuẫn: 9x9 cắt CẢ
# HAI trục, và DIRS là trục không được phép cắt.)
#
# CẢ HAI bộ lập kế hoạch đều đọc giá trị này, và chúng nằm trong các chế độ khác nhau.
# v0 chỉ kích hoạt phát bắn trên các tiếp cận đảo hướng (GOAL_SHOT_MIN_REVERSAL_DEG),
# nơi hình nón co về các biên của nó, do đó giá trị 3 là hoàn toàn miễn phí ở đó — cả
# hai lần quét chuẩn đều giống nhau từng bit vì phát bắn không bao giờ kích hoạt trên
# chúng. Bộ lập kế hoạch CHÍNH vẫn kích hoạt phát bắn trên mọi nhiệm vụ đích cố định,
# bao gồm các nhiệm vụ cùng hướng nơi hình nón mịn hơn thỉnh thoảng có ích, nên nó phải
# trả một chút chi phí: đo đạc 300 seed cố định, 243 -> 243 bài giải được, chiều dài
# +0.0261% (21 seed dài hơn, 1 ngắn hơn, 221 giống hệt, seed tệ nhất 138 tăng +1.457%),
# số lần lặp +3.23%. Việc áp dụng cùng cửa chặn đảo hướng cho main sẽ loại bỏ chi phí
# đó; cho đến lúc đó thì đây là cái giá của việc duy trì một hằng số với một ý nghĩa duy
# nhất.
GOAL_SHOT_CONE = 3

# Chỉ kích hoạt phát bắn trên các nhiệm vụ có HƯỚNG TIẾP CẬN ĐẢO NGƯỢC: góc giữa
# goal_heading và phương vị start->goal phải đạt tối thiểu bấy nhiêu độ. Được đọc bởi
# core/kinodynamic_astar_v0.py; 0.0 sẽ kích hoạt trên mọi nhiệm vụ đích cố định (hành vi
# kế thừa cũ, và là những gì bộ lập kế hoạch chính vẫn đang làm).
#
# ALPHA_MAX là ngưỡng có lý do chặt chẽ, không phải qua tinh chỉnh mò mẫm. Dưới ngưỡng
# đó, một đoạn bay thẳng vào đích vẫn có thể rẽ vào goal_heading chỉ bằng MỘT góc rẽ,
# thứ mà ứng viên đích Chiến lược A thông thường đã tự dựng được; trên ngưỡng đó, một
# góc rẽ không thể làm được và pha cuối cần 2 góc rẽ mà phát bắn tổng hợp. Vì vậy điều
# này kích hoạt phát bắn chính xác tại nơi mà nó là giải pháp duy nhất hoàn thành nhiệm
# vụ.
#
# Điều này rất quan trọng vì phát bắn là BẢO HIỂM, không phải công cụ tăng tốc, và phí
# bảo hiểm là khá đắt. Đo đạc trên v0, đích cố định, 300 seed ngẫu nhiên: phát bắn được
# thử 55,184 lần, kết nối 4,663 lần, và 87 trong số đó (0.16% số lần thử) lọt vào đường
# bay bàn giao -- đổi lại -0.22% chiều dài với chi phí +26% thời gian thực. Trên các
# nhiệm vụ thực sự đảo hướng, chính đoạn code đó giúp giải thêm 10 nhiệm vụ và giảm -77%
# thời gian thực (bộ kiểm thử 144 ca hướng bất lợi: 131/144 tại 1,177,550 lần lặp khi
# không có, 141/144 tại 78,979 lần lặp khi có).
#
# Cả hai bộ benchmark hiện tại đều không chứa hướng tiếp cận đảo ngược: 16 kịch bản định
# danh cao nhất là 45 độ và đợt quét 300 seed đạt tối đa 89.5 độ. Vì vậy cửa chặn này
# TẮT phát bắn ở mọi nơi mà benchmark đo đạc và BẬT cho trường hợp mà nó được viết ra để
# xử lý -- đây là chủ đích thiết kế, không phải sơ suất. ĐỪNG hiểu "nó không bao giờ
# kích hoạt trên benchmark" là "nó là code chết"; hãy hiểu là "benchmark không chứa
# nhiệm vụ quay đầu".
GOAL_SHOT_MIN_REVERSAL_DEG = ALPHA_MAX

# ====== KHOẢNG AN TOÀN CUNG LƯỢN (trong lúc tìm kiếm) ======
# Trong quá trình mở rộng, tìm kiếm chỉ kiểm tra va chạm đoạn THẲNG của mỗi trạng thái
# kế tiếp; cung lượn fillet bán kính R bo góc cua TẠI waypoint hiện tại được để lại cho
# oracle cuối cùng. Cung của một góc cua tự do phồng vào trong một khoảng
# R*(1/cos(alpha/2)-1) và có thể lấn vào vật cản gần đó mà các đoạn thẳng đã tránh được,
# khiến tìm kiếm cam kết một đường bay mà sau đó chỉ có oracle mới từ chối (lỗi
# path_self_collision) - không mức độ đa dạng hóa trạng thái kế tiếp nào sửa được. Khi
# bật cờ này, cung rẽ của mỗi trạng thái kế tiếp được kiểm tra an toàn đối với các vật
# cản ĐÃ GIÃN NỞ -- cùng tập hợp mà các đoạn thẳng kiểm tra, đồng nhất với
# path_validation.arcs_clear.
#
# Trước đây từng dùng tập vật cản THÔ, và comment này từng giữ nguyên rất lâu sau khi
# không còn đúng nữa: trước đây khi phép giãn nở mang số hạng góc rẽ
# R*(1/cos(alpha_max/2)-1), cung fillet được THIẾT KẾ để phồng đúng vào dải đệm đó, do
# đó tập thô là mốc tham chiếu chuẩn xác. Khi số hạng quay bị loại bỏ thì dải đệm không
# còn, và việc kiểm tra với tập thô sẽ làm cung rẽ lấn vào khoảng cách an toàn của người
# vận hành (đo đạc thực tế: chỉ còn 97.9 m khoảng cách an toàn thực trên lần chạy cấu
# hình 500 m). ARC_CLEARANCE_CHECK = False là chế độ kế thừa cũ.
ARC_CLEARANCE_CHECK = True

# Số đoạn lấy mẫu cung lượn cho phép kiểm tra cung góc rẽ trong lúc tìm kiếm. Oracle lấy
# mẫu 24 đoạn; 12 là đủ để phát hiện độ phồng nông (vài chục mét) gặp trong thực tế đồng
# thời giảm một nửa chi phí mỗi góc rẽ. Các đoạn thẳng (chứ không chỉ các điểm) đều được
# kiểm tra, do đó một vật cản mỏng giữa các điểm lấy mẫu vẫn bị chặn dọc theo dây cung.
ARC_CHECK_SAMPLES = 12

# ====== HIỂN THỊ TRỰC QUAN ======
PLOT_BUFFER_ZONES = True
PLOT_START_END_MARKERS = True

# Độ phân giải DPI khi lưu đồ họa
FIGURE_DPI = 200

# ====== SINH KỊCH BẢN (map_generator) ======
# Bán kính phát hiện chướng ngại vật (m)
OBSTACLE_RADIUS_MIN = 10000.0
OBSTACLE_RADIUS_MAX = 50000.0

# Kích thước đảo đa giác (m)
ISLAND_SIZE_MIN = 5000.0
ISLAND_SIZE_MAX = 30000.0

# Số đỉnh của đa giác bất quy tắc
ISLAND_VERTICES_MIN = 4
ISLAND_VERTICES_MAX = 8

# Khoảng cách tối thiểu (m) giữa hai đảo được sinh ra, tương tự quy tắc cách ly mà bộ
# sinh hình tròn luôn áp dụng. Không có quy tắc này, các đảo sẽ chồng lấn tự do: đo đạc
# trên 200 kịch bản, 183 kịch bản chứa các cặp chồng lấn (trung vị 7, tối đa 62) và
# 21.2% đỉnh bao lồi đa giác nằm VÙI BÊN TRONG đa giác khác — các ứng viên mà tìm kiếm
# phải liên tục kiểm tra và từ chối ở mỗi lượt mở rộng. Đây là ràng buộc về hình dạng
# hình học, không phải khả năng bay: tại R = 8000 m thì hành lang 500 m đều không bay
# được theo cách nào, mục đích là để tập chướng ngại vật được định hình chuẩn xác về mặt
# hình học.
ISLAND_MIN_SEPARATION_M = 500.0

# Cùng quy tắc với hình tròn, đo đạc giữa hai ĐƯỜNG BIÊN: hai vị trí được coi là cách ly
# khi dist(tâm) >= r_i + r_j + giá trị này. Code cũ so sánh với ngưỡng cố định
# 2*OBSTACLE_RADIUS_MAX + 500 = 100.5 km, tính cho mọi cặp bán kính xấu nhất — trên bản
# đồ 500 km điều này giới hạn tối đa chỉ ~13 hình tròn bất kể số lượng yêu cầu là bao
# nhiêu (đo đạc: trung vị 6, tối đa 13 khi yêu cầu 0-50).
CIRCLE_MIN_SEPARATION_M = 500.0

# Khoảng cách an toàn tối thiểu (m) bắt buộc giữa start/goal và bất kỳ chướng ngại vật
# nào được sinh ra. Giá trị này trước đây dùng config.EPS, tức là 1e-6 m — vùng đệm cho
# phép chướng ngại vật chạm sát điểm xuất phát. Đo đạc tại cấu hình đó: 16% kịch bản đặt
# start hoặc goal gần vật cản hơn khoảng cách L0, khiến chặng bay cất cánh hoặc tự dẫn
# bắt buộc bị chặn ngay từ đầu.
SPAWN_CLEARANCE_M = 5000.0

# ====== TIỆN ÍCH ======


def deg_to_rad(degrees: float) -> float:
    """Chuyển đổi góc từ độ sang radian.

    Args:
        degrees: Giá trị góc tính bằng độ.

    Returns:
        Góc tương đương tính bằng radian.
    """
    return math.radians(degrees)


def rad_to_deg(radians: float) -> float:
    """Chuyển đổi góc từ radian sang độ.

    Args:
        radians: Giá trị góc tính bằng radian.

    Returns:
        Góc tương đương tính bằng độ.
    """
    return math.degrees(radians)


# Tính trước các giá trị thường dùng
ALPHA_MAX_RAD = deg_to_rad(ALPHA_MAX)

# ====== BẢO VỆ LÀM TRÒN SỐ THỰC (chỉ ở khâu dựng hình) ======
# Khoảng đệm làm tròn số thực Float64. Chúng tồn tại để hình học được DỰNG sát sạt một
# giới hạn không bị từ chối bởi phép kiểm tra chính xác ngay sau đó: một dây cung được
# dựng tiếp tuyến với một hình tròn có khoảng-cách-đến-tâm == r theo số học chính xác,
# nhưng trên thực tế rơi lệch vài ULP ở một trong hai phía. Đo đạc trên bộ lập kế hoạch
# này (2440 tiếp tuyến, tọa độ ~2e5 m): |dist - r| có trung vị 7.3e-12 m, tối đa 7.5e-11
# m ~= 1 ULP, và 43.3% tiếp tuyến rơi VÀO TRONG hình tròn, tức là tự đánh trượt trước
# kiểm tra chính xác `dist < radius`.
#
# CHỈ sử dụng chúng khi DỰNG hình học, không bao giờ dùng để nới lỏng kiểm tra — một
# kiểm tra có dung sai tha thứ cho sự xâm phạm thực sự và phá hủy đảm bảo rằng khoảng
# cách an toàn báo cáo là khoảng cách thực tế.
#
# Đệm VỀ PHÍA KHẢ THI, không phải lúc nào cũng là dấu "+": cộng thêm vào bán kính vật
# cản, nhưng TRỪ đi khỏi giới hạn góc rẽ (dựng alpha <= alpha_max - GEOM_EPS_RAD) và
# dựng các đoạn bay thẳng DÀI HƠN sàn đoản trình của chúng. Cộng vào alpha_max sẽ dựng
# chính sự vi phạm mà khoảng đệm muốn ngăn ngừa.
#
# Độ lớn: một khoảng đệm tuyệt đối phải vượt qua một ULP tại tọa độ lớn nhất được xét.
# ULP xấp xỉ 1.2e-10 m tại biên bản đồ 500 km và ~2.3e-10 m tại y ~ 1.15e6 (nhiệm vụ
# thực tế), do đó 1e-8 m duy trì khoảng trống an toàn ~25 lần trong khi chỉ bằng 1e-12
# của bán kính quay vòng 8 km, tức là hoàn toàn không có ý nghĩa sai lệch hình học. ĐỪNG
# tăng lên cỡ milimét để "cho an toàn": đo đạc thực tế, đệm 1e-3 m đã làm tăng 1.7-2.5%
# chiều dài đường bay, và đệm 1 m cũng tốn chừng đó mà không đem lại lợi ích gì thêm.
# Khoản chi phí đó là KHOẢNG CÁCH AN TOÀN, vốn là nhiệm vụ của SAFE_MARGIN /
# CONSTRUCTION_CLEARANCE_M.
GEOM_EPS_M = 1e-8

# Chiều dài cạnh ứng viên Chiến lược A ngắn nhất đáng để đánh giá (m). Ứng viên nằm gần
# waypoint hiện tại hơn mức này sẽ được bỏ qua trước khi chạy _pivot_candidate.
#
# Hai bộ lập kế hoạch từng bất đồng ở điểm này, và không phải do phong cách: main bỏ qua
# bất kỳ điểm nào trong phạm vi 100 m (một hằng số `< 10000` mét vuông trong vòng lặp
# trạng thái kế tiếp), v0 chỉ bỏ qua cạnh có chiều dài thực sự suy biến bằng 0. Do đó
# main từng loại bỏ các ứng viên mà v0 đánh giá, đây là sự khác biệt trong đồ thị tìm
# kiếm, không phải sở thích.
#
# Cận dưới là GEOM_EPS_M bất kể giá trị này: một cạnh độ dài bằng 0 không có góc hướng
# và không bao giờ được chạm tới _pivot_candidate. Đặt 0 để "chỉ bỏ qua cạnh suy biến"
# (quy tắc lịch sử của v0).
#
# ĐO ĐẠC THỰC TẾ trên 300 seed x cả hai chế độ đích, main ở mức 100 m vs mức 0: - mọi
# đường bay đều giống nhau từng bit, mọi nhiệm vụ đều giải được, cùng chiều dài - số lần
# gọi _pivot_candidate: 3,836,890 -> 3,845,944, tức là +0.236% Vì vậy hai quy tắc có kết
# quả tương đương trên phân phối này và mức bỏ qua 100 m chỉ tiết kiệm được 0.2% số lần
# đánh giá ứng viên. Mức 0 thắng về mặt giảm thiểu rủi ro thay vì thời gian: 100 m là
# con số không có dẫn xuất lý thuyết, và nó lặng lẽ loại bỏ một tiếp điểm hoặc đỉnh bao
# lồi hợp lệ trên bất kỳ bản đồ nào có các đặc trưng địa hình gần nhau hơn mức đó -- các
# bản đồ này có các đảo 20-60 km, nhưng thực tế không nhất thiết như vậy.
#
# Đừng tinh chỉnh theo trực giác nếu bạn xem xét lại điều này: chất lượng không đơn điệu
# theo số lượng trạng thái kế tiếp ở đây (xem NUM_FAN_DISTANCES), vì vậy hãy đo đạc lại,
# và đo SỐ LƯỢNG LẦN GỌI thay vì thời gian thực (config.TIME_BUDGET_S khiến kết quả thời
# gian thực phụ thuộc máy chạy).
CANDIDATE_MIN_DIST_M = 0.0

# Bản sao góc của GEOM_EPS_M. Đo đạc thực tế: 0.31% quyết định góc rẽ nằm trong phạm vi
# 1e-12 rad của alpha_max (chúng là các nấc quạt nan và góc xuất phát được dựng để đáp
# ứng chính xác alpha_max), và việc mở rộng dải từ 1e-12 lên 1e-3 rad chỉ làm tăng tập
# hợp đó từ 1410 lên 1707 — do đó quần thể thực sự nằm ĐÚNG tại giới hạn, và 1e-9 rad
# vượt qua nó với dự phòng 3 bậc độ lớn. 1e-9 rad trên bán kính 8 km tương đương 8
# micron cung tròn.
GEOM_EPS_RAD = 1e-9

# ====== VALIDATION & ORACLE TOLERANCES ======
# Ngưỡng dung sai phân biệt tiếp xúc biên và thấu giao cắt đa giác trong oracle (m)
ORACLE_POLYGON_TOUCH_TOL_M = 1e-6

# Ngưỡng dung sai dự trữ cung lượn nhận diện đổi hướng bay trong oracle (m)
ORACLE_TURN_RESERVE_TOL_M = 1e-6

# Số đoạn lấy mẫu cung lượn fillet arc khi kiểm tra va chạm độc lập trong oracle
ORACLE_ARC_SAMPLES = 24
