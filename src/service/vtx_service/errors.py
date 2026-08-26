"""Lỗi nội bộ: traceback ở lại server, client nhận một mã tra cứu.

``PlanReply.detail`` đi thẳng ra dây DDS tới mọi client trên domain. Trước
module này, ``runner._child`` nhét ``traceback.format_exc()`` vào đó, nên một
lỗi bất kỳ trong planner phát tán đường dẫn tuyệt đối trên server và cấu trúc
thư mục của nó ra ngoài - thông tin không client nào cần và không client nào
nên có.

Cách sửa không phải là xoá traceback: làm vậy thì người vận hành mất đúng thứ
duy nhất chẩn đoán được, và đây là service chạy không người trực. Traceback
được CHUYỂN CHỖ - vào log của service, cạnh một mã ngẫu nhiên ngắn mà client
cũng cầm. Client báo "request của tôi trả về internal error a1b2c3d4", người
vận hành ``grep a1b2c3d4`` trong journal và thấy nguyên văn.

Lưu ý cái này KHÔNG áp cho ``INVALID_REQUEST``: thông báo validate
(``turn_radius_m phải dương``) là thứ client cần để tự sửa, không tiết lộ gì về
server, và giấu nó đi chỉ tạo thêm một vòng hỏi đáp.
"""

from __future__ import annotations

import logging
import secrets

ERROR_ID_LEN = 8
"""Số ký tự hex của một mã lỗi.

8 hex = 32 bit. Mã chỉ cần phân biệt các lỗi trong cửa sổ giữ log của một
service, không cần chống va chạm mật mã học, và nó phải NGẮN vì con người đọc
nó qua điện thoại từ một client.
"""


def new_error_id() -> str:
    """Sinh một mã tra cứu mới.

    Returns:
        Chuỗi hex ``ERROR_ID_LEN`` ký tự.
    """
    return secrets.token_hex(ERROR_ID_LEN // 2)


def report_internal_error(
    log: logging.Logger, request_id: bytes, context: str, exc_text: str
) -> str:
    """Ghi đầy đủ lỗi vào log, trả về phần an toàn để gửi cho client.

    Args:
        log: Logger của service.
        request_id: Định danh 16 byte của request, để nối báo cáo của client
            với dòng log.
        context: Chỗ lỗi xảy ra, ví dụ ``"khi dịch reply ra kiểu trên dây"``.
        exc_text: Traceback hoặc mô tả lỗi. Chỉ tới log, không bao giờ tới dây.

    Returns:
        Chuỗi đặt vào ``PlanReply.detail``: mang ngữ cảnh và mã tra cứu, không
        mang traceback.
    """
    error_id = new_error_id()
    log.error(
        "request %s lỗi %s [%s]:\n%s",
        request_id.hex()[:8],
        context,
        error_id,
        exc_text,
    )
    return f"internal error {context} [{error_id}] - tra mã này trong log service"
