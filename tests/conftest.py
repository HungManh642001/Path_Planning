"""Cấu hình và fixtures kiểm thử toàn cục cho toàn bộ dự án."""

from __future__ import annotations

import random
from collections.abc import Generator

import pytest


@pytest.fixture(autouse=True)
def fixed_seed() -> Generator[None, None, None]:
    """Cố định seed ngẫu nhiên mặc định để đảm bảo tính quyết định (determinism).

    Yields:
        None sau khi đã thiết lập seed ngẫu nhiên.
    """
    random.seed(42)
    yield
    random.seed(42)
