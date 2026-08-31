"""Kiểm thử đơn vị cho động cơ tìm kiếm A* trong module search.astar."""

from __future__ import annotations

from path_planning import config
from path_planning.collision.detector import CollisionDetector
from path_planning.search.astar import AstarSearchEngine
from path_planning.search.state import State
from path_planning.search.successors import SuccessorGenerator
from path_planning.types import PreprocessedScenario


def test_astar_search_engine_initialization(
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra khởi tạo động cơ AstarSearchEngine với đầy đủ tham số cấu hình."""
    # Arrange
    detector = CollisionDetector(sample_preprocessed_scenario)
    generator = SuccessorGenerator(sample_preprocessed_scenario, detector)
    corners = generator.seed_start_corners()
    goal_state = State(
        sample_preprocessed_scenario["goal_state"]["waypoint"],
        sample_preprocessed_scenario["goal_state"]["heading"],
    )

    # Act
    engine = AstarSearchEngine(
        corners,
        goal_state,
        generator,
        detector,
        time_budget_s=15.0,
        origin=sample_preprocessed_scenario["start_pos"],
        target=sample_preprocessed_scenario["goal_pos"],
        alpha_build=sample_preprocessed_scenario["alpha_max_rad"],
    )

    # Assert
    assert len(engine.start_corners) == len(corners)
    assert engine.time_budget_s == 15.0
    assert engine.is_budget_bound is False
    assert engine.is_search_failed is False


def test_astar_reconstruct_path_walks_parent_pointers_in_order() -> None:
    """Kiểm tra hàm reconstruct_path truy vết đúng thứ tự từ gốc xuất phát tới đích."""
    # Arrange
    st1 = State((0.0, 0.0), 0.0)
    st2 = State((10000.0, 0.0), 0.0)
    st2.parent = st1
    st3 = State((20000.0, 10000.0), 0.785)
    st3.parent = st2

    # Fake engine to invoke reconstruct_path
    detector = None  # type: ignore[assignment]
    generator = None  # type: ignore[assignment]
    engine = AstarSearchEngine(
        [st1],
        st3,
        generator,
        detector,
        time_budget_s=5.0,
        origin=(0.0, 0.0),
        target=(20000.0, 10000.0),
        alpha_build=config.ALPHA_MAX_RAD,
    )

    # Act
    path = engine.reconstruct_path(st3)

    # Assert
    assert len(path) == 3
    assert path[0][0] == (0.0, 0.0)
    assert path[1][0] == (10000.0, 0.0)
    assert path[2][0] == (20000.0, 10000.0)


def test_astar_search_finds_valid_path_in_clean_scenario(
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra động cơ A* tìm thấy đường bay khả thi trong kịch bản mẫu."""
    # Arrange
    detector = CollisionDetector(sample_preprocessed_scenario)
    generator = SuccessorGenerator(sample_preprocessed_scenario, detector)
    corners = generator.seed_start_corners()
    goal_state = State(
        sample_preprocessed_scenario["goal_state"]["waypoint"],
        sample_preprocessed_scenario["goal_state"]["heading"],
    )
    engine = AstarSearchEngine(
        corners,
        goal_state,
        generator,
        detector,
        time_budget_s=15.0,
        origin=sample_preprocessed_scenario["start_pos"],
        target=sample_preprocessed_scenario["goal_pos"],
        alpha_build=sample_preprocessed_scenario["alpha_max_rad"],
    )

    # Act
    path = engine.search()

    # Assert
    assert path is not None
    assert len(path) >= 2
    stats = engine.get_search_stats()
    assert stats["iterations"] > 0
    assert stats["is_search_failed"] is False
