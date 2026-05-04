"""adapter_v4 단위 테스트 — ROS2 / pymycobot 없는 환경에서도 실행 가능 (mock 모드)."""

from unittest.mock import MagicMock, patch
import sys
import pytest
from pydantic import ValidationError

# =============================================================================
# 1️⃣  외부 의존 모듈 mock (ROS2, pymycobot)
# =============================================================================
sys.modules["rclpy"]                      = MagicMock()
sys.modules["rclpy.action"]               = MagicMock()
sys.modules["rclpy.node"]                 = MagicMock()
sys.modules["action_msgs"]                = MagicMock()
sys.modules["action_msgs.msg"]            = MagicMock()
sys.modules["geometry_msgs"]              = MagicMock()
sys.modules["geometry_msgs.msg"]          = MagicMock()
sys.modules["nav2_msgs"]                  = MagicMock()
sys.modules["nav2_msgs.action"]           = MagicMock()
sys.modules["pymycobot"]                  = MagicMock()
sys.modules["pymycobot.mycobot280"]       = MagicMock()

import main_service.management.services.adapters.robot_adapter_v4 as mod
from main_service.management.services.adapters.robot_adapter_v4 import (
    # 테이블
    DOCK_STATIONS,
    MANUFACTURING_WAYPOINTS,
    PUTAWAY_WAYPOINTS,
    # Pydantic 모델
    ManufacturingWaypointInput,
    GripperInput,
    GoHomeInput,
    PutawayPickInput,
    PutawayWaypointInput,
    MoveZRelInput,
    SendCommandResult,
    # Adapter 클래스
    Adapter,
)

# =============================================================================
# 2️⃣  공용 fixture
# =============================================================================

@pytest.fixture
def adapter() -> Adapter:
    """ROS2·pymycobot을 완전히 mock한 Adapter 인스턴스."""
    with patch.object(mod, "MyCobot280", return_value=MagicMock()), \
         patch.object(mod, "rclpy",      MagicMock()), \
         patch.object(mod, "Node",       MagicMock()):
        return Adapter()


@pytest.fixture
def mc() -> MagicMock:
    """pymycobot MyCobot280 인스턴스 mock."""
    return MagicMock()


@pytest.fixture(autouse=True)   # ← 여기 추가
def no_sleep(monkeypatch):
    monkeypatch.setattr("main_service.management.services.adapters.robot_adapter_v4.time.sleep", lambda _: None)
# =============================================================================
# 3️⃣  Pydantic Input 모델 — 유효성 검증
# =============================================================================

class TestGripperInput:
    def test_defaults(self):
        inp = GripperInput()
        assert inp.speed == 50
        assert inp.delay == 1.0

    def test_custom_values(self):
        inp = GripperInput(speed=80, delay=0.5)
        assert inp.speed == 80
        assert inp.delay == 0.5

    def test_speed_below_min_raises(self):
        with pytest.raises(ValidationError):
            GripperInput(speed=0)

    def test_speed_above_max_raises(self):
        with pytest.raises(ValidationError):
            GripperInput(speed=101)

    def test_negative_delay_raises(self):
        with pytest.raises(ValidationError):
            GripperInput(delay=-0.1)


class TestGoHomeInput:
    def test_defaults(self):
        inp = GoHomeInput()
        assert inp.speed == 50
        assert inp.delay == 1.0

    def test_speed_boundary(self):
        assert GoHomeInput(speed=1).speed   == 1
        assert GoHomeInput(speed=100).speed == 100

    def test_speed_out_of_range_raises(self):
        with pytest.raises(ValidationError):
            GoHomeInput(speed=0)
        with pytest.raises(ValidationError):
            GoHomeInput(speed=101)


class TestMoveZRelInput:
    def test_delta_z_required(self):
        with pytest.raises(ValidationError):
            MoveZRelInput()

    def test_valid(self):
        inp = MoveZRelInput(delta_z=10.0)
        assert inp.delta_z == 10.0
        assert inp.speed   == 30
        assert inp.delay   == 2.0

    def test_negative_delta_z_allowed(self):
        inp = MoveZRelInput(delta_z=-5.0)
        assert inp.delta_z == -5.0

    def test_speed_out_of_range_raises(self):
        with pytest.raises(ValidationError):
            MoveZRelInput(delta_z=1.0, speed=0)


class TestPutawayWaypointInput:
    def test_floor_and_cell_required(self):
        with pytest.raises(ValidationError):
            PutawayWaypointInput()

    def test_valid(self):
        inp = PutawayWaypointInput(floor=2, cell=3)
        assert inp.floor == 2
        assert inp.cell  == 3
        assert inp.speed == 30
        assert inp.delay == 3.0

    @pytest.mark.parametrize("floor", [0, 4])
    def test_floor_out_of_range_raises(self, floor):
        with pytest.raises(ValidationError):
            PutawayWaypointInput(floor=floor, cell=1)

    @pytest.mark.parametrize("cell", [0, 7])
    def test_cell_out_of_range_raises(self, cell):
        with pytest.raises(ValidationError):
            PutawayWaypointInput(floor=1, cell=cell)

    def test_floor_boundary(self):
        assert PutawayWaypointInput(floor=1, cell=1).floor == 1
        assert PutawayWaypointInput(floor=3, cell=1).floor == 3

    def test_cell_boundary(self):
        assert PutawayWaypointInput(floor=1, cell=1).cell == 1
        assert PutawayWaypointInput(floor=1, cell=6).cell == 6


class TestPutawayPickInput:
    def test_default_speed(self):
        assert PutawayPickInput().speed == 50

    def test_speed_out_of_range_raises(self):
        with pytest.raises(ValidationError):
            PutawayPickInput(speed=0)


class TestManufacturingWaypointInput:
    def test_defaults(self):
        inp = ManufacturingWaypointInput()
        assert inp.speed == 50
        assert inp.delay == 3.0

    def test_speed_out_of_range_raises(self):
        with pytest.raises(ValidationError):
            ManufacturingWaypointInput(speed=101)


# =============================================================================
# 5️⃣  좌표 테이블 — 키 존재 및 길이 검증
# =============================================================================

class TestDockStations:
    @pytest.mark.parametrize("task_type", list(DOCK_STATIONS.keys()))
    def test_all_keys_have_four_elements(self, task_type):
        x, y, theta, dock_type = DOCK_STATIONS[task_type]
        assert isinstance(x,         float)
        assert isinstance(y,         float)
        assert isinstance(theta,     float)
        assert isinstance(dock_type, str)

    def test_dock_type_is_valid(self):
        valid = {"test_dock", "STRG_dock"}
        for key, (_, _, _, dock_type) in DOCK_STATIONS.items():
            assert dock_type in valid, f"{key}: unexpected dock_type '{dock_type}'"


class TestManufacturingWaypoints:
    @pytest.mark.parametrize("key", list(MANUFACTURING_WAYPOINTS.keys()))
    def test_all_waypoints_have_six_joints(self, key):
        assert len(MANUFACTURING_WAYPOINTS[key]) == 6, f"{key} should have 6 joint values"


class TestPutawayWaypoints:
    @pytest.mark.parametrize("key", list(PUTAWAY_WAYPOINTS.keys()))
    def test_all_waypoints_have_six_joints(self, key):
        assert len(PUTAWAY_WAYPOINTS[key]) == 6, f"{key} should have 6 joint values"

    def test_all_floors_are_valid(self):
        for (_, floor, _) in PUTAWAY_WAYPOINTS:
            assert 1 <= floor <= 3

    def test_all_cells_are_valid(self):
        for (_, _, cell) in PUTAWAY_WAYPOINTS:
            assert 1 <= cell <= 6


# =============================================================================
# 6️⃣  send_command — unknown robot_id
# =============================================================================

class TestSendCommandRouting:
    def test_unknown_robot_id_returns_failed_result(self, adapter):
        result = adapter.send_command("UNKNOWN", "GO_HOME")
        assert isinstance(result, SendCommandResult)
        assert result.succeeded is False
        assert "unknown robot_id" in result.message

    def test_amr_unknown_task_type_returns_failed_result(self, adapter):
        result = adapter.send_command("TAT1", "FLY_TO_MOON")
        assert result.succeeded is False
        assert "unknown AMR task_type" in result.message


# =============================================================================
# 7️⃣  _handle_primitive — mc mock 직접 주입
# =============================================================================

class TestHandlePrimitive:
    def test_gripper_open(self, adapter, mc):
        result = adapter._handle_primitive(mc, "GRIPPER_OPEN", {})
        mc.set_gripper_value.assert_called_once_with(100, 50)
        assert result.succeeded is True
        assert result.message == "gripper opened"

    def test_gripper_close(self, adapter, mc):
        result = adapter._handle_primitive(mc, "GRIPPER_CLOSE", {})
        mc.set_gripper_value.assert_called_once_with(0, 50)
        assert result.succeeded is True
        assert result.message == "gripper closed"

    def test_gripper_custom_speed(self, adapter, mc):
        result = adapter._handle_primitive(mc, "GRIPPER_OPEN", {"speed": 80})
        mc.set_gripper_value.assert_called_once_with(100, 80)
        assert result.succeeded is True

    def test_go_home(self, adapter, mc):
        result = adapter._handle_primitive(mc, "GO_HOME", {})
        mc.send_angles.assert_called_once()
        assert result.succeeded is True
        assert result.message == "moved to home"

    def test_move_z_rel_success(self, adapter, mc):
        mc.get_coords.return_value = [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]
        result = adapter._handle_primitive(mc, "MOVE_Z_REL", {"delta_z": 10.0})
        assert result.succeeded is True
        assert "10.0" in result.message

    def test_move_z_rel_missing_delta_z_raises_validation_error(self, adapter, mc):
        result = adapter._handle_primitive(mc, "MOVE_Z_REL", {})
        assert result.succeeded is False
        assert "primitive failed" in result.message

    def test_move_z_rel_empty_coords(self, adapter, mc):
        mc.get_coords.return_value = []
        result = adapter._handle_primitive(mc, "MOVE_Z_REL", {"delta_z": 5.0})
        assert result.succeeded is False
        assert "get_coords returned empty" in result.message

    def test_gripper_invalid_speed_returns_failed(self, adapter, mc):
        result = adapter._handle_primitive(mc, "GRIPPER_OPEN", {"speed": 0})
        assert result.succeeded is False
        assert "primitive failed" in result.message


# =============================================================================
# 8️⃣  _handle_manufacturing — mc mock 직접 주입
# =============================================================================

class TestHandleManufacturing:
    @pytest.mark.parametrize("task_type", list(MANUFACTURING_WAYPOINTS.keys()))
    def test_known_waypoint_calls_send_angles(self, adapter, mc, task_type):
        result = adapter._handle_manufacturing(mc, task_type, {})
        mc.send_angles.assert_called_once()
        assert result.succeeded is True
        assert task_type in result.message

    def test_unknown_waypoint_returns_failed(self, adapter, mc):
        result = adapter._handle_manufacturing(mc, "UNKNOWN_STEP", {})
        assert result.succeeded is False
        assert "unknown manufacturing waypoint" in result.message

    def test_mc_exception_returns_failed(self, adapter, mc):
        mc.send_angles.side_effect = RuntimeError("serial error")
        result = adapter._handle_manufacturing(mc, "MOLD_P1_PICK", {})
        assert result.succeeded is False
        assert "manufacturing failed" in result.message


# =============================================================================
# 9️⃣  _handle_putaway — mc mock 직접 주입
# =============================================================================

class TestHandlePutaway:
    def test_pick_calls_send_angles_three_times(self, adapter, mc):
        result = adapter._handle_putaway(mc, "PICK", {})
        assert mc.send_angles.call_count == 3  # pre → target → pre (gripper 제외)
        assert result.succeeded is True
        assert result.message == "picked"

    @pytest.mark.parametrize("task_type, floor, cell", [
        ("APPROACH", 3, 1),
        ("PLACE",    3, 1),
        ("MIDDLE",   2, 1),
        ("MIDDLE_1", 1, 1),
        ("MIDDLE_2", 1, 1),
        ("PLACE",    1, 6),
    ])
    def test_known_waypoint_calls_send_angles(self, adapter, mc, task_type, floor, cell):
        result = adapter._handle_putaway(mc, task_type, {"floor": floor, "cell": cell})
        mc.send_angles.assert_called_once()
        assert result.succeeded is True

    def test_unknown_waypoint_returns_failed(self, adapter, mc):
        result = adapter._handle_putaway(mc, "APPROACH", {"floor": 1, "cell": 99})
        assert result.succeeded is False

    def test_missing_floor_returns_failed(self, adapter, mc):
        result = adapter._handle_putaway(mc, "APPROACH", {"cell": 1})
        assert result.succeeded is False
        assert "invalid putaway payload" in result.message

    def test_missing_cell_returns_failed(self, adapter, mc):
        result = adapter._handle_putaway(mc, "APPROACH", {"floor": 1})
        assert result.succeeded is False
        assert "invalid putaway payload" in result.message

    def test_mc_exception_returns_failed(self, adapter, mc):
        mc.send_angles.side_effect = RuntimeError("serial error")
        result = adapter._handle_putaway(mc, "APPROACH", {"floor": 3, "cell": 1})
        assert result.succeeded is False
        assert "putaway failed" in result.message