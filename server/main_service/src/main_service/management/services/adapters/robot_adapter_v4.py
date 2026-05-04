"""adapter_v4 — Robot Executor 와 실제 로봇 사이의 어댑터.

설계 원칙
---------
- Robot Executor 는 task_type + payload 만 던진다.
- Adapter 는 단일 동작 단위로만 받아서 즉시 실행한다
- AMR  → ROS2 nav2_msgs/action/DockRobot (opennav_docking by-pose mode)
- RA → pymycobot 라이브러리 직접 호출 (mc.send_angles 등)

로봇 역할 분담
--------------
- TAT1/TAT2/TAT3 : 물류 이동 (도크 스테이션 단위, AMR 카테고리)
- MAT  : manufacturing 전담  (몰드 제작/패터닝/주입/탈형)
- PAT  : putaway 전담 (선반 적재)
"""

import math
import time
from typing import Any, Dict, Literal, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import DockRobot
from pydantic import BaseModel, Field
from rclpy.action import ActionClient
from rclpy.node import Node

from pymycobot.mycobot280 import MyCobot280

from ..core.contracts.models import *

# ---------------------------------------------------------------------------
# 로봇 역할 검증 테이블
# ---------------------------------------------------------------------------

AMR: set[str] = {"TAT1", "TAT2", "TAT3"}

RA_ROLE: Dict[str, Literal["manufacturing", "putaway"]] = {
    "MAT": "manufacturing",
    "PAT": "putaway",
}

# ---------------------------------------------------------------------------
# AMR 도크 좌표 테이블 (map frame)
# 값: (x, y, theta, dock_type)
#   - theta    : rad
#   - dock_type: "test_dock" 또는 "STRG_dock" (docking_server YAML plugin id)
# ---------------------------------------------------------------------------

DOCK_STATIONS: Dict[str, Tuple[float, float, float, str]] = {
    # key           x        y      theta   dock_type
    "ToINSP":  (-0.670, -0.100, -1.57, "test_dock"),
    "ToSHIP":  (-0.670,  0.450,  1.57, "test_dock"),
    "ToCAST1": (-0.256,  0.200,  1.57, "test_dock"),
    "ToCAST2": (-0.413,  0.200,  1.57, "test_dock"),
    "ToCHG1":  ( 0.044,  0.095,  0.00, "test_dock"),
    "ToCHG2":  ( 0.044, -0.027,  0.00, "test_dock"),
    "ToCHG3":  ( 0.044, -0.179,  0.00, "test_dock"),
    "ToSTRG1": (-0.100, -0.465, -1.57, "STRG_dock"),
    "ToSTRG2": (-0.223, -0.465, -1.57, "test_dock"),
    "ToPP1":   (-0.447, -1.050,  3.14, "test_dock"),
    "ToPP2":   (-0.447, -1.130,  3.14, "test_dock"),
}

# ---------------------------------------------------------------------------
# MAT Manufacturing 좌표 테이블
# 값: (j1, j2, j3, j4, j5, j6) — 단위: deg
# ---------------------------------------------------------------------------

MANUFACTURING_WAYPOINTS: Dict[str, Tuple[float, ...]] = {
    # key                          j1      j2      j3       j4      j5     j6
    # ----- Mold making: 패턴 1 -----
    "MOLD_P1_PICK":          ( 90.00,  17.50, -144.80,  38.00,   0.00,  45.00),
    "MOLD_P1_PATTERN_READY": (  0.00,   0.00,    0.00, -17.31,   0.00, -45.00),
    # "":                      (  0.00,  -76.6,    0.00, -17.31,   0.00, -45.00),
    "MOLD_P1_PATTERNING":    (  0.00, -21.30,  -97.70,  28.70,   0.00, -45.00),
    "MOLD_P1_DROP":          ( 90.00,  25.20, -111.50,  -7.00,   0.00,  45.00),
    # ----- Mold making: 패턴 2 -----
    "MOLD_P2_PICK":          ( 90.00, -16.00, -114.00,  42.00,   0.00,  45.00),
    "MOLD_P2_PATTERN_READY": (  0.00,   0.00,    0.00, -17.31,   0.00, -45.00),
    "MOLD_P2_PATTERNING":    (  0.00, -76.60,    0.00, -17.31,   0.00, -45.00),
    "MOLD_P2_DROP":          ( 90.00, -10.00,  -63.00, -17.00,   0.00,  45.00),
    "MOLD_P2_DROP_RELEASE":  ( 90.00, -11.20,  -63.30, -22.50,   0.00,  45.00),
    # ----- Mold making: 패턴 3 -----
    "MOLD_P3_PRE_PICK":      ( 90.00,   0.00,    0.00, -90.00,   0.00,  45.00),
    "MOLD_P3_PICK":          ( 90.00, -43.00,  -69.00,  23.00,   0.00,  45.00),
    "MOLD_P3_PATTERN_READY": (  0.00,   0.00,    0.00, -17.31,   0.00, -45.00),
    "MOLD_P3_PATTERNING":    (  0.00, -76.60,    0.00, -17.31,   0.00, -45.00),
    "MOLD_P3_PRE_DROP":      ( 90.00,   0.00,    0.00, -90.00,   0.00,  45.00),
    "MOLD_P3_DROP":          ( 90.00, -36.12,  -63.28,   9.05,   0.00,  45.00),
    # ----- Pouring -----
    "POURING_PICK_READY":    (-90.00,   0.00,    0.00,   0.00,   0.00,  45.00),
    "POURING_PICK":          (-90.00, -86.00,    0.00,  90.00,   0.00,  45.00),
    "POURING_RELEASE":       (-90.00, -87.00,    0.00,  90.00,   0.00,  45.00),
    "POURING_TILT_READY":    (  0.00,   0.00, -142.00, 143.00,   1.00,  45.00),
    "POURING_TILT":          (  0.00,   0.00, -142.00, 143.00,   1.00, 125.00),
    # ----- Demolding -----
    "DEMOLD_APPROACH":       (  0.00,   0.00,    0.00, -17.31,   0.00,  45.00),
    "DEMOLD_PICK":           (  0.00, -76.60,    0.00, -17.31,   0.00,  45.00),
    "DEMOLD_DROP":           (-30.90, -60.30,    0.00,  24.00,   0.00,  45.00),
}

# ---------------------------------------------------------------------------
# PAT Putaway 좌표 테이블
# 값: (j1, j2, j3, j4, j5, j6) — 단위: deg
# 단계 구성:
#   3층 : APPROACH → PLACE
#   2층 : APPROACH → MIDDLE → PLACE
#   1층 : APPROACH → MIDDLE_1 → MIDDLE_2 → PLACE
# ---------------------------------------------------------------------------

PUTAWAY_WAYPOINTS: Dict[Tuple[str, int, int], Tuple[float, ...]] = {
    # key                   j1      j2      j3       j4      j5     j6
    # ===== 3층 =====
    ("APPROACH", 3, 1): (  4.00,   0.00,    0.00,   0.00,   0.00,  45.00),
    ("PLACE",    3, 1): (  4.00,   0.00,  -40.00,  35.00,   0.00,  46.00),
    ("APPROACH", 3, 2): (-10.00,   0.00,    0.00,   0.00,   0.00,  45.00),
    ("PLACE",    3, 2): (-10.00, -11.00,  -40.00,  53.00,   0.00,  46.00),
    ("APPROACH", 3, 3): (-35.00,   0.00,    0.00,   0.00,   0.00,  45.00),
    ("PLACE",    3, 3): (-35.00, -37.70,    8.40,  32.30,  -1.80,  46.00),
    ("APPROACH", 3, 4): (-47.00,   0.00,    0.00,   0.00,   0.00,  45.00),
    ("PLACE",    3, 4): (-47.00, -51.50,   20.30,  36.50,  -1.40,  46.00),
    ("APPROACH", 3, 5): (-70.00,   0.00,    0.00,   0.00,   0.00,  45.00),
    ("PLACE",    3, 5): (-70.00, -50.71,   20.30,  36.50,  -2.80,  46.00),
    ("APPROACH", 3, 6): (-81.00,   0.00,    0.00,   0.00,   0.00,  45.00),
    ("PLACE",    3, 6): (-74.10, -51.50,   13.00,  50.70, -12.00,  45.00),
    # ===== 2층 =====
    ("APPROACH", 2, 1): (  4.00,   0.00,    0.00,   0.00,   0.00,  45.00),
    ("MIDDLE",   2, 1): (  4.00,  65.40, -134.80,  66.80,   3.00,  45.00),
    ("PLACE",    2, 1): (  4.00,   9.20, -109.30, 100.00,   0.00,  45.00),
    ("APPROACH", 2, 2): (-10.00,   0.00,    0.00,   0.00,   0.00,  45.00),
    ("MIDDLE",   2, 2): (-10.00,  65.40, -134.80,  66.80,   0.00,  45.00),
    ("PLACE",    2, 2): (-10.00,  -1.00, -106.20, 109.00,   0.00,  45.00),
    ("APPROACH", 2, 3): (-35.00,   0.00,    0.00,   0.00,   0.00,  45.00),
    ("MIDDLE",   2, 3): (-35.00,  65.40, -134.80,  56.80,   0.00,  45.00),
    ("PLACE",    2, 3): (-35.00, -10.80,  -94.30, 109.00,   0.00,  45.00),
    ("APPROACH", 2, 4): (-47.00,   0.00,    0.00,   0.00,   0.00,  45.00),
    ("MIDDLE",   2, 4): (-47.00,  65.40, -134.80,  56.80,   0.00,  45.00),
    ("PLACE",    2, 4): (-47.00, -18.10,  -89.20, 112.00,  -1.00,  45.00),
    ("APPROACH", 2, 5): (-70.00,   0.00,    0.00,   0.00,   0.00,  45.00),
    ("MIDDLE",   2, 5): (-70.00,  65.40, -134.80,  56.80,   0.00,  45.00),
    ("PLACE",    2, 5): (-70.00, -25.90,  -81.90, 115.00,  -2.80,  45.00),
    ("APPROACH", 2, 6): (-81.00,   0.00,    0.00,   0.00,   0.00,  45.00),
    ("MIDDLE",   2, 6): (-81.00,  65.40, -134.80,  56.80,  10.00,  45.00),
    ("PLACE",    2, 6): (-74.00, -21.00,  -90.00, 115.00, -12.00,  45.00),
    # ===== 1층 =====
    ("APPROACH", 1, 1): (  4.00,   0.00,    0.00,   0.00,   0.00,  45.00),
    ("MIDDLE_1", 1, 1): (  4.00,  61.50, -150.00,  93.20,   0.00,  45.00),
    ("MIDDLE_2", 1, 1): (  4.00,  61.50, -150.00,  70.00,   0.00,  45.00),
    ("PLACE",    1, 1): (  4.00,   8.30, -127.30,  91.70,   0.00,  45.00),
    ("APPROACH", 1, 2): (-10.00,   0.00,    0.00,   0.00,   0.00,  45.00),
    ("MIDDLE_1", 1, 2): (-10.00,  61.50, -150.00,  93.20,   0.00,  45.00),
    ("MIDDLE_2", 1, 2): (-10.00,  61.50, -150.00,  70.00,   0.00,  45.00),
    ("PLACE",    1, 2): (-10.00,   2.50, -129.60, 100.60,   0.00,  45.00),
    ("APPROACH", 1, 3): (-35.00,   0.00,    0.00,   0.00,   0.00,  45.00),
    ("MIDDLE_1", 1, 3): (-35.00,  61.50, -150.00,  93.20,   0.00,  45.00),
    ("MIDDLE_2", 1, 3): (-35.00,  61.50, -150.00,  70.00,   0.00,  45.00),
    ("PLACE",    1, 3): (-35.00, -13.40, -116.20, 108.80,   0.00,  45.00),
    ("APPROACH", 1, 4): (-47.00,   0.00,    0.00,   0.00,   0.00,  45.00),
    ("MIDDLE_1", 1, 4): (-47.00,  61.50, -150.00,  93.20,   0.00,  45.00),
    ("MIDDLE_2", 1, 4): (-47.00,  61.50, -150.00,  70.00,   0.00,  45.00),
    ("PLACE",    1, 4): (-47.00, -28.70, -112.50, 126.70,   0.00,  45.00),
    ("APPROACH", 1, 5): (-70.00,   0.00,    0.00,   0.00,   0.00,  45.00),
    ("MIDDLE_1", 1, 5): (-70.00,  61.50, -150.00,  93.20,   0.00,  45.00),
    ("MIDDLE_2", 1, 5): (-70.00,  61.50, -150.00,  70.00,   0.00,  45.00),
    ("PLACE",    1, 5): (-70.00, -21.40, -108.10, 108.10,  -2.80,  45.00),
    ("APPROACH", 1, 6): (-81.00,   0.00,    0.00,   0.00,   0.00,  45.00),
    ("MIDDLE_1", 1, 6): (-81.00,  61.50, -150.00,  93.20,   0.00,  45.00),
    ("MIDDLE_2", 1, 6): (-81.00,  61.50, -150.00,  70.00,   0.00,  45.00),
    ("PLACE",    1, 6): (-79.60, -42.40, -100.50, 131.70,  -3.30,  45.00),
}

# ---------------------------------------------------------------------------
# Primitive 동작 상수
# ---------------------------------------------------------------------------

PRIMITIVE_TASKS = {"GRIPPER_OPEN", "GRIPPER_CLOSE", "MOVE_Z_REL", "GO_HOME"}

HOME_ANGLES: Tuple[float, ...] = (0.00, 0.00, 0.00, 0.00, 0.00, 45.00)

PICK_PRE_ANGLES:    Tuple[float, ...] = (90.00,   0.00,   0.00,  0.00, 0.00, 45.00)
PICK_TARGET_ANGLES: Tuple[float, ...] = (90.00, -20.39, -36.56, -7.99, 0.00, 45.00)



# ---------------------------------------------------------------------------
# Adapter
# ---------------------------------------------------------------------------

class Adapter:
    """Robot Executor 와 실제 로봇 사이의 어댑터.

    초기화 시
    - RA 인스턴스 (pymycobot) 연결, 메모리 보관
    - ROS2 노드 1개 생성 (AMR action client 용)

    Example
    -------
    adapter = Adapter()
    adapter.send_command("TAT1", "ToCAST1")
    adapter.send_command("PAT", "APPROACH", {"floor": 2, "cell": 3})
    adapter.send_command("PAT", "PICK", {})
    """

    def __init__(self,
                 ra_ports: Dict[str, str] | None = None,
                 baudrate: int = 1000000,
                 node_name: str = "adapter_node") -> None:
        if ra_ports is None:
            ra_ports = {"MAT": "/dev/ttyJETCOBOT_001",
                          "PAT": "/dev/ttyJETCOBOT_002"}

        # --- RA 연결 ---
        self._mc_instances: Dict[str, MyCobot280] = {}
        for robot_id, port in ra_ports.items():
            mc = MyCobot280(port, baudrate)
            mc.thread_lock = True
            self._mc_instances[robot_id] = mc

        # --- ROS2 초기화 ---
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = Node(node_name)
        self._dock_clients: Dict[str, ActionClient] = {}

    def shutdown(self) -> None:
        """ROS2 노드 정리. 프로그램 종료 시 호출."""
        try:
            self._node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()

    # -------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------

    def send_command(self,
                     robot_id: str,
                     task_type: str,
                     payload: Dict[str, Any] | None = None,
                     timeout_sec: float = 60.0) -> SendCommandResult:
        """단일 동작 1회 실행."""
        payload = payload or {}

        if robot_id in AMR:
            return self._handle_amr(robot_id, task_type, timeout_sec)

        if robot_id in RA_ROLE:
            return self._handle_ra(robot_id, task_type, payload)

        return SendCommandResult(succeeded=False, message=f"unknown robot_id: {robot_id}")
    
    # -------------------------------------------------------------------
    # AMR: ROS2 DockRobot action
    # -------------------------------------------------------------------

    def _handle_amr(self,
                    robot_id: str,
                    task_type: str,
                    timeout_sec: float) -> SendCommandResult:
        if task_type not in DOCK_STATIONS:
            return SendCommandResult(succeeded=False, message=f"unknown AMR task_type: {task_type}")

        x, y, theta, dock_type = DOCK_STATIONS[task_type]

        client = self._dock_clients.get(robot_id)
        if client is None:
            client = ActionClient(self._node, DockRobot, f"/{robot_id}/dock_robot")
            self._dock_clients[robot_id] = client

        if not client.wait_for_server(timeout_sec=5.0):
            return SendCommandResult(succeeded=False, message=f"dock action server unavailable: {robot_id}")

        ps = PoseStamped()
        ps.header.stamp        = self._node.get_clock().now().to_msg()
        ps.header.frame_id     = "map"
        ps.pose.position.x     = x
        ps.pose.position.y     = y
        ps.pose.position.z     = 0.0
        ps.pose.orientation.z  = math.sin(theta * 0.5)
        ps.pose.orientation.w  = math.cos(theta * 0.5)

        goal = DockRobot.Goal()
        goal.use_dock_id              = False
        goal.dock_pose                = ps
        goal.dock_type                = dock_type
        goal.navigate_to_staging_pose = True

        return self._send_and_wait_action(client, goal, timeout_sec, success_label="docked")

    def _send_and_wait_action(self,
                              client: ActionClient,
                              goal: Any,
                              timeout_sec: float,
                              success_label: str) -> SendCommandResult:
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_future, timeout_sec=5.0)

        if not send_future.done():
            return SendCommandResult(succeeded=False, message="send_goal timeout")

        handle = send_future.result()
        if not handle.accepted:
            return SendCommandResult(succeeded=False, message="goal rejected by action server")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            handle.cancel_goal_async()
            return SendCommandResult(succeeded=False, message=f"result timeout after {timeout_sec}s")

        wrapped = result_future.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return SendCommandResult(succeeded=False, message=f"action failed (status={wrapped.status})")

        if hasattr(wrapped.result, "success") and not wrapped.result.success:
            err     = getattr(wrapped.result, "error_code",  "?")
            retries = getattr(wrapped.result, "num_retries", "?")
            return SendCommandResult(succeeded=False, message=f"action result failed (error_code={err}, retries={retries})")

        return SendCommandResult(succeeded=True, message=success_label)

    # -------------------------------------------------------------------
    # RA: pymycobot 직접 호출
    # -------------------------------------------------------------------

    def _handle_ra(self,
                     robot_id: str,
                     task_type: str,
                     payload: Dict[str, Any]) -> SendCommandResult:
        mc = self._mc_instances.get(robot_id)
        if mc is None:
            return SendCommandResult(succeeded=False, message=f"unknown RA robot_id: {robot_id}")

        role = RA_ROLE.get(robot_id)
        if role is None:
            return SendCommandResult(succeeded=False, message=f"RA role not configured: {robot_id}")

        if task_type in PRIMITIVE_TASKS:
            return self._handle_primitive(mc, task_type, payload)

        if role == "putaway":
            return self._handle_putaway(mc, task_type, payload)

        if role == "manufacturing":
            return self._handle_manufacturing(mc, task_type, payload)

        return SendCommandResult(succeeded=False, message=f"unsupported RA role: {role}")

    def _handle_primitive(self,
                          mc: MyCobot280,
                          task_type: str,
                          payload: Dict[str, Any]) -> SendCommandResult:
        try:
            if task_type in ("GRIPPER_OPEN", "GRIPPER_CLOSE"):
                inp   = GripperInput(**payload)
                value = 100 if task_type == "GRIPPER_OPEN" else 0
                mc.set_gripper_value(value, inp.speed)
                time.sleep(inp.delay)
                label = "gripper opened" if task_type == "GRIPPER_OPEN" else "gripper closed"
                return SendCommandResult(succeeded=True, message=label)

            if task_type == "GO_HOME":
                inp = GoHomeInput(**payload)
                mc.send_angles(list(HOME_ANGLES), inp.speed)
                time.sleep(max(inp.delay, 1.0))
                return SendCommandResult(succeeded=True, message="moved to home")

            if task_type == "MOVE_Z_REL":
                inp       = MoveZRelInput(**payload)
                coords    = mc.get_coords()

                if not coords:
                    return SendCommandResult(succeeded=False, message="get_coords returned empty (robot not ready?)")

                target     = list(coords)
                target[2] += inp.delta_z
                mc.send_coords(target, inp.speed, 0)
                time.sleep(max(inp.delay, 2.0))
                return SendCommandResult(succeeded=True, message=f"moved z by {inp.delta_z}")
        except Exception as exc:
            return SendCommandResult(succeeded=False, message=f"primitive failed: {type(exc).__name__}: {exc}")

        return SendCommandResult(succeeded=False, message=f"unhandled primitive: {task_type}")

    def _handle_putaway(self,
                          mc: MyCobot280,
                          task_type: str,
                          payload: Dict[str, Any]) -> SendCommandResult:
        if task_type == "PICK":
            try:
                inp = PutawayPickInput(**payload)
                mc.send_angles(list(PICK_PRE_ANGLES),    inp.speed)
                time.sleep(1.0)
                mc.send_angles(list(PICK_TARGET_ANGLES), 30)
                time.sleep(1.0)
                mc.set_gripper_value(0, 50)
                time.sleep(1.0)
                mc.send_angles(list(PICK_PRE_ANGLES),    inp.speed)
                time.sleep(1.0)
                return SendCommandResult(succeeded=True, message="picked")
            except Exception as exc:
                return SendCommandResult(succeeded=False, message=f"putaway PICK failed: {type(exc).__name__}: {exc}")

        try:
            inp = PutawayWaypointInput(**payload)
        except Exception as exc:
            return SendCommandResult(succeeded=False, message=f"invalid putaway payload: {exc}")

        key = (task_type, inp.floor, inp.cell)
        if key not in PUTAWAY_WAYPOINTS:
            return SendCommandResult(succeeded=False, message=f"unknown putaway waypoint: {key}")

        try:
            mc.send_angles(list(PUTAWAY_WAYPOINTS[key]), inp.speed)
            time.sleep(inp.delay)
            return SendCommandResult(succeeded=True, message=f"putaway {task_type} ({inp.floor},{inp.cell}) done")
        except Exception as exc:
            return SendCommandResult(succeeded=False, message=f"putaway failed: {type(exc).__name__}: {exc}")

    def _handle_manufacturing(self,
                        mc: MyCobot280,
                        task_type: str,
                        payload: Dict[str, Any]) -> SendCommandResult:
        if task_type not in MANUFACTURING_WAYPOINTS:
            return SendCommandResult(succeeded=False, message=f"unknown manufacturing waypoint: {task_type}")

        try:
            inp = ManufacturingWaypointInput(**payload)
        except Exception as exc:
            return SendCommandResult(succeeded=False, message=f"invalid manufacturing payload: {exc}")

        try:
            mc.send_angles(list(MANUFACTURING_WAYPOINTS[task_type]), inp.speed)
            time.sleep(inp.delay)
            return SendCommandResult(succeeded=True, message=f"manufacturing {task_type} done")
        except Exception as exc:
            return SendCommandResult(succeeded=False, message=f"manufacturing failed: {type(exc).__name__}: {exc}")