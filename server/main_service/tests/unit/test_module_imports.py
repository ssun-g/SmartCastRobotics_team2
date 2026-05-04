from main_service.management.services.core.event_bridge import EventBridge
from main_service.management.services.core.orchestrator import Orchestrator
from main_service.management.services.core.monitor_agent import MonitorAgent
from main_service.management.services.core.robot_adapter import RobotAdapter
from main_service.management.services.core.state_manager import StateManager
from main_service.management.services.core.task_allocator import TaskAllocator
from main_service.management.services.core.task_executor import TaskExecutor
from main_service.management.services.core.task_manager import TaskManager
from main_service.management.services.core.traffic_manager import TrafficManager


def test_main_service_modules_are_importable():
    assert Orchestrator
    assert TaskManager
    assert TaskAllocator
    assert TaskExecutor
    assert RobotAdapter
    assert StateManager
    assert EventBridge
    assert TrafficManager
    assert MonitorAgent

