"""Exercise delayed goal/arrival events against already revoked executions."""
import importlib.util
import sys
from pathlib import Path
from collections import deque
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from auto_mode_manager_runtime import AutoModeManager, Execution
from contract_core import RequestState, STANDBY, FAIL


class Future:
    def __init__(self, result):
        self.value = result
    def result(self):
        return self.value
    def add_done_callback(self, callback):
        self.callback = callback


class DelayedGoal:
    accepted = True
    def __init__(self):
        self.cancel_calls = 0
        self.terminal = Future(None)
    def get_result_async(self):
        return self.terminal
    def cancel_goal_async(self):
        self.cancel_calls += 1
        return Future(None)


def coordinator(execution):
    manager = object.__new__(AutoModeManager)
    manager.core = RequestState(state=STANDBY, generation=10, cancelling=True)
    manager.execution = execution
    manager.events = deque()
    return manager


def test_late_goal_acceptance_canceled_and_arrival_not_adopted():
    execution = Execution(9, 1, 0, goal_pending=True, terminal=False, cleared=False)
    manager = coordinator(execution)
    handle = DelayedGoal()
    manager.events.append(('accepted', execution, Future(handle)))
    manager.process_events()
    assert not execution.goal_pending and handle.cancel_calls == 1
    assert not execution.terminal and manager.core.cancelling
    outcome = SimpleNamespace(status=4, result=SimpleNamespace(success=True, generation=9, reason='Late success'))
    manager.events.append(('result', execution, Future(outcome)))
    manager.process_events()
    assert execution.terminal
    assert manager.core.state == STANDBY and manager.core.generation == 10
    assert not execution.cleared  # Action terminal alone never confirms history removal.


def test_invalidated_plan_and_fault_cannot_be_overwritten():
    execution = Execution(9, 1, 0)
    manager = coordinator(execution)
    manager.core.state = FAIL
    manager.events.append(('plan', execution, Future(SimpleNamespace(success=True, generation=9, path=object()))))
    manager.process_events()
    assert execution.plan is None and manager.core.state == FAIL
