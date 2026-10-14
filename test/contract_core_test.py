"""Contract boundary tests with controlled monotonic time; no implementation mirrors."""
import importlib.util
from pathlib import Path

module = importlib.util.spec_from_file_location('contract_core', Path(__file__).parents[1] / 'scripts/contract_core.py')
import sys
core = importlib.util.module_from_spec(module)
sys.modules[module.name] = core
module.loader.exec_module(core)


def release(state, start=1.0, command=core.RELEASE):
    for elapsed in (0, .05, .10, .151):
        state.observe(command, 0, start + elapsed, True)
        if state.cancelling:
            state.cancel_complete()  # This helper models no pending Action or goal receipt.
        state.update_ready(start + elapsed, True)


def test_boot_hold_goal_and_same_target_rearm():
    state = core.RequestState()
    assert not state.observe(core.MOVE, 1, 0, True)
    for elapsed in range(100):
        assert not state.observe(core.MOVE, 1, elapsed * .05, True)
    release(state, 6)
    assert state.state == core.READY
    assert state.observe(core.MOVE, 1, 6.2, True)
    generation = state.generation
    state.state = core.GOAL
    for elapsed in range(100):
        assert not state.observe(core.MOVE, 1, 6.25 + elapsed * .05, True)
    assert state.generation == generation
    state.observe(core.RELEASE, 0, 12, True)
    assert state.cancelling
    assert not state.observe(core.MOVE, 2, 12.01, True)
    state.cancel_complete()
    release(state, 13)
    assert state.observe(core.MOVE, 1, 13.2, True)
    assert state.generation > generation


def test_short_release_gap_and_quality_rejection_consumes_move():
    state = core.RequestState()
    for elapsed in (0, .05, .1):
        state.observe(core.RELEASE, 0, 1 + elapsed, True)
        state.update_ready(1 + elapsed, True)
    assert not state.observe(core.MOVE, 1, 1.14, True)
    release(state, 2)
    assert not state.observe(core.MOVE, 1, 2.2, False)
    assert not state.observe(core.MOVE, 1, 2.25, True)
    state.observe(core.RELEASE, 0, 3, True)
    state.observe(core.RELEASE, 0, 3.16, True)
    state.update_ready(3.16, True)
    assert not state.armed


def test_cancel_fault_priority_reset_and_no_auto_resume():
    state = core.RequestState()
    release(state)
    assert state.observe(core.MOVE, 7, 1.2, True)
    state.observe(core.RELEASE, 0, 1.21, True)
    assert state.cancelling and state.state != core.GOAL
    state.cancel(1.22, 'Input loss', fault=True)
    state.cancel_complete()
    release(state, 2)
    state.update_ready(2.2, True)
    assert state.state == core.FAIL
    assert state.reset(2.2, False)[0] is False
    assert state.reset(2.2, True)[0]
    assert state.state == core.READY and state.target_id == 7
    assert not state.cancelling


def test_production_age_duplicates_restart_and_retirement():
    fresh = core.Freshness(.1)
    assert fresh.accept(1, 1, 1000000000, 1050000000, 5) == (True, False)
    assert fresh.fresh(1060000000, 5.049)
    assert not fresh.accept(1, 1, 1000000000, 1060000000, 5.04)[0]
    assert not fresh.fresh(1060000000, 5.051)
    assert fresh.accept(2, 1, 1070000000, 1070000000, 5.07) == (True, True)
    assert not fresh.accept(1, 2, 1080000000, 1080000000, 5.08)[0]
    assert not fresh.fresh(1060000000, 5.08)


def test_angles_and_generation_overflow():
    import math
    assert core.wire_angle(math.radians(.5), 1, 0) == 1
    assert core.wire_angle(math.radians(-.5), 1, 0) == -1
    assert core.wire_angle(math.radians(20.01), 1, 0) is None
    assert core.wire_angle(float('nan'), 1, 0) is None
    assert abs(core.wrap(-math.pi - .1) - (math.pi - .1)) < 1e-9
    state = core.RequestState(generation=core.MAX_SEQUENCE - 1)
    release(state)
    assert not state.observe(core.MOVE, 1, 1.2, True)
    assert state.overflow and state.state == core.FAIL
