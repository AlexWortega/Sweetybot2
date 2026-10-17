"""Running evidence must exclude contact glitches and stationary jumping."""
import numpy as np

from learning.evaluate import COMMANDS, RUNNING_COMMANDS, FlightEvidence, running_successes


def tick(evidence, *, air=True, bad=0, up=.99, height=.20, vx=.45, active=True):
    evidence.update([[not air]*4], [bad], [up], [height], [vx], [active])


def test_sustained_intervals_and_trailing_interval_counted_once():
    evidence = FlightEvidence(1, .02)
    tick(evidence)  # Single-tick glitch cannot count.
    tick(evidence, air=False)
    assert evidence.intervals.tolist() == [0]
    tick(evidence)
    tick(evidence)
    tick(evidence, air=False)
    assert evidence.intervals.tolist() == [1]
    for _ in range(3):
        tick(evidence)
    evidence.finish()
    evidence.finish()
    assert evidence.intervals.tolist() == [2]
    assert evidence.valid_ticks.tolist() == [5]
    np.testing.assert_allclose(evidence.longest_ticks*.02, [.06])


def test_only_clean_forward_upright_flight_qualifies():
    # At every index another exclusion holds: stance, body contact, tipped,
    # low, implausibly high, stationary jump, failed/previously fallen world.
    evidence = FlightEvidence(7, .02)
    for _ in range(3):
        evidence.update(
            [[True]*4] + [[False]*4]*6,
            [0, 1, 0, 0, 0, 0, 0],
            [.99, .99, .5, .99, .99, .99, .99],
            [.20, .20, .20, .14, .29, .20, .20],
            [.45, .45, .45, .45, .45, 0., .45],
            [True, True, True, True, True, True, False],
        )
    evidence.finish()
    assert not evidence.intervals.any()
    assert not evidence.valid_ticks.any()


def test_body_contact_breaks_flight_interval():
    evidence = FlightEvidence(1, .02)
    tick(evidence)
    tick(evidence, bad=1)
    tick(evidence)
    evidence.finish()
    assert evidence.intervals.tolist() == [0]


def test_running_requires_survival_speed_repeated_flight_and_stable_end():
    # Positive control, stationary jump, fast shuffle, one jump, fall after
    # running, unstable final state, tiny air fraction, poor command tracking.
    accepted = running_successes(
        [True, True, True, True, False, True, True, True],
        [True, True, True, True, True, False, True, True],
        [.45, 0., .45, .45, .45, .45, .45, .36],
        [.55]*8,
        [2, 4, 0, 1, 2, 2, 2, 2],
        [.04, .20, 0., .04, .04, .04, .02, .04],
    )
    assert accepted.tolist() == [True, False, False, False, False, False, False, False]


def test_running_commands_keep_existing_locomotion_suite_unchanged():
    assert tuple(COMMANDS) == ("stand", "forward", "fast_forward", "backward", "sideways", "turn")
    assert tuple(RUNNING_COMMANDS) == ("forward", "fast_forward", "run_turn")
    assert all(command[0] >= .4 for command in RUNNING_COMMANDS.values())
