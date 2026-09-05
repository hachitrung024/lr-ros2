"""Future and stale messages must not replace eligible cycle inputs."""

from types import SimpleNamespace

from prediction_ros.input_history import InputHistory


def message(sec, ns=0, frame='map'):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame, stamp=SimpleNamespace(sec=sec, nanosec=ns))
    )


def test_select_past_sample_instead_of_future_and_reject_wrong_frame():
    history = InputHistory(0.5)
    valid = message(9, 800_000_000)
    history.append(valid)
    history.append(message(10, 100_000_000))
    history.append(message(10, frame='odom'))
    assert history.select(message(10))[0] is valid
    assert history.select(message(11))[0] is None
    history.clear()
    assert history.select(message(10))[0] is None


def test_history_has_a_hard_memory_bound():
    history = InputHistory(None, max_samples=3)
    for stamp in range(20):
        history.append(message(stamp))
    assert len(history.messages) == 3
