"""ROS timestamp conversion without recording or cache dependencies."""


def stamp_to_nanoseconds(stamp) -> int:
    """Convert a builtin_interfaces Time message to integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def nanoseconds_to_stamp(value: int, stamp) -> None:
    """Populate a builtin_interfaces Time message from nanoseconds."""
    stamp.sec = int(value) // 1_000_000_000
    stamp.nanosec = int(value) % 1_000_000_000
