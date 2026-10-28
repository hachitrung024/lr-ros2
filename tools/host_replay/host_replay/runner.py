"""One monotonic scheduler for SVO frames, replay clock and causal telemetry."""

import argparse
from contextlib import ExitStack
from pathlib import Path
import signal
import sqlite3
import sys
import time

from .clock import ClockSender
from .live import LiveTime
from .records import Recording
from .source import SvoSource
from .telemetry import TelemetrySender, validate_recording


def port(value):
    number = int(value)
    if not 1 <= number <= 65535:
        raise argparse.ArgumentTypeError('port must be in [1, 65535]')
    return number


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--svo-path', required=True)
    parser.add_argument('--mavlink-db-path', required=True)
    parser.add_argument('--stream-port', required=True, type=port)
    parser.add_argument('--mavlink-bind-address', required=True)
    parser.add_argument('--mavlink-bind-port', required=True, type=port)
    parser.add_argument('--mavlink-destination', required=True)
    parser.add_argument('--mavlink-destination-port', required=True, type=port)
    parser.add_argument('--time-mode', choices=('replay', 'live'), default='replay')
    parser.add_argument('--clock-destination')
    parser.add_argument('--clock-destination-port', type=port)
    parser.add_argument('--play-from-frame', type=int, default=0)
    args = parser.parse_args(argv)
    if args.stream_port % 2 or args.stream_port > 65534:
        parser.error('--stream-port must be even and in [2, 65534]')
    if args.play_from_frame < 0:
        parser.error('--play-from-frame must be non-negative')
    if args.time_mode == 'replay' and (
            not args.clock_destination or args.clock_destination_port is None):
        parser.error('replay requires --clock-destination and --clock-destination-port')
    if args.time_mode == 'live' and (
            args.clock_destination is not None or args.clock_destination_port is not None):
        parser.error('live does not accept clock endpoints')
    for name in ('mavlink_bind_address', 'mavlink_destination'):
        if not getattr(args, name).strip():
            parser.error(name.replace('_', '-') + ' must be nonempty')
    return args


def run(source, telemetry, clock=None, *, monotonic=time.monotonic, sleep=time.sleep):
    """Emit each frame once, with clock before telemetry and no catch-up bursts."""
    frames = 0
    deadline = monotonic()
    anchor = deadline
    first_stamp = source.stamp_ns
    while True:
        if clock is not None:
            clock.send(source.stamp_ns)
        telemetry.send_frame(source.stamp_ns)
        frames += 1
        deadline = (anchor + (source.stamp_ns - first_stamp) / 1e9 + source.period
                    if clock is None else deadline + source.period)
        now = monotonic()
        if deadline <= now:
            deadline = now + source.period
        while monotonic() < deadline:
            telemetry.service()
            sleep(max(0.0, min(0.01, deadline - monotonic())))
        if not source.tick():
            return frames


def main(argv=None):
    args = arguments(argv)

    def interrupted(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        coverage = validate_recording(args.mavlink_db_path)
        recording = Recording(Path(args.mavlink_db_path).expanduser())
        with ExitStack() as resources:
            source = SvoSource(args.svo_path, args.stream_port, args.play_from_frame)
            resources.callback(source.close)
            # Match the existing explicit-DB selection tolerance, including mid-session joins.
            if not coverage[0] - 60_000_000_000 <= source.stamp_ns <= coverage[1] + 60_000_000_000:
                raise ValueError('The selected MAVLink DB does not cover the first SVO timestamp')
            live = LiveTime(source.stamp_ns) if args.time_mode == 'live' else None
            options = {'live': live} if live is not None else {}
            telemetry = TelemetrySender(
                recording, args.mavlink_bind_address, args.mavlink_bind_port,
                args.mavlink_destination, args.mavlink_destination_port, **options,
            )
            resources.callback(telemetry.close)
            clock = None
            if args.time_mode == 'replay':
                clock = ClockSender(
                    args.mavlink_bind_address, args.clock_destination, args.clock_destination_port,
                )
                resources.callback(clock.close)
            print(f'SVO NET port={args.stream_port}, fps={source.fps:g}, '
                  f'frame={source.frame_id}/{source.total_frames}, '
                  f'epoch_us={recording.epoch_us}, mode={args.time_mode}, '
                  f'session={live.session_id if live else clock.session_id}', flush=True)
            frames = run(source, telemetry, clock)
            print(f'EOF: {frames} frames, {telemetry.sent} sensor packets', flush=True)
        return 0
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, RuntimeError, ImportError, sqlite3.Error) as error:
        print(f'Replay failed: {error}', file=sys.stderr, flush=True)
        return 1
