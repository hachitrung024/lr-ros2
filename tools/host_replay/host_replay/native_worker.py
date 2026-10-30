"""Independent FCU process; SDK setup cannot block time or mission responses."""

import json
from pathlib import Path
import select
import subprocess
import sys
import time

from .interactive import TerminalCommands
from .native import NativeFcu
from .records import Recording


def worker():
    config = json.loads(sys.stdin.readline())
    server = NativeFcu(Recording(config['mavlink_db_path']), config)
    commands = TerminalCommands()
    due, reboot = 0., 0
    print('READY', flush=True)
    try:
        while True:
            command = commands.poll()
            if command == 'quit':
                break
            if command:
                data = json.loads(command)
                if data['action'] == 'anchor':
                    server.reanchor(data['record_ns'], data['monotonic_ns'])
                elif data['action'] == 'pause':
                    server.paused = True
            if time.monotonic() >= due:
                if config.get('control_file'):
                    try:
                        faults = json.loads(Path(config['control_file']).read_text())
                        if faults.get('reboot', reboot) != reboot:
                            reboot = faults.get('reboot', 0)
                            server.origin = server.now()-1_000_000_000
                            server.recording.reset()
                        server.faults = faults
                    except (FileNotFoundError, ValueError):
                        pass
                if config.get('native_status_file'):
                    path = Path(config['native_status_file'])
                    report = dict(version=1, protocol='native',
                        private_packets=server.emitted_types.get('TUNNEL', 0),
                        network_errors=server.network_errors,
                        emitted_types=server.emitted_types,
                        record_ns=server.clock(), boot_ns=server.boot(),
                        system_id=server.mav.srcSystem, component_id=server.mav.srcComponent,
                        mission_count=len(server.mission.items), current_seq=server.seq,
                        paused=server.paused, counts=server.counts, faults=server.faults)
                    temporary = path.with_suffix(path.suffix+'.tmp')
                    temporary.write_text(json.dumps(report, indent=2)+'\n')
                    temporary.replace(path)
                due = time.monotonic()+.1
            server.service()
            time.sleep(.002)
    finally:
        server.close()


class NativeProcess:
    independent = True
    def __init__(self, args):
        self.process = subprocess.Popen([
            sys.executable, str(Path(__file__).resolve().parents[1]/'replay.py'), '--native-worker',
        ], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        try:
            self.process.stdin.write(json.dumps(vars(args))+'\n')
            self.process.stdin.flush()
            if not select.select([self.process.stdout], [], [], 10.)[0] or (
                    self.process.stdout.readline().strip() != 'READY'):
                raise RuntimeError('Native MAVLink worker failed to start')
        except BaseException:
            self.close()
            raise

    def send(self, data):
        self.service()
        self.process.stdin.write(json.dumps(data)+'\n')
        self.process.stdin.flush()

    def service(self):
        if self.process.poll() is not None:
            raise RuntimeError('Native MAVLink worker exited')

    def send_frame(self, _stamp):
        self.service()

    def close(self):
        try:
            if self.process.poll() is None:
                self.process.stdin.close()
                self.process.wait(timeout=3.)
        except (OSError, subprocess.TimeoutExpired):
            self.process.kill()
            self.process.wait(timeout=3.)
        finally:
            self.process.stdout.close()
