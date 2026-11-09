"""Read-only TMCM-1111 input monitor.

Polls every input the board exposes and reports which ones change, so you can
press/release a switch by hand and see which register it is wired to.
Nothing is written to the board: the motor is not moved and outputs are not touched.

Usage (from the project root):
    python b_diagnostics/io_monitor.py                 # wheel board, port from motor_config.yaml
    python b_diagnostics/io_monitor.py --board spindle
    python b_diagnostics/io_monitor.py --port COM7     # override the COM port
Stop with Ctrl+C; a summary of every input that toggled is printed on exit.
"""
import argparse
import time
from pathlib import Path

import yaml
from pytrinamic.connections import ConnectionManager

CONFIG_PATH = Path(__file__).resolve().parent.parent / 'a_config' / 'motor_config.yaml'

# (label, kind, index). kind: 'ap' = axis parameter (GAP n, 0), 'di' = GIO n, 0, 'ai' = GIO n, 1
SWITCH_SIGNALS = [
    ('H  (AP 9  HomeSwitch)',   'ap', 9),
    ('R  (AP 10 RightEndstop)', 'ap', 10),
    ('L  (AP 11 LeftEndstop)',  'ap', 11),
]
DIGITAL_PORTS = range(8)   # GIO <port>, 0
ANALOG_PORTS = range(4)    # GIO <port>, 1
ANALOG_THRESHOLD = 200     # counts a reading must move to count as a change (0..4095 for 12-bit)


def read(iface, kind, index):
    if kind == 'ap':
        return iface.get_axis_parameter(index, 0)
    if kind == 'di':
        return iface.get_digital_input(index)
    return iface.get_analog_input(index)


def probe(iface):
    """Keep only the signals this firmware answers; unsupported ports raise a TMCL error."""
    candidates = list(SWITCH_SIGNALS)
    candidates += [(f'DI {p}  (GIO {p}, 0)', 'di', p) for p in DIGITAL_PORTS]
    candidates += [(f'AI {p}  (GIO {p}, 1)', 'ai', p) for p in ANALOG_PORTS]
    signals = []
    for label, kind, index in candidates:
        try:
            read(iface, kind, index)
            signals.append((label, kind, index))
        except Exception as e:
            print(f"  skipping {label.strip():<24} ({type(e).__name__})")
    return signals


def changed(kind, old, new):
    if kind == 'ai':
        return abs(new - old) >= ANALOG_THRESHOLD
    return new != old


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--board', default='wheel', choices=['wheel', 'spindle'])
    ap.add_argument('--port', help='COM port override (default: from motor_config.yaml)')
    ap.add_argument('--interval', type=float, default=0.05, help='poll period in seconds')
    args = ap.parse_args()

    config = yaml.safe_load(CONFIG_PATH.read_text())
    port = args.port or config[args.board]['port']
    print(f"Connecting to {args.board} board on {port} ...")
    iface = ConnectionManager(f"--interface usb_tmcl --port {port}").connect()

    try:
        signals = probe(iface)
        baseline = {s: read(iface, s[1], s[2]) for s in signals}
        toggles = {s: 0 for s in signals}

        print("\nBaseline (raw values; switch polarity varies, so watch for CHANGES):")
        for s in signals:
            print(f"  {s[0]:<26} = {baseline[s]}")
        print("\nNow press/release the switch. Ctrl+C to stop.\n")

        last = dict(baseline)
        t0 = time.time()
        while True:
            for s in signals:
                try:
                    v = read(iface, s[1], s[2])
                except Exception as e:
                    print(f"  read error on {s[0].strip()}: {e}")
                    continue
                if changed(s[1], last[s], v):
                    toggles[s] += 1
                    print(f"[{time.time() - t0:7.2f}s] {s[0]:<26} {last[s]} -> {v}   (toggles: {toggles[s]})")
                    last[s] = v
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        iface.close()

    print("\nSummary:")
    for s in signals:
        status = f"TRIGGERED ({toggles[s]} changes)" if toggles[s] else "not triggered"
        print(f"  {s[0]:<26} {status}")


if __name__ == '__main__':
    main()
