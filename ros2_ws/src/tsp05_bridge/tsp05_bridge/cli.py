"""Test / debug tool for the TSP05 bridge.

    tsp05 check [--auto]     guided hardware test with a PASS/FAIL summary
    tsp05 watch              one-line live view of all sensors and motors
    tsp05 mode gui|sensor|toggle
    tsp05 servo <deg>        (GUI mode)
    tsp05 step <delta deg>   (GUI mode)
    tsp05 dc <rpm>           (GUI mode)
    tsp05 dcdeg <delta deg>  move the DC motor by degrees (GUI mode, PID)
    tsp05 safety <cm>        (0 disables)
    tsp05 calibrate          guided flex (force sensor) calibration
    tsp05 characterize sharp|tof|flex|pot
                             record converted output vs known reference values
                             (measured error + transfer function data)
    tsp05 dctest [--forward yes|no]
                             DC motor polarity + speed-control test (motor spins!)

Requires tsp05_serial_node to be running (scripts/tsp05 starts it).
"""

import argparse
import csv
import json
import math
import os
import select
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32, String

from . import protocol

PREFIX = '/tsp05'

GREEN = '\033[32m'
RED = '\033[31m'
YELLOW = '\033[33m'
BOLD = '\033[1m'
RESET = '\033[0m'


class Client(Node):
    """Keeps the latest Arduino state and sends commands."""

    def __init__(self):
        super().__init__('tsp05_cli')
        self.lock = threading.Lock()
        self.state = None
        self.state_time = 0.0
        self.state_count = 0
        self.events = []

        self.create_subscription(
            String, f'{PREFIX}/state_json', self.on_state, 10)
        self.create_subscription(
            String, f'{PREFIX}/events', self.on_event, 10)

        self.pubs = {
            'mode': self.create_publisher(String, f'{PREFIX}/cmd/mode', 10),
            'servo': self.create_publisher(
                Float32, f'{PREFIX}/cmd/servo_angle', 10),
            'step': self.create_publisher(
                Float32, f'{PREFIX}/cmd/stepper_delta', 10),
            'dc': self.create_publisher(Float32, f'{PREFIX}/cmd/dc_rpm', 10),
            'dcdeg': self.create_publisher(
                Float32, f'{PREFIX}/cmd/dc_deg', 10),
            'dc_pwm': self.create_publisher(
                Float32, f'{PREFIX}/cmd/dc_pwm', 10),
            'dc_pol': self.create_publisher(
                String, f'{PREFIX}/cmd/dc_polarity', 10),
            'safety': self.create_publisher(
                Float32, f'{PREFIX}/cmd/safety_threshold', 10),
            'cal': self.create_publisher(
                String, f'{PREFIX}/cmd/flex_calibrate', 10),
        }

    def on_state(self, msg):
        parsed = protocol.parse_line(msg.data)
        if parsed is None:
            return
        with self.lock:
            self.state = parsed
            self.state_time = time.monotonic()
            self.state_count += 1

    def on_event(self, msg):
        try:
            event = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.events.append(event)

    def get(self):
        with self.lock:
            return self.state

    def fresh(self, max_age=1.0):
        with self.lock:
            return (self.state is not None and
                    time.monotonic() - self.state_time < max_age)

    def take_events(self):
        with self.lock:
            events, self.events = self.events, []
        return events

    def send(self, name, value, timeout=3.0):
        """Publish a command; returns the Arduino's ack dict (or None)."""
        pub = self.pubs[name]
        end = time.monotonic() + timeout
        while pub.get_subscription_count() == 0 and time.monotonic() < end:
            time.sleep(0.05)
        if pub.get_subscription_count() == 0:
            return None

        self.take_events()
        if name in ('mode', 'cal', 'dc_pol'):
            msg = String(data=str(value))
        else:
            msg = Float32(data=float(value))
        pub.publish(msg)

        cmd = {'mode': 'MODE', 'servo': 'SERVO', 'step': 'STEP',
               'dc': 'DC', 'dcdeg': 'DCDEG', 'dc_pwm': 'DCPWM', 'dc_pol': 'DCPOL',
               'safety': 'SAFETY', 'cal': 'CAL'}[name]
        while time.monotonic() < end:
            for event in self.take_events():
                if event.get('t') == 'ack' and event.get('cmd') == cmd:
                    return event
            time.sleep(0.02)
        return None


def start_client():
    rclpy.init()
    client = Client()
    executor = SingleThreadedExecutor()
    executor.add_node(client)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return client, executor, thread


def wait_for_state(client, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if client.fresh():
            return True
        time.sleep(0.05)
    return False


def fmt(v, digits=1):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return '  --'
    return f'{v:.{digits}f}'


def status_line(s):
    tof = fmt(s['tof_cm']) if s['tof_ok'] else 'ERR'
    sharp = fmt(s['sharp_dist_cm'], 0) if s['sharp_valid'] else s['sharp_st']
    return (
        f"{s['mode']:<6} {'SAFETY' if s['safety'] else 'ok':<6} | "
        f"flex {s['flex_angle']:5.1f}° ({s['flex_kohm']:4.1f}k) | "
        f"pot {s['pot_deg']:5.1f}° | tof {tof:>5}cm | sharp {sharp:>9} | "
        f"btn {'X' if s['btn'] else '_'}{s['btn_n']:<3} | "
        f"servo {s['servo_deg']:3d}° step {s['step_deg']:6.1f}° "
        f"dc {s['dc_tgt']:5.1f}->{s['dc_rpm']:5.1f}rpm {s['dc_deg']:7.1f}° pwm {s['dc_pwm']:4d}"
        f"{' STALL' if s['dc_stall'] else ''}")


# ----------------------------------------------------------------------
# Guided check
# ----------------------------------------------------------------------

class Check:
    def __init__(self, client, interactive):
        self.c = client
        self.interactive = interactive
        self.results = []

    def record(self, name, ok, detail=''):
        if ok is None:
            tag = f'{YELLOW}SKIP{RESET}'
        elif ok:
            tag = f'{GREEN}PASS{RESET}'
        else:
            tag = f'{RED}FAIL{RESET}'
        print(f'  [{tag}] {name}' + (f' - {detail}' if detail else ''))
        self.results.append((name, ok, detail))

    def watch_until(self, prompt, condition, timeout=20.0):
        """Show live values until condition(state, start_state) is true.

        Returns True/False, or None if the user pressed Enter to skip.
        """
        print(f'\n  {BOLD}>> {prompt}{RESET}  (Enter to skip, {timeout:.0f} s)')
        start = self.c.get()
        end = time.monotonic() + timeout
        try:
            while time.monotonic() < end:
                s = self.c.get()
                if s is not None:
                    sys.stdout.write('\r     ' + status_line(s) + '  ')
                    sys.stdout.flush()
                    if condition(s, start):
                        print()
                        return True
                if sys.stdin.isatty():
                    ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                    if ready:
                        sys.stdin.readline()
                        print()
                        return None
                else:
                    time.sleep(0.1)
        except KeyboardInterrupt:
            print()
            raise
        print()
        return False

    def ask_yes(self, question):
        if not sys.stdin.isatty():
            return None
        answer = input(f'  {BOLD}?? {question} [y/n/Enter=skip]{RESET} ').strip().lower()
        if answer.startswith('y'):
            return True
        if answer.startswith('n'):
            return False
        return None

    def collect(self, seconds):
        """Collect states for a few seconds."""
        states = []
        end = time.monotonic() + seconds
        last = None
        while time.monotonic() < end:
            s = self.c.get()
            if s is not None and s is not last:
                states.append(s)
                last = s
            time.sleep(0.02)
        return states

    def set_mode(self, mode):
        ack = self.c.send('mode', mode)
        return ack is not None and ack.get('ok')

    # -- sections -------------------------------------------------------

    def run(self):
        print(f'{BOLD}TSP05 hardware check{RESET}')
        if not self.connection():
            return self.summary()

        start = self.c.get()
        original_mode = start['mode']
        original_safety = start['safety_cm']
        try:
            self.sensors_auto()
            if self.interactive:
                self.sensors_interactive()
                self.motors_gui()
                self.sensor_mode()
            else:
                print(f'\n  {YELLOW}--auto: skipping interactive checks{RESET}')
        finally:
            # Restore the board to how we found it
            self.c.send('safety', original_safety)
            self.set_mode(original_mode)
        return self.summary()

    def connection(self):
        print('\n1. Connection')
        if not wait_for_state(self.c, 6.0):
            self.record('Receiving data from bridge', False,
                        'no /tsp05/state_json - is tsp05_serial_node running '
                        'and the Arduino plugged in?')
            return False
        n0 = self.c.state_count
        time.sleep(2.0)
        rate = (self.c.state_count - n0) / 2.0
        self.record('Receiving data from bridge', True)
        self.record('State rate ~10 Hz', 8.0 <= rate <= 12.0, f'{rate:.1f} Hz')
        return True

    def sensors_auto(self):
        print('\n2. Sensor readings (automatic)')
        states = self.collect(2.0)
        s = states[-1]

        flex = [x['flex_adc'] for x in states]
        self.record('Flex sensor connected', 100 <= min(flex) and max(flex) <= 1000,
                    f"ADC {min(flex):.0f}-{max(flex):.0f}, {s['flex_kohm']:.1f} kOhm "
                    '(straight ~13 kOhm)')

        pot = [x['pot_raw'] for x in states]
        self.record('Potentiometer stable', max(pot) - min(pot) <= 40,
                    f"{s['pot_v']:.2f} V, {s['pot_pct']}%, spread {max(pot) - min(pot)}")

        tof_ok = all(x['tof_ok'] for x in states)
        self.record('VL53L1X responding', tof_ok,
                    f"{s['tof_cm']:.1f} cm" if tof_ok else 'not detected / timeout')

        sharp_v = [x['sharp_v'] for x in states]
        self.record('Sharp IR output', True,
                    f"{min(sharp_v):.2f}-{max(sharp_v):.2f} V, {s['sharp_st']}")

        self.record('DC motor', None if not s['dc_on'] else True,
                    'disabled in firmware (DC_MOTOR_CONNECTED = false)'
                    if not s['dc_on'] else '')

    def sensors_interactive(self):
        print('\n3. Sensors (interactive)')

        # Button toggles the mode, so remember it and put it back
        mode_before = self.c.get()['mode']
        ok = self.watch_until(
            'Press and release the BUTTON once',
            lambda s, s0: s['btn_n'] > s0['btn_n'] and not s['btn'])
        if ok:
            time.sleep(0.3)
            s = self.c.get()
            self.record('Button press counted', True, f"count {s['btn_n']}")
            self.record('Button toggles mode', s['mode'] != mode_before,
                        f"{mode_before} -> {s['mode']}")
            self.set_mode(mode_before)
        else:
            self.record('Button press counted', ok)

        lo, hi = [101], [-1]

        def pot_sweep(s, s0):
            lo[0] = min(lo[0], s['pot_pct'])
            hi[0] = max(hi[0], s['pot_pct'])
            return lo[0] <= 5 and hi[0] >= 95
        ok = self.watch_until('Turn the POTENTIOMETER fully to both ends', pot_sweep)
        self.record('Potentiometer full range', ok,
                    f'{lo[0]}-{hi[0]}%' if ok is not None else '')

        peak = [0.0]

        def flex_bend(s, s0):
            peak[0] = max(peak[0], s['flex_kohm'] - s0['flex_kohm'])
            return s['flex_deg'] >= 45
        ok = self.watch_until('BEND the flex sensor (to >= 45 deg)', flex_bend)
        self.record('Flex sensor responds to bending', ok,
                    f"+{peak[0]:.1f} kOhm" if ok is not None else '')
        if ok is False:
            print(f'     {YELLOW}Tip: run "scripts/tsp05 calibrate" so a full bend '
                  f'= 100 % force{RESET}')
        if ok:
            self.watch_until('Straighten the flex sensor',
                             lambda s, s0: s['flex_deg'] < 10, timeout=10)

        span = [math.inf, -math.inf]

        def tof_move(s, s0):
            if s['tof_ok']:
                span[0] = min(span[0], s['tof_cm'])
                span[1] = max(span[1], s['tof_cm'])
            return span[1] - span[0] >= 15
        ok = self.watch_until(
            'Move your hand towards/away from the VL53L1X (>= 15 cm change)',
            tof_move)
        self.record('VL53L1X tracks distance', ok,
                    f'{span[0]:.0f}-{span[1]:.0f} cm' if ok else '')

        self.c.send('safety', 20)
        ok = self.watch_until(
            'Hold an object ~10-15 cm in front of the SHARP sensor',
            lambda s, s0: s['safety'])
        self.record('Safety stop triggers (Sharp < 20 cm)', ok)
        if ok:
            ok = self.watch_until('Remove the object',
                                  lambda s, s0: not s['safety'], timeout=10)
            self.record('Safety stop releases', ok)

    def motors_gui(self):
        print('\n4. Motors in GUI mode')
        if not self.set_mode('GUI'):
            self.record('Switch to GUI mode', False)
            return
        if self.c.get()['safety']:
            ok = self.watch_until(
                'Safety stop is active - clear the area in front of the Sharp',
                lambda s, s0: not s['safety'])
            if not ok:
                self.record('Motors (GUI)', None, 'safety stop active')
                return

        for angle in (0, 180, 90):
            ack = self.c.send('servo', angle)
            time.sleep(1.0)
            self.record(f'Servo command {angle} deg',
                        bool(ack and ack.get('ok')) and
                        self.c.get()['servo_deg'] == angle,
                        (ack or {}).get('err', ''))
        self.record('Servo moved 0 -> 180 -> 90 (seen)',
                    self.ask_yes('Did the servo swing 0 -> 180 -> 90?'))

        start = self.c.get()['step_deg']
        ack = self.c.send('step', 90)
        ok = self.watch_until(
            'Stepper turning +90 deg...',
            lambda s, s0: abs(s['step_deg'] - (start + 90)) < 0.5, timeout=5)
        self.record('Stepper +90 deg', bool(ack and ack.get('ok')) and ok)
        ack = self.c.send('step', -90)
        ok = self.watch_until(
            'Stepper turning back -90 deg...',
            lambda s, s0: abs(s['step_deg'] - start) < 0.5, timeout=5)
        self.record('Stepper -90 deg', bool(ack and ack.get('ok')) and ok)
        self.record('Stepper moved +90 then back (seen)',
                    self.ask_yes('Did the stepper turn a quarter turn and back?'))

        ack = self.c.send('dc', 40)
        self.record('DC command accepted', bool(ack and ack.get('ok')),
                    'target only, motor disabled' if not self.c.get()['dc_on'] else '')
        if self.c.get()['dc_on']:
            ok, got = dc_settle(self, 40.0)
            self.record('DC holds 40 rpm', ok,
                        f'measured {got:+.1f} (wrong sign? run: tsp05 dctest)')
        self.c.send('dc', 0)

    def sensor_mode(self):
        print('\n5. SENSOR mode (sensors drive motors)')
        if not self.set_mode('SENSOR'):
            self.record('Switch to SENSOR mode', False)
            return

        # Let SENSOR mode take over the servo first (it may still be at a
        # GUI-commanded angle), so the check below can't pass by accident
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            s = self.c.get()
            if abs(s['servo_deg'] - s['flex_deg']) <= 10:
                break
            time.sleep(0.1)

        ok = self.watch_until(
            'BEND the flex sensor - the SERVO should follow',
            lambda s, s0: (s['flex_deg'] >= 45 and
                           abs(s['servo_deg'] - s['flex_deg']) <= 10))
        self.record('Flex -> servo', ok)
        if ok:
            self.watch_until('Straighten the flex sensor',
                             lambda s, s0: s['servo_deg'] == 0, timeout=10)

        def pot_follow(s, s0):
            return (abs(s['step_tgt'] - s0['step_tgt']) >= 45 and
                    abs(s['step_deg'] - s['step_tgt']) < 1.0)
        ok = self.watch_until(
            'Turn the POTENTIOMETER - the STEPPER should follow', pot_follow)
        self.record('Potentiometer -> stepper', ok)

        def dc_follow(s, s0):
            return abs(s['dc_tgt'] - s0['dc_tgt']) >= 15
        ok = self.watch_until(
            'Move your hand in front of the VL53L1X - DC target speed changes',
            dc_follow)
        self.record('VL53L1X -> DC target speed', ok)

        self.record('Servo/stepper followed (seen)',
                    self.ask_yes('Did the servo and stepper physically follow?'))

    def summary(self):
        passed = sum(1 for _, ok, _ in self.results if ok is True)
        failed = [(n, d) for n, ok, d in self.results if ok is False]
        skipped = sum(1 for _, ok, _ in self.results if ok is None)
        print(f'\n{BOLD}Summary:{RESET} {GREEN}{passed} passed{RESET}, '
              f'{RED if failed else ""}{len(failed)} failed{RESET}, '
              f'{skipped} skipped')
        for name, detail in failed:
            print(f'  {RED}FAIL{RESET} {name}' + (f' - {detail}' if detail else ''))
        return 1 if failed else 0


# ----------------------------------------------------------------------
# DC motor polarity + speed control test
# ----------------------------------------------------------------------

DC_TEST_PWM = 180   # motor needs ~140 to start
DC_TEST_RPM = 30.0


def dc_open_loop(chk, pwm):
    """Run the motor open loop; returns the mean measured rpm (or None)."""
    ack = chk.c.send('dc_pwm', pwm)
    if not (ack and ack.get('ok')):
        print(f"  {RED}DCPWM rejected: {(ack or {}).get('err', 'no reply')}{RESET}")
        return None
    chk.collect(0.8)                      # PWM ramps up (30 per 200 ms)
    rpms = [x['dc_rpm'] for x in chk.collect(1.0)]
    chk.collect(0.6)                      # Arduino stops it after 2 s
    return sum(rpms) / len(rpms) if rpms else None


def dc_settle(chk, target, timeout=4.0, tol_frac=0.2):
    """Wait until the rpm stays within tolerance for 1 s."""
    tol = max(abs(target) * tol_frac, 3.0)
    end = time.monotonic() + timeout
    good_since = None
    last = None
    while time.monotonic() < end:
        s = chk.c.get()
        if s is not last:
            last = s
            sys.stdout.write('\r     ' + status_line(s) + '  ')
            sys.stdout.flush()
            if abs(s['dc_rpm'] - target) <= tol:
                good_since = good_since or time.monotonic()
                if time.monotonic() - good_since >= 1.0:
                    print()
                    return True, s['dc_rpm']
            else:
                good_since = None
        time.sleep(0.02)
    print()
    s = chk.c.get()
    if s['dc_stall']:
        print(f'     {RED}Motor stalled (driven, encoder sees no motion) - output '
              f'cut. Check motor power / driver / encoder wiring.{RESET}')
    return False, s['dc_rpm']


def dctest(client, forward):
    chk = Check(client, interactive=sys.stdin.isatty())
    print(f'{BOLD}DC motor test{RESET} - {YELLOW}the motor will spin, '
          f'keep hands clear{RESET}')
    s = client.get()
    if not s['dc_on']:
        print(f'{RED}DC motor is disabled in the firmware (DC_MOTOR_CONNECTED){RESET}')
        return 1
    original_mode = s['mode']
    try:
        if not chk.set_mode('GUI'):
            chk.record('Switch to GUI mode', False)
            return chk.summary()
        if client.get()['safety']:
            ok = chk.watch_until(
                'Safety stop is active - clear the area in front of the Sharp',
                lambda st, s0: not st['safety'])
            if not ok:
                chk.record('DC test', None, 'safety stop active')
                return chk.summary()

        pol = client.get()['dc_pol']
        motor_inv, enc_inv = bool(pol & 1), bool(pol & 2)
        print(f'\n1. Open loop, PWM +{DC_TEST_PWM} (polarity now: '
              f'motor_invert={int(motor_inv)} encoder_invert={int(enc_inv)})')
        rpm = dc_open_loop(chk, DC_TEST_PWM)
        if rpm is None:
            chk.record('Open-loop run', False)
            return chk.summary()
        spins = abs(rpm) >= 3.0
        chk.record('Encoder sees the motor turning', spins, f'{rpm:+.1f} rpm')
        if not spins:
            print('     Check motor power, the driver wiring (ENA/IN1/IN2) and '
                  'encoder A->D3, B->D4.')
            return chk.summary()

        if forward is None:
            forward = chk.ask_yes('Did the motor turn in the FORWARD direction?')
        if forward is None:
            forward = True
            print(f'  {YELLOW}No answer - keeping the current motor direction{RESET}')

        if not forward:
            motor_inv = not motor_inv
            rpm = -rpm                    # flipping the motor flips the reading
        if rpm < 0:
            enc_inv = not enc_inv         # make forward read as positive rpm
        ack = client.send('dc_pol', f'{int(motor_inv)} {int(enc_inv)}')
        chk.record('Polarity saved', bool(ack and ack.get('ok')),
                   f'motor_invert={int(motor_inv)} encoder_invert={int(enc_inv)}')
        time.sleep(0.3)

        print(f'\n2. Check: open loop +{DC_TEST_PWM} should now read positive')
        rpm = dc_open_loop(chk, DC_TEST_PWM)
        chk.record('+PWM gives +rpm', rpm is not None and rpm >= 3.0,
                   f'{rpm:+.1f} rpm' if rpm is not None else '')

        print('\n3. Closed-loop speed control')
        for target in (DC_TEST_RPM, -DC_TEST_RPM, 0.0):
            client.send('dc', target)
            ok, got = dc_settle(chk, target)
            chk.record(f'Holds {target:+.0f} rpm', ok, f'measured {got:+.1f}')
        if forward is not False and chk.interactive:
            chk.record('Forward at +rpm (seen)', chk.ask_yes(
                'Did it spin forward first, then backward?'))
    finally:
        client.send('dc', 0)
        chk.set_mode(original_mode)
    return chk.summary()


# ----------------------------------------------------------------------
# Calibration
# ----------------------------------------------------------------------

def calibrate(client):
    """Guided two-point calibration of the flex (force) sensor."""
    s = client.get()
    print(f'{BOLD}Flex (force sensor) calibration{RESET}')
    print(f"Current: straight = {s['flex_flat_k']:.2f} kOhm, "
          f"fully bent = {s['flex_bent_k']:.2f} kOhm\n")

    for point, prompt in (
            ('FLAT', 'Hold the flex sensor STRAIGHT (no force)'),
            ('BENT', 'Bend the flex sensor as far as it will go in use '
                     '(maximum force) and HOLD it')):
        print(f'{BOLD}>> {prompt}, then press Enter{RESET}  (Ctrl+C to abort)')
        stop = threading.Event()

        def show():
            while not stop.is_set():
                st = client.get()
                sys.stdout.write(f"\r     {st['flex_kohm']:6.2f} kOhm  "
                                 f"(ADC {st['flex_adc']:5.1f})   ")
                sys.stdout.flush()
                time.sleep(0.1)
        t = threading.Thread(target=show, daemon=True)
        t.start()
        try:
            input()
        finally:
            stop.set()
            t.join()
        ack = client.send('cal', point)
        if ack is None or not ack.get('ok'):
            err = (ack or {}).get('err', 'no reply from Arduino')
            print(f'{RED}{point} failed: {err}{RESET}')
            return 1
        time.sleep(0.3)
        print(f'{GREEN}{point} saved{RESET}')

    s = client.get()
    print(f"\n{GREEN}Calibrated:{RESET} straight = {s['flex_flat_k']:.2f} kOhm (0 %), "
          f"fully bent = {s['flex_bent_k']:.2f} kOhm (100 % = servo 180 deg). "
          'Saved on the Arduino.')
    return 0


# ----------------------------------------------------------------------
# Characterization (measured error + transfer function data)
# ----------------------------------------------------------------------

# ref: what you measure with a ruler/protractor, meas: converted output,
# record: electrical/raw fields saved for the transfer function plot
SENSORS = {
    'sharp': dict(ref='distance', unit='cm', meas='sharp_cm',
                  record=('sharp_v',),
                  points=(10, 15, 20, 30, 40, 50, 60, 70, 80)),
    'tof': dict(ref='distance', unit='cm', meas='tof_cm',
                record=('tof_mm',),
                points=(10, 25, 50, 75, 100, 150, 200)),
    'flex': dict(ref='bend angle', unit='deg', meas='flex_angle',
                 record=('flex_adc', 'flex_kohm'),
                 points=(0, 15, 30, 45, 60, 75, 90)),
    'pot': dict(ref='shaft angle', unit='deg', meas='pot_deg',
                record=('pot_raw', 'pot_v'),
                points=(0, 45, 90, 135, 180, 225, 270)),
}

CHAR_DIR = os.path.join(os.environ.get('TSP05_REPO', '/tsp05'), 'characterization')


def collect_states(client, seconds):
    """All distinct state messages received in the next `seconds`."""
    states, last = [], None
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        s = client.get()
        if s is not last:
            states.append(s)
            last = s
        time.sleep(0.02)
    return states


def mean_std(values):
    values = [float(v) for v in values if not math.isnan(float(v))]
    if not values:
        return math.nan, math.nan
    m = sum(values) / len(values)
    return m, math.sqrt(sum((v - m) ** 2 for v in values) / len(values))


def save_rows(path, columns, rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)
    # The CLI runs as root in the container: hand the files to the repo owner
    owner = os.stat(os.path.dirname(CHAR_DIR))
    for p in (os.path.dirname(path), path):
        try:
            os.chown(p, owner.st_uid, owner.st_gid)
        except OSError:
            pass


def characterize(client, name, seconds=2.0):
    """Record converted output vs a known reference at several points."""
    cfg = SENSORS[name]
    unit = cfg['unit']
    path = os.path.join(CHAR_DIR, f'{name}.csv')
    columns = (['true', 'measured', 'measured_std', 'error', 'n'] +
               [c for k in cfg['record'] for c in (k, f'{k}_std')])

    rows = []
    if os.path.exists(path):
        with open(path) as f:
            rows = list(csv.DictReader(f))
        print(f'{YELLOW}Continuing {path} ({len(rows)} points so far, '
              f'"clear" to start over){RESET}')

    print(f"{BOLD}Characterize {name}{RESET}: set a known {cfg['ref']} "
          f'(ruler/protractor), hold still, type the true value + Enter.')
    print(f"Suggested points ({unit}): {', '.join(map(str, cfg['points']))}")
    print('Commands: undo, clear, Enter on empty line = finish\n')

    while True:
        s = client.get()
        live = s.get(cfg['meas'], math.nan)
        try:
            text = input(f'[now {fmt(float(live))} {unit}] true {cfg["ref"]} ({unit}): ').strip()
        except EOFError:
            break
        if not text:
            break
        if text == 'undo':
            if rows:
                rows.pop()
                save_rows(path, columns, rows)
            print(f'  removed last point ({len(rows)} left)')
            continue
        if text == 'clear':
            rows = []
            save_rows(path, columns, rows)
            print('  cleared')
            continue
        try:
            true = float(text)
        except ValueError:
            print(f'{RED}  not a number{RESET}')
            continue

        states = collect_states(client, seconds)
        meas, meas_std = mean_std([st[cfg['meas']] for st in states])
        row = {'true': true, 'measured': round(meas, 3),
               'measured_std': round(meas_std, 3),
               'error': round(meas - true, 3), 'n': len(states)}
        for k in cfg['record']:
            m, sd = mean_std([st[k] for st in states])
            row[k], row[f'{k}_std'] = round(m, 4), round(sd, 4)
        rows.append(row)
        save_rows(path, columns, rows)

        raw = '  '.join(f'{k} {row[k]}' for k in cfg['record'])
        print(f"  measured {fmt(meas)} {unit} (±{fmt(meas_std)})  "
              f"error {row['error']:+.1f} {unit}  | {raw}")

    errors = [float(r['error']) for r in rows if r['error'] not in ('', 'nan')]
    errors = [e for e in errors if not math.isnan(e)]
    if errors:
        mae = sum(abs(e) for e in errors) / len(errors)
        rms = math.sqrt(sum(e * e for e in errors) / len(errors))
        print(f'\n{BOLD}{name}: {len(errors)} points{RESET}  mean |error| '
              f'{mae:.2f} {unit}, RMS {rms:.2f} {unit}, '
              f'max |error| {max(abs(e) for e in errors):.2f} {unit}')
    print(f'Saved {path}  (plots + tables: scripts/tsp05 analyze)')
    return 0


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(prog='tsp05', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)
    check = sub.add_parser('check', help='guided hardware test')
    check.add_argument('--auto', action='store_true',
                       help='only the automatic checks (no prompts)')
    sub.add_parser('watch', help='live one-line view')
    sub.add_parser('calibrate', help='guided flex (force) calibration')
    p = sub.add_parser('characterize',
                       help='record measured error vs known reference values')
    p.add_argument('sensor', choices=sorted(SENSORS))
    p = sub.add_parser('dctest', help='DC motor polarity + speed test')
    p.add_argument('--forward', choices=['yes', 'no'],
                   help='answer for "did it spin forward" (else asked)')
    p = sub.add_parser('mode')
    p.add_argument('value', choices=['gui', 'sensor', 'toggle', 'GUI', 'SENSOR', 'TOGGLE'])
    for name, help_text in (('servo', 'angle deg'), ('step', 'relative deg'),
                            ('dc', 'rpm'), ('dcdeg', 'relative deg'), ('safety', 'threshold cm, 0 = off')):
        p = sub.add_parser(name)
        p.add_argument('value', type=float, help=help_text)
    args = parser.parse_args(argv)

    client, executor, thread = start_client()
    try:
        if args.cmd == 'check':
            return Check(client, interactive=not args.auto).run()

        if not wait_for_state(client, 6.0):
            print(f'{RED}No data on {PREFIX}/state_json - is the bridge running?{RESET}')
            return 1

        if args.cmd == 'calibrate':
            return calibrate(client)

        if args.cmd == 'characterize':
            return characterize(client, args.sensor)

        if args.cmd == 'dctest':
            forward = None if args.forward is None else args.forward == 'yes'
            return dctest(client, forward)

        if args.cmd == 'watch':
            print('Ctrl+C to stop')
            last = None
            while True:
                s = client.get()
                if s is not last:
                    sys.stdout.write('\r' + status_line(s) + '  ')
                    sys.stdout.flush()
                    last = s
                for event in client.take_events():
                    print(f'\n{YELLOW}{json.dumps(event)}{RESET}')
                time.sleep(0.05)

        value = args.value.upper() if args.cmd == 'mode' else args.value
        ack = client.send(args.cmd, value)
        if ack is None:
            print(f'{RED}No reply from the Arduino{RESET}')
            return 1
        if not ack.get('ok'):
            print(f"{RED}Rejected: {ack.get('err')}{RESET}")
            return 1
        print(f'{GREEN}OK{RESET}')
        return 0
    except KeyboardInterrupt:
        print()
        return 130
    finally:
        # Stop the spin thread before destroying the node
        if rclpy.ok():
            rclpy.shutdown()
        executor.shutdown()
        thread.join(timeout=2.0)
        client.destroy_node()


if __name__ == '__main__':
    sys.exit(main())
