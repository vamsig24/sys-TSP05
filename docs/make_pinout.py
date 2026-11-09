#!/usr/bin/env python3
"""Generate docs/pinout.svg: Arduino Uno wiring for TSP05.

Run:  python3 docs/make_pinout.py
Pins must match sys-TSP05.ino.
"""

import os

W, H = 1100, 760

# Board and headers (Uno R3, USB on the left, as seen from above)
BX, BY, BW, BH = 330, 280, 440, 230
TOP_Y = BY + 18
BOT_Y = BY + BH - 18

TOP_PINS = ['SCL', 'SDA', 'AREF', 'GND', '13', '12', '11', '10', '9', '8',
            None, '7', '6', '5', '4', '3', '2', '1', '0']
BOT_PINS = ['NC', 'IOREF', 'RESET', '3.3V', '5V', 'GND', 'GND', 'VIN',
            None, 'A0', 'A1', 'A2', 'A3', 'A4', 'A5']


def header_x(pins, start):
    xs, x = {}, start
    for p in pins:
        if p is None:
            x += 14
            continue
        xs[p if p not in xs else p + '_2'] = x
        x += 20
    return xs


TOP_X = header_x(TOP_PINS, BX + 46)
BOT_X = header_x(BOT_PINS, BX + 90)

STEPPER = '#8e24aa'
SERVO = '#1e88e5'
DC = '#d81b60'
BUTTON = '#fb8c00'
FLEX = '#e53935'
SHARP = '#2e7d32'
POT = '#6d4c41'
TOF = '#00897b'

# pin -> colour of the device using it (for the pin dots)
USED = {
    ('top', '13'): STEPPER, ('top', '12'): STEPPER, ('top', '10'): SERVO,
    ('top', '8'): DC, ('top', '7'): DC, ('top', '6'): DC, ('top', '4'): DC,
    ('top', '3'): DC,
    ('top', '2'): BUTTON,
    ('bot', 'A0'): FLEX, ('bot', 'A1'): SHARP, ('bot', 'A2'): POT,
    ('bot', 'A4'): TOF, ('bot', 'A5'): TOF,
    ('bot', '5V'): '#d32f2f', ('bot', 'GND'): '#212121',
    ('bot', 'GND_2'): '#212121', ('top', 'GND'): '#212121',
    ('top', '1'): '#5c6bc0', ('top', '0'): '#5c6bc0',
}

out = []


def add(s):
    out.append(s)


def text(x, y, s, size=13, color='#212121', weight='normal', anchor='start',
         rotate=None, italic=False):
    tr = f' transform="rotate({rotate} {x} {y})"' if rotate is not None else ''
    style = ' font-style="italic"' if italic else ''
    s = s.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')
    add(f'<text x="{x}" y="{y}" font-size="{size}" fill="{color}" '
        f'font-weight="{weight}" text-anchor="{anchor}"{style}{tr}>{s}</text>')


def wire(points, color, dashed=False):
    pts = ' '.join(f'{x},{y}' for x, y in points)
    dash = ' stroke-dasharray="7 5"' if dashed else ''
    add(f'<polyline points="{pts}" fill="none" stroke="{color}" '
        f'stroke-width="3" stroke-linejoin="round"{dash}/>')


def box(x, y, w, h, color, title, lines, dashed=False):
    dash = ' stroke-dasharray="7 5"' if dashed else ''
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" '
        f'fill="#ffffff" stroke="{color}" stroke-width="2.5"{dash}/>')
    add(f'<rect x="{x}" y="{y}" width="{w}" height="24" rx="8" fill="{color}"/>')
    add(f'<rect x="{x}" y="{y + 14}" width="{w}" height="10" fill="{color}"/>')
    text(x + 10, y + 17, title, 13, '#ffffff', 'bold')
    for i, line in enumerate(lines):
        text(x + 10, y + 42 + i * 17, line, 12)


def to_left(pin_x, pin_y, lane_y, box_x):
    return [(pin_x, pin_y), (pin_x, lane_y), (box_x, lane_y)]


# ----------------------------------------------------------------------
add(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
    f'viewBox="0 0 {W} {H}" font-family="Helvetica, Arial, sans-serif">')
add(f'<rect width="{W}" height="{H}" fill="#ffffff"/>')
text(W / 2, 32, 'TSP05 - Arduino Uno wiring', 22, '#212121', 'bold', 'middle')
text(W / 2, 54, 'pins as used in sys-TSP05.ino',
     13, '#616161', anchor='middle')

# ---- wires (drawn first, under boxes and board) ----
# Left top: stepper (D13, D12), servo (D10)
wire(to_left(TOP_X['13'], TOP_Y, 222, 300), STEPPER)
wire(to_left(TOP_X['12'], TOP_Y, 246, 300), STEPPER)
wire(to_left(TOP_X['10'], TOP_Y, 118, 300), SERVO)

# Right top: DC driver (D8, D7, D6), encoder (D4 = B, D3 = A), button (D2)
wire([(TOP_X['8'], TOP_Y), (TOP_X['8'], 96), (800, 96)], DC)
wire([(TOP_X['7'], TOP_Y), (TOP_X['7'], 116), (800, 116)], DC)
wire([(TOP_X['6'], TOP_Y), (TOP_X['6'], 136), (800, 136)], DC)
wire([(TOP_X['4'], TOP_Y), (TOP_X['4'], 194), (800, 194)], DC)
wire([(TOP_X['3'], TOP_Y), (TOP_X['3'], 212), (800, 212)], DC)
wire([(TOP_X['2'], TOP_Y), (TOP_X['2'], 252), (800, 252)], BUTTON)

# Left bottom: flex (A0), Sharp (A1), pot (A2)
wire(to_left(BOT_X['A0'], BOT_Y, 562, 300), FLEX)
wire(to_left(BOT_X['A1'], BOT_Y, 640, 300), SHARP)
wire(to_left(BOT_X['A2'], BOT_Y, 712, 300), POT)

# Right bottom: VL53L1X (A5 = SCL shallower, A4 = SDA deeper)
wire([(BOT_X['A5'], BOT_Y), (BOT_X['A5'], 566), (800, 566)], TOF)
wire([(BOT_X['A4'], BOT_Y), (BOT_X['A4'], 588), (800, 588)], TOF)

# ---- board ----
add(f'<rect x="{BX}" y="{BY}" width="{BW}" height="{BH}" rx="14" '
    f'fill="#00838f" stroke="#005f66" stroke-width="3"/>')
add(f'<rect x="{BX - 22}" y="{BY + 34}" width="46" height="56" rx="4" '
    f'fill="#b0bec5" stroke="#78909c" stroke-width="2"/>')
text(BX - 16, BY + 66, 'USB', 11, '#37474f', 'bold')
add(f'<rect x="{BX - 14}" y="{BY + 150}" width="38" height="46" rx="4" '
    f'fill="#424242"/>')
text(BX - 10, BY + 177, 'DC in', 10, '#eeeeee')
text(BX + BW / 2 + 20, BY + BH / 2 - 4, 'ARDUINO UNO', 24, '#ffffff', 'bold',
     'middle')
text(BX + BW / 2 + 20, BY + BH / 2 + 18, 'USB serial to ROS 2 (D0/D1)', 12,
     '#e0f7fa', anchor='middle')


def header(xs, y, side, label_dir):
    first, last = min(xs.values()), max(xs.values())
    add(f'<rect x="{first - 10}" y="{y - 10}" width="{last - first + 20}" '
        f'height="20" fill="#263238" rx="2"/>')
    for name, x in xs.items():
        color = USED.get((side, name))
        fill = color if color else '#90a4ae'
        add(f'<circle cx="{x}" cy="{y}" r="6" fill="{fill}" '
            f'stroke="#ffffff" stroke-width="1.5"/>')
        label = name.replace('_2', '')
        if side == 'top' and label.isdigit():
            label = 'D' + label
        weight = 'bold' if color else 'normal'
        lcolor = '#ffffff' if color else '#b2dfdb'
        if label_dir == 'down':
            text(x + 4, y + 16, label, 11, lcolor, weight, 'end', rotate=-90)
        else:
            text(x + 4, y - 16, label, 11, lcolor, weight, 'start', rotate=-90)


header(TOP_X, TOP_Y, 'top', 'down')
header(BOT_X, BOT_Y, 'bot', 'up')

# ---- device boxes ----
box(40, 70, 260, 96, SERVO, 'Servo (HS-311)', [
    'signal  <- D10',
    'V+  -> 5V (external supply if it',
    '        browns out),  GND -> GND'])
box(40, 182, 260, 90, STEPPER, 'Stepper driver (EasyDriver)', [
    'DIR   <- D13',
    'STEP  <- D12',
    'GND -> GND, motor power separate'])

box(800, 70, 260, 96, DC, 'DC motor driver (L298)', [
    'IN1 <- D8,  IN2 <- D7,  ENA <- D6 (PWM)',
    'motor power from its own supply,',
    'GND common with the Arduino'])
box(800, 176, 260, 52, DC, 'DC motor encoder', [
    'A -> D3 (interrupt),  B -> D4'])
box(800, 236, 260, 70, BUTTON, 'Push button (mode toggle)', [
    'D2 <- button <- 5V',
    'D2 -- 10k pull-down -- GND'])

box(40, 524, 260, 64, FLEX, 'Flex sensor (force sensor)', [
    '5V -- flex -- A0 -- 10k -- GND'])
box(40, 600, 260, 74, SHARP, 'Sharp GP2Y0A21 IR (safety)', [
    'Vo -> A1,  Vcc -> 5V,  GND -> GND',
    '10 uF across Vcc/GND recommended'])
box(40, 686, 260, 64, POT, 'Potentiometer', [
    'wiper -> A2,  ends -> 5V and GND'])

box(800, 536, 260, 84, TOF, 'VL53L1X ToF (I2C)', [
    'SCL -> A5,   SDA -> A4',
    'VIN -> supply per breakout (5V/3.3V)',
    'GND -> GND'])
box(800, 636, 260, 112, '#546e7a', 'Notes', [
    'All grounds common (Arduino, servo,',
    'driver and sensor supplies).',
    'Free: D5, D9, D11, A3',
    '(D9/D10 PWM off: Servo uses Timer1)'])

add('</svg>')

path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pinout.svg')
with open(path, 'w') as f:
    f.write('\n'.join(out) + '\n')
print(f'wrote {path}')
