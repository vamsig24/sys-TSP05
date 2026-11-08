#!/usr/bin/env python3
"""Generate the TSP05 Foxglove layout (foxglove/tsp05_layout.json).

Run:  python3 foxglove/make_layout.py
Then in Foxglove: Layouts (top bar) -> Import from file... -> tsp05_layout.json

Layout (like section 6 of the TSP05 design image):
  left   - mode + safety indicators, mode buttons, motor controls (GUI mode),
           force-sensor calibration (tabs)
  middle - live sensor readings and motor states (gauges)
  right  - flex raw vs filtered, force transfer function, distances, motors
"""

import json
import os

P = '/tsp05'
STRING = 'std_msgs/msg/String'
FLOAT = 'std_msgs/msg/Float32'

BLUE = '#1e88e5'
GREEN = '#43a047'
RED = '#e53935'
GREY = '#757575'
ORANGE = '#fb8c00'

config = {}


def panel(panel_id, title, **cfg):
    cfg['foxglovePanelTitle'] = title
    config[panel_id] = cfg
    return panel_id


def split(direction, first, second, pct=50):
    return {'direction': direction, 'first': first, 'second': second,
            'splitPercentage': pct}


def column(*items):
    """Stack items vertically in equal parts."""
    if len(items) == 1:
        return items[0]
    return split('column', items[0], column(*items[1:]), 100 / len(items))


def row(*items):
    """Place items side by side in equal parts."""
    if len(items) == 1:
        return items[0]
    return split('row', items[0], row(*items[1:]), 100 / len(items))


def button(panel_id, title, topic, datatype, data, text, color=None,
           editable=False):
    cfg = dict(topicName=topic, datatype=datatype, buttonText=text,
               advancedView=editable,
               value=json.dumps({'data': data}, indent=2))
    if color:
        cfg['buttonColor'] = color
    return panel(panel_id, title, **cfg)


def gauge(panel_id, title, path, lo, hi, tick, color_map='turbo'):
    return panel(panel_id, title, path=path, style='dial', minValue=lo,
                 maxValue=hi, tickInterval=tick, showTicks=True,
                 colorMode='colormap', colorMap=color_map, reverse=False,
                 reverseDirection=False)


def series(path, label, color, **extra):
    return dict(value=path, label=label, color=color, enabled=True,
                timestampMethod='receiveTime', **extra)


def plot(panel_id, title, paths, y_label, lo=None, hi=None, seconds=30):
    cfg = dict(paths=paths, xAxisVal='timestamp', timeWindowMode='sliding',
               followingViewWidth=seconds, showLegend=True,
               legendDisplay='floating', floatingLegendPosition='top-left',
               showPlotValuesInLegend=True, isSynced=True,
               yAxisLabel=y_label, showYAxisGridLines=True)
    if lo is not None:
        cfg['minYValue'] = lo
    if hi is not None:
        cfg['maxYValue'] = hi
    return panel(panel_id, title, **cfg)


# ----------------------------------------------------------------------
# Left: mode, controls, calibration
# ----------------------------------------------------------------------

mode = panel(
    'Indicator!mode', 'Mode', path=f'{P}/mode.data', style='background',
    fontSize=22, fallbackColor=GREY, fallbackLabel='no data',
    rules=[
        dict(operator='=', rawValue='GUI', color=BLUE, label='GUI MODE'),
        dict(operator='=', rawValue='SENSOR', color=GREEN, label='SENSOR MODE'),
    ])
safety = panel(
    'Indicator!safety', 'Safety', path=f'{P}/safety/active.data',
    style='background', fontSize=16, fallbackColor=GREEN,
    fallbackLabel='Clear',
    rules=[dict(operator='=', rawValue='true', color=RED,
                label='SAFETY STOP (obstacle at Sharp)')])
connected = panel(
    'Indicator!connected', 'Arduino', path=f'{P}/connected.data',
    style='bulb', fontSize=14, fallbackColor=RED,
    fallbackLabel='Arduino not connected',
    rules=[dict(operator='=', rawValue='true', color=GREEN,
                label='Arduino connected')])

mode_gui = button('Publish!mode_gui', 'Switch mode', f'{P}/cmd/mode', STRING,
                  'GUI', 'GUI mode', BLUE)
mode_sensor = button('Publish!mode_sensor', 'Switch mode', f'{P}/cmd/mode',
                     STRING, 'SENSOR', 'SENSOR mode', GREEN)

servo = button('Publish!servo', 'Servo angle (0-180 deg)',
               f'{P}/cmd/servo_angle', FLOAT, 90.0, 'Set servo angle',
               editable=True)
servo_0 = button('Publish!servo_0', 'Servo', f'{P}/cmd/servo_angle', FLOAT,
                 0.0, 'Servo 0 deg')
servo_90 = button('Publish!servo_90', 'Servo', f'{P}/cmd/servo_angle', FLOAT,
                  90.0, 'Servo 90 deg')
servo_180 = button('Publish!servo_180', 'Servo', f'{P}/cmd/servo_angle',
                   FLOAT, 180.0, 'Servo 180 deg')
step_minus = button('Publish!step_minus', 'Stepper', f'{P}/cmd/stepper_delta',
                    FLOAT, -15.0, 'Stepper -15 deg')
step_plus = button('Publish!step_plus', 'Stepper', f'{P}/cmd/stepper_delta',
                   FLOAT, 15.0, 'Stepper +15 deg')
step = button('Publish!step', 'Stepper relative move (deg)',
              f'{P}/cmd/stepper_delta', FLOAT, 90.0, 'Move stepper',
              editable=True)
dc = button('Publish!dc', 'DC speed (RPM, -100..100) - motor not connected',
            f'{P}/cmd/dc_rpm', FLOAT, 40.0, 'Set DC speed', editable=True)

controls = column(
    row(mode_gui, mode_sensor),
    servo,
    row(servo_0, servo_90, servo_180),
    row(step_minus, step_plus),
    step,
    dc,
)

cal_help = panel(
    'Markdown!cal_help', 'How to calibrate', renderMode='static',
    markdown=(
        '**Force sensor (flex) calibration**\n\n'
        '1. Hold the flex sensor **straight** and press *Straight = 0 %*.\n'
        '2. Bend it as far as it goes in use, **hold**, and press '
        '*Bent = 100 %*.\n\n'
        'Saved on the Arduino. Force 0-100 % maps to servo 0-180 deg in '
        'SENSOR mode. Check the result on the transfer-function plot.\n\n'
        'Stored points: see *Calibration* gauges.'))
cal_flat = button('Publish!cal_flat', 'Calibrate', f'{P}/cmd/flex_calibrate',
                  STRING, 'FLAT', 'Straight = 0 %', BLUE)
cal_bent = button('Publish!cal_bent', 'Calibrate', f'{P}/cmd/flex_calibrate',
                  STRING, 'BENT', 'Bent = 100 %', ORANGE)
cal_reset = button('Publish!cal_reset', 'Calibrate', f'{P}/cmd/flex_calibrate',
                   STRING, 'RESET', 'Reset to defaults', GREY)
cal_flat_value = gauge('Gauge!cal_flat', 'Calibration: straight (kOhm)',
                       f'{P}/sensors/flex/cal_flat_kohm.data', 0, 60, 10)
cal_bent_value = gauge('Gauge!cal_bent', 'Calibration: bent (kOhm)',
                       f'{P}/sensors/flex/cal_bent_kohm.data', 0, 60, 10)
calibration = column(
    cal_help,
    row(cal_flat, cal_bent),
    cal_reset,
    row(cal_flat_value, cal_bent_value),
)

events = panel('RawMessages!events', 'Arduino replies / events',
               topicPath=f'{P}/events', fontSize=12, expansion='all')

tabs = panel('Tab!left', 'Controls', activeTabIdx=0, tabs=[
    dict(title='Motor controls (GUI mode)', layout=controls),
    dict(title='Calibrate force sensor', layout=calibration),
])
config['Tab!left'].pop('foxglovePanelTitle')  # Tab uses per-tab titles

# Indicators on top (16 %), tabs, then the Arduino replies at the bottom
left = split('column',
             split('column', mode, split('column', safety, connected, 50), 45),
             split('column', tabs, events, 78), 16)

# ----------------------------------------------------------------------
# Middle: live readings
# ----------------------------------------------------------------------

force = gauge('Gauge!force', 'Force (flex) %', f'{P}/sensors/flex/force_pct.data',
              0, 100, 10, 'red-yellow-green')
pot = gauge('Gauge!pot', 'Potentiometer %', f'{P}/sensors/pot/position_pct.data',
            0, 100, 10)
tof = gauge('Gauge!tof', 'VL53L1X distance (cm)',
            f'{P}/sensors/tof/distance_cm.data', 0, 200, 20)
sharp = gauge('Gauge!sharp', 'Sharp IR distance (cm)',
              f'{P}/sensors/sharp/distance_cm.data', 10, 80, 10)
servo_angle = gauge('Gauge!servo', 'Servo angle (deg)',
                    f'{P}/motors/servo/angle_deg.data', 0, 180, 30)
stepper_angle = gauge('Gauge!stepper', 'Stepper angle (deg)',
                      f'{P}/motors/stepper/angle_deg.data', -360, 360, 90)
dc_target = gauge('Gauge!dc', 'DC target speed (RPM)',
                  f'{P}/motors/dc/target_rpm.data', -100, 100, 25)
button_count = panel('Indicator!button', 'Hardware button',
                     path=f'{P}/button/pressed.data', style='bulb',
                     fontSize=14, fallbackColor=GREY, fallbackLabel='released',
                     rules=[dict(operator='=', rawValue='true', color=ORANGE,
                                 label='PRESSED')])

middle = column(
    row(force, pot),
    row(tof, sharp),
    row(servo_angle, stepper_angle),
    row(dc_target, button_count),
)

# ----------------------------------------------------------------------
# Right: plots
# ----------------------------------------------------------------------

raw_vs_filtered = plot(
    'Plot!flex_raw', 'Force sensor: raw vs filtered ADC', [
        series(f'{P}/sensors/flex/adc_raw.data', 'Raw', '#9e9e9e'),
        series(f'{P}/sensors/flex/adc_filtered.data', 'Filtered (median + EMA)',
               BLUE, lineSize=2),
    ], 'ADC (0-1023)')

transfer = panel(
    'Plot!transfer', 'Force transfer function (ADC -> force %)',
    paths=[dict(value=f'{P}/sensors/flex/transfer.y',
                xValuePath=f'{P}/sensors/flex/transfer.x',
                label='force % vs ADC', color=ORANGE, enabled=True,
                showLine=False, timestampMethod='receiveTime')],
    xAxisVal='custom', xAxisLabel='Filtered ADC', yAxisLabel='Force %',
    minYValue=0, maxYValue=100, showLegend=True, legendDisplay='floating',
    floatingLegendPosition='top-left', showYAxisGridLines=True)

distances = plot(
    'Plot!distances', 'Distances', [
        series(f'{P}/sensors/tof/distance_cm.data', 'VL53L1X', BLUE),
        series(f'{P}/sensors/sharp/distance_cm.data', 'Sharp IR', GREEN),
        series(f'{P}/safety/threshold_cm.data', 'Safety threshold', RED,
               lineStyle='dashed'),
    ], 'cm', 0, 150)

motors = plot(
    'Plot!motors', 'Motors', [
        series(f'{P}/motors/servo/angle_deg.data', 'Servo deg', BLUE),
        series(f'{P}/motors/stepper/angle_deg.data', 'Stepper deg', GREEN),
        series(f'{P}/motors/stepper/target_deg.data', 'Stepper target deg',
               GREEN, lineStyle='dashed'),
        series(f'{P}/motors/dc/target_rpm.data', 'DC target RPM', ORANGE),
    ], 'deg / RPM')

right = column(raw_vs_filtered, transfer, distances, motors)

layout = {
    'configById': config,
    'globalVariables': {},
    'userNodes': {},
    'playbackConfig': {'speed': 1.0},
    'layout': split('row', left, split('row', middle, right, 42), 28),
}


def leaves(node):
    if isinstance(node, str):
        yield node
    else:
        yield from leaves(node['first'])
        yield from leaves(node['second'])


def check(layout):
    used = set(leaves(layout['layout']))
    for tab in config['Tab!left']['tabs']:
        used |= set(leaves(tab['layout']))
    missing = used - set(config)
    unused = set(config) - used
    assert not missing, f'panels in layout without config: {missing}'
    assert not unused, f'configured panels not placed: {unused}'


check(layout)
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tsp05_layout.json')
with open(out, 'w') as f:
    json.dump(layout, f, indent=2)
print(f'wrote {out} ({len(config)} panels)')
