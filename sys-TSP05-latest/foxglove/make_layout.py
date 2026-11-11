#!/usr/bin/env python3
"""Generate the TSP05 Foxglove layout (foxglove/tsp05_layout.json).

Run:  python3 foxglove/make_layout.py
Then in Foxglove: Layouts (top bar) -> Import from file... -> tsp05_layout.json

Layout (like section 6 of the TSP05 design image):
  left   - mode/safety, minimal editable Publish controls and command replies
  middle - one reading per sensor and essential motor feedback
  right  - noise filtering evidence, user-supplied ROS Sharp curve, DC PID plots
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
# Minimal rubric-oriented layout: only motor controls and required feedback.
# ----------------------------------------------------------------------
mode = panel('Indicator!mode', 'Control mode', path=f'{P}/mode.data',
             style='background', fontSize=17, fallbackColor=GREY,
             fallbackLabel='Not connected', rules=[
                 dict(operator='=', rawValue='GUI', color=BLUE, label='GUI'),
                 dict(operator='=', rawValue='SENSOR', color=GREEN, label='SENSOR')])
safety = panel('Indicator!safety', 'Sharp safety', path=f'{P}/safety/active.data',
               style='bulb', fallbackColor=GREEN, fallbackLabel='Clear',
               rules=[dict(operator='=', rawValue='true', color=RED,
                           label='SAFETY STOP')])
connected = panel('Indicator!connected', 'Arduino connected',
                  path=f'{P}/connected.data', style='bulb',
                  fallbackColor=RED, fallbackLabel='Disconnected',
                  rules=[dict(operator='=', rawValue='true', color=GREEN,
                              label='Connected')])

mode_gui = button('Publish!mode_gui', 'Mode', f'{P}/cmd/mode', STRING,
                  'GUI', 'GUI mode', BLUE)
mode_sensor = button('Publish!mode_sensor', 'Mode', f'{P}/cmd/mode', STRING,
                     'SENSOR', 'SENSOR mode', GREEN)
# All four motor commands use editable Publish panels; the user types the
# requested value in the std_msgs/Float32 data field and presses Publish.
servo = button('Publish!servo', 'Servo angle: enter 0..180 deg',
               f'{P}/cmd/servo_angle', FLOAT, 90.0, 'Publish servo angle',
               editable=True)
step = button('Publish!step', 'Stepper: enter relative degrees (+/-)',
              f'{P}/cmd/stepper_delta', FLOAT, 45.0,
              'Publish stepper delta', editable=True)
dc = button('Publish!dc', 'DC velocity: enter signed RPM',
            f'{P}/cmd/dc_rpm', FLOAT, 20.0,
            'Publish DC RPM', editable=True)
dc_pos = button('Publish!dc_pos', 'DC position: enter relative degrees (+/-)',
                f'{P}/cmd/dc_position_delta', FLOAT, 90.0,
                'Publish relative angle', editable=True)
dc_stop = button('Publish!dc_stop', 'DC stop', f'{P}/cmd/dc_rpm', FLOAT,
                 0.0, 'STOP DC', RED)
replies = panel('RawMessages!events', 'Arduino acknowledgments / button / safety',
                topicPath=f'{P}/events', fontSize=12, expansion='all')

left = column(
    row(mode, safety, connected),
    row(mode_gui, mode_sensor),
    servo, step, dc, dc_pos, dc_stop,
    replies,
)

# Sensor values and motor targets use exactly the endpoint ranges shown
# in the live mapping-summary text panel. Pot endstops are measured via POTCAL.
flex_angle = gauge('Gauge!flex_angle', 'Calibrated flex bend (0-100%)',
                   f'{P}/sensors/flex/force_pct.data', 0, 100, 20)
pot_angle = gauge('Gauge!pot_angle', 'Pot in calibrated endstops (0-100%)',
                  f'{P}/sensors/pot/mapped_pct.data', 0, 100, 20)
tof = gauge('Gauge!tof', 'ToF distance (cm)',
            f'{P}/sensors/tof/distance_cm.data', 0, 200, 20)
sharp = gauge('Gauge!sharp', 'Sharp filtered distance (cm)',
              f'{P}/sensors/sharp/distance_cm.data', 10, 80, 10)
servo_angle = gauge('Gauge!servo', 'Servo commanded angle [0-180 deg]',
                    f'{P}/motors/servo/angle_deg.data', 0, 180, 30)
stepper_target = gauge('Gauge!step_target', 'Stepper mapped target [0-360 deg]',
                       f'{P}/mapping/pot_stepper_ideal_deg.data', 0, 360, 60)
stepper_angle = gauge('Gauge!stepper', 'Stepper reported angle (deg)',
                      f'{P}/motors/stepper/angle_deg.data', -360, 360, 90)
dc_measured = gauge('Gauge!dc_rpm', 'DC encoder RPM',
                    f'{P}/motors/dc/rpm.data', -60, 60, 10)
dc_position = gauge('Gauge!dc_position', 'DC encoder position (deg)',
                    f'{P}/motors/dc/position_deg.data', -360, 360, 90)
button_count = panel('Indicator!button', 'Debounced hardware switch',
                     path=f'{P}/button/pressed.data', style='bulb',
                     fallbackColor=GREY, fallbackLabel='Released',
                     rules=[dict(operator='=', rawValue='true', color=ORANGE,
                                 label='Pressed')])

middle = column(
    row(flex_angle, pot_angle),
    row(servo_angle, stepper_target),
    row(tof, sharp),
    row(stepper_angle, dc_measured),
    row(dc_position, button_count),
)

# Raw/filtered evidence for the noise reduction rubric.
raw_vs_filtered = plot(
    'Plot!flex_raw', 'Flex: raw vs filtered ADC', [
        series(f'{P}/sensors/flex/adc_raw.data', 'Raw', GREY),
        series(f'{P}/sensors/flex/adc_filtered.data', 'Filtered', BLUE,
               lineSize=2),
    ], 'ADC counts')

# This is deliberately NOT titled an empirical transfer function: the
# distance values were copied from /sensors/sharp/distance_cm, not a ruler.
transfer = panel(
    'Plot!transfer', 'Sharp: ROS-calculated distance vs ADC (NOT ruler data)',
    paths=[dict(value=f'{P}/sensors/sharp/adc_vs_reported_cm.points[:].y',
                xValuePath=f'{P}/sensors/sharp/adc_vs_reported_cm.points[:].x',
                label='11 supplied calculated-distance/ADC pairs',
                color=ORANGE, enabled=True, showLine=True,
                timestampMethod='receiveTime')],
    xAxisVal='custom', timeRange='latest',
    xAxisLabel='ROS-calculated distance (cm) – NOT independent measurement',
    yAxisLabel='Raw Sharp ADC counts (0..1023)',
    minXValue=10, maxXValue=70, minYValue=0, maxYValue=400,
    showLegend=True, legendDisplay='floating',
    floatingLegendPosition='top-left', showYAxisGridLines=True)

dc_speed = plot('Plot!dc_speed', 'DC PID velocity: target vs measured', [
    series(f'{P}/motors/dc/target_rpm.data', 'Target RPM', ORANGE,
           lineStyle='dashed'),
    series(f'{P}/motors/dc/rpm.data', 'Encoder RPM', RED, lineSize=2),
], 'RPM', -60, 60)

dc_position_plot = plot(
    'Plot!dc_position', 'DC PID position: encoder vs target', [
        series(f'{P}/motors/dc/position_deg.data', 'Encoder (deg)', BLUE),
        series(f'{P}/motors/dc/position_target_deg.data', 'Target (deg)',
               ORANGE, lineStyle='dashed'),
    ], 'degrees')

# Single plain-text live panel displays the calibrated min/max and exact
# 1-to-1 sensor -> motor mappings. No redundant editable position controls.
mapping_summary = panel(
    'RawMessages!mapping', 'Sensor ranges and mappings (LIVE)',
    topicPath=f'{P}/mapping/status', fontSize=12, expansion='all')
right = split('column', mapping_summary,
              column(raw_vs_filtered, transfer, dc_speed, dc_position_plot), 32)

layout = {
    'configById': config,
    'globalVariables': {},
    'userNodes': {},
    'playbackConfig': {'speed': 1.0},
    'layout': split('row', left, split('row', middle, right, 42), 29),
}


def leaves(node):
    if isinstance(node, str):
        yield node
    else:
        yield from leaves(node['first'])
        yield from leaves(node['second'])


used = set(leaves(layout['layout']))
assert used == set(config), f'panels missing/unused: {used ^ set(config)}'
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       'tsp05_layout.json'), 'w') as f:
    json.dump(layout, f, indent=2)
print(f'Wrote minimal Foxglove layout ({len(config)} panels)')
