"""Offline correctness checks. Synthetic test rows never enter calibration evidence."""
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('sensor_validation', ROOT / 'tools' / 'sensor_validation.py')
validation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(validation)


class TestValidation(unittest.TestCase):
    def test_error_metrics_are_based_on_true_measured_reference(self):
        # Artificial unit-test fixture only, not real lab data.
        rows = []
        for truth, raw, report in [(10, 500, 12), (10, 498, 10),
                                   (20, 400, 21), (20, 402, 21)]:
            rows.append(dict(sensor='sharp', truth=truth, raw=raw, reported=report))
        data = validation.summarize(rows)['sharp']
        self.assertEqual(data['count'], 2)
        self.assertAlmostEqual(data['mae'], 1.0)
        self.assertAlmostEqual(data['max_abs'], 1.0)
        self.assertEqual(data['levels'][0]['raw'], 499)
        self.assertEqual(data['levels'][0]['observed'], 11)
        self.assertEqual(data['levels'][1]['truth'], 20)
        self.assertAlmostEqual(data['levels'][1]['raw_std'], 2**0.5)

    def test_plot_from_csv_temp_location_only(self):
        try:
            import matplotlib
        except ImportError:
            self.skipTest('python3-matplotlib not installed')
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / 'measurements.csv'
            with path.open('w', newline='') as fh:
                w = csv.DictWriter(fh, fieldnames=validation.COLUMNS)
                w.writeheader()
                for truth, adc in [(10, 600), (20, 430), (40, 240), (80, 110)]:
                    for i in range(3):
                        w.writerow(dict(sensor='sharp', truth=truth, raw=adc + i,
                                        filtered='', reported=truth + 1,
                                        arduino_ms=i, utc='unit-test'))
            validation.plot(type('Args', (), {'csv': path})())
            self.assertTrue((path.parent / 'sharp_transfer_function.png').exists())
            self.assertTrue((path.parent / 'conversion_errors.png').exists())
            report = (path.parent / 'error_report.md').read_text()
            self.assertIn('sharp', report)
            self.assertIn('4', report)

    def test_foxglove_layout_contract(self):
        layout = json.loads((ROOT / 'foxglove' / 'tsp05_layout.json').read_text())
        transfer = layout['configById']['Plot!transfer']
        self.assertEqual(transfer['xAxisVal'], 'custom')
        self.assertIn('/sensors/sharp/adc_vs_reported_cm.points[:].x',
                      transfer['paths'][0]['xValuePath'])
        self.assertIn('/sensors/sharp/adc_vs_reported_cm.points[:].y',
                      transfer['paths'][0]['value'])
        self.assertIn('NOT ruler', transfer['foxglovePanelTitle'])
        self.assertIn('ADC', transfer['yAxisLabel'])
        self.assertIn('cm', transfer['xAxisLabel'])
        self.assertIn('dc_position_delta',
                      layout['configById']['Publish!dc_pos']['topicName'])
        self.assertTrue(layout['configById']['Publish!dc_pos']['advancedView'])
        self.assertNotIn('Publish!dc_pos_plus', layout['configById'])
        self.assertNotIn('Gauge!dc_position_target', layout['configById'])

    def test_ros_sharp_points_match_user_reported_calculated_distances(self):
        path = ROOT / 'calibration' / 'sharp_ros_reported_11_points.csv'
        with path.open(newline='') as f:
            rows = [(int(r['reported_distance_cm']), int(r['raw_adc']))
                    for r in csv.DictReader(f)]
        self.assertEqual(rows, [(15,342),(20,255),(25,225),(30,180),
                                (35,171),(40,156),(45,140),(50,125),
                                (55,112),(60,105),(65,98)])
        src = (ROOT / 'ros2_ws/src/tsp05_bridge/tsp05_bridge/serial_node.py').read_text()
        for dist, raw in rows:
            self.assertIn(f'({float(dist)}, {float(raw)})', src)
        self.assertIn('adc_vs_reported_cm', src)

    def test_sensor_mode_has_independent_1_to_1_mappings(self):
        src = (ROOT / 'sys-TSP05-fixed.ino').read_text()
        body = src.split('void runSensorMode() {', 1)[1].split('// ====================================\n// Serial commands', 1)[0]
        self.assertIn('flexSensor.getForcePercent()', body)
        self.assertIn('writeServo(servoRequest)', body)
        self.assertIn('potentiometer.getFilteredADC()', body)
        self.assertIn('stepperMotor.setTargetDegrees(', body)
        self.assertIn('distanceToRPM()', body)
        self.assertIn('dcTargetRPM = rpm', body)

    def test_layout_uses_calibrated_percent_ranges_and_status(self):
        config = json.loads((ROOT / 'foxglove/tsp05_layout.json').read_text())['configById']
        self.assertEqual(config['Gauge!flex_angle']['maxValue'], 100)
        self.assertEqual(config['Gauge!pot_angle']['maxValue'], 100)
        self.assertEqual(config['Gauge!step_target']['maxValue'], 360)
        self.assertEqual(config['RawMessages!mapping']['topicPath'],
                         '/tsp05/mapping/status')
        self.assertIn('servo/angle_deg', config['Gauge!servo']['path'])
        self.assertNotIn('Publish!dc_pos_plus', config)

    def test_pot_calibration_and_flex_hysteresis_in_firmware(self):
        src = (ROOT / 'sys-TSP05-fixed.ino').read_text()
        self.assertIn('handlePotCalibration(arg)', src)
        self.assertIn('loadPotCalibration()', src)
        self.assertIn('potCalFlags', src)
        self.assertIn('servoFollow.command(', src)
        self.assertIn('TSP05Mapping::potToStepper(', src)
        self.assertIn('pot_cal_min', src)
        self.assertIn('pot_cal_max', src)


if __name__ == '__main__':
    unittest.main()
