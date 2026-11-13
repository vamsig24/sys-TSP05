import math
import unittest

from tsp05_bridge import protocol

STATE = (
    '{"t":"state","ms":42700,"mode":"GUI","safety":true,"safety_cm":20,'
    '"flex_raw":459,"flex_adc":469.1,"flex_kohm":11.81,"flex_deg":0,'
    '"flex_force":0.0,"flex_flat_k":13.0,"flex_bent_k":34.0,'
    '"pot_raw":1023,"pot_adc":1000.0,"pot_v":5.00,"pot_pct":100,'
    '"pot_cal_min":100,"pot_cal_max":1000,"pot_cal_flags":3,'
    '"tof_ok":true,"tof_mm":133,'
    '"sharp_v":1.92,"sharp_cm":12,"sharp_st":"VALID","btn":false,"btn_n":0,'
    '"servo_deg":90,"step_deg":0.0,"step_tgt":0.0,"dc_on":false,'
    '"dc_tgt":0.0,"dc_rpm":0.0,"dc_pwm":0,"dc_pol":0,"dc_stall":false,'
    '"sharp_raw":390,"pot_deg":270.0,"step_move":0.0,"dc_pos":0.0,'
    '"dc_pos_tgt":0.0,"dc_pos_mode":false,"dc_pos_done":false,'
    '"dc_active_rpm":0.0,"flex_angle_max":90.0}')


class TestParse(unittest.TestCase):
    def test_state(self):
        msg = protocol.parse_line(STATE)
        self.assertEqual(msg['t'], 'state')
        self.assertEqual(msg['mode'], 'GUI')
        self.assertTrue(msg['safety'])
        self.assertAlmostEqual(msg['tof_cm'], 13.3)
        self.assertTrue(msg['sharp_valid'])
        self.assertEqual(msg['sharp_dist_cm'], 12.0)

    def test_bytes(self):
        self.assertIsNotNone(protocol.parse_line(STATE.encode() + b'\r\n'))

    def test_invalid_sensors_are_nan(self):
        line = (STATE.replace('"tof_ok":true', '"tof_ok":false')
                     .replace('"sharp_st":"VALID"', '"sharp_st":"TOO_FAR"'))
        msg = protocol.parse_line(line)
        self.assertTrue(math.isnan(msg['tof_cm']))
        self.assertFalse(msg['sharp_valid'])
        self.assertTrue(math.isnan(msg['sharp_dist_cm']))

    def test_truncated_state_rejected(self):
        # Port opened mid-line: missing leading fields
        self.assertIsNone(protocol.parse_line(
            '{"t":"state","dc_tgt":0.0,"dc_rpm":0.0}'))
        self.assertIsNone(protocol.parse_line(STATE[:-20]))

    def test_ack_and_event(self):
        ack = protocol.parse_line(
            '{"t":"ack","cmd":"SERVO","ok":false,"err":"not in GUI mode"}')
        self.assertFalse(ack['ok'])
        ev = protocol.parse_line(
            '{"t":"event","event":"mode_changed","mode":"SENSOR"}')
        self.assertEqual(ev['mode'], 'SENSOR')

    def test_garbage(self):
        for line in ('', 'VL53L1X initialized', '[1,2]', '{"no_type":1}',
                     b'\xff\xfe'):
            self.assertIsNone(protocol.parse_line(line))


class TestCommands(unittest.TestCase):
    def test_mode(self):
        self.assertEqual(protocol.format_mode('sensor'), 'MODE SENSOR\n')
        self.assertEqual(protocol.format_mode(' toggle '), 'MODE TOGGLE\n')
        with self.assertRaises(ValueError):
            protocol.format_mode('auto')

    def test_servo_clamped(self):
        self.assertEqual(protocol.format_servo(90.4), 'SERVO 90\n')
        self.assertEqual(protocol.format_servo(250), 'SERVO 180\n')
        self.assertEqual(protocol.format_servo(-5), 'SERVO 0\n')

    def test_dc_position(self):
        self.assertEqual(protocol.format_dc_position(90), 'DCPOS 90.0\n')
        self.assertEqual(protocol.format_dc_position(-45.1), 'DCPOS -45.1\n')
        for bad in (math.nan, math.inf, 9999):
            with self.assertRaises(ValueError):
                protocol.format_dc_position(bad)

    def test_dc_and_step(self):
        self.assertEqual(protocol.format_dc(150), 'DC 100.0\n')
        self.assertEqual(protocol.format_step(-15), 'STEP -15.0\n')

    def test_dc_pwm_and_polarity(self):
        self.assertEqual(protocol.format_dc_pwm(90), 'DCPWM 90\n')
        self.assertEqual(protocol.format_dc_pwm(-999), 'DCPWM -230\n')
        self.assertEqual(protocol.format_dc_polarity('1 0'), 'DCPOL 1 0\n')
        with self.assertRaises(ValueError):
            protocol.format_dc_polarity('2 0')
        with self.assertRaises(ValueError):
            protocol.format_dc_polarity('1')

    def test_safety(self):
        self.assertEqual(protocol.format_safety(20), 'SAFETY 20\n')
        self.assertEqual(protocol.format_safety(-3), 'SAFETY 0\n')

    def test_calibration(self):
        self.assertEqual(protocol.format_calibration('flat'), 'CAL FLAT\n')
        self.assertEqual(protocol.format_calibration('BENT'), 'CAL BENT\n')
        self.assertEqual(protocol.format_calibration('ANGLE 90'), 'CAL ANGLE 90.0\n')
        with self.assertRaises(ValueError):
            protocol.format_calibration('middle')

    def test_pot_endpoints_and_mapping(self):
        self.assertEqual(protocol.format_pot_calibration('min'), 'POTCAL MIN\n')
        self.assertEqual(protocol.format_pot_calibration('MAX'), 'POTCAL MAX\n')
        with self.assertRaises(ValueError):
            protocol.format_pot_calibration('banana')
        state = protocol.parse_line(STATE)
        self.assertEqual(state['pot_map_pct'], 100)
        self.assertEqual(state['step_ideal_deg'], 360)
        self.assertEqual(state['servo_ideal_deg'], 0)
        text = protocol.mapping_summary(state)
        self.assertIn('both endstops measured', text)
        self.assertIn('100..1000', text)
        self.assertIn('FLEX -> SERVO', text)
        self.assertIn('POT -> STEPPER', text)
        self.assertIn('TOF -> DC', text)

    def test_pot_mapping_clamps_and_default_label(self):
        line = (STATE.replace('\"pot_adc\":1000.0', '\"pot_adc\":1100.0')
                     .replace('\"pot_cal_flags\":3', '\"pot_cal_flags\":0'))
        state = protocol.parse_line(line)
        self.assertAlmostEqual(state['pot_map_pct'], 100.0)
        self.assertIn('NOT measured', protocol.mapping_summary(state))

    def test_non_finite_rejected(self):
        for fmt in (protocol.format_servo, protocol.format_dc,
                    protocol.format_step, protocol.format_safety):
            with self.assertRaises(ValueError):
                fmt(float('nan'))


if __name__ == '__main__':
    unittest.main()
