import math
import unittest

from sharp_ir_test.protocol import parse_sensor_line


class TestSharpIRProtocol(unittest.TestCase):
    def test_valid_reading(self):
        result = parse_sensor_line(
            '{"type":"state","sharp_distance_cm":42,'
            '"sharp_valid":true,"sharp_status":"VALID"}')
        self.assertTrue(result['valid'])
        self.assertEqual(result['distance_cm'], 42.0)
        self.assertEqual(result['status'], 'VALID')

    def test_too_far_is_invalid(self):
        result = parse_sensor_line(
            '{"type":"state","sharp_distance_cm":null,'
            '"sharp_valid":false,"sharp_status":"TOO_FAR"}')
        self.assertFalse(result['valid'])
        self.assertIsNone(result['distance_cm'])
        self.assertEqual(result['status'], 'TOO_FAR')

    def test_out_of_range_is_rejected_even_if_declared_valid(self):
        result = parse_sensor_line(
            '{"distance_cm":95,"valid":true,"status":"VALID"}')
        self.assertFalse(result['valid'])
        self.assertIsNone(result['distance_cm'])
        self.assertEqual(result['status'], 'OUT_OF_RANGE')

    def test_non_json_is_ignored(self):
        self.assertIsNone(parse_sensor_line('Mean Distance: 42 cm'))


if __name__ == '__main__':
    unittest.main()
