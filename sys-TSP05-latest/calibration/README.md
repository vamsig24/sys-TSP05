# Real calibration evidence only

`measurements.csv`, `error_report.md`, and `sharp_transfer_function.png` are **not included** until you collect physical data using `tools/sensor_validation.py`. Never submit invented measurements. The JSON Foxglove Sharp XY panel is populated by the bridge from `measurements.csv` after you capture data.

For Sharp, put a flat white card at physically measured 10, 15, 20, 30, 40, 60, and 80 cm; run `python3 tools/sensor_validation.py record sharp N --seconds 3` at each point. Plot using `python3 tools/sensor_validation.py plot`.

Also record ToF at 10/20/40/80/120 cm, potentiometer shaft with a protractor at 0/45/90/135/180/225/270 degrees (adapt for actual mechanical travel), flex bending angle with a protractor at 0/30/60/90/120/150/180 degrees if safely achievable. The configured flex endpoint interpolation (now scaled to physically measured maximum bend angle using `CAL ANGLE`) is still a first approximation, and the potentiometer 270-degree sweep is not guaranteed for every part. Use the recorded error report to document accuracy and adjust calibration constants if needed.

The Sharp analog voltage/counts vs **physical distance** is a rubric-valid transfer-function plot; ToF already reports distance digitally and must not be used for that item.
