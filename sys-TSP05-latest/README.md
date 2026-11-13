> **Focused Nov 11 fixes:** DC relative-position anti-hunt latch, Sharp spike-filtered distance, one-to-one SENSOR-mode endpoint mappings, and lean Foxglove. See [FIXES_2026-11-11.md](FIXES_2026-11-11.md). The 11 Sharp distance/ADC pairs are **ROS-calculated values, not ruler measurements**.

# TSP05 Sensors & Motor Control — Arduino Uno + native ROS 2 Humble + Foxglove

**This repo is an updated implementation based on the original `sys-TSP05.zip`, targeting the four sensors and three motors already documented in that ZIP.** All Arduino device work runs in a **single sketch on an Arduino Uno**; the ROS 2 nodes run on a laptop using **native Ubuntu 22.04 / ROS 2 Humble**, NOT Docker. The assignment is the uploaded *Sensors & Motor Control_Guidelines_2026.pdf* (specification on pages 2–3).

> **Status:** Python protocol tests and host C++ syntax checks can run without hardware. The Arduino AVR compilation, live motor control, sensor accuracy, Foxglove version-specific GUI behavior, and final grading cannot be certified without physically flashing and measuring the device. Sensor measurements are **not invented** in this repository. The Sharp measured transfer plot stays empty until you record ruler-measured data. A six-person team must supply a fifth distinct sensor: this kit configuration has only four, and the button does not count.

## Quick start, WITHOUT Docker

See [NATIVE_HUMBLE.md](NATIVE_HUMBLE.md) for first-time host installation, port permissions, Arduino IDE/CLI, wiring and troubleshooting. See [RUBRIC_CHECKLIST.md](RUBRIC_CHECKLIST.md) for the **one-by-one demonstration and evidence checklist**.

On a native Ubuntu 22.04 laptop with ROS 2 Humble, Arduino CLI, ROS packages, and permissions already installed:

```bash
cd /path/to/sys-TSP05
export TSP05_PORT=/dev/ttyACM0       # or your stable /dev/serial/by-id/... path
scripts/tsp05 unit                     # protocol tests, NO hardware
./tools/firmware_syntax_check.sh      # mock-header HOST syntax only
scripts/tsp05 flash                    # REAL Arduino Uno build and USB upload
scripts/tsp05 build                    # native colcon build
scripts/tsp05 foxglove                 # native ROS2 bridge and GUI connection instructions
scripts/tsp05 check                    # guided live hardware tests
```

In Foxglove desktop, connect to **`ws://localhost:8765`** as a *Foxglove WebSocket*, then **Layouts → Import from file → `foxglove/tsp05_layout.json`**. Do not leave Arduino Serial Monitor open; it and the bridge cannot access the port at once.

## Hardware: the four sensors already in the ZIP

| Distinct sensor | Arduino connection | Physical-unit output | Motor in SENSOR mode |
|---|---|---|---|
| Flex bend sensor | A0, divider `5V – flex – A0 – 10kΩ – GND` | **Approximate bend angle (degrees)**, 0–measured max angle (1–180°) configured by user-stored straight/bent resistance endpoints and physical maximum bend angle; verify with protractor | HS-311 servo 0–180° |
| Potentiometer | A2 wiper, outer pins 5V/GND | **Approximate shaft angle (degrees)**, currently assumes 270° mechanical travel; measure YOUR potentiometer | EasyDriver stepper absolute target 0–360° |
| VL53L1X ToF | A4 SDA / A5 SCL; correct breakout supply | **Distance (centimeters)**; built-in digital physical-unit sensor | Encoder DC motor speed target: 10–100 cm maps 0 / 20–50 RPM |
| Sharp GP2Y0A21 analog IR | A1 Vo; sensor 5V/GND | **Distance (centimeters)**, 10–80 cm nominal; electrical raw ADC and voltage also available | Independent safety interlock for all motors (trip near 20 cm) |
| Pushbutton (**does not count** towards the four sensors) | D2; button to 5V; external 10kΩ pull-down to GND | Debounced press event/count | GUI ↔ SENSOR mode |

**Do not call the flex sensor a calibrated force sensor (newtons).** Its normalization percentage is a bent/straight fraction, not a calibrated mechanical force. The older `/tsp05/sensors/flex/force_pct` topic retains its historical name for compatibility; use `/tsp05/sensors/flex/bend_deg` for the rubric's physical-unit display and explain the sensor as a *bend sensor*. Potentiometer angle and flex bend angle are only approximations until independently measured; the attached validation workflow quantifies their errors.

### Motor pins (Arduino Uno only)

| Pin | Part | Pin | Part |
|---|---|---|---|
| D3 | DC encoder A (interrupt) | D4 | DC encoder B |
| D6 | L298 ENA PWM | D7/D8 | L298 IN2/IN1 |
| D10 | Servo signal | D12/D13 | EasyDriver STEP/DIR |
| D2 | Button | D0/D1 | USB serial (reserved) |

The annotated diagram is [`docs/pinout.svg`](docs/pinout.svg); regenerate with `python3 docs/make_pinout.py`. The servo on D10 is fine; however, Arduino **Servo library disables hardware PWM on D9 and D10**, so DC PWM stays on D6.

**CRITICAL power:** externally power **all three motors** from their suitable motor supplies; **servo needs an external regulated 5–6 V supply**, not Uno 5 V. DC and stepper motors must NOT be powered by the Uno 5 V rail. Connect **every motor-supply ground to Arduino GND**, configure EasyDriver current limiting BEFORE applying power, and never rewire under power. The correct EasyDriver motor-coil pairing must be checked electrically, not assumed solely from datasheet color labels. The DC Hall encoder may need pull-up resistors to produce clean pulses.

## Controls

The default boot mode is **GUI** (USB/Foxglove commands). SENSOR mode connects each of three different sensors to its own motor; a debounced physical button or the GUI buttons switch modes. Sharp safety interrupts movements and rejects GUI motor commands while active (Sharp usable range is 10–80 cm; test 10–20 cm in practice).

| Action | Native CLI command | ROS 2 publish topic and type |
|---|---|---|
| GUI mode | `scripts/tsp05 mode gui` | `/tsp05/cmd/mode` String (`GUI`) |
| SENSOR mode | `scripts/tsp05 mode sensor` | `/tsp05/cmd/mode` String (`SENSOR`) |
| Servo absolute position | `scripts/tsp05 servo 90` | `/tsp05/cmd/servo_angle` Float32 (degrees) |
| Stepper relative position | `scripts/tsp05 step -45` | `/tsp05/cmd/stepper_delta` Float32 (degrees, **±**) |
| DC signed RPM | `scripts/tsp05 dc -30` | `/tsp05/cmd/dc_rpm` Float32 (RPM, **±**) |
| **DC encoder relative position** | `scripts/tsp05 dcpos 90` | **`/tsp05/cmd/dc_position_delta` Float32** (degrees, **±**) |
| Stop DC | `scripts/tsp05 dc 0` | `/tsp05/cmd/dc_rpm` Float32 (0) |
| Flex endpoints | `scripts/tsp05 calibrate` | `/tsp05/cmd/flex_calibrate` String (`FLAT` / `BENT`) |
| Safety trip distance | `scripts/tsp05 safety 20` | `/tsp05/cmd/safety_threshold` Float32 (cm) |

For raw ROS command testing (after sourcing `ros2_ws/install/setup.bash`):

```bash
ros2 topic pub --once /tsp05/cmd/dc_position_delta std_msgs/msg/Float32 '{data: 90.0}'
ros2 topic echo /tsp05/motors/dc/position_deg
ros2 topic echo /tsp05/motors/dc/position_reached
ros2 topic echo /tsp05/motors/stepper/last_actual_delta_deg
ros2 topic echo /tsp05/events
```

DC motor control has a **signed encoder-based speed PID loop** (P+I+D with derivative on encoder measurement, gains in `setup()`) and a **position proportional-derivative outer loop** that requests signed RPM until the encoder position is within tolerance. Check actual output-shaft pulses/revolution and tune PI/PID constants on YOUR unit, which cannot be confirmed from code alone. A command `dcpos 90` is relative to **current measured encoder position**; a subsequent `dcpos -90` requests the reverse movement. Resetting the Uno zeros incremental position. When switching to SENSOR mode, a previous GUI DC position goal is canceled.

The stepper reports commanded **quantized** position `/tsp05/motors/stepper/target_deg`, estimated position `/tsp05/motors/stepper/angle_deg`, and `/tsp05/motors/stepper/last_actual_delta_deg`. Stepper **does not have physical encoder feedback**: mark and photograph the shaft to verify it did not skip steps. Microsteps/revolution is set to 1600 (200 full steps × 8 microsteps assumed); check EasyDriver MS1/MS2 and your actual motor specification.

## Real measured transfer function and measured conversion error

The assignment specifically wants **physical quantity X, raw electrical signal Y, based on your own physical measurements** (page 2, item 2). The original flex chart had these axes reversed and was merely plotting computed live values, not recorded independently measured truth. This update uses the **Sharp GP2Y0A21 (analog) sensor**:

- **X:** ruler-measured distance from sensor face to target, in **cm**.
- **Y:** median **10-bit raw electrical ADC reading**, 0–1023, sent as `sharp_raw`.
- Each plotted point is the **mean of actual live samples at that physical reference**; error bars are the standard deviation of the electrical readings. No fake calibration points are included.

After starting the ROS bridge and verifying the sensor is within its valid 10–80 cm range, hold a matte target at 10 cm and run:

```bash
scripts/tsp05 measure sharp 10 --seconds 3
scripts/tsp05 measure sharp 15 --seconds 3
scripts/tsp05 measure sharp 20 --seconds 3
scripts/tsp05 measure sharp 30 --seconds 3
scripts/tsp05 measure sharp 40 --seconds 3
scripts/tsp05 measure sharp 60 --seconds 3
scripts/tsp05 measure sharp 80 --seconds 3
```

Then record **independent** measured truth values for the other three sensors (example angles/distances only; do not type an angle that you have not actually measured):

```bash
scripts/tsp05 measure tof 40 --seconds 3
scripts/tsp05 measure pot 90 --seconds 3
scripts/tsp05 measure flex 45 --seconds 3
```

Use at least **five physical reference settings for each sensor across its feasible range** (see `calibration/README.md`). After collecting:

```bash
scripts/tsp05 plot
```

Creates **`calibration/sharp_transfer_function.png`** (a printable grading-valid plot), **`calibration/error_report.md`** (per-sensor MAE/RMSE/max error and per-reference errors), and **`calibration/conversion_errors.png`**. The calibration CSV becomes `calibration/measurements.csv` (real data). The native ROS bridge also reloads the CSV into `/tsp05/calibration/sharp_transfer` (`geometry_msgs/Polygon`, points `x=true_cm, y=raw_adc`) for the right-side **MEASURED Sharp IR** Foxglove XY panel. If the panel is empty, first acquire data and confirm its topic with `ros2 topic echo /tsp05/calibration/sharp_transfer --once`.

The live raw-vs-filtered plot uses flex raw ADC and **median + EMA filtered ADC**, overlaid on the same time axis. Hold the sensor steady and save a screenshot; calculate/describe noise reduction qualitatively or by recording many samples. **Do not claim every sensor is accurately calibrated until `error_report.md` is filled with your own measurements and you confirm the tolerances.**

## Validation and limitations

Run `scripts/tsp05 unit` for protocol and measurement-file tests and `tools/firmware_syntax_check.sh` for C++ syntax with a host shim. **Those are not equivalent to** `arduino-cli compile --fqbn arduino:avr:uno`, which must be performed on your laptop with the actual SparkFun dependency. Run `scripts/tsp05 check` to walk through GUI controls, servo extremes, stepper signed motion, DC speed and DC encoder position, safety, button, and sensor mappings. Tests that depend on a person observing motor shafts cannot be automated entirely.

The **assignment grading includes** GitHub team commits/PRs with individual technical details and challenges, and a 5–7 minute talk in a 15 minute lab slot (including ≥1 minute project application). See [RUBRIC_CHECKLIST.md](RUBRIC_CHECKLIST.md) for every check and what counts as observable PASS. A single team's four-sensor system is **not sufficient for a six-person team** (five distinct sensors required). Do not add a fictitious sensor.
