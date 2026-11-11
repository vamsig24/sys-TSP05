# TSP05 assignment: one-by-one grading verification checklist

**Evidence rule:** check a box only after you personally observe it on the **real Arduino Uno / motors / sensors**, save a screenshot/video, or have truthful GitHub history. Static code tests do **not** prove the kit works. This checklist follows *Sensors & Motor Control_Guidelines_2026.pdf*, **pages 1–4**. The grading specification states 4 total points (2 GitHub contribution, 2 presentation/demo); the lab demo subtasks are numbered **1–7**.

Before any test: power OFF; wire Arduino, sensors, L298, EasyDriver, button and servo per `docs/pinout.svg`. Feed each motor from a properly rated **external supply** with common grounds. Configure EasyDriver current limit, mark shafts with tape, and **mount motors**. Only then apply power, flash and run. Set `export TSP05_PORT=...` to the correct USB Uno device. Do not unplug wiring while powered.

## 0. Environment and architecture (assignment pages 1, 3–4)

- [ ] **0.1 Arduino Uno ONLY.** Inspect hardware. `arduino-cli board list` shows an Uno; no ESP32, Mega, Raspberry Pi or second MCU on the motor-control circuit. Evidence: photo of wired Uno.
- [ ] **0.2 One unified MCU program.** `scripts/tsp05 flash` uploads `sys-TSP05.ino` plus its C++ modules as **one sketch**. All sensors and motors operate concurrently in one firmware loop, not one sketch per sensor. Evidence: successful Uno AVR compile/upload log + running demo.
- [ ] **0.3 ROS 2 on laptop over serial.** `scripts/tsp05 build && scripts/tsp05 start`, then `ros2 topic echo /tsp05/connected` shows `true`. Verify `ros2 topic list | grep tsp05` and state topics change. Evidence: terminal screenshot.
- [ ] **0.4 Native Ubuntu 22.04 / Humble, no Docker.** `echo $ROS_DISTRO` in sourced terminal is `humble`; `ros2 pkg prefix tsp05_bridge` succeeds. Launch `scripts/tsp05 start`. Evidence: native terminal.
- [ ] **0.5 Kit power and mounting.** Motors don't move the board/circuit across bench; shafts visibly marked, servo uses external regulated **5–6 V**, DC/stepper separate motor supply, all grounds common, EasyDriver current limit set. Evidence: clear picture.

## 1. Sensor processing — physical units and *measured error* for EACH (page 2, item 1)

Exactly these four count: flex, potentiometer, VL53L1X, Sharp IR. **Pushbutton is not a counted sensor.** If your team has **6 people**, five distinct sensors are required; a fifth sensor must be added and validated before claiming compliance. One chosen sensor must be analog (several here are).

- [ ] **1.1 Four sensors present, distinct and updating.** `scripts/tsp05 watch` and Foxglove show flex bend degrees, pot shaft degrees, VL53L1X centimeters, Sharp centimeters. Move each **one at a time**; verify only its matching feedback changes. Evidence: four screenshots or video.
- [ ] **1.2 Flex bend angle (degrees).** `scripts/tsp05 calibrate` straight/bent endpoints **and protractor-measured maximum angle**. Use a **protractor** for at least five true bending angles within safe feasible range, at each angle `scripts/tsp05 measure flex <true_degrees> --seconds 3`. Recorded `flex_deg` tracks true angles. Evidence: `calibration/measurements.csv` and error report; don't describe the normalized `force_pct` as measured newtons.
- [ ] **1.3 Potentiometer shaft angle (degrees).** Mark shaft, independently measure actual range; adjust `POT_TRAVEL_DEG` in `Potentiometer.h` from default 270° if different, reflash. At ≥5 measured positions `scripts/tsp05 measure pot <true_degrees> --seconds 3`. Evidence: calibrated angle + report.
- [ ] **1.4 VL53L1X range (centimeters).** Test ≥5 **ruler-measured** distances within its practical reliable range, e.g., 10/20/40/80/120 cm, run `scripts/tsp05 measure tof <true_cm> --seconds 3`. Verify `tof_ok=true` and display in cm; out-of-range invalid. Evidence: screenshot + error report.
- [ ] **1.5 Sharp IR range (centimeters).** Test ≥5 and preferably 7 **ruler-measured** distances across nominal 10–80 cm, with matte target, run `scripts/tsp05 measure sharp <true_cm> --seconds 3`. Verify `sharp_st=VALID` and display in cm; out-of-range is flagged, not used as valid distance. Evidence: CSV + report.
- [ ] **1.6 Measured error across range FOR ALL FOUR.** `scripts/tsp05 plot` creates `calibration/error_report.md`: check **all four sensors appear** and each has ≥5 distinct physical truth points, meaningful MAE, RMSE, maximum absolute error, per-reference bias. State actual numeric errors to TA and acknowledge whether accuracy meets your project's threshold. Code cannot pre-certify that error is small. Evidence: report and pictures of ruler/protractor setups.

## 2. Graded SENSOR transfer-function plot — independent physical X, electrical Y (page 2, item 2)

- [ ] **2.1 Valid analog sensor:** choose **Sharp GP2Y0A21** A1 (not ToF). State it is an analog voltage-output sensor; y is its **raw ADC** before the distance conversion.
- [ ] **2.2 Physically measured data:** capture at least five distinct **ruler** positions (recommend 10, 15, 20, 30, 40, 60, 80 cm), stable target, ≥3 seconds each: `scripts/tsp05 measure sharp 10`, etc. Confirm nonempty `calibration/measurements.csv` containing real `sensor=sharp, truth, raw, reported` rows. **No synthetic points.**
- [ ] **2.3 Correct axis orientation:** `scripts/tsp05 plot`; open `calibration/sharp_transfer_function.png`. **Horizontal = measured physical distance (cm)**, **vertical = raw analog ADC (counts)**; markers are measured average raw ADC and error bars are measured SD. Show printed copy or screen to TA.
- [ ] **2.4 Foxglove live connection to measured graph:** `ros2 topic echo /tsp05/calibration/sharp_transfer --once` shows points `x=TRUE_DISTANCE_CM`, `y=RAW_ADC`. Open dashboard **MEASURED Sharp IR** XY plot, verify axes. If your Foxglove version changes import behavior, fix XY fields as in `NATIVE_HUMBLE.md` or present the standalone PNG (explicitly permitted on screen/printed by assignment).

## 3. Noise filtering on at least one sensor (page 2, item 3)

- [ ] **3.1 Flex raw and filtered side by side.** Foxglove **Flex bend: raw vs filtered ADC** shows two simultaneously updating curves from `/tsp05/sensors/flex/adc_raw` and `/tsp05/sensors/flex/adc_filtered`.
- [ ] **3.2 Noise actually reduced.** Hold flex at fixed angle for at least 20–30 seconds. Inspect variation in the gray raw curve versus the median+EMA filtered curve and screenshot; optionally run `scripts/tsp05 measure flex <stationary_true_degrees> --seconds 30` then `scripts/tsp05 plot` for `calibration/flex_raw_filtered.png` and SD in the report. Explain median rejection of spikes and EMA smoothing; any lag in response is expected. Raw and filtered traces need **demonstrated** difference, not just code.

## 4. GUI controls all THREE motors (page 3, item 4)

- [ ] **4.1 GUI mode selected.** In Foxglove click **GUI mode**, verify `/tsp05/mode` = `GUI` and the physical button can return it.
- [ ] **4.2 RC positional servo extremes and intermediate:** in Foxglove use **Servo 0**, **Servo 180**, then `Set servo angle` with data `90` and e.g. `37`. Physically observe both achievable mechanical extremes and intermediate; verify `/tsp05/motors/servo/angle_deg`. If actual HS-311 cannot safely travel exact 0/180 using nominal pulse limits, document mechanical limits rather than forcing it into a stop.
- [ ] **4.3 DC signed RPM via encoder PID:** in Foxglove command +30 RPM, -30 RPM and zero; compare measured `/tsp05/motors/dc/rpm` with `/tsp05/motors/dc/target_rpm`, correct sign, and settle reasonably. Run `scripts/tsp05 dctest` if polarity incorrect. Check PWM stall behavior. Verify physical spin direction and RPM, not only software setpoints.
- [ ] **4.4 DC encoder position +degrees:** first calibrate real motor `PULSES_PER_REV` in `DcMotor.h` and reflash. Foxglove **DC +90 degrees** / editable position command, verify `/tsp05/motors/dc/position_deg` changes approximately +90°, `position_reached` becomes true, physical shaft marker moves ~¼ turn. Compare actual protractor angle, not just encoder calculation.
- [ ] **4.5 DC encoder position -degrees:** Foxglove **DC -90 degrees**, verify measured relative change approximately -90°, physical shaft returns. Explain **outer position PD + inner encoder PID speed loop**, not merely open-loop PWM.
- [ ] **4.6 Stepper signed relative angle:** Foxglove `+15°`, `-15°`, editable `+90°`, `-90°`, and a non-step-aligned angle (e.g. 7°). Physically observe direction, compare `last_actual_delta_deg` (quantized requested angle), `target_deg`, `angle_deg`. Verify microstep settings correctly reflect wiring; software step counts alone don't detect skips.

## 5. Three DISTINCT sensors control THREE motors, with clear mode selector (page 3, item 5)

- [ ] **5.1 Flex → servo:** choose SENSOR mode; bend/straighten flex, confirm actual servo angle follows measured bend in degrees. Evidence: live demo.
- [ ] **5.2 Pot → stepper:** rotate pot across range; the stepper visibly follows to different angles and stabilizes. Evidence: video/marked shaft.
- [ ] **5.3 VL53L1X → DC RPM:** change ToF distance 10→40→100 cm; DC motor target changes, measured speed follows (and motor stops at near distance). Evidence: dual RPM trace and actual motor direction/velocity.
- [ ] **5.4 Distinct mapping demonstrated:** flex/pot/ToF physically separate; each motor controlled by **different** sensor; Sharp IR is fourth safety sensor. Explain to TA.
- [ ] **5.5 Mode toggle / arbitration:** switch GUI→SENSOR via Foxglove button and back via button or GUI; verify sensor changes affect motors only in SENSOR mode. In SENSOR mode, try GUI motor command; `/tsp05/events` acknowledges rejection (`not in GUI mode`). Evidence: screenshot + explanation.
- [ ] **5.6 Safety interlock:** with Sharp target ~10–15 cm, verify `/tsp05/safety/active` true, DC/stepper stop and GUI commands are rejected. Remove target and verify latch clears after ~0.5 sec. Safety is a lab demonstration, **not** a certified failsafe.

## 6. Debounced physical switch changes motor state (page 3, item 6)

- [ ] **6.1 One deliberate button press → one mode transition** (`GUI↔SENSOR`), button count increments by exactly one. Hold pressed and verify no repeated toggles; release and press again to toggle back. Evidence: `/tsp05/button/count`, mode and event screenshot. Repeat a few times and check bounces don't create duplicates.
- [ ] **6.2 Explain software debounce** (minimum stable interval in `Button.cpp`, 50 ms, non-blocking) and that mode changes whether sensors are commanding motors.

## 7. GUI shows meaningful live sensor/motor data AND sends commands (page 3, item 7)

- [ ] **7.1 Foxglove opens native bridge** at `ws://localhost:8765`; imported layout `foxglove/tsp05_layout.json` renders without broken panels. Screenshot.
- [ ] **7.2 GUI graphical outputs:** physical units for all four sensors; gauges for servo, stepper, DC target/measured RPM and DC position; mode and safety indicators; plots for raw/filtered, measured Sharp transfer, motor position/speed. Confirm data change live.
- [ ] **7.3 GUI graphical inputs:** publish buttons work for mode, servo extremes/intermediate, arbitrary signed stepper delta, arbitrary signed DC RPM and relative DC degrees; event/ack feedback from Uno shows accepted/rejected commands. Confirm real actuators respond.
- [ ] **7.4 GUI stays responsive under simultaneous sensor readings and stepper motion**, no serial corruption / disconnects when motors move. Check `scripts/tsp05 logs` and `/tsp05/connected` during the demo.

## Submission and presentation (assignment page 1 and page 4)

- [ ] **G1 Public GitHub repo, all teammates added.** `git remote -v`, repository visibility public, all team members collaborators as appropriate.
- [ ] **G2 Substantive independent contributions.** EACH person has actual authored code changes in own GitHub account's commits/PRs with descriptive commit messages and PR/repo detail; do not manufacture commit authors or history.
- [ ] **G3 Contributions/challenges documented with technical specifics.** README/PR describes each person's technical work and the root cause, investigation or suggested remedy of actual project challenges. See `GITHUB_TEAM_EVIDENCE.md` template to fill with factual team information.
- [ ] **G4 Supporting figures**: upload wiring photo, marked motor shafts, raw-filter noise screenshot, actual Sharp transfer PNG, error report, Foxglove dashboard screenshot. The assignment allows photos, flowcharts, plots, schematic, etc.
- [ ] **P1 Lab demo** during **15-minute** assigned slot, NSH B506 or as your posted Canvas schedule directs; one teammate gives a **5–7 minute** presentation including live demo.
- [ ] **P2 At least 1 minute of the talk** explains realistic future uses of your sensors, motors and motor control in YOUR system (not just a generic list). Prepare your team's actual application.
- [ ] **P3 Q&A** for remainder of lab period; know motor power limits, PID/encoder calibration, analog transfer X/Y and sensor measurement error.
- [ ] **P4 Return/disassemble all kit parts** after demo per lab notes; power off first.

## What can and cannot be called "complete"

**Software present**: one Uno sketch; four sensor input paths; analog output and physical-unit conversions; filtered+raw; three motor types; encoder-based signed velocity PID & relative position control; software debounce; safety; ROS2 bridge; Foxglove layout; native launcher; live measurement recorder; truthful error report/plot generator.

**Still requires your own evidence**: real measured sensor conversion error at multiple physical input levels; actual sensor transfer data; calibration of motor encoder pulses/rev and pot mechanical range; physical performance of each motor, mode switch and safety; Foxglove version on your laptop; GitHub contributors and demonstrated presentation; fifth sensor if six-person team.

**Source:** TSP05 2026 lab handout, pages 1–4. It also warns Uno Servo library disables PWM on D9/D10, servo needs external 5–6 V power, stepper current limiting and Hall-encoder pull-ups. Do not skip these checks.
