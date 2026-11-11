# TSP05 update — flex deadzone and calibrated sensor-to-motor mappings

This update is based on the working `sys-TSP05-position-sharp-foxglove-fixed-CORRECTED.zip` version. It **does not modify** the DC motor PID or Sharp IR filtering modules, pins, serial transport, command topics for existing motor controls, or your previously supplied Sharp XY points. It modifies only flex-follow decision logic, calibratable potentiometer endpoints, live mapping telemetry/GUI display, their tests, and documentation.

## What changed

### Flex sensor -> servo (SENSOR mode only)

- Sensor driver unchanged: 5-sample median plus EMA filter.
- The filtered, calibrated bend percentage maps linearly: **0% -> 0° servo**, **50% -> 90°**, **100% -> 180°**.
- Adds a command deadband of **8 servo degrees**, evaluation every **60 ms**, **two consecutive** out-of-deadband samples in the **same direction** before a command, and minimum **180 ms** between servo commands. That reduces flutter from a stationary flex sensor but still responds to intentional bends.
- Adds endpoint hysteresis: when commanded to 0°, the servo remains there until target >16°; when commanded to 180°, it remains there until the target <169°. It can still reach both 0 and 180.
- GUI-mode servo commands are unaffected. These numbers are in `SensorMapping.h` if you need to tune them.
- A servo may still twitch if its **power supply / common ground / mechanical mount** is unstable: software deadband does not fix pulse-width or supply noise.

**Important calibration:** use `scripts/tsp05 calibrate` to save straight / maximum-bend sensor resistances to Arduino EEPROM. Measure max physical bend with a protractor in the existing guided calibration; if left at default 180°, that displayed physical maximum is **configured, not independently verified**. The **0..100%** bend percentage, not assumed 0..180° physical flex angle, is what drives the servo.

### Potentiometer -> stepper (SENSOR mode only)

- Input is the **filtered potentiometer ADC**. It is normalized between the **stored actual endpoint values**, then mapped linearly to **0..360° commanded stepper angle**, independent of flex and ToF.
- Pot endpoint calibration is now possible from ROS 2. The previous 0..1023 ADC default remains until you calibrate. Values are saved in Arduino EEPROM and survive unplugging the Uno.
- The stepper's zero is the **position on firmware boot** (no home switch / encoder); the actual position is estimated from commanded steps. Do not claim absolute mechanical position without homing.
- Stepper target changes smaller than the existing **3°** deadband are still ignored. The `step_tgt` value is step-quantized by the EasyDriver step calculation.
- GUI-mode relative stepper movement commands are unchanged.

#### Calibrate the actual potentiometer endpoints (recommended)

1. Start the bridge and enter GUI mode. Keep your finger off the motor supply switch while checking wiring.
2. Turn the potentiometer **fully to its minimum mechanical stop**; leave it still for **at least 1.2 seconds** so its 10-sample moving average catches up.
3. Run `scripts/tsp05 potcal min`.
4. Turn it **fully to its maximum mechanical stop**; leave it still for **at least 1.2 seconds**.
5. Run `scripts/tsp05 potcal max`.
6. Check `ros2 topic echo /tsp05/mapping/status --once`. It should state **both endstops measured** and show actual ADC min and max.
7. Set `scripts/tsp05 mode sensor`. At minimum, the target should be approximately 0°; midpoint ~180°; maximum ~360°.

The `POTCAL` commands are allowed only in GUI mode; this prevents accidental recalibration while the stepper follows the pot. If necessary, `scripts/tsp05 potcal reset` restores the default ADC 0..1023 range. Calibration requires the observed max-minus-min span to be at least 100 ADC counts. Your terminal will report a clear rejection if not.

## Foxglove live mapping panel

Connect to `ws://localhost:8765` and **import the NEW** `foxglove/tsp05_layout.json` into Foxglove (previously imported layouts will not update themselves). Look for **Sensor ranges and mappings (LIVE)**. It contains:

- Flex resistance calibration min/max (kΩ), configured max physical bend angle, current bend **0..100%**, calculated ideal servo **0..180°**, and actually commanded servo angle (may differ by up to the deadband).
- Pot filtered ADC, **current captured min/max ADC**, endpoint-calibration status (defaults / MIN only / MAX only / both measured), normalized **0..100%**, ideal stepper **0..360°**, and actual step-quantized target.
- ToF distance and commanded DC RPM, with the unchanged 10..100 cm / 0..50 RPM mapping.

The flex and potentiometer gauges now use 0..100% **input normalization** rather than suggesting that the pot always sweeps 270 physical degrees or that your flex sensor always bends through 180 physical degrees. Servo commands retain 0..180°; stepper mapped target uses 0..360°.

ROS 2 topics:

```text
/tsp05/mapping/status                  std_msgs/msg/String
/tsp05/mapping/flex_servo_ideal_deg   std_msgs/msg/Float32
/tsp05/mapping/pot_stepper_ideal_deg  std_msgs/msg/Float32
/tsp05/sensors/flex/force_pct         std_msgs/msg/Float32
/tsp05/sensors/pot/adc_filtered       std_msgs/msg/Float32
/tsp05/sensors/pot/cal_min_adc        std_msgs/msg/Float32
/tsp05/sensors/pot/cal_max_adc        std_msgs/msg/Float32
/tsp05/sensors/pot/cal_flags          std_msgs/msg/Int32 (3=both measured)
/tsp05/sensors/pot/mapped_pct         std_msgs/msg/Float32
/tsp05/motors/servo/angle_deg         std_msgs/msg/Float32
/tsp05/motors/stepper/target_deg      std_msgs/msg/Float32
```

The original Sharp ROS-calculated-distance versus ADC graph remains present, **not** mislabeled as ruler-measured calibration. The DC velocity and encoder-position PID plots remain.

## Native ROS 2 Humble install/launch

Stop your OLD bridge if it is still running from the old extracted directory, and close Serial Monitor. Then:

```bash
cd ~/Downloads/sys-TSP05-flex-servo-mapping/sys-TSP05-fixed
chmod +x scripts/tsp05
source /opt/ros/humble/setup.bash
export TSP05_PORT=/dev/ttyACM0
scripts/tsp05 stop
scripts/tsp05 unit
bash tools/tests/run_control_sim.sh
scripts/tsp05 flash
scripts/tsp05 build
scripts/tsp05 start
scripts/tsp05 mode gui
scripts/tsp05 calibrate       # recommended flex calibration
scripts/tsp05 potcal min      # pot physically at minimum; wait 1.2s first
scripts/tsp05 potcal max      # pot physically at maximum; wait 1.2s first
scripts/tsp05 mode sensor
scripts/tsp05 foxglove
```

Run `scripts/tsp05 potcal min` and `max` **one at a time**, moving the pot between commands; do not execute both without physically moving it. Open the Foxglove layout file from this new directory.

## Verification checklist

- [ ] Hold flex stationary in SENSOR mode for 20 seconds: servo commands remain stable (except after deliberately moving the flex).
- [ ] Bend flex intentionally: servo moves smoothly; a larger bend eventually produces a new servo command. At calibrated flat/full bend it can still reach 0°/180°.
- [ ] `scripts/tsp05 calibrate` saved the flex endpoint resistances; Foxglove reports calibrated min/max in kΩ. Verify any measured physical max angle with a protractor.
- [ ] `potcal min` and `max` accepted and the Foxglove text reports actual ADC calibration endpoints and flag 3.
- [ ] Turn pot to minimum/midpoint/maximum: mapping panel shows normalized 0/50/100%; stepper ideal target about 0/180/360°, commanded stepper target matches within step quantization + 3° deadband.
- [ ] Observe servo angle and stepper target alongside both sensor gauges in Foxglove.
- [ ] Confirm the three mappings stay distinct: flex→servo, pot→stepper, ToF→DC. Sharp remains safety sensor.
- [ ] Confirm DC position PID is unchanged (one +45° move, no automatic reversal), and existing Sharp filter and measured-value plot remain functional.
- [ ] Check Uno compilation memory summary and several minutes of no resets or serial disconnections; host tests do **not** prove AVR SRAM headroom or hardware stability.

This package passed the Python protocol/layout tests, a **host-side** C++ firmware syntax check, and **host simulated** mapping / DC position / Sharp spike tests. Real AVR compile/flash and physical servo/stepper behavior must be verified using your Arduino and sensors.
