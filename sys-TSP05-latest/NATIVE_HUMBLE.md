> **Updated package:** see [FIXES_2026-11-11.md](FIXES_2026-11-11.md) for the new DC position controller, Sharp low-pass filtering, simplified Foxglove dashboard and tests. Reflash Uno **and rebuild ROS bridge**.

# Native Ubuntu 22.04 / ROS 2 Humble setup (NO Docker)

These instructions are for a **native Ubuntu 22.04 Jammy 64-bit** laptop. Humble binaries target Ubuntu 22.04; if your laptop has Windows or macOS, you would need an appropriate Linux installation or USB-enabled Linux VM. This project does **not** require or launch Docker.

## 1. Install ROS 2 Humble and required tools

First install ROS 2 Humble from the official **Ubuntu Debian packages** instructions: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html (setup the apt repository and `ros-humble-ros-base` / `ros-humble-desktop` as preferred). Once `/opt/ros/humble/setup.bash` exists:

```bash
sudo apt update
sudo apt install -y ros-humble-ros-base ros-humble-foxglove-bridge \
  ros-humble-std-msgs ros-humble-geometry-msgs \
  python3-colcon-common-extensions python3-serial python3-matplotlib \
  python3-pytest build-essential git
source /opt/ros/humble/setup.bash
ros2 --help | head
```

Install `arduino-cli` via https://arduino.github.io/arduino-cli/latest/installation/ and make sure `arduino-cli version` works. Then:

```bash
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli lib install "SparkFun VL53L1X 4m Laser Distance Sensor"
arduino-cli board list
```

If `arduino-cli lib install` cannot find the SparkFun VL53L1X library, install it via **Arduino IDE → Library Manager**, or search `arduino-cli lib search VL53L1X`; verify `SparkFun_VL53L1X.h` is available to the IDE build.

## 2. Serial permission and wiring

Unplug motor power **before connecting wires**. Follow `docs/pinout.svg` and README pin table. **Servo** needs its own externally regulated 5–6 V; stepper and DC also need correctly rated **external** supplies. Connect grounds together. Set EasyDriver current limit before powering it. Do not run motors loose on the desk: mount them and mark each shaft for visible rotation. Verify L298 motor terminal polarity and the encoder Hall signal via oscilloscope/logic analyzer if needed.

```bash
sudo usermod -aG dialout "$USER"
# LOG OUT / LOG IN so the new dialout membership takes effect.
groups | grep dialout
ls -l /dev/serial/by-id/ 2>/dev/null || true
arduino-cli board list
```

Use the Uno's actual port, for example:

```bash
export TSP05_PORT=/dev/ttyACM0
# For persistent names, prefer a valid /dev/serial/by-id/... entry.
```

On some Linux machines the Uno appears on `/dev/ttyUSB0`. **Close the Arduino IDE Serial Monitor** and kill any old node that owns the port before flashing or running. If ModemManager intercepts the Arduino, fix that serial conflict using system-specific instructions before proceeding.

## 3. Build / upload / launch

From the extracted ZIP (all instructions assume its top-level `sys-TSP05` folder):

```bash
cd ~/Downloads/sys-TSP05             # change to YOUR extracted path
chmod +x scripts/tsp05 tools/*.sh
scripts/tsp05 unit                   # no hardware, tests Python files
./tools/firmware_syntax_check.sh    # host g++ syntax only
scripts/tsp05 flash                  # Arduino Uno AVR compile AND upload
scripts/tsp05 build                  # native ROS 2 Humble colcon build
scripts/tsp05 start                  # native ROS2 bridge + foxglove_bridge
scripts/tsp05 watch                  # live status; Ctrl+C quits the monitor
```

The launcher sources ROS 2 automatically, and starts a native background `ros2 launch` process. For **manual** launching (without helper):

```bash
source /opt/ros/humble/setup.bash
cd ~/Downloads/sys-TSP05/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch tsp05_bridge tsp05.launch.py \
  port:=/dev/ttyACM0 foxglove:=true \
  calibration_csv:=$HOME/Downloads/sys-TSP05/calibration/measurements.csv
```

If you run manual launch, keep the terminal open and use Ctrl+C to stop; **do not run `scripts/tsp05 start` concurrently**, because only one process can own the serial port.

To stop the native helper-launched service (e.g., before reflashing):

```bash
scripts/tsp05 stop
scripts/tsp05 logs            # only while it is/was running
```

## 4. Foxglove desktop dashboard

Install Foxglove desktop (https://foxglove.dev/download), open the app:

1. Choose **Open connection → Foxglove WebSocket**.
2. Enter `ws://localhost:8765` and connect.
3. Choose **Layouts → Import from file** and select `foxglove/tsp05_layout.json` from the extracted ZIP.
4. Watch **Arduino connected** and the four sensor panels. Open the **Motor controls (GUI mode)** tab; switch to GUI before sending servo/stepper/DC commands.
5. Test the three motor controls, the mode buttons and the live sensor plots.
6. Open **MEASURED Sharp IR: distance → raw ADC** after `calibration/measurements.csv` has real recorded rows. The X axis must say **TRUE ruler distance (cm)** and Y must say **raw electrical ADC**. If your Foxglove release changes the imported Plot settings, configure the panel to **Expression / XY**, Y `/tsp05/calibration/sharp_transfer.points[:].y`, X `/tsp05/calibration/sharp_transfer.points[:].x`. The standalone PNG generated by `scripts/tsp05 plot` remains a guaranteed static alternative accepted by the assignment.

Open a second sourced native terminal to inspect:

```bash
source /opt/ros/humble/setup.bash
source /full/path/to/sys-TSP05/ros2_ws/install/setup.bash
ros2 topic list | grep tsp05
ros2 topic echo /tsp05/connected
ros2 topic echo /tsp05/sensors/sharp/distance_cm
ros2 topic echo /tsp05/motors/dc/position_deg
ros2 topic echo /tsp05/events
```

## 5. Three important calibrations before demo

**A. Flex endpoint calibration:** `scripts/tsp05 calibrate`; hold straight when prompted, then bend to your largest safe measurable angle, and enter that angle from your protractor (for example 90, not automatically 180). Flex resistance is non-linear in general, so report measured error across the physical bend range (do not infer N of force).

**B. DC Hall encoder output-shaft calibration:** the code uses `PULSES_PER_REV = 210` **only as an initial assumption**, in `DcMotor.h`. With motor power disconnected and shaft free (if mechanically feasible), mark the output shaft and rotate **one physical output revolution**; observe delta of `/tsp05/motors/dc/position_deg`. If the reading isn't ~360°, update `PULSES_PER_REV` to `old_PPR × (reported_degrees / 360)` (or count the A-rising edges directly), reflash, and verify. For 90-degree commands, physically measure the rotation and tune the position/speed PID gains safely. Because of gearing and motor friction, software feedback is not proof of physical calibration. `scripts/tsp05 dctest` helps correct motor and encoder polarity, with checks in both directions.

**C. Potentiometer sweep:** `POT_TRAVEL_DEG` in `Potentiometer.h` currently assumes 270°. Measure YOUR physical shaft travel from a marked 0° orientation, change constant if needed, reflash, and record error at multiple angles.

For Sharp actual electrical transfer, ToF/Sharp distance error, flex angle error and pot angle error, see `calibration/README.md` and `RUBRIC_CHECKLIST.md`. Recalibrate if the error table is unacceptable, and never present `measurements.csv` rows you have not physically measured.

## 6. Troubleshooting

- `ros2: command not found`: install Humble on Ubuntu 22.04 and `source /opt/ros/humble/setup.bash`.
- `Package 'tsp05_bridge' not found`: from project root run `scripts/tsp05 build` and source `ros2_ws/install/setup.bash` in manual terminals.
- `Package 'foxglove_bridge' not found`: `sudo apt install ros-humble-foxglove-bridge`.
- `No module named serial`: `sudo apt install python3-serial`.
- `Serial port ... permission denied`: join `dialout`, LOG OUT/IN; ensure the port is unused.
- `No data from Arduino`: verify correct port, 115200 baud, proper Uno upload, and USB cable; view `.tsp05_native.log`.
- No Foxglove topics: check bridge log and websocket; restart after rebuilding using `scripts/tsp05 restart`.
- Sharp says `TOO_CLOSE`/`TOO_FAR`: test at **10–80 cm**, move matte target perpendicular to sensor. Very close readings can be non-monotonic; don't use Sharp as a certified safety device.
- DC moves wrong direction / measured RPM negative for positive command: `scripts/tsp05 dctest`, verify Hall pull-ups and power. Then verify output-shaft PPR.
- Stepper stalls or shakes: unpower before checking coil pairing, supply and driver current limit; verify EasyDriver microstepping and speed.
- Servo moves but Arduino resets: do not power servo from Arduino 5V, use a separate correctly rated 5–6 V supply with common ground.
- Flex transfer chart shows nothing: the **graded Sharp transfer** must be measured and plotted using `scripts/tsp05 measure ...` and `scripts/tsp05 plot`; it is intentionally not fabricated.

## 7. Shutdown

`Ctrl+C` stops a foreground `watch` / manual launch; `scripts/tsp05 stop` stops the helper-managed native bridge. Then safely remove motor power, disconnect and disassemble the kit after the demo, per assignment.
