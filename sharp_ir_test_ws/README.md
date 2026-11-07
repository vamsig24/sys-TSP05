# Sharp IR Individual ROS 2 Test Workspace

A minimal, standalone workspace for testing one Sharp GP2Y0A21YK0F analog distance sensor (SharpIR library model `1080`) on an Arduino Uno, publishing its readings to ROS 2, and viewing them in Foxglove.

This is intentionally separate from the full TSP05 project. It does not control motors and does not require the ESP32.

## Data path

`Sharp GP2Y0A21YK0F -> Arduino Uno A0 -> USB serial (JSON, 9600 baud) -> ROS 2 Python node -> Foxglove Bridge -> Foxglove`

## Hardware

| Sharp sensor | Arduino Uno |
|---|---|
| VCC | 5V |
| GND | GND |
| OUT | A0 |

Place the sensor's recommended supply bypass capacitor (at least 10 uF) close to its VCC/GND pins. Observe electrolytic capacitor polarity. Do not connect the sensor output to a 5 V tolerant assumption on another board; this workspace is for the Arduino Uno.

## 1. Upload the Arduino sketch

1. Install the `SharpIR` library in Arduino IDE (Library Manager, if available).
2. Open `arduino/sharp_ir_json.ino`.
3. Select the correct Arduino Uno board and serial port.
4. Upload the sketch.
5. Close Arduino IDE Serial Monitor/Plotter before starting ROS 2. Only one program can own the serial port at a time.

The sketch uses model `1080`, a nominal valid range of 10-80 cm, and 9600 baud. It emits one JSON object per line. It does not add a custom calibration table.

Example valid line:

```json
{"type":"state","sharp_distance_cm":42,"sharp_valid":true,"sharp_status":"VALID"}
```

Example invalid line:

```json
{"type":"state","sharp_distance_cm":null,"sharp_valid":false,"sharp_status":"TOO_FAR"}
```

The range checks classify the library's reported distance; they cannot guarantee detection of every object outside the sensor's rated range.

## 2. Install ROS 2 dependencies

This workspace targets ROS 2 Jazzy on Ubuntu. If using another ROS 2 distribution, source that distribution and install the matching Foxglove Bridge package.

```bash
sudo apt update
sudo apt install python3-serial ros-jazzy-foxglove-bridge
```

If your account cannot open the Arduino port:

```bash
sudo usermod -a -G dialout "$USER"
```

Log out and back in after adding the group. Check the port with:

```bash
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

Typical Arduino Uno port: `/dev/ttyACM0`.

## 3. Build the standalone ROS 2 workspace

From this folder:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 4. Run the serial-to-ROS 2 node

Make sure the Arduino is connected and its IDE Serial Monitor is closed.

```bash
ros2 launch sharp_ir_test sharp_ir.launch.py port:=/dev/ttyACM0 baud:=9600
```

Change `/dev/ttyACM0` if your Arduino appears on another port.

## 5. Verify ROS 2 topics

Open another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/sharp_ir_test_ws/ros2_ws/install/setup.bash
ros2 topic list | grep sharp
```

Expected topics:

- `/lab/sensors/sharp/distance_cm` (`std_msgs/msg/Float32`)
- `/lab/sensors/sharp/valid` (`std_msgs/msg/Bool`)
- `/lab/sensors/sharp/status` (`std_msgs/msg/String`)
- `/lab/sensors/sharp/raw_json` (`std_msgs/msg/String`)

Try:

```bash
ros2 topic echo /lab/sensors/sharp/distance_cm
ros2 topic echo /lab/sensors/sharp/valid
ros2 topic echo /lab/sensors/sharp/status
```

When a reading is invalid, `distance_cm` is published as NaN, `valid` is false, and `status` explains why. NaN prevents a stale valid distance from being mistaken for the current reading.

## 6. View it in Foxglove

Terminal 1: keep the serial node running as above.

Terminal 2: start Foxglove Bridge:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Open Foxglove (web app or desktop), select **Open connection -> Foxglove WebSocket**, and connect to:

```text
ws://localhost:8765
```

Create these panels:

1. **Gauge**: topic `/lab/sensors/sharp/distance_cm`, field `data`, min 10, max 80, units `cm`.
2. **Plot**: topic `/lab/sensors/sharp/distance_cm`, field `data`.
3. **Raw Messages**: topic `/lab/sensors/sharp/status`.
4. **Raw Messages**: topic `/lab/sensors/sharp/valid`.

The Gauge/Plot are for valid distance readings. Use `valid` and `status` to identify invalid readings; do not interpret NaN as a measured distance.

## 7. Run the protocol unit tests (no Arduino or ROS 2 required)

From this folder:

```bash
python3 -m unittest discover -s ros2_ws/src/sharp_ir_test/test -v
```

## Troubleshooting

- **Permission denied:** add your user to `dialout`, then log out/in.
- **Port busy:** close Arduino Serial Monitor, Serial Plotter, and any other serial terminal.
- **No messages:** verify the selected port, 9600 baud, USB cable, and that the sketch was uploaded.
- **JSON parse warnings:** confirm that the Arduino is running `arduino/sharp_ir_json.ino`, not the older human-readable `Serial.print()` sketch.
- **No Foxglove topics:** verify the ROS 2 node is publishing, then start `foxglove_bridge` and connect to port 8765.

## Scope and safety

This workspace is for individual sensor visualization only. It does not implement motor control, collision-rated safety, or the full TSP05 multi-sensor system. The Sharp sensor's response outside its nominal 10-80 cm range is not reliable; the software range labels are not a substitute for a physical safety system.
