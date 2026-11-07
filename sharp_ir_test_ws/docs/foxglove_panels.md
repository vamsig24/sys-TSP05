# Foxglove panel setup

Connect to `ws://localhost:8765` after starting `foxglove_bridge`.

| Panel | Topic | Field / setup |
|---|---|---|
| Gauge | `/lab/sensors/sharp/distance_cm` | `data`; min 10; max 80; unit cm |
| Plot | `/lab/sensors/sharp/distance_cm` | `data` |
| Raw Messages | `/lab/sensors/sharp/status` | Inspect VALID / TOO_CLOSE / TOO_FAR / INVALID_READING / NO_SERIAL_DATA |
| Raw Messages | `/lab/sensors/sharp/valid` | Inspect Bool `data` |
| Raw Messages | `/lab/sensors/sharp/raw_json` | Original Arduino JSON line |

The distance topic publishes NaN for invalid measurements. Treat the validity and status topics as authoritative for whether the distance is usable.
