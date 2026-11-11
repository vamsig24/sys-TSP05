// =====================================================================
// TSP05: Sensors & Motor Control - unified Arduino Uno program
//
// Implements the TSP05 demo flow with the hardware actually on the bench:
//   FSR             -> replaced by FLEX sensor (A0) used as the force
//                      sensor: normalized bend to calibrated degrees -> servo angle
//   Omron slot      -> replaced by POTENTIOMETER (A2) -> stepper angle
//   VL53L1X (I2C)   -> DC motor target speed (encoder PI speed control)
//   VL53L5CX matrix -> not available; SHARP IR (A1) is the safety sensor
//   Hardware button (pin 2, debounced) -> GUI / SENSOR mode toggle
//
// Two modes:
//   GUI    - motors are commanded from ROS 2 / Foxglove over serial
//   SENSOR - motors are driven by the sensors (mapping above)
// In both modes the safety interlock stops the motors while an obstacle
// is closer than the safety threshold.
//
// Serial (115200 baud), one message per line:
//   Arduino -> host: JSON objects ("t":"state" at 10 Hz, "ack", "event", "boot")
//   Host -> Arduino: text commands
//       MODE GUI | MODE SENSOR | MODE TOGGLE
//       SERVO <deg 0-180>          (GUI mode)
//       DC <rpm -100..100>         (GUI mode)
//       DCPOS <relative degrees>     (GUI mode, encoder position control)
//       STEP <delta deg>           (GUI mode, relative stepper move)
//       SAFETY <cm>                (safety threshold, 0 = disabled)
//       DCPWM <pwm -230..230>      (GUI mode, open-loop DC test, stops after 2 s)
//       DCPOL <motor 0/1> <encoder 0/1>
//                                  (DC polarity, saved in EEPROM)
//       CAL FLAT | CAL BENT | CAL ANGLE <measured_deg> | CAL RESET
//                                  (flex/force calibration, saved in EEPROM)
//       POTCAL MIN | POTCAL MAX | POTCAL RESET
//                                  (pot ADC endstops; GUI mode only; EEPROM)
// =====================================================================

#include <Servo.h>
#include <EEPROM.h>
#include "DcMotor.h"
#include "StepperMotor.h"
#include "DistanceSensor.h"
#include "FlexSensor.h"
#include "SharpIRSensor.h"
#include "Potentiometer.h"
#include "Button.h"
#include "SensorMapping.h"

// ====================================
// Pin configuration
// ====================================

// DC motor: ENA, IN1, IN2, Encoder A, Encoder B
// Encoder A must be an interrupt pin (2 or 3 on the Uno); B is only read
// for direction. Pin 2 is the button, so A is on 3 and B on 4.
const bool DC_MOTOR_CONNECTED = true;
DCMotor dcMotor(6, 8, 7, 3, 4);

// Stepper motor: DIR, STEP
StepperMotor stepperMotor(13, 12);

// Servo motor
const int SERVO_PIN = 10;
Servo servoMotor;

// VL53L1X distance sensor: I2C (Uno: SDA = A4, SCL = A5)
DistanceSensor distanceSensor;

// Flex sensor: 5V -- flex -- A0 -- 10k -- GND
FlexSensor flexSensor(A0, 10000.0f);

// Sharp GP2Y0A21YK0F IR distance sensor (10-80 cm)
SharpIRSensor sharpSensor(A1);

// Potentiometer wiper on A2 (outer legs to 5V and GND)
Potentiometer potentiometer(A2);

// Push button on pin 2: button to 5V, external pull-down (pressed = HIGH)
Button button(2);

// ====================================
// Control configuration
// ====================================

// Flex bend calibration -> servo full 0..180 degree travel (SENSOR mode)
// Default calibration from that branch; replace with CAL FLAT / CAL BENT
const float FLEX_DEFAULT_FLAT_OHMS = 13000.0f;
const float FLEX_DEFAULT_BENT_OHMS = 34000.0f;
const float FLEX_MIN_SPAN_OHMS = 500.0f;   // bent must exceed flat by this
// Flex->servo jitter guard is in SensorMapping.h: 8 degree deadband,
// 60 ms evaluation, 2 readings, 180 ms command spacing, hysteresis.
TSP05Mapping::ServoDeadband servoFollow;

// Potentiometer physical endstops are CALIBRATABLE and persisted in EEPROM.
// Until calibrated, the conservative electrical range is ADC 0..1023.
float potAdcMin = 0.0f;
float potAdcMax = 1023.0f;
uint8_t potCalFlags = 0;  // bit 0: measured MIN, bit 1: measured MAX
const float POT_MIN_SPAN_ADC = 100.0f;
const float STEPPER_DEADBAND_DEG = 3.0f;
const float STEPPER_SPEED = 400.0f;  // microsteps/second

// VL53L1X -> DC motor speed (closer object -> lower speed)
const float DC_NEAR_CM = 10.0f;      // at or below: stop
const float DC_FAR_CM = 100.0f;      // at or above: max speed
const float DC_MIN_RPM = 20.0f;
const float DC_MAX_RPM = 50.0f;      // motor tops out ~55 rpm at max PWM
// The PI controller resets its integrator whenever the target changes, so
// ignore small changes caused by ToF noise (otherwise it never settles)
const float DC_TARGET_DEADBAND_RPM = 2.0f;
// Default polarity (overridden by DCPOL, stored in EEPROM)
const bool DC_MOTOR_INVERT = false;
const bool DC_ENCODER_INVERT = false;
const unsigned long DC_OPEN_LOOP_MS = 2000;   // open-loop test auto-stop

// Safety interlock (Sharp IR, stands in for VL53L5CX)
float safetyThresholdCm = 20.0f;     // 0 disables
const uint8_t SAFETY_TRIP_READINGS = 2;        // consecutive close readings
const unsigned long SAFETY_CLEAR_MS = 500;     // clear time before release

// Serial output
const unsigned long STATE_INTERVAL_MS = 100;   // 10 Hz

// ====================================
// State
// ====================================

enum Mode { MODE_GUI, MODE_SENSOR };
Mode mode = MODE_GUI;

int servoAngle = 90;
float dcTargetRPM = 0.0f;
bool dcPositionMode = false;
float stepLastActualDeg = 0.0f;
float stepperTargetDeg = 0.0f;

bool dcOpenLoop = false;
int dcOpenLoopPWM = 0;
unsigned long dcOpenLoopStart = 0;

bool safetyActive = false;
uint8_t safetyCloseCount = 0;
unsigned long safetyLastClose = 0;

unsigned long lastStateSend = 0;

// ====================================
// Non-blocking serial output
// ====================================
// Serial.print() blocks once the 64-byte TX buffer is full, which would
// stall the stepper. Short messages (acks, events) are queued in outBuf.
// The large state message is written one field at a time, only when the
// TX buffer has room for a whole field (see drainSerial()).

const int OUT_CAPACITY = 160;
char outBuf[OUT_CAPACITY];
int outHead = 0;   // next byte to send
int outTail = 0;   // end of queued data
int msgStart = -1; // start of message being built
bool msgOverflow = false;

int stateField = -1;       // next state field to send, -1 = idle
const int FIELD_ROOM = 34; // longest state field + margin

void beginMessage() {
    // Make room at the end of the buffer by discarding sent bytes
    if (outHead > 0) {
        memmove(outBuf, outBuf + outHead, outTail - outHead);
        outTail -= outHead;
        outHead = 0;
    }
    msgStart = outTail;
    msgOverflow = false;
}

void put(const char* s) {
    while (*s) {
        if (outTail >= OUT_CAPACITY) {
            msgOverflow = true;
            return;
        }
        outBuf[outTail++] = *s++;
    }
}

void putBool(bool v) {
    put(v ? "true" : "false");
}

// ,"key":
void key(const char* k) {
    put(",\"");
    put(k);
    put("\":");
}

// Drops the message if it did not fit
void endMessage() {
    put("}\n");
    if (msgOverflow) {
        outTail = msgStart;
    }
    msgStart = -1;
}

bool queueEmpty() {
    return outHead == outTail;
}

// ====================================
// Helpers
// ====================================

const char* modeName() {
    return mode == MODE_GUI ? "GUI" : "SENSOR";
}

void sendEvent(const char* name) {
    beginMessage();
    put("{\"t\":\"event\",\"event\":\"");
    put(name);
    put("\"");
    key("mode");
    put("\"");
    put(modeName());
    put("\"");
    endMessage();
}

void sendAck(const char* cmd, bool ok, const char* err) {
    beginMessage();
    put("{\"t\":\"ack\",\"cmd\":\"");
    put(cmd);
    put("\"");
    key("ok");
    putBool(ok);
    if (!ok) {
        key("err");
        put("\"");
        put(err);
        put("\"");
    }
    endMessage();
}

void setMode(Mode newMode) {
    if (newMode == mode) {
        return;
    }
    mode = newMode;

    // Entering GUI mode: hold everything where it is until commanded
    if (mode == MODE_GUI) {
        stepperTargetDeg = stepperMotor.getPositionDegrees();
        stepperMotor.setTargetDegrees(stepperTargetDeg);
        dcTargetRPM = 0.0f;
    }
    dcPositionMode = false;
    dcMotor.cancelPositionMove();
    dcOpenLoop = false;
    if (mode == MODE_SENSOR) servoFollow.reset(millis());

    sendEvent("mode_changed");
}

void writeServo(int angle) {
    servoAngle = constrain(angle, 0, 180);
    servoMotor.write(servoAngle);
}

// ====================================
// Flex (force) calibration in EEPROM
// ====================================

struct FlexCalibration {
    uint16_t magic;
    float flatOhms;
    float bentOhms;
    float maxAngleDegrees;
};
const uint16_t FLEX_CAL_MAGIC = 0x7C06;  // new EEPROM format with measured angle
const int FLEX_CAL_ADDR = 0;

void loadFlexCalibration() {
    FlexCalibration cal;
    EEPROM.get(FLEX_CAL_ADDR, cal);
    if (cal.magic == FLEX_CAL_MAGIC &&
        cal.flatOhms > 0.0f && cal.bentOhms < 1000000.0f &&
        cal.bentOhms - cal.flatOhms >= FLEX_MIN_SPAN_OHMS &&
        cal.maxAngleDegrees >= 1.0f && cal.maxAngleDegrees <= 180.0f) {
        flexSensor.setCalibration(cal.flatOhms, cal.bentOhms);
        flexSensor.setMaxBendDegrees(cal.maxAngleDegrees);
    } else {
        flexSensor.setCalibration(FLEX_DEFAULT_FLAT_OHMS, FLEX_DEFAULT_BENT_OHMS);
        flexSensor.setMaxBendDegrees(180.0f);
    }
}

void saveFlexCalibration() {
    FlexCalibration cal = {
        FLEX_CAL_MAGIC, flexSensor.getFlatOhms(), flexSensor.getBentOhms(),
        flexSensor.getMaxBendDegrees()
    };
    EEPROM.put(FLEX_CAL_ADDR, cal);
}

// CAL FLAT: current reading = 0 % force; CAL BENT: current reading = 100 %
void handleCalibration(char* arg) {
    if (arg == nullptr) {
        sendAck("CAL", false, "expected FLAT, BENT or RESET");
        return;
    }
    for (char* p = arg; *p; p++) *p = toupper(*p);

    float now = flexSensor.getResistanceOhms();
    float flat = flexSensor.getFlatOhms();
    float bent = flexSensor.getBentOhms();

    if (strncmp(arg, "ANGLE ", 6) == 0) {
        char* endptr;
        float degrees = strtod(arg + 6, &endptr);
        while (*endptr == ' ') endptr++;
        if (endptr == arg + 6 || *endptr || !isfinite(degrees) ||
            degrees < 1.0f || degrees > 180.0f) {
            sendAck("CAL", false, "ANGLE expects 1..180 physical degrees");
            return;
        }
        flexSensor.setMaxBendDegrees(degrees);
        saveFlexCalibration();
        sendAck("CAL", true, "");
        return;
    }

    if (strcmp(arg, "FLAT") == 0) {
        flat = now;
        // Keep the span valid if the old bent point is now too close
        if (bent - flat < FLEX_MIN_SPAN_OHMS) {
            bent = flat + FLEX_MIN_SPAN_OHMS;
        }
    } else if (strcmp(arg, "BENT") == 0) {
        if (now - flat < FLEX_MIN_SPAN_OHMS) {
            sendAck("CAL", false, "bend further (or CAL FLAT first)");
            return;
        }
        bent = now;
    } else if (strcmp(arg, "RESET") == 0) {
        flat = FLEX_DEFAULT_FLAT_OHMS;
        bent = FLEX_DEFAULT_BENT_OHMS;
        flexSensor.setMaxBendDegrees(180.0f);
    } else {
        sendAck("CAL", false, "expected FLAT, BENT or RESET");
        return;
    }

    flexSensor.setCalibration(flat, bent);
    saveFlexCalibration();
    sendAck("CAL", true, "");
}

// ====================================
// Potentiometer actual endstop calibration in EEPROM
// ====================================
struct PotCalibration {
    uint16_t magic;
    uint16_t minADC;
    uint16_t maxADC;
    uint8_t flags;
};
const uint16_t POT_CAL_MAGIC = 0x50CA;
const int POT_CAL_ADDR = 48;  // distinct from flex (0) and DC polarity (32)

void loadPotCalibration() {
    PotCalibration cal;
    EEPROM.get(POT_CAL_ADDR, cal);
    if (cal.magic == POT_CAL_MAGIC && cal.maxADC <= 1023 &&
        cal.maxADC >= cal.minADC + POT_MIN_SPAN_ADC &&
        cal.flags <= 3) {
        potAdcMin = cal.minADC;
        potAdcMax = cal.maxADC;
        potCalFlags = cal.flags;
    }
}
void savePotCalibration() {
    PotCalibration cal = {POT_CAL_MAGIC,
        (uint16_t)(potAdcMin + 0.5f), (uint16_t)(potAdcMax + 0.5f),
        potCalFlags};
    EEPROM.put(POT_CAL_ADDR, cal);
}
void handlePotCalibration(char* arg) {
    if (mode != MODE_GUI) {
        sendAck("POTCAL", false, "switch to GUI mode first");
        return;
    }
    if (arg == nullptr) {
        sendAck("POTCAL", false, "expected MIN, MAX or RESET");
        return;
    }
    for (char* p = arg; *p; ++p) *p = toupper(*p);
    float reading = potentiometer.getFilteredADC();
    if (strcmp(arg, "RESET") == 0) {
        potAdcMin = 0.0f;
        potAdcMax = 1023.0f;
        potCalFlags = 0;
    } else if (strcmp(arg, "MIN") == 0 &&
               reading <= potAdcMax - POT_MIN_SPAN_ADC) {
        potAdcMin = reading;
        potCalFlags |= 1;
    } else if (strcmp(arg, "MAX") == 0 &&
               reading >= potAdcMin + POT_MIN_SPAN_ADC) {
        potAdcMax = reading;
        potCalFlags |= 2;
    } else {
        sendAck("POTCAL", false, "bad endpoint / span < 100 ADC");
        return;
    }
    savePotCalibration();
    sendAck("POTCAL", true, "");
}

// ====================================
// DC motor polarity in EEPROM
// ====================================

struct DcPolarity {
    uint16_t magic;
    uint8_t motorInverted;
    uint8_t encoderInverted;
};
const uint16_t DC_POL_MAGIC = 0xDC05;
const int DC_POL_ADDR = 32;

void loadDcPolarity() {
    DcPolarity pol;
    EEPROM.get(DC_POL_ADDR, pol);
    if (pol.magic == DC_POL_MAGIC) {
        dcMotor.setInverted(pol.motorInverted, pol.encoderInverted);
    } else {
        dcMotor.setInverted(DC_MOTOR_INVERT, DC_ENCODER_INVERT);
    }
}

void handleDcPolarity(char* arg) {
    int m, e;
    if (arg == nullptr || sscanf(arg, "%d %d", &m, &e) != 2 ||
        m < 0 || m > 1 || e < 0 || e > 1) {
        sendAck("DCPOL", false, "expected: DCPOL <0|1> <0|1>");
        return;
    }
    // Stop first so the direction does not flip while running
    dcOpenLoop = false;
    dcTargetRPM = 0.0f;
    dcPositionMode = false;
    dcMotor.cancelPositionMove();
    dcMotor.setClosedLoop();
    dcMotor.setTargetRPM(0.0f);
    dcMotor.setInverted(m, e);
    DcPolarity pol = {DC_POL_MAGIC, (uint8_t)m, (uint8_t)e};
    EEPROM.put(DC_POL_ADDR, pol);
    sendAck("DCPOL", true, "");
}

// ====================================
// Safety interlock
// ====================================

bool sharpObstacle() {
    if (safetyThresholdCm <= 0.0f) {
        return false;
    }
    SharpIRSensor::Status s = sharpSensor.getStatus();
    if (s == SharpIRSensor::TOO_CLOSE) {
        return true;
    }
    return s == SharpIRSensor::VALID &&
           sharpSensor.getDistanceCm() < safetyThresholdCm;
}

void updateSafety() {
    static unsigned long lastCheck = 0;
    unsigned long now = millis();

    // Sharp updates every 50 ms; check at the same rate
    if (now - lastCheck < 50) {
        return;
    }
    lastCheck = now;

    if (sharpObstacle()) {
        safetyLastClose = now;
        if (safetyCloseCount < 255) {
            safetyCloseCount++;
        }
        if (!safetyActive && safetyCloseCount >= SAFETY_TRIP_READINGS) {
            safetyActive = true;

            // Stop motors where they are
            stepperTargetDeg = stepperMotor.getPositionDegrees();
            stepperMotor.setTargetDegrees(stepperTargetDeg);
            dcTargetRPM = 0.0f;
            dcPositionMode = false;
            dcOpenLoop = false;
            if (DC_MOTOR_CONNECTED) dcMotor.emergencyStop();

            sendEvent("safety_stop");
        }
    } else {
        safetyCloseCount = 0;
        if (safetyActive && now - safetyLastClose >= SAFETY_CLEAR_MS) {
            safetyActive = false;
            sendEvent("safety_clear");
        }
    }
}

// ====================================
// Sensor mode control
// ====================================

float distanceToRPM() {
    if (!distanceSensor.isOk()) {
        return 0.0f;
    }
    float cm = distanceSensor.getDistanceCm();
    if (cm <= DC_NEAR_CM) {
        return 0.0f;
    }
    if (cm >= DC_FAR_CM) {
        return DC_MAX_RPM;
    }
    return DC_MIN_RPM + (cm - DC_NEAR_CM) * (DC_MAX_RPM - DC_MIN_RPM) /
                        (DC_FAR_CM - DC_NEAR_CM);
}

void runSensorMode() {

    // One-to-one SENSOR mapping 1: calibrated flex 0..100% -> servo 0..180.
    // Keep the existing median/EMA; suppress tiny servo commands and apply
    // hysteresis near 0/180 so sensor fluctuations cannot chatter the horn.
    int servoRequest = servoFollow.command(
        flexSensor.getForcePercent(), servoAngle, millis());
    if (servoRequest >= 0) writeServo(servoRequest);

    // One-to-one SENSOR mapping 2: ACTUAL calibrated pot endstops -> 0..360.
    // Stepper '0 degrees' is its position when the Arduino booted, NOT a
    // homed mechanical zero. The reported target is step-quantized.
    float potDeg = TSP05Mapping::potToStepper(
        potentiometer.getFilteredADC(), potAdcMin, potAdcMax);
    if (fabs(potDeg - stepperTargetDeg) >= STEPPER_DEADBAND_DEG) {
        stepperMotor.setTargetDegrees(potDeg);
        stepperTargetDeg = stepperMotor.getTargetDegrees();
    }

    // One-to-one SENSOR mapping 3: VL53L1X near/far -> DC target RPM.
    // The Sharp IR sensor is ONLY the independent safety interlock.
    float rpm = roundf(distanceToRPM());
    if (fabs(rpm - dcTargetRPM) >= DC_TARGET_DEADBAND_RPM ||
        (rpm == 0.0f && dcTargetRPM != 0.0f)) {
        dcPositionMode = false;
        dcTargetRPM = rpm;
    }
}

// ====================================
// Serial commands
// ====================================

const int IN_CAPACITY = 40;
char inBuf[IN_CAPACITY];
int inLen = 0;

void handleCommand(char* line) {

    // Split "<CMD> <ARG>"
    char* arg = strchr(line, ' ');
    if (arg != nullptr) {
        *arg++ = '\0';
        while (*arg == ' ') arg++;
    }
    for (char* p = line; *p; p++) {
        *p = toupper(*p);
    }

    if (strcmp(line, "MODE") == 0) {
        if (arg == nullptr) {
            sendAck("MODE", false, "missing argument");
            return;
        }
        for (char* p = arg; *p; p++) *p = toupper(*p);
        if (strcmp(arg, "GUI") == 0) {
            setMode(MODE_GUI);
        } else if (strcmp(arg, "SENSOR") == 0) {
            setMode(MODE_SENSOR);
        } else if (strcmp(arg, "TOGGLE") == 0) {
            setMode(mode == MODE_GUI ? MODE_SENSOR : MODE_GUI);
        } else {
            sendAck("MODE", false, "expected GUI, SENSOR or TOGGLE");
            return;
        }
        sendAck("MODE", true, "");
        return;
    }

    if (strcmp(line, "CAL") == 0) {
        handleCalibration(arg);
        return;
    }
    if (strcmp(line, "POTCAL") == 0) {
        handlePotCalibration(arg);
        return;
    }

    if (strcmp(line, "DCPOL") == 0) {
        handleDcPolarity(arg);
        return;
    }

    if (strcmp(line, "SAFETY") == 0) {
        if (arg == nullptr) {
            sendAck("SAFETY", false, "missing argument");
            return;
        }
        safetyThresholdCm = max(0.0f, (float)atof(arg));
        sendAck("SAFETY", true, "");
        return;
    }

    bool isMotorCmd = strcmp(line, "SERVO") == 0 ||
                      strcmp(line, "DC") == 0 ||
                      strcmp(line, "DCPWM") == 0 ||
                      strcmp(line, "DCPOS") == 0 ||
                      strcmp(line, "STEP") == 0;
    if (!isMotorCmd) {
        sendAck(line, false, "unknown command");
        return;
    }
    if (arg == nullptr) {
        sendAck(line, false, "missing argument");
        return;
    }
    if (mode != MODE_GUI) {
        sendAck(line, false, "not in GUI mode");
        return;
    }
    if (safetyActive) {
        sendAck(line, false, "safety stop active");
        return;
    }

    // Reject malformed / non-finite commands instead of interpreting them
    // as 0 or accepting NaN as a motor target.
    char* endptr;
    float value = strtod(arg, &endptr);
    while (*endptr == ' ') endptr++;
    if (endptr == arg || *endptr != '\0' || isnan(value) || isinf(value)) {
        sendAck(line, false, "expected finite number");
        return;
    }

    if (strcmp(line, "SERVO") == 0) {
        writeServo((int)(value + 0.5f));
    } else if (strcmp(line, "DC") == 0) {
        dcPositionMode = false;
        dcOpenLoop = false;
        dcTargetRPM = constrain(value, -100.0f, 100.0f);
        dcMotor.clearStall();   // explicit command = retry
    } else if (strcmp(line, "DCPOS") == 0) {
        if (!DC_MOTOR_CONNECTED) {
            sendAck(line, false, "DC motor disabled");
            return;
        }
        if (fabs(value) > 3600.0f) {
            sendAck(line, false, "limit: 3600 deg per move");
            return;
        }
        dcOpenLoop = false;
        dcPositionMode = true;
        dcTargetRPM = 0.0f;
        dcMotor.moveRelativeDegrees(value);
    } else if (strcmp(line, "DCPWM") == 0) {
        dcPositionMode = false;
        dcOpenLoop = true;
        dcOpenLoopPWM = constrain((int)value, -230, 230);
        dcOpenLoopStart = millis();
        dcTargetRPM = 0.0f;
    } else {  // STEP: relative to the previous quantized target
        if (fabs(value) > 3600.0f) {
            sendAck(line, false, "limit: 3600 deg per move");
            return;
        }
        float before = stepperMotor.getTargetDegrees();
        stepperMotor.moveDegrees(value);
        stepperTargetDeg = stepperMotor.getTargetDegrees();
        stepLastActualDeg = stepperTargetDeg - before;
    }
    sendAck(line, true, "");
}

void readCommands() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            inBuf[inLen] = '\0';
            if (inLen > 0) {
                handleCommand(inBuf);
            }
            inLen = 0;
        } else if (inLen < IN_CAPACITY - 1) {
            inBuf[inLen++] = c;
        }
    }
}

// ====================================
// State message (written field by field)
// ====================================

void field(const __FlashStringHelper* k) {
    Serial.print(F(",\""));
    Serial.print(k);
    Serial.print(F("\":"));
}

void fieldBool(const __FlashStringHelper* k, bool v) {
    field(k);
    Serial.print(v ? F("true") : F("false"));
}

void fieldStr(const __FlashStringHelper* k, const char* v) {
    field(k);
    Serial.print('"');
    Serial.print(v);
    Serial.print('"');
}

// Writes state field i; returns false when there are no more fields
bool writeStateField(int i) {
    switch (i) {
        case 0:  Serial.print(F("{\"t\":\"state\",\"ms\":")); Serial.print(millis()); break;
        case 1:  fieldStr(F("mode"), modeName()); break;
        case 2:  fieldBool(F("safety"), safetyActive); break;
        case 3:  field(F("safety_cm")); Serial.print(safetyThresholdCm, 0); break;
        case 4:  field(F("flex_raw")); Serial.print(flexSensor.getRawADC()); break;
        case 5:  field(F("flex_adc")); Serial.print(flexSensor.getADC(), 1); break;
        case 6:  field(F("flex_kohm")); Serial.print(flexSensor.readResistance(), 2); break;
        case 7:  field(F("flex_deg")); Serial.print(flexSensor.getBendAngle()); break;
        case 8:  field(F("pot_raw")); Serial.print(potentiometer.getRawADC()); break;
        case 9:  field(F("pot_v")); Serial.print(potentiometer.getVoltage(), 2); break;
        case 10: field(F("pot_pct")); Serial.print(potentiometer.getPositionPercent()); break;
        case 11: fieldBool(F("tof_ok"), distanceSensor.isOk()); break;
        case 12: field(F("tof_mm")); Serial.print(distanceSensor.getDistanceMm()); break;
        case 13: field(F("sharp_v")); Serial.print(sharpSensor.getVoltage(), 2); break;
        case 14: field(F("sharp_cm")); Serial.print(sharpSensor.getDistanceCm()); break;
        case 15: fieldStr(F("sharp_st"), SharpIRSensor::statusToString(sharpSensor.getStatus())); break;
        case 16: fieldBool(F("btn"), button.isPressed()); break;
        case 17: field(F("btn_n")); Serial.print(button.getPressCount()); break;
        case 18: field(F("servo_deg")); Serial.print(servoAngle); break;
        case 19: field(F("step_deg")); Serial.print(stepperMotor.getPositionDegrees(), 1); break;
        case 20: field(F("step_tgt")); Serial.print(stepperTargetDeg, 1); break;
        case 21: fieldBool(F("dc_on"), DC_MOTOR_CONNECTED); break;
        case 22: field(F("dc_tgt")); Serial.print(dcTargetRPM, 1); break;
        case 23: field(F("dc_rpm")); Serial.print(DC_MOTOR_CONNECTED ? dcMotor.getRPM() : 0.0f, 1); break;
        case 24: field(F("flex_force")); Serial.print(flexSensor.getForcePercent(), 1); break;
        case 25: field(F("flex_flat_k")); Serial.print(flexSensor.getFlatOhms() / 1000.0f, 2); break;
        case 26: field(F("flex_bent_k")); Serial.print(flexSensor.getBentOhms() / 1000.0f, 2); break;
        case 27: field(F("dc_pwm")); Serial.print(DC_MOTOR_CONNECTED ? dcMotor.getPWM() : 0); break;
        case 28: field(F("dc_pol")); Serial.print((dcMotor.isMotorInverted() ? 1 : 0) + (dcMotor.isEncoderInverted() ? 2 : 0)); break;
        case 29: fieldBool(F("dc_stall"), DC_MOTOR_CONNECTED && dcMotor.isStalled()); break;
        case 30: field(F("sharp_raw")); Serial.print(sharpSensor.getRawADC()); break;
        case 31: field(F("pot_deg")); Serial.print(potentiometer.getAngleDegrees(), 1); break;
        case 32: field(F("step_move")); Serial.print(stepLastActualDeg, 2); break;
        case 33: field(F("dc_pos")); Serial.print(DC_MOTOR_CONNECTED ? dcMotor.getPositionDegrees() : 0.0f, 2); break;
        case 34: field(F("dc_pos_tgt")); Serial.print(DC_MOTOR_CONNECTED ? dcMotor.getTargetPositionDegrees() : 0.0f, 2); break;
        case 35: fieldBool(F("dc_pos_mode"), dcPositionMode); break;
        case 36: fieldBool(F("dc_pos_done"), DC_MOTOR_CONNECTED && dcMotor.positionReached()); break;
        case 37: field(F("dc_active_rpm")); Serial.print(DC_MOTOR_CONNECTED ? dcMotor.getActiveTargetRPM() : 0.0f, 1); break;
        case 38: field(F("flex_angle_max")); Serial.print(flexSensor.getMaxBendDegrees(), 1); break;
        case 39: field(F("pot_cal_min")); Serial.print(potAdcMin, 0); break;
        case 40: field(F("pot_cal_max")); Serial.print(potAdcMax, 0); break;
        case 41: field(F("pot_cal_flags")); Serial.print((int)potCalFlags); break;
        case 42: field(F("pot_adc")); Serial.print(potentiometer.getFilteredADC(), 1); break;
        case 43: Serial.print("}\n"); break;
        default: return false;
    }
    return true;
}

void drainSerial() {

    // State message in progress: send fields while there is room
    if (stateField >= 0) {
        while (Serial.availableForWrite() >= FIELD_ROOM) {
            if (!writeStateField(stateField)) {
                stateField = -1;
                break;
            }
            stateField++;
        }
        return;
    }

    // Otherwise send queued acks / events
    int room = Serial.availableForWrite();
    while (room-- > 0 && outHead < outTail) {
        Serial.write(outBuf[outHead++]);
    }
    if (queueEmpty() && msgStart < 0) {
        outHead = outTail = 0;
    }
}

// ====================================
// Setup
// ====================================

void setup() {
    Serial.begin(115200);

    // Motors
    if (DC_MOTOR_CONNECTED) {
        dcMotor.begin();
        dcMotor.setPIDGains(1.0, 1.5, 0.02);
        dcMotor.setTargetRPM(0);
        loadDcPolarity();
    }
    stepperMotor.begin();
    stepperMotor.setSpeed(STEPPER_SPEED);
    stepperMotor.setTargetDegrees(0);
    servoMotor.attach(SERVO_PIN);
    writeServo(90);

    // Sensors
    flexSensor.begin();
    loadFlexCalibration();
    sharpSensor.begin();
    potentiometer.begin();
    loadPotCalibration();
    button.begin();
    bool tofOk = distanceSensor.begin();

    Serial.print(F("{\"t\":\"boot\""));
    fieldBool(F("tof_ok"), tofOk);
    fieldBool(F("dc_on"), DC_MOTOR_CONNECTED);
    fieldStr(F("mode"), modeName());
    Serial.print("}\n");

    lastStateSend = millis();
}

// ====================================
// Main loop
// ====================================

void loop() {

    // 1. Read all sensors
    distanceSensor.update();
    flexSensor.update();
    sharpSensor.update();
    potentiometer.update();
    button.update();

    // 2. Mode selection: hardware button or serial command
    if (button.wasPressed()) {
        setMode(mode == MODE_GUI ? MODE_SENSOR : MODE_GUI);
    }
    readCommands();

    // 3. Safety interlock
    updateSafety();

    // 4. Control motors
    if (mode == MODE_SENSOR && !safetyActive) {
        runSensorMode();
    }
    if (safetyActive) {
        dcTargetRPM = 0.0f;
        dcPositionMode = false;
        if (DC_MOTOR_CONNECTED) dcMotor.emergencyStop();
    }
    if (dcOpenLoop && millis() - dcOpenLoopStart >= DC_OPEN_LOOP_MS) {
        dcOpenLoop = false;   // open-loop test auto-stop
    }
    if (DC_MOTOR_CONNECTED) {
        if (dcOpenLoop) {
            dcMotor.setOpenLoopPWM(dcOpenLoopPWM);
        } else {
            if (dcMotor.isOpenLoop()) {
                dcMotor.setClosedLoop();
            }
            if (!dcPositionMode) dcMotor.setTargetRPM(dcTargetRPM);
        }
        dcMotor.update();
        static bool wasStalled = false;
        if (dcMotor.isStalled() && !wasStalled) {
            sendEvent("dc_stall");
        }
        wasStalled = dcMotor.isStalled();
    }
    stepperMotor.update();

    // 5. Publish state to ROS 2
    // Start a new state message when due and nothing else is being sent
    if (millis() - lastStateSend >= STATE_INTERVAL_MS &&
        stateField < 0 && queueEmpty()) {
        lastStateSend = millis();
        stateField = 0;
    }
    drainSerial();
}
