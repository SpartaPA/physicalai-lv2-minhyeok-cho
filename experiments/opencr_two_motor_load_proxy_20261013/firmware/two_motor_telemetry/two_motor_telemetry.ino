// FOUR_CONNECTED_ONE_ACTIVE_PAIR_V2
// Active: W350 ID11 + W210 ID1. Inactive: W350 ID12 + W210 ID2.
// All four must be connected and pass verification.
// External bus current is NOT measured by this sketch.
// USB serial: 115200. DXL TTL: 1000000, Protocol 2.0. Boots disarmed.
// Commands: HELLO, STATUS, ARM, RUN IDLE|STEER|DRIVE|BOTH, HB, STOP, DISARM
// Telemetry: TEL,ms,mode,steer_mA,drive_mA,steer_V,drive_V,
//            steer_pos_raw,drive_vel_raw,steer_temp_C,drive_temp_C,steer_pwm_raw,drive_pwm_raw
#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <math.h>
#include <string.h>
using namespace ControlTableItem;

constexpr uint8_t STEER_ID = 11;      // Active XM430-W350
constexpr uint8_t DRIVE_ID = 1;       // Active XM430-W210
constexpr uint8_t ALL_IDS[] = {11, 1, 12, 2};
constexpr int32_t EXPECTED_MODELS[] = {1020, 1030, 1020, 1030};
constexpr uint32_t DXL_BAUD = 1000000; // Verified discovery baud
constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t HOST_TIMEOUT_MS = 1200;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 200; // target 5Hz; actual rate measured on Pi
constexpr uint32_t MOTION_INTERVAL_MS = 50;
constexpr float STEER_AMPLITUDE_TICKS = 60.0f; // about 5.3 degrees motor shaft
constexpr uint32_t STEER_PERIOD_MS = 4000;
constexpr float DRIVE_VELOCITY_RAW = 15.0f; // 15*0.229 = ~3.4 motor rpm
constexpr int32_t MOTOR_CURRENT_LIMIT_RAW = 250; // ~673mA; NOT a guaranteed supply-current limit
constexpr int32_t MOTOR_PWM_LIMIT_RAW = 180;     // ~20.3% output PWM
constexpr int32_t STEER_GOAL_CURRENT_RAW = 180; // ~484mA, in current-based position mode
constexpr float MIN_MOTOR_V = 10.0f;
constexpr float MAX_MOTOR_V = 14.8f;
constexpr int MAX_TEMP_C = 60;
constexpr float PI2 = 6.28318530718f;

Dynamixel2Arduino dxl(Serial3, 84); // OpenCR uses Serial3 and DIR pin 84
bool availableMotors = false;
bool armed = false;
uint32_t lastHeartbeat = 0, lastMotion = 0, lastTelemetry = 0, lastPing = 0, phaseStart = 0;
uint8_t invalidVoltageSamples = 0;
int32_t steerCenter = 2048;
char input[48];
uint8_t inputLen = 0;
enum Mode { IDLE, STEER, DRIVE, BOTH };
Mode mode = IDLE;

const char* modeName() {
  switch (mode) {case STEER: return "STEER"; case DRIVE: return "DRIVE";
                 case BOTH: return "BOTH"; default: return "IDLE";}
}
// CHECKED_READ_AND_STOP_V1
bool readError = false;
bool powerCutLatched = false;

bool lastTransactionOK() {
  return dxl.getLastLibErrCode() == 0 &&
         dxl.getLastStatusPacketError() == 0;
}

int32_t readChecked(uint8_t item, uint8_t id) {
  int32_t value = dxl.readControlTableItem(item, id);
  if (!lastTransactionOK()) {
    readError = true;
    Serial.print("READ_ERROR,ID=");
    Serial.print(id);
    Serial.print(",ITEM=");
    Serial.print(item);
    Serial.print(",LIB=");
    Serial.print((int)dxl.getLastLibErrCode());
    Serial.print(",STATUS=");
    Serial.println((int)dxl.getLastStatusPacketError());
  }
  return value;
}


bool haltMotion() {
  armed = false;
  mode = IDLE;

  // Request torque-off on every motor before doing verification reads.
  for (uint8_t id : ALL_IDS) {
    dxl.torqueOff(id);
  }

  bool allOff = true;
  for (uint8_t id : ALL_IDS) {
    int32_t torque = dxl.readControlTableItem(TORQUE_ENABLE, id);
    bool verified = lastTransactionOK() && torque == 0;
    if (!verified) {
      allOff = false;
      Serial.print("STOP_UNVERIFIED,ID=");
      Serial.println(id);
    }
  }

  if (!allOff) {
    digitalWrite(BDPIN_DXL_PWR_EN, LOW);
    powerCutLatched = true;
    availableMotors = false;
    Serial.println("STOP_UNVERIFIED,DXL_POWER_DISABLE_REQUESTED,RESET_REQUIRED");
  }
  return allOff;
}
void fault(const char* reason) {
  availableMotors = false; // Any fault requires reset and startup checks.
  haltMotion();
  Serial.print("FAULT,"); Serial.println(reason);
}

bool setEEPROMIfDifferent(uint8_t id, uint8_t item, int32_t desired) {
  if (readError) return false;
  int32_t original = readChecked(item, id);
  if (readError) return false;

  Serial.print("CFG,"); Serial.print(id); Serial.print(',');
  Serial.print(item); Serial.print(','); Serial.print(original);
  Serial.print(','); Serial.println(desired);

  if (original != desired) {
    bool written = dxl.writeControlTableItem(item, id, desired);
    if (!written || !lastTransactionOK()) return false;
  }
  int32_t actual = readChecked(item, id);
  return !readError && actual == desired;
}

// Read all four torque states, supply voltages, temperatures and HW errors.
// Expected torque: active pair on only while armed; inactive pair always off.
bool verifyFourMotorState(bool activeOn) {
  for (uint8_t id : ALL_IDS) {
    bool active = id == STEER_ID || id == DRIVE_ID;
    int32_t expectedTorque = activeOn && active ? 1 : 0;
    int32_t torque = readChecked(TORQUE_ENABLE, id);
    float voltage = 0.1f * readChecked(PRESENT_INPUT_VOLTAGE, id);
    int32_t temperature = readChecked(PRESENT_TEMPERATURE, id);
    int32_t hwError = readChecked(HARDWARE_ERROR_STATUS, id);

    if (readError || torque != expectedTorque ||
        voltage < MIN_MOTOR_V || voltage > MAX_MOTOR_V ||
        temperature >= MAX_TEMP_C || hwError != 0) {
      Serial.print("CHECK_FAILED,ID="); Serial.print(id);
      Serial.print(",TORQUE="); Serial.print(torque);
      Serial.print(",EXPECTED="); Serial.print(expectedTorque);
      Serial.print(",V="); Serial.print(voltage);
      Serial.print(",TEMP="); Serial.print(temperature);
      Serial.print(",HW="); Serial.println(hwError);
      return false;
    }
  }
  return true;
}
void announce() {
  Serial.println("READY,opencr-two-motor-telemetry-v3,STEER_ID=11,DRIVE_ID=1,INACTIVE_IDS=12|2");
  Serial.print("STATUS,"); Serial.print(availableMotors ? "MOTORS_OK" : "MOTORS_MISSING");
  Serial.print(','); Serial.print(armed ? "ARMED" : "DISARMED");
  Serial.print(','); Serial.println(modeName());
}

void configureMotors() {
  availableMotors = false;
  if (!haltMotion()) {
    Serial.println("FAULT,BOOT_TORQUE_OFF_NOT_VERIFIED");
    return;
  }

  readError = false;
  for (unsigned int i = 0; i < sizeof(ALL_IDS) / sizeof(ALL_IDS[0]); ++i) {
    uint8_t id = ALL_IDS[i];
    int32_t model = readChecked(MODEL_NUMBER, id);
    int32_t secondary = (uint8_t)readChecked(SECONDARY_ID, id);
    Serial.print("INVENTORY,ID="); Serial.print(id);
    Serial.print(",MODEL="); Serial.print(model);
    Serial.print(",SECONDARY_ID="); Serial.println(secondary);

    // Require Secondary ID addressing to be disabled on every motor.
    if (readError || model != EXPECTED_MODELS[i] ||
        secondary < 253 || secondary > 255) {
      fault("MODEL_OR_SECONDARY_ID_CHECK_FAILED");
      return;
    }
  }

  if (!verifyFourMotorState(false)) {
    fault("BOOT_FOUR_MOTOR_CHECK_FAILED");
    return;
  }

  // Configure only the active pair. Inactive EEPROM is left unchanged.
  // Mode changes must precede limits because they can reset PWM settings.
  if (!setEEPROMIfDifferent(STEER_ID, OPERATING_MODE, 5) ||
      !setEEPROMIfDifferent(DRIVE_ID, OPERATING_MODE, 1) ||
      !setEEPROMIfDifferent(STEER_ID, CURRENT_LIMIT, MOTOR_CURRENT_LIMIT_RAW) ||
      !setEEPROMIfDifferent(DRIVE_ID, CURRENT_LIMIT, MOTOR_CURRENT_LIMIT_RAW) ||
      !setEEPROMIfDifferent(STEER_ID, PWM_LIMIT, MOTOR_PWM_LIMIT_RAW) ||
      !setEEPROMIfDifferent(DRIVE_ID, PWM_LIMIT, MOTOR_PWM_LIMIT_RAW)) {
    fault("CONFIG_FAILED");
    return;
  }

  steerCenter = readChecked(PRESENT_POSITION, STEER_ID);
  if (readError || steerCenter < 300 || steerCenter > 3795) {
    fault("STEER_CENTER_OUTSIDE_SAFE_MARGIN");
    return;
  }

  availableMotors = true;
  Serial.print("CFG,CENTER_TICKS,"); Serial.println(steerCenter);
  Serial.println("INFO,BOOT_DISARMED_ALL_FOUR_TORQUE_OFF");
}
void updateMotion(uint32_t now) {
  if (!armed || !availableMotors) return;
  if ((uint32_t)(now - lastMotion) < MOTION_INTERVAL_MS) return;
  lastMotion = now;
  float target = (float)steerCenter;
  if (mode == STEER || mode == BOTH) {
    float angle = PI2 * (float)((now - phaseStart) % STEER_PERIOD_MS) / (float)STEER_PERIOD_MS;
    target += STEER_AMPLITUDE_TICKS * sinf(angle);
  }
  const float vel = (mode == DRIVE || mode == BOTH) ? DRIVE_VELOCITY_RAW : 0.0f;
  if (!dxl.setGoalPosition(STEER_ID, target, UNIT_RAW) ||
      !dxl.setGoalVelocity(DRIVE_ID, vel, UNIT_RAW)) {
    fault("WRITE_FAILED");
  }
}
void emitTelemetry(uint32_t now) {
  if (!availableMotors || (uint32_t)(now - lastTelemetry) < TELEMETRY_INTERVAL_MS) return;
  lastTelemetry = now;
  readError = false;
  if (!verifyFourMotorState(armed)) {
    fault("FOUR_MOTOR_STATE_CHECK_FAILED");
    return;
  }
  Serial.println("GUARD,INACTIVE_12=OFF,INACTIVE_2=OFF");
  float cs = (2.69f * (int16_t)readChecked(PRESENT_CURRENT, STEER_ID));
  float cd = (2.69f * (int16_t)readChecked(PRESENT_CURRENT, DRIVE_ID));
  float vs = 0.1f * readChecked(PRESENT_INPUT_VOLTAGE, STEER_ID);
  float vd = 0.1f * readChecked(PRESENT_INPUT_VOLTAGE, DRIVE_ID);
  int32_t ps = readChecked(PRESENT_POSITION, STEER_ID);
  float vdRaw = readChecked(PRESENT_VELOCITY, DRIVE_ID);
  int32_t ts = readChecked(PRESENT_TEMPERATURE, STEER_ID);
  int32_t td = readChecked(PRESENT_TEMPERATURE, DRIVE_ID);
  int32_t pws = (int16_t)readChecked(PRESENT_PWM, STEER_ID);
  int32_t pwd = (int16_t)readChecked(PRESENT_PWM, DRIVE_ID);
  if (readError) {
    fault("TELEMETRY_READ_FAILED");
    availableMotors = false;  // Require reset before another trial.
    return;
  }
  // Register reads succeeded; now validate operating conditions.
  if (vs < MIN_MOTOR_V || vd < MIN_MOTOR_V || vs > MAX_MOTOR_V || vd > MAX_MOTOR_V) {
    if (invalidVoltageSamples < 2) ++invalidVoltageSamples;
    if (invalidVoltageSamples >= 2 && armed) {
      fault("VOLTAGE_OUT_OF_RANGE_OR_READ_FAILED");
      return;
    }
  } else invalidVoltageSamples = 0;
  if ((ts >= MAX_TEMP_C || td >= MAX_TEMP_C) && armed) {
    fault("TEMPERATURE_LIMIT"); return;
  }
  Serial.print("TEL,"); Serial.print(now); Serial.print(','); Serial.print(modeName());
  Serial.print(','); Serial.print(cs, 2); Serial.print(','); Serial.print(cd, 2);
  Serial.print(','); Serial.print(vs, 2); Serial.print(','); Serial.print(vd, 2);
  Serial.print(','); Serial.print(ps); Serial.print(','); Serial.print(vdRaw, 2);
  Serial.print(','); Serial.print(ts); Serial.print(','); Serial.print(td);
  Serial.print(','); Serial.print(pws); Serial.print(','); Serial.println(pwd);
}
// EXPLICIT_INIT_V3
bool initAttempted = false;

void handle(const char* cmd) {
  if (!strcmp(cmd, "INIT")) {
    if (armed) {
      Serial.println("ERR,DISARM_FIRST");
      return;
    }
    if (powerCutLatched || initAttempted) {
      Serial.println("ERR,RESET_REQUIRED");
      return;
    }
    initAttempted = true;
    Serial.println("INIT,BEGIN");
    digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
    delay(500);
    configureMotors();
    Serial.println(availableMotors ? "INIT,OK" : "INIT,FAILED");
    announce();
    return;
  }
  if (!strcmp(cmd, "HELLO") || !strcmp(cmd, "STATUS")) { announce(); return; }
  if (!strcmp(cmd, "DISARM") || !strcmp(cmd, "STOP")) {
    if (haltMotion()) {
      Serial.println("ACK,DISARMED");
    } else {
      Serial.println("FAULT,DISARM_NOT_VERIFIED");
    }
    return;
  }
  if (!strcmp(cmd, "ARM")) {
    if (powerCutLatched) {
      Serial.println("ERR,RESET_REQUIRED");
      return;
    }
    if (armed) {
      Serial.println("ERR,ALREADY_ARMED");
      return;
    }
    readError = false;
    if (!availableMotors) { Serial.println("ERR,RESET_REQUIRED"); return; }
    if (!verifyFourMotorState(false)) {
      fault("ARM_FOUR_MOTOR_CHECK_FAILED");
      return;
    }
    if (!dxl.ping(STEER_ID) || !dxl.ping(DRIVE_ID)) {fault("PING_FAILED"); return;}
    float v1 = 0.1f * readChecked(PRESENT_INPUT_VOLTAGE, STEER_ID);
    float v2 = 0.1f * readChecked(PRESENT_INPUT_VOLTAGE, DRIVE_ID);
    int32_t t1 = readChecked(PRESENT_TEMPERATURE, STEER_ID);
    int32_t t2 = readChecked(PRESENT_TEMPERATURE, DRIVE_ID);
    if (readError || v1 < MIN_MOTOR_V || v2 < MIN_MOTOR_V ||
        v1 > MAX_MOTOR_V || v2 > MAX_MOTOR_V ||
        t1 >= MAX_TEMP_C || t2 >= MAX_TEMP_C) {
      fault("ARM_PRECHECK_VOLTAGE_OR_TEMPERATURE"); return;
    }
    // Re-latch the center at the physical angle reached while disarmed.
    steerCenter = readChecked(PRESENT_POSITION, STEER_ID);
    if (readError || steerCenter < 300 || steerCenter > 3795) {fault("UNSAFE_CENTER"); return;}
    bool goalsOk = true;
    goalsOk &= dxl.writeControlTableItem(GOAL_PWM, STEER_ID, MOTOR_PWM_LIMIT_RAW);
    goalsOk &= dxl.writeControlTableItem(GOAL_PWM, DRIVE_ID, MOTOR_PWM_LIMIT_RAW);
    goalsOk &= dxl.setGoalCurrent(STEER_ID, STEER_GOAL_CURRENT_RAW, UNIT_RAW);
    goalsOk &= dxl.setGoalPosition(STEER_ID, steerCenter, UNIT_RAW);
    goalsOk &= dxl.setGoalVelocity(DRIVE_ID, 0, UNIT_RAW);
    if (!goalsOk) {
      fault("ARM_GOAL_WRITE_FAILED");
      return;
    }
    if (!dxl.torqueOn(STEER_ID) || !dxl.torqueOn(DRIVE_ID)) {fault("TORQUE_ENABLE_FAILED"); return;}
    readError = false;
    if (!verifyFourMotorState(true)) {
      fault("ARM_TORQUE_STATE_NOT_VERIFIED");
      return;
    }
    armed = true; mode = IDLE; phaseStart = millis(); lastHeartbeat = millis();
    Serial.println("ACK,ARMED"); return;
  }
  if (!strcmp(cmd, "HB")) {
    if (armed) lastHeartbeat = millis();
    return;
  }
  if (!strncmp(cmd, "RUN ", 4)) {
    if (!armed) { Serial.println("ERR,DISARMED"); return; }
    const char* arg = cmd + 4;
    if (!strcmp(arg, "IDLE")) mode = IDLE;
    else if (!strcmp(arg, "STEER")) mode = STEER;
    else if (!strcmp(arg, "DRIVE")) mode = DRIVE;
    else if (!strcmp(arg, "BOTH")) mode = BOTH;
    else { Serial.println("ERR,BAD_MODE"); return; }
    phaseStart = millis(); lastHeartbeat = millis();
    Serial.print("ACK,RUN,"); Serial.println(modeName()); return;
  }
  Serial.println("ERR,UNKNOWN_COMMAND");
}
void pollUSB() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      input[inputLen] = '\0';
      if (inputLen) handle(input);
      inputLen = 0;
    } else if (inputLen < sizeof(input) - 1) {
      input[inputLen++] = c;
    } else {
      inputLen = 0;
      Serial.println("ERR,LINE_OVERFLOW");
    }
  }
}
void setup() {
  Serial.begin(USB_BAUD);
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, LOW);
  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(2.0);
  digitalWrite(BDPIN_DXL_PWR_EN, LOW);
  Serial.println("INFO,WAITING_FOR_INIT,MOTOR_POWER_DISABLE_REQUESTED");
  announce();
}
void loop() {
  pollUSB();
  uint32_t now = millis();
  if (armed && (uint32_t)(now - lastHeartbeat) > HOST_TIMEOUT_MS) {
    fault("HOST_HEARTBEAT_TIMEOUT");
  }
  if (availableMotors && armed && (uint32_t)(now - lastPing) > 1000) {
    lastPing = now;
    if (!dxl.ping(STEER_ID) || !dxl.ping(DRIVE_ID)) fault("MOTOR_PING_FAILED");
  }
  updateMotion(now);
  emitTelemetry(now);
}
