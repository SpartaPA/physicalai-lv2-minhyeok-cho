// Two motor load-PROXY experiment: OpenCR 1.0 + XM430-W350-T (ID1) + XM430-W210-T (ID2)
// External bus current is NOT measured by this sketch.
// USB serial: 115200. DXL TTL: 57600, Protocol 2.0. Boots disarmed.
// Commands: HELLO, STATUS, ARM, RUN IDLE|STEER|DRIVE|BOTH, HB, STOP, DISARM
// Telemetry: TEL,ms,mode,steer_mA,drive_mA,steer_V,drive_V,
//            steer_pos_raw,drive_vel_raw,steer_temp_C,drive_temp_C,steer_pwm_raw,drive_pwm_raw
#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <math.h>
#include <string.h>
using namespace ControlTableItem;

constexpr uint8_t STEER_ID = 1;       // XM430-W350
constexpr uint8_t DRIVE_ID = 2;       // XM430-W210
constexpr uint32_t DXL_BAUD = 57600;  // Both motors must already use this baud
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
void haltMotion() {
  mode = IDLE;
  if (availableMotors) {
    dxl.setGoalVelocity(DRIVE_ID, 0, UNIT_RAW);
    dxl.torqueOff(DRIVE_ID);
    dxl.torqueOff(STEER_ID);
  }
  armed = false;
}
void fault(const char* reason) {
  haltMotion();
  Serial.print("FAULT,"); Serial.println(reason);
}
bool setEEPROMIfDifferent(uint8_t id, uint8_t item, int32_t desired) {
  int32_t original = dxl.readControlTableItem(item, id);
  Serial.print("CFG,"); Serial.print(id); Serial.print(',');
  Serial.print(item); Serial.print(','); Serial.print(original);
  Serial.print(','); Serial.println(desired);
  if (original == desired) return true;
  return dxl.writeControlTableItem(item, id, desired);
}
void announce() {
  Serial.println("READY,opencr-two-motor-telemetry-v1,STEER_ID=1,DRIVE_ID=2");
  Serial.print("STATUS,"); Serial.print(availableMotors ? "MOTORS_OK" : "MOTORS_MISSING");
  Serial.print(','); Serial.print(armed ? "ARMED" : "DISARMED");
  Serial.print(','); Serial.println(modeName());
}
void configureMotors() {
  // Request torque OFF on any reachable motor, even if the other is missing.
  dxl.torqueOff(STEER_ID);
  dxl.torqueOff(DRIVE_ID);
  // Never start if both uniquely addressed motors cannot be found.
  if (!dxl.ping(STEER_ID) || !dxl.ping(DRIVE_ID)) {
    Serial.println("FAULT,MISSING_ID1_OR_ID2_CHECK_IDS_BAUD_CABLES");
    return;
  }
  availableMotors = true;
  dxl.torqueOff(STEER_ID);
  dxl.torqueOff(DRIVE_ID);
  bool ok = true;
  // EEPROM writes persist across power cycles. Original values logged via CFG.
  ok &= setEEPROMIfDifferent(STEER_ID, CURRENT_LIMIT, MOTOR_CURRENT_LIMIT_RAW);
  ok &= setEEPROMIfDifferent(DRIVE_ID, CURRENT_LIMIT, MOTOR_CURRENT_LIMIT_RAW);
  ok &= setEEPROMIfDifferent(STEER_ID, PWM_LIMIT, MOTOR_PWM_LIMIT_RAW);
  ok &= setEEPROMIfDifferent(DRIVE_ID, PWM_LIMIT, MOTOR_PWM_LIMIT_RAW);
  ok &= dxl.setOperatingMode(STEER_ID, OP_CURRENT_BASED_POSITION);
  ok &= dxl.setOperatingMode(DRIVE_ID, OP_VELOCITY);
  if (!ok) {fault("CONFIG_FAILED"); availableMotors = false; return;}
  steerCenter = (int32_t)dxl.getPresentPosition(STEER_ID, UNIT_RAW);
  if (steerCenter < 300 || steerCenter > 3795) {
    fault("STEER_CENTER_OUTSIDE_SAFE_MARGIN");
    availableMotors = false;
    return;
  }
  Serial.print("CFG,CENTER_TICKS,"); Serial.println(steerCenter);
  Serial.println("INFO,BOOT_DISARMED_TORQUE_OFF");
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
  float cs = dxl.getPresentCurrent(STEER_ID, UNIT_MILLI_AMPERE);
  float cd = dxl.getPresentCurrent(DRIVE_ID, UNIT_MILLI_AMPERE);
  float vs = 0.1f * dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, STEER_ID);
  float vd = 0.1f * dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, DRIVE_ID);
  int32_t ps = dxl.readControlTableItem(PRESENT_POSITION, STEER_ID);
  float vdRaw = dxl.getPresentVelocity(DRIVE_ID, UNIT_RAW);
  int32_t ts = dxl.readControlTableItem(PRESENT_TEMPERATURE, STEER_ID);
  int32_t td = dxl.readControlTableItem(PRESENT_TEMPERATURE, DRIVE_ID);
  int32_t pws = dxl.readControlTableItem(PRESENT_PWM, STEER_ID);
  int32_t pwd = dxl.readControlTableItem(PRESENT_PWM, DRIVE_ID);
  // Sanity validation identifies grossly invalid reads but not every bus error.
  if (vs < MIN_MOTOR_V || vd < MIN_MOTOR_V || vs > MAX_MOTOR_V || vd > MAX_MOTOR_V) {
    if (++invalidVoltageSamples >= 2 && armed) { fault("VOLTAGE_OUT_OF_RANGE_OR_READ_FAILED"); return; }
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
void handle(const char* cmd) {
  if (!strcmp(cmd, "HELLO") || !strcmp(cmd, "STATUS")) { announce(); return; }
  if (!strcmp(cmd, "DISARM") || !strcmp(cmd, "STOP")) {
    haltMotion(); Serial.println("ACK,DISARMED"); return;
  }
  if (!strcmp(cmd, "ARM")) {
    if (!availableMotors) { Serial.println("ERR,MOTORS_MISSING"); return; }
    if (!dxl.ping(STEER_ID) || !dxl.ping(DRIVE_ID)) {fault("PING_FAILED"); return;}
    float v1 = 0.1f * dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, STEER_ID);
    float v2 = 0.1f * dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, DRIVE_ID);
    int32_t t1 = dxl.readControlTableItem(PRESENT_TEMPERATURE, STEER_ID);
    int32_t t2 = dxl.readControlTableItem(PRESENT_TEMPERATURE, DRIVE_ID);
    if (v1 < MIN_MOTOR_V || v2 < MIN_MOTOR_V ||
        v1 > MAX_MOTOR_V || v2 > MAX_MOTOR_V ||
        t1 >= MAX_TEMP_C || t2 >= MAX_TEMP_C) {
      fault("ARM_PRECHECK_VOLTAGE_OR_TEMPERATURE"); return;
    }
    // Re-latch the center at the physical angle reached while disarmed.
    steerCenter = (int32_t)dxl.getPresentPosition(STEER_ID, UNIT_RAW);
    if (steerCenter < 300 || steerCenter > 3795) {fault("UNSAFE_CENTER"); return;}
    bool goalsOk = true;
    goalsOk &= dxl.setGoalCurrent(STEER_ID, STEER_GOAL_CURRENT_RAW, UNIT_RAW);
    goalsOk &= dxl.setGoalPosition(STEER_ID, steerCenter, UNIT_RAW);
    goalsOk &= dxl.setGoalVelocity(DRIVE_ID, 0, UNIT_RAW);
    if (!goalsOk) {
      fault("ARM_GOAL_WRITE_FAILED");
      return;
    }
    if (!dxl.torqueOn(STEER_ID) || !dxl.torqueOn(DRIVE_ID)) {fault("TORQUE_ENABLE_FAILED"); return;}
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
  delay(200);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  delay(350);
  configureMotors();
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
