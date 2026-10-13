#include <Arduino.h>
#include <Dynamixel2Arduino.h>

using namespace ControlTableItem;

Dynamixel2Arduino dxl(Serial3, 84);

// Initial search covers the two likely baud rates.
// A negative result does not rule out other baud rates or protocols.
const uint32_t BAUDS[] = {57600, 1000000};

bool transactionOK() {
  return dxl.getLastLibErrCode() == 0 &&
         dxl.getLastStatusPacketError() == 0;
}

bool printItem(uint8_t id, uint8_t item, const char* name) {
  int32_t value = dxl.readControlTableItem(item, id);
  if (!transactionOK()) {
    Serial.print("READ_FAILED,");
    Serial.println(name);
    return false;
  }
  Serial.print(name);
  Serial.print('=');
  Serial.println(value);
  return true;
}

void scanMotor() {
  Serial.println("SCAN_BEGIN,CONNECT_ONLY_ONE_MOTOR");
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  delay(500);

  unsigned int found = 0;

  for (unsigned int b = 0; b < sizeof(BAUDS) / sizeof(BAUDS[0]); ++b) {
    dxl.begin(BAUDS[b]);
    dxl.setPortProtocolVersion(2.0);
    delay(100);

    Serial.print("SCANNING_BAUD=");
    Serial.println(BAUDS[b]);

    for (unsigned int id = 0; id <= 252; ++id) {
      if (!dxl.ping((uint8_t)id)) continue;

      ++found;
      Serial.print("FOUND,ID=");
      Serial.print(id);
      Serial.print(",BAUD=");
      Serial.println(BAUDS[b]);

      // Request torque-off only. Never command movement.
      dxl.torqueOff((uint8_t)id);

      int32_t torque = dxl.readControlTableItem(
          TORQUE_ENABLE, (uint8_t)id);
      if (!transactionOK() || torque != 0) {
        digitalWrite(BDPIN_DXL_PWR_EN, LOW);
        Serial.println("FAULT,TORQUE_OFF_UNVERIFIED,POWER_DISABLE_REQUESTED");
        return;
      }

      Serial.println("TORQUE_OFF_VERIFIED");

      bool ok = true;
      ok &= printItem(id, MODEL_NUMBER, "MODEL_NUMBER");
      ok &= printItem(id, FIRMWARE_VERSION, "FIRMWARE_VERSION");
      ok &= printItem(id, ID, "ID");
      ok &= printItem(id, BAUD_RATE, "BAUD_RATE_REGISTER");
      ok &= printItem(id, OPERATING_MODE, "OPERATING_MODE");
      ok &= printItem(id, CURRENT_LIMIT, "CURRENT_LIMIT_RAW");
      ok &= printItem(id, PWM_LIMIT, "PWM_LIMIT_RAW");
      ok &= printItem(id, PRESENT_POSITION, "PRESENT_POSITION_RAW");
      ok &= printItem(id, PRESENT_INPUT_VOLTAGE, "INPUT_VOLTAGE_RAW");
      ok &= printItem(id, PRESENT_TEMPERATURE, "TEMPERATURE_C");

      if (!ok) {
        digitalWrite(BDPIN_DXL_PWR_EN, LOW);
        Serial.println("FAULT,CONFIG_READ_FAILED,POWER_DISABLE_REQUESTED");
        return;
      }
    }
  }

  digitalWrite(BDPIN_DXL_PWR_EN, LOW);
  Serial.print("SCAN_DONE,RESPONSES=");
  Serial.println(found);
  Serial.println("DXL_POWER_DISABLE_REQUESTED");
}

void setup() {
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, LOW);

  Serial.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);
  digitalWrite(BDPIN_DXL_PWR_EN, LOW);
}

void loop() {
  if (Serial.available()) {
    char command = (char)Serial.read();
    if (command == 'H') {
      Serial.println("READY,motor-discovery-v1,S=SCAN,H=HELLO");
    } else if (command == 'S') {
      scanMotor();
    }
  }
}
