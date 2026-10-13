// ONE-TIME MOTOR ID PROVISIONING. Connect ONLY the W210 motor to the OpenCR.
// Change the TARGET_ID only after confirming your motor currently responds at SOURCE_ID.
// Flash this program, inspect USB serial at 115200, and remove power before reconnecting.
#include <Dynamixel2Arduino.h>
using namespace ControlTableItem;
Dynamixel2Arduino dxl(Serial3, 84); // OpenCR TTL bus
constexpr uint8_t SOURCE_ID = 1;
constexpr uint8_t TARGET_ID = 2;
constexpr uint32_t DXL_BAUD = 57600;

void setup() {
  Serial.begin(115200);
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, LOW);
  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(2.0);
  delay(250);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  delay(500);
  Serial.println("ID_SETUP: Only the W210 must be connected; all torque is OFF.");
  if (!dxl.ping(SOURCE_ID)) {
    Serial.println(dxl.ping(TARGET_ID)
        ? "ID_SETUP_OK: Motor already responds at ID 2; nothing changed."
        : "ID_SETUP_ERROR: Neither ID 1 nor ID 2 found; nothing changed.");
    return;
  }
  dxl.torqueOff(SOURCE_ID);
  const bool ok = dxl.writeControlTableItem(ID, SOURCE_ID, TARGET_ID);
  delay(100);
  Serial.println(ok && dxl.ping(TARGET_ID)
      ? "ID_SETUP_OK: W210 is now ID 2. Power off before connecting the W350."
      : "ID_SETUP_ERROR: ID change not confirmed. Check only one motor connected.");
}
void loop() { delay(1000); }
