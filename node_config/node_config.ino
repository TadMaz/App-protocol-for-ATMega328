
#include <EEPROM.h>

// change this to be the ID of your node in the mesh network
uint8_t NODE_ID = 1;
uint8_t DISTANCE_FROM_GATEWAY = 100

void setup() {
  Serial.begin(9600);
  while (!Serial) ; // Wait for serial port to be available

  Serial.println("setting nodeId...");

  EEPROM.write(0, NODE_ID);
  EEPROM.write(1, DISTANCE_FROM_GATEWAY);
  
  Serial.print(F("set nodeId = "));
  Serial.println(NODE_ID);
  Serial.print(F("set distance to gateway = "));
  Serial.println(DISTANCE_FROM_GATEWAY);
  
  uint8_t readVal = EEPROM.read(0);
  Serial.print(F("read nodeId: "));
  Serial.println(readVal);

  if (nodeId != readVal) {
    Serial.println(F("*** FAIL ***"));
  } else {
    Serial.println(F("SUCCESS"));
  }
}

void loop() {

}
