#include "Arduino.h"
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#include <app_routing.h>
#include <messages.h>
#include <node_config.h>
#include <rx.h>
#include <tx.h>
#include <util.h>

#include <SPI.h>
#include <RH_RF95.h>

#define RED_LED 7
#define GREEN_LED 8
#define BLINK_DELAY 500


// Singleton instance of the radio driver
RH_RF95 rf95(10, 2);
AppRouter router;

void setup()
{
  //Setup Green and Red LED as output
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  //Start serial port and wait for it to be available
//  Serial.begin(9600);
//  while (!Serial) ;

  //Initialise the LoRa RFM95 module
  if (!rf95.init()) {
//    Serial.println("init failed");
    blink(RED_LED,2);
    return;
  }

  //Configure Power and Frequency of LoRa RFM95 module
  rf95.setFrequency(915.0);
  rf95.setTxPower(23, true);

  blink(GREEN_LED,2);
//  Serial.println("Intialisation Complete.");

}

void loop()
{
  sendMessageToGateway(HEARTBEAT_MESSAGE);
}


void sendMessageToGateway(uint8_t messageType){

  if (messageType > 4 || messageType < 0){
//      Serial.println("Illegal Message Type. Check messages.h for message types.");
  }
  
  uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
  uint16_t destID = router.getBestDestination();

  if( messageType == VIBRATION_SIGNATURE_MESSAGE ){
      Tx::constructVibrationMessage(data, destID);
//      Serial.println ("Sending Vibration Signature . . ");
  }

  if (messageType == HEARTBEAT_MESSAGE){
      Tx::constructHeartbeatMessage(data, destID);
//      Serial.println ("Sending Heartbeat . . ");
  }

  rf95.setModeTx();
  rf95.send(data, sizeof(data));
  rf95.waitPacketSent();
//  Serial.println("Message sent");
}

void blink(int LED, int n_times) {

  for (int i = 0; i < n_times; i++) {
    digitalWrite(LED, HIGH);
    delay(BLINK_DELAY);
    digitalWrite(LED, LOW);
    delay(BLINK_DELAY);
  }
}
