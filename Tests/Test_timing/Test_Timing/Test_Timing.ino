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
#define BLINK_DELAY 100
#define RX_WAIT_TIME 10000

// Initial System Timing
unsigned long startTime = millis();
unsigned long previousBroadcastTime  = startTime;
unsigned long previousHeartbeatTime  = startTime;
unsigned long previousVibrationSignatureTime = startTime;
unsigned long lastBatteryStatusCheck = startTime;
unsigned long currentTime = startTime;


// Singleton instance of the radio driver
RH_RF95 rf95(10, 2);
AppRouter router;

void setup()
{
  //Setup Green and Red LED as output
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  //Start serial port and wait for it to be available
  Serial.begin(9600);
  while (!Serial) ;

  //Initialise the LoRa RFM95 module
  if (!rf95.init()) {
    Serial.println("init failed");
  }

  //Configure Power and Frequency of LoRa RFM95 module
  rf95.setFrequency(915.0);
  rf95.setTxPower(23, true);

  // Setup Watchdog Timer
  //setupWatchDogTimer();
  
  Serial.println("Intialisation Complete.");

}

void loop()
{
  currentTime = millis();
  if(currentTime>1200000){
    Serial.println("Experiment Complete");
    return;
  }
  
  runMode();
}

void runMode(){

  /*-----------------------------INITIALISE BUFFER---------------------------------------------------*/
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

    
/*-----------------------------SEARCH FOR POSSIBLE ROUTES-------------------------------------------*/
  currentTime = millis();
  
  // Send Beacons every 30 seconds
  if (currentTime - previousBroadcastTime > 30000){
      Serial.print("Sending Broadcast Message. The time is:");
      Serial.println(currentTime - previousBroadcastTime, DEC);
      sendBroadcast();  
      previousBroadcastTime = currentTime;
  }
  
  // Send Heartbeat every 1 minute 
  if (currentTime - previousHeartbeatTime > 60000){
      Serial.print("Sending HeartBeat Message. The time is:");
      Serial.println(currentTime - previousHeartbeatTime, DEC);
      sendMessageToGateway(HEARTBEAT_MESSAGE);
      previousHeartbeatTime = currentTime;
  }

  // Send Vibration Signature every 2 minutes 
  if (currentTime - previousVibrationSignatureTime > 120000){
      Serial.print("Sending Vibration Message. The time is:");
      Serial.println(currentTime - previousVibrationSignatureTime, DEC);
      sendMessageToGateway(VIBRATION_SIGNATURE_MESSAGE);
      previousVibrationSignatureTime = currentTime;
      
  }

  

  
  /*-----------------------------RECEIVE MODE---------------------------------------------------------*/

  rf95.setModeRx();
  if (rf95.waitAvailableTimeout(10000))
  {
 

  }
  
}

//--------------------------------- APPLICATION METHODS ------------------------------

void sendBroadcast(){

  // Reset Routing Table
  router.resetRoutingTable();

  //Send a beacon 3 times.
  for (char i = 0; i < 3 ; i++) {
    sendBroadcastMessage();
  }
 
}

void sendBroadcastMessage() {
  uint8_t data[4];
  Tx::constructBroadcastMessage(&data[0]);
  rf95.setModeTx();
  rf95.send(data, sizeof(data));
  rf95.waitPacketSent();

//  Serial.println("Sent : BR");

}

void sendMessageToGateway(uint8_t messageType){

  if (messageType > 4 || messageType < 0){
      Serial.println("Illegal Message Type. Check messages.h for message types.");
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
