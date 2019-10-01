//--------------------------------------------------------------------------------------
// INCLUDE REQUIRED LIBRARIES
#include <app_routing.h>
#include <messages.h>
#include <node_config.h>
#include <rx.h>
#include <tx.h>
#include <util.h>

#include <SPI.h>
#include <RH_RF95.h>
//--------------------------------------------------------------------------------------
// DEFINE SYSTEM PROPERTIES
#define RED_LED 7
#define GREEN_LED 8
#define BLINK_DELAY 50

//--------------------------------------------------------------------------------------
//Define Utility methods
void blink(int LED, int n_times);
void sendBroadcastMessage();
void sendVibrationSignature();
//--------------------------------------------------------------------------------------
//Define objects to be used in application

// Singleton instance of the radio driver
RH_RF95 rf95(10, 2);

//--------------------------------------------------------------------------------------
//SETUP MCU
void setup() 
{
   //Setup Green and Red LED as output
   pinMode(7, OUTPUT);
   pinMode(8, OUTPUT);

  //Start serial port and wait for it to be available
  Serial.begin(9600);
  while (!Serial) ; 

  //Initialise the LoRa RFM95 module
  if (!rf95.init()){
    blink(RED_LED, 2);
    // Serial.println("init failed");
  }

  //Configure Power and Frequency of LoRa RFM95 module
  rf95.setFrequency(915.0);
  rf95.setTxPower(14, true);
  
  // Serial.println("Intialisation Complete.");
  blink(GREEN_LED,1);
  delay(BLINK_DELAY);
}

//--------------------------------------------------------------------------------------
// START THE APPLICATION
void loop()
{
  // 1. Broadcast Status to potential recipient nodes
  sendBroadcastMessage();
  sendVibrationSignature();
  
  // Serial.println("Message sent!");

  // Now wait for a reply

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  
  rf95.setModeRx();
  if (rf95.waitAvailableTimeout(5000))
  { 
    // Should be a reply message for us now  
    if (rf95.recv(buf, &len))
   {  
      if (Util::isBroadcastMessage(buf)){
        blink(RED_LED, 1);  
      }

      if(Util::isVibrationSignature(buf)){
        blink(RED_LED, 2);
      }
          
    }
    else
    {
      // Serial.println("recv failed");
    }
  }
  else
  {
    // Serial.println("No reply, is rf95_server running?");
  }
}

// Blinks LED of choice n-times
void blink(int LED, int n_times){
  
    for (int i = 0; i<n_times; i++){
      digitalWrite(LED, HIGH);
      delay(BLINK_DELAY);
      digitalWrite(LED, LOW);
      delay(BLINK_DELAY);
    }
}


void sendBroadcastMessage(){
  uint8_t data[4];
  Tx::constructBroadcastMessage(&data[0]);
  rf95.setModeTx();
  rf95.send(data, sizeof(data));
  rf95.waitPacketSent();
  blink(GREEN_LED, 2);  
}

void sendVibrationSignature(){

  uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
  Tx::constructVibrationMessage(data);
  rf95.send(data, sizeof(data));
  rf95.waitPacketSent();
  blink(GREEN_LED, 2);  
   
}
