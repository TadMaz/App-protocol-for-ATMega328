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
#define RX_WAIT_TIME 500
//--------------------------------------------------------------------------------------
//Define Utility methods
void blink(int LED, int n_times);
void sendBroadcastMessage();
void sendVibrationSignature();
//--------------------------------------------------------------------------------------
//Define objects to be used in application

// Singleton instance of the radio driver
RH_RF95 rf95(10, 2);
AppRouter router;

//--------------------------------------------------------------------------------------
//SETUP MCU
void setup() 
{
   //Setup Green and Red LED as output
   pinMode(7, OUTPUT);
   pinMode(8, OUTPUT);

  //Start serial port and wait for it to be available
//  Serial.begin(9600);
//  while (!Serial) ; 

  //Initialise the LoRa RFM95 module
  if (!rf95.init()){
    blink(RED_LED, 2);
    // Serial.println("init failed");
  }

  //Configure Power and Frequency of LoRa RFM95 module
  rf95.setFrequency(915.0);
  rf95.setTxPower(17, false);
  
  // Serial.println("Intialisation Complete.");
  blink(GREEN_LED,1);
  delay(BLINK_DELAY);
}

//--------------------------------------------------------------------------------------
// START THE APPLICATION
void loop()
{
  /*-----------------------------INITIALISE BUFFER---------------------------------------------------*/
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  
  /*-----------------------------SEARCH FOR POSSIBLE ROUTES-------------------------------------------*/
  // 1. Broadcast Status to potential recipient nodes
  
//  Serial.println("Establishing potential routes . . .");
  
  for (char i = 0; i < 3 ; i++){
    sendBroadcastMessage(); 
    
    rf95.setModeRx();
    if( rf95.waitAvailableTimeout(RX_WAIT_TIME) ){
       
       if( rf95.recv(buf, &len) && Util::isBroadcastMessage(buf) ){
/*uint8_t id = */
            Rx::extractBroadcastMessage(buf,rf95.lastRssi(),&router);
       }
    }
  }

//*-----------------------------SIMULATE PACKET SEND-------------------------------------------------*/
  for( int i = 0; i< 3; i++)
    sendVibrationSignature();

 /*-----------------------------RECEIVE MODE---------------------------------------------------------*/
rf95.setModeRx();
  if (rf95.waitAvailableTimeout(RX_WAIT_TIME))
  { 
    // Should be a reply message for us now  
    
    if (rf95.recv(buf, &len))
   {  

      // 1. Send Broadcast if Broadcast Message is received
      if (Util::isBroadcastMessage(buf)){
          
          uint16_t id = Rx::extractBroadcastMessage(buf,rf95.lastRssi(),&router);
          blink(RED_LED, 1);
          
      }
      //Is packet addressed to this node and is one of the gateway messages?
      else if (Util::isVibrationSignature(buf)){
          blink(RED_LED, 2); 
//          Serial.println("Vibration Signature received");
          relayMessage(buf);        
      }
     
    }
  }
  
}

//--------------------------------- APPLICATION METHODS ------------------------------
void sendBroadcastMessage(){
  uint8_t data[4];
  Tx::constructBroadcastMessage(&data[0]);
  rf95.setModeTx();
  rf95.send(data, sizeof(data));
  rf95.waitPacketSent();
  blink(GREEN_LED, 1);  
}

void sendVibrationSignature(){

  uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
  uint16_t destID = router.getBestDestination();
  
  Tx::constructVibrationMessage(data, destID);
  
  rf95.setModeTx();
  rf95.send(data, sizeof(data));
  rf95.waitPacketSent();
  blink(GREEN_LED, 2);  
   
}

void relayMessage(uint8_t* buf){
  // Change the destination of this packet to the new destination and relay the packet
  
  uint16_t destID = router.getBestDestination();
  Util::writeNodeIdToBuffer(destID, buf, DESTINATION_ID_START);
  rf95.setModeTx();
  rf95.send(buf, sizeof(buf));
  rf95.waitPacketSent();
  blink(GREEN_LED, 3);
  
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
