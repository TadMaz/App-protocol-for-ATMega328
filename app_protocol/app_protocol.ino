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
  if (!rf95.init()){
    blink(RED_LED, 2);
    Serial.println("init failed");
  }

  //Configure Power and Frequency of LoRa RFM95 module
  rf95.setFrequency(915.0);
  rf95.setTxPower(14, true);
  
  Serial.println("Intialisation Complete.");
 
}

void loop()
{

  /*-----------------------------TRANSMIT MODE-------------------------------------------*/
  // 1. Broadcast Status to potential recipient nodes
//  uint8_t reading[] = "000000000000000100100";

  sendBroadcastMessage(); 
  sendVibrationSignature();
  uint8_t dataToSend[RH_RF95_MAX_MESSAGE_LEN];
  

  /*-----------------------------RECEIVE MODE-------------------------------------------*/
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  
  rf95.setModeRx();
  if (rf95.waitAvailableTimeout(10000))
  { 
    // Should be a reply message for us now  
    Serial.println("Waiting for a reply . . "); 
    if (rf95.recv(buf, &len))
   {  
      Serial.println("got reply: ");
      
      if (Util::isBroadcastMessage(buf)){
          uint32_t pay = Rx::extractBroadcastMessage(buf,rf95.lastRssi(),&router);
          Serial.println(pay, DEC);
          Serial.println("Route Added to table");
          printRoutingTable(&router);
      }
      else if (Util::isVibrationSignature(buf)){
          Serial.println("Vibration Signature received");
          
      }
     
    }
    else
    {
      Serial.println("recv failed");
    }
  }
  else
  {
    Serial.println("No reply, is rf95_server running?");
  }
  delay(400);
}
//--------------------------------- APPLICATION METHODS ------------------------------
void sendBroadcastMessage(){
  uint8_t data[4];
  Tx::constructBroadcastMessage(&data[0]);
  rf95.setModeTx();
  rf95.send(data, sizeof(data));
  rf95.waitPacketSent();
  
  Serial.print("Sent : ");
  for (int i=0; i<4; i++){
    Serial.print(data[i]);
  }
  Serial.println("");
    
}

void sendVibrationSignature(){

  uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
  Tx::constructVibrationMessage(data);
  rf95.send(data, sizeof(data));
  rf95.waitPacketSent();
  Serial.println("Vibration signature sent");  
   
}
void printRoutingTable(AppRouter* router){
  Serial.println("Destination ID | RSSI | Distance To Gateway");
  Serial.print(router->getDestinationIDAt(0), DEC);
  Serial.print("             |");
  Serial.print(router->getRssiAt(0), DEC);
  Serial.print("        |");
  Serial.println(router->getDistanceToGatewayAt(0), DEC);
  Serial.print(router->getDestinationIDAt(1), DEC);
  Serial.print("             |");
  Serial.print(router->getRssiAt(1), DEC);
  Serial.print("        |");
  Serial.println(router->getDistanceToGatewayAt(1), DEC);
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
