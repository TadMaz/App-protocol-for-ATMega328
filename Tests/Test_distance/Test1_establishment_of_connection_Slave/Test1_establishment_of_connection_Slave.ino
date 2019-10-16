#include <SPI.h>
#include <RH_RF95.h>

#include <app_routing.h>
#include <messages.h>
#include <node_config.h>
#include <rx.h>
#include <tx.h>
#include <util.h>

//System parameters
#define RED_LED 7
#define GREEN_LED 8
#define BLINK_DELAY 500

//Test parameters
#define NO_OF_PACKETS 25
int packets_received = 0;
int acknowledgements_sent = 0;

// Instance of Radio driver
RH_RF95 rf95(10, 2);

void setup() {

//    Serial.begin(9600);
//    while (!Serial) ;
    //Setup Green and Red LED as output
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
//    pinMode(53, OUTPUT);
    if (!rf95.init()) {
      blink(RED_LED, 2);
      Serial.println("init failed");
    }
  
    rf95.setFrequency(915.0);
    rf95.setTxPower(23);
    //rf95.setSpreadingFactor(12);
    
    blink(GREEN_LED, 2);
}

void loop() {
    
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
  
    //Wait in receive mode
    rf95.setModeRx();
    if (rf95.waitAvailableTimeout(60000)){
        
        blink(GREEN_LED,1);
        
        if(rf95.recv(buf, &len)){
  //        Serial.println("Received Packet");
          packets_received++; 
          blink(RED_LED,1);
          
          // send acknowledgement back to Master node
          blink(GREEN_LED,1);
          sendPacket(buf);
          acknowledgements_sent++;
  //        Serial.println("Acknowlegement sent");
          
        }  
    }
}

void sendPacket(uint8_t* buf){
  
  rf95.setModeTx();
  rf95.send(buf, sizeof(buf));
  rf95.waitPacketSent();
//  Serial.println("Packet sent");
}

void blink(int LED, int n_times) {

  for (int i = 0; i < n_times; i++) {
    digitalWrite(LED, HIGH);
    delay(BLINK_DELAY);
    digitalWrite(LED, LOW);
    delay(BLINK_DELAY);
  }
}
