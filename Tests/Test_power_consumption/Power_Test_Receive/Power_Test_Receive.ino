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

RH_RF95 rf95(10, 2);
AppRouter router;

void setup() {
  // put your setup code here, to run once:
  //Setup Green and Red LED as output
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
  
    //Start serial port and wait for it to be available
//    Serial.begin(9600);
//    while (!Serial) ;
  
    //Initialise the LoRa RFM95 module
    if (!rf95.init()) {
//      Serial.println("init failed");
      blink(RED_LED,2);
    }
  
    //Configure Power and Frequency of LoRa RFM95 module
    rf95.setFrequency(915.0);
    rf95.setTxPower(23,true);

    blink(GREEN_LED,2);
//    Serial.println("Intialisation Complete.");
}

void loop() {
  // put your main code here, to run repeatedly:
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    rf95.setModeRx();
//    Serial.println("Receive Mode");
    if(rf95.waitAvailableTimeout(15000)){

      if (rf95.recv(buf, &len)){

//          Rssi[no_of_acknownledgements] = rf95.lastRssi();
//          Serial.print("Acknowledgement received. RSSI:");
//          Serial.println(rf95.lastRssi(), DEC);
//          no_of_acknownledgements++;
////              Serial.println((char*) buf);
      }
      
    }
}
void blink(int LED, int n_times) {

  for (int i = 0; i < n_times; i++) {
    digitalWrite(LED, HIGH);
    delay(BLINK_DELAY);
    digitalWrite(LED, LOW);
    delay(BLINK_DELAY);
  }
}
