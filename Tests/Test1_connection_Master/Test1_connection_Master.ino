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

/// Test parameters
#define NO_OF_PACKETS 25
int packets_sent = 0;
int no_of_acknownledgements = 0;
int16_t Rssi[NO_OF_PACKETS];

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
  rf95.setTxPower(23,true);
  
  Serial.println("Intialisation Complete.");

}

void loop() {

    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    //Send packet every 10s 
    while ( packets_sent < NO_OF_PACKETS ){
        //send packet
       
        sendPacket();
        packets_sent++;
        
        //wait for acknowlegement - if 20 seconds lapse, Timeout
        
        
        rf95.setModeRx();
        if(rf95.waitAvailableTimeout(15000)){

          if (rf95.recv(buf, &len)){

              Rssi[no_of_acknownledgements] = rf95.lastRssi();
              Serial.print("Acknowledgement received. RSSI:");
              Serial.println(rf95.lastRssi(), DEC);
              no_of_acknownledgements++;
//              Serial.println((char*) buf);
          }
          
        }
        delay(500);
    }
    Serial.println("Test complete. 25 packets sent.");
    Serial.print("Acknowlegements received : ");
    Serial.println(no_of_acknownledgements, DEC);
    while(true);
}
void sendPacket(){
  
  uint8_t data[RH_RF95_MAX_MESSAGE_LEN] = {"Hello Back to you."};
  
  //Tx::constructVibrationMessage(data, 1);
  
  rf95.setModeTx();
  rf95.send(data, sizeof(data));
  
  rf95.waitPacketSent();
  Serial.println("Packet sent");
}

void blink(int LED, int n_times) {

  for (int i = 0; i < n_times; i++) {
    digitalWrite(LED, HIGH);
    delay(BLINK_DELAY);
    digitalWrite(LED, LOW);
    delay(BLINK_DELAY);
  }
}
