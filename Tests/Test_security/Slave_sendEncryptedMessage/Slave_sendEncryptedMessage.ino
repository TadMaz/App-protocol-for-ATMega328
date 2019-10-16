#include <aes.h>
#include <aes128_dec.h>
#include <aes128_enc.h>
#include <aes192_dec.h>
#include <aes192_enc.h>
#include <aes256_dec.h>
#include <aes256_enc.h>
#include <AESLib.h>
#include <aes_dec.h>
#include <aes_enc.h>
#include <aes_invsbox.h>
#include <aes_keyschedule.h>
#include <aes_sbox.h>
#include <aes_types.h>
#include <bcal-basic.h>
#include <bcal-cbc.h>
#include <bcal-cmac.h>
#include <bcal-ofb.h>
#include <bcal_aes128.h>
#include <bcal_aes192.h>
#include <bcal_aes256.h>
#include <blockcipher_descriptor.h>
#include <gf256mul.h>
#include <keysize_descriptor.h>
#include <memxor.h>

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

// Instance of Radio driver
RH_RF95 rf95(10, 2);
AppRouter router;


//key
uint8_t key[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};

void setup() {
  
    Serial.begin(9600);
    while (!Serial);
    Serial.println("Setting up");
    
    //Setup Green and Red LED as output
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
  
    if (!rf95.init()) {
      blink(RED_LED, 2);
      Serial.println("init failed");
      return;
    }

    //RF Settings
    rf95.setFrequency(915.0);
    rf95.setTxPower(23);
    
    Serial.println("Initialisation Done");
}

void loop() {
  
//  uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
// 
//  
//  Tx::constructVibrationMessage(data, 1);

//  char data[] = {"1234567891"};
//  
//  aes128_enc_single(aes_key,data);
//  Serial.print("encrypted:");
//  Serial.println(data);
//  
//  aes128_dec_single(aes_key, data);
//  Serial.print("decrypted:");
//  Serial.println(data);

  
//  char data[251] = "01234567890123450123456789012345"; //16 chars == 16 bytes
  uint8_t data[251] = {0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5};
//  aes128_enc_single(key, data);
//  Serial.print("encrypted:");
//  Serial.println(data);
//  aes128_dec_single(key, data);
//  Serial.print("decrypted:");
//  Serial.println(data);
//  encryptMessage((uint8_t*) data);
//  char ex [16];
  
//  Serial.println(ex);
//  Serial.print("encrypted:");
//  Serial.println(data);

//  decryptMessage((uint8_t*) data);
//  uint8_t reloaded[] = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};
//  reload_bytes((char*)reloaded,(uint8_t*)data,6,11);
//  
//  for(int i =6;i<6+11;i++){
//        Serial.println((uint8_t)data[i]);
//  }
//
//  char ex [16];
//  extract_bytes((char*)ex,(uint8_t*)data,16,16);
//  for (int i = 0; i<16;i++){
//    Serial.println((uint8_t) ex[i],DEC);
//  }
  while(1);
}

void encryptMessage(uint8_t* buf){

    
    char data[16];

    //encrypt first 240 bytes
    for (int i = 0; i<15; i++){
      extract_bytes((char*)&data, buf, (i*16), 16); 
      aes128_enc_single(key, data);
      reload_bytes((char*)&data, buf, (i*16)-1, 16); 
    }

    //encrypt last 11 bytes
    char data1[11];
    extract_bytes((char*)&data1, buf, 240, 11);
    aes128_enc_single(key, data1);
    reload_bytes((char*)&data1, buf, 239, 11);  
    

}

void decryptMessage(uint8_t* buf){

    char data[16];

    //decrypt first 240 bytes
    for (int i = 0; i<15 ; i++){
      extract_bytes((char*)&data, buf, (i*16), 16);
      aes128_enc_single(key, data);
      reload_bytes((char*)&data, buf, (i*16), 16); 
    }

    //decrypt last 11 bytes
    char data1[11];
    extract_bytes((char*)&data1, buf, 240, 11);
    aes128_dec_single(key, data1);
    reload_bytes((char*)&data1, buf, 240, 11);
}


//extract "no_of_bytes" from buffer from position "start_index"
void extract_bytes(char* output, uint8_t* buf, int start_index, int no_of_bytes){

    int buf_index = start_index;
//    Serial.print("Started On :");
//    Serial.println(buf_index,DEC); 
    for (int i = 0; i<no_of_bytes; i++){
      output[i] = buf[buf_index];
//      Serial.println(i,DEC);
      buf_index++;   
    }
//    Serial.print("Ended On :");
//    Serial.println(buf_index,DEC);
}

//reload "no_of_bytes" to buffer 
void reload_bytes(char* input, uint8_t* buf, int start_index, int no_of_bytes){

  int buf_index = start_index;
  
  for (int i = 0; i<no_of_bytes; i++){
      buf[buf_index] = input[i];
      buf_index++;   
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
