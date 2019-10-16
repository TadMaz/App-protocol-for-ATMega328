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

/*------------------------------SYSTEM CONSTANTS-----------------------------*/
#define RED_LED 7
#define GREEN_LED 8
#define BLINK_DELAY 100
#define RX_WAIT_TIME 10000

/*---------------------Initial System Timing----------------------------------*/

unsigned long startTime = millis();
unsigned long previousBroadcastTime  = startTime;
unsigned long previousHeartbeatTime  = startTime;
unsigned long previousVibrationSignatureTime = startTime;
unsigned long lastBatteryStatusCheck = startTime;
unsigned long currentTime;

/*--------------------- WATCHDOG TIMER SETTINGS -------------------------------*/
// This variable is made volatile because it is changed inside
// an interrupt function
volatile int f_wdt=1;

/*--------------------INITIALISE THE RFM95 and the routing table---------------*/
RH_RF95 rf95(10, 2);
AppRouter router;
/*-----------------------------------------------------------------------------*/
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
    blink(RED_LED, 2);
    Serial.println("init failed");
  }

  //Configure Power and Frequency of LoRa RFM95 module
  rf95.setFrequency(915.0);
  rf95.setTxPower(14, true);

  // Setup Watchdog Timer
  //setupWatchDogTimer();
  
  Serial.println("Intialisation Complete.");

}

void loop()
{ 
    //Initialise the buffer
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    
    Serial.println("Application protocol starting. . .");
    runProtocol((uint8_t*)buf);
}

void runProtocol(uint8_t* buf){

  /*-----------------------------INITIALISE BUFFER---------------------------------------------------*/
 
  uint8_t len = sizeof(buf);

/*-----------------------------SEARCH FOR POSSIBLE ROUTES-------------------------------------------*/

  
  // Send Beacons every 30 seconds
  if (currentTime - previousBroadcastTime > 30000){
      sendBroadcast();  
      previousBroadcastTime = currentTime;
  }

  
  // Send Heartbeat every 1 minute (for now) ---> should be 10 minutes
  if (currentTime - previousBroadcastTime > 60000){
      sendMessageToGateway(HEARTBEAT_MESSAGE);
      previousHeartbeatTime = currentTime;
  }

  // Send Vibration Signature every 2 minutes (for now) --> should be 15 minutes
  if (currentTime - previousBroadcastTime > 120000){
      sendMessageToGateway(VIBRATION_SIGNATURE_MESSAGE);
      previousVibrationSignatureTime = currentTime;
      
  }

  
  /*-----------------------------RECEIVE MODE---------------------------------------------------------*/

  rf95.setModeRx();
  if (rf95.waitAvailableTimeout(20000))
  {
    // Should be a reply message for us now

    if (rf95.recv(buf, &len))
    {
     
      // 1. Send Broadcast if Broadcast Message is received
      if (Util::isBroadcastMessage(buf))
      {

        // extract the contents of the broadcast Message and add them to the router
        uint16_t id = Rx::extractBroadcastMessage(buf, rf95.lastRssi(), &router);

      // Check if the message is addressed to this node, 
      //and is a Gateway Message (Vibration Signature or Heartbeat Message)
      
      }else if (Util::matchIDs(buf) && (
                Util::isBroadcastMessage(buf) || Util::isHeartbeat(buf)))
              
      {   
          // Relay -> Change the destination on the packet to the best node and send this packet 
          // The source address is maintained             
          relayMessage(buf);
      }

    }
  }
 
}
//--------------------------------- WATCH DOG TIMER ----------------------------------
// Setup the Watch Dog Timer (WDT)
void setupWatchDogTimer() {
  // The MCU Status Register (MCUSR) is used to tell the cause of the last
  // reset, such as brown-out reset, watchdog reset, etc.
  // NOTE: for security reasons, there is a timed sequence for clearing the
  // WDE and changing the time-out configuration. If you don't use this
  // sequence properly, you'll get unexpected results.

  // Clear the reset flag on the MCUSR, the WDRF bit (bit 3).
  MCUSR &= ~(1<<WDRF);

  // Configure the Watchdog timer Control Register (WDTCSR)
  // The WDTCSR is used for configuring the time-out, mode of operation, etc

  // In order to change WDE or the pre-scaler, we need to set WDCE (This will
  // allow updates for 4 clock cycles).

  // Set the WDCE bit (bit 4) and the WDE bit (bit 3) of the WDTCSR. The WDCE
  // bit must be set in order to change WDE or the watchdog pre-scalers.
  // Setting the WDCE bit will allow updates to the pre-scalers and WDE for 4
  // clock cycles then it will be reset by hardware.
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  /**
   *  Setting the watchdog pre-scaler value with VCC = 5.0V and 16mHZ
   *  WDP3 WDP2 WDP1 WDP0 | Number of WDT | Typical Time-out at Oscillator Cycles
   *  0    0    0    0    |   2K cycles   | 16 ms
   *  0    0    0    1    |   4K cycles   | 32 ms
   *  0    0    1    0    |   8K cycles   | 64 ms
   *  0    0    1    1    |  16K cycles   | 0.125 s
   *  0    1    0    0    |  32K cycles   | 0.25 s
   *  0    1    0    1    |  64K cycles   | 0.5 s
   *  0    1    1    0    |  128K cycles  | 1.0 s
   *  0    1    1    1    |  256K cycles  | 2.0 s
   *  1    0    0    0    |  512K cycles  | 4.0 s
   *  1    0    0    1    | 1024K cycles  | 8.0 s
  */
  WDTCSR  = (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);
  // Enable the WD interrupt (note: no reset).
  WDTCSR |= _BV(WDIE);
}


// Enters the arduino into sleep mode.
void enterSleep(void)
{
  // There are five different sleep modes in order of power saving:
  // SLEEP_MODE_IDLE - the lowest power saving mode
  // SLEEP_MODE_ADC
  // SLEEP_MODE_PWR_SAVE
  // SLEEP_MODE_STANDBY
  // SLEEP_MODE_PWR_DOWN - the highest power saving mode
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  // Now enter sleep mode.
  sleep_mode();

  // The program will continue from here after the WDT timeout

  // First thing to do is disable sleep.
  sleep_disable();

  // Re-enable the peripherals.
  power_all_enable();
}

ISR(WDT_vect) {
  if(f_wdt < 4) {
    // here we can implement a counter the can set the f_wdt to true if
    // the watchdog cycle needs to run longer than the maximum of eight
    // seconds.
    f_wdt++;
    Serial.println("8 seconds");delay(100);
  }
}

//--------------------------------- APPLICATION METHODS ------------------------------


/**
  This method resets the router and sends a Broadcast Message 3 time 
*/
void sendBroadcast(){

  // Reset Routing Table
  router.resetRoutingTable();

  //Send a beacon 3 times.
  for (char i = 0; i < 3 ; i++) {
    sendBroadcastMessage();
  }
 
}

/**
 * This is a helper method to sendBroadcast - it sends the broadcast Message
 */
void sendBroadcastMessage() {
  uint8_t data[4];
  Tx::constructBroadcastMessage(&data[0]);
  rf95.setModeTx();
  rf95.send(data, sizeof(data));
  rf95.waitPacketSent();


}
/*
 * This method sends a message to the gateway
 * Valid gateway messages are Vibration Signature Messages and Heartbeat Message
 */
void sendMessageToGateway(uint8_t messageType){

  if (messageType > 4 || messageType < 0){
      Serial.println("Illegal Message Type. Check messages.h for message types.");
  }
  
  uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
  uint16_t destID = router.getBestDestination();

  if( messageType == VIBRATION_SIGNATURE_MESSAGE ){
      Tx::constructVibrationMessage(data, destID);
  }

  if (messageType == HEARTBEAT_MESSAGE){
      Tx::constructHeartbeatMessage(data, destID);
  }

  rf95.setModeTx();
  rf95.send(data, sizeof(data));
  rf95.waitPacketSent();
}

/* 
 *This method modifies the destination address with the best route and sends the packet 
 */
void relayMessage(uint8_t* buf) {
    // Change the destination of this packet to the new destination and relay the packet
  
    uint16_t destID = router.getBestDestination();
  
    if (!destID) {
      return;
    }
  
    Util::writeNodeIdToBuffer(destID, buf, DESTINATION_ID_START);
    rf95.setModeTx();
    rf95.send(buf, sizeof(buf));
    rf95.waitPacketSent();
}


// Blinks LED of choice n-times
void blink(int LED, int n_times) {

  for (int i = 0; i < n_times; i++) {
    digitalWrite(LED, HIGH);
    delay(BLINK_DELAY);
    digitalWrite(LED, LOW);
    delay(BLINK_DELAY);
  }
}

/*
 * This method may be used to view the entries on the routing table.
 */
void printRoutingTable(AppRouter* router) {

  Serial.println("Destination ID | RSSI | Distance To Gateway");

  for (char i = 0; i < ROUTER_TABLE_LENGTH; i++) {

    Serial.print(router->getDestinationIDAt(i), DEC);
    Serial.print("             |");
    Serial.print(router->getRssiAt(i), DEC);
    Serial.print("        |");
    Serial.println(router->getDistanceToGatewayAt(i), DEC);
  }

}
