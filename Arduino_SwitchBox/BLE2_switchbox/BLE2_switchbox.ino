/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE" 
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second. 

let data_table2 =[ 
['node1det', 'NO', '21300', '33', 'buttonid0'],
['node2det', 'NO', '21301', '3', 'buttonid1'],
['node3det', 'NO', '21302', '1', 'buttonid2'],
['node4det', 'NO', '21303', '100', 'buttonid3'],
['vdd12vdet', 'NO', '21304', '500', 'buttonid4'],
['Wet sensor', 'NO', '21305', '200', 'buttonid5'],
['Light sensor', 'NO', '21306', '400', 'buttonid6'],
['Temp sensor', 'NO', '21307', '300', 'buttonid7'],
['Bandgap', 'NO', '21308', '434', 'buttonid8'],
['Light sensor limit', 'Y', '21309', '450', 'buttonid9'],
['Moist Detector limit', 'Y', '21310', '500', 'buttonid10'],
['node1 on time', 'Y', '21311', '5', 'buttonid11'],
['node2 on time', 'Y', '21312', '5', 'buttonid12'],
['node3 on time', 'Y', '21313', '5', 'buttonid13'],
['node4 on time', 'Y', '21314', '5', 'buttonid14'],
['node1 off time', 'Y', '21315', '15', 'buttonid15'],
['node2 off time', 'Y', '21316', '15', 'buttonid16'],
['node3 off time', 'Y', '21317', '15', 'buttonid17'],
['node4 off time', 'Y', '21318', '15', 'buttonid18'],
['onmult', 'Y', '21319', '1', 'buttonid19'],
['offmult', 'Y', '21320', '1', 'buttonid20'],
['readvalhold', 'NO', '21321', '0', 'buttonid21'],
['readvalhold_1', 'NO', '21322', '0', 'buttonid22'],
['readvalhold_2', 'NO', '21323', '0', 'buttonid23']
]
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <Arduino.h>
#include <esp_timer.h>

#include "GPIO.h"
#include <driver/gpio.h>
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc.h"
#include "driver/rtc_io.h"

#include <cstring>
#include <iostream>
#include <string.h>
#include <stdio.h>

#include <bits/stdc++.h>
#include <EEPROM.h>
#define EEPROM_SIZE 28

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);

#include "freertos/FreeRTOS.h"

#define NOP() asm volatile ("nop")

BLEServer *pServer = NULL;
BLECharacteristic * pCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool eepromRead = false;

//uint8_t txValue = 0;
uint16_t txValue = 0;
uint16_t rx_Value = 0;
float txFloat = 5e-3;
int i,j,k = 0;
std::string txString;
std::string rxString;
std::string rxValue1;
uint16_t wordsread[105];
char buffer1[200];
char buffer2[200];
int analogValue, tempntcValue, tempValue, progValue1, progValue2, ldrValue;
int analogVolts;
uint8_t connectnow=0;
uint8_t initwrite=0;
uint8_t progstate=0;     // during programming keep state
uint8_t progldrtemp=0;   // is 1 when programming ldr or temp
int switch1return=0;
int switch2return=0;
int switch3return=0;
int switch4return=0;
int CheckSwitch1Return=0;
int CheckSwitch2Return=0;
int CheckSwitch3Return=0;
int CheckSwitch4Return=0;



uint16_t saveeprom=0;
 
char rxCopy[256];  // for receive string BLE
char* tokens[500] = {0};  // Initialize tokens to NULL for splitting receive string
size_t tokenCount = 0;     // Split the string in tokens and count the tokens:
static size_t const max_token_count = 50;  // max 25 


uint16_t vbat=1;  // battery voltage
uint16_t vbatinit=1;  // battery voltage initial value

//std::string rxString;
//std::string rxValue1; // current read value

//char txString[2049];

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID            "BD115970-EDB9-4F6A-AEA4-DCE32BE9F470"  //"0A5BB5A0-0000-1000-8000-00805F9B34FB" 0A5BB5A0-0000-1000-8000-00805F9B34FB UART service UUID
#define CHARACTERISTIC_UUID_RX  "BD115971-EDB9-4F6A-AEA4-DCE32BE9F470"  //"0A5BB5A1-0000-1000-8000-00805F9B34FB" // 0A5BB5A1-0000-1000-8000-00805F9B34FB
#define CHARACTERISTIC_UUID_RX2 "BD115972-EDB9-4F6A-AEA4-DCE32BE9F470"  // "00004444-0000-1000-8000-00805F9B34FB" // 0A5BB5A1-0000-1000-8000-00805F9B34FB
#define CHARACTERISTIC_UUID_TX  "BD115973-EDB9-4F6A-AEA4-DCE32BE9F470"  //"0A5BB5A2-0000-1000-8000-00805F9B34FB" // 0A5BB5A2-0000-1000-8000-00805F9B34FB
// "BD115974-EDB9-4F6A-AEA4-DCE32BE9F470"

//#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
//#define CHARACTERISTIC_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

//#define SERVICE_UUID "FFE0"
//#define CHARACTERISTIC_UUID "FFE1"



//#define CLK  12    // clock
//#define MISO 14    // data
//#define CS   27    // chip select
//#define MOSI 26    // data out
//Boot is interfered

#define CLK  23    // clock
#define MISO 22    // data
#define CS   21    // chip select
#define MOSI 19    // data out
#define SWITCH1RET  39    // Switch output return
#define SWITCH2RET  36    // Switch output return
#define SWITCH3RET  35    // Switch output return
#define SWITCH4RET  34    // Switch output return
#define SWITCHRST  17 //26    // Switch output
#define LDRIN  25    // LDROUT return 
#define SENSOUT  18    // SENSOUT output supply for all sensors
#define TEMPIN  13    // TEMPIN return 
#define PROGVAL1  4    // Programming input
#define PROGVAL2  33    // Programming input
#define CONNECTBLE  13    // Enable BLE connect
#define LED  19       // LED output
#define VDD3V3  4    // VIN connected through a 10k resistor


#define SWITCHMAX      2400  // Maximum time in minutes 
#define SWITCHMIN      1     // Minimum time in minutes 
#define BUTTONON       3000  // Button is high limit 
#define SWITCHRETMIN   50    // SWITCH=OFF: Return value for connection to switch output 
#define SWITCHRETSHORT 144    // SWITCH=ON:  Return value for SOA MAX LOAD 0.5v*att=100k/430k*ADCMAX=4096/vdd=3.3=144
#define VDD3V3MIN      3276   // GPIO4 = VINMIN=0.8*4096=3276

#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex
#define uS_TO_S_FACTOR 1000000ULL    /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60            /* Time ESP32 will go to sleep (in seconds) */

#define HYSTERESIS  15               /* hysteresis for LDR and temperature */

#define LDRLIM   1500
#define TEMPLIM  500
//#define SHORTLIM 7           // 93%
#define SHORTLIM 5             // 95%
#define SWITCHRETOFFSHORT 144    // SWITCH=ON:  Return value for SOA MAX LOAD 0.5v*att=100k/430k*ADCMAX=4096/vdd=3.3=144


RTC_DATA_ATTR uint16_t bootCount = 0;
RTC_DATA_ATTR uint16_t bleOnCounter = 0;
RTC_DATA_ATTR uint16_t fastrunCounter = 0;
RTC_DATA_ATTR uint8_t switch1state = 0;   // switch1state = 1 is active
RTC_DATA_ATTR uint8_t switch2state = 0;   // switch2state = 1 is active
RTC_DATA_ATTR uint8_t switch3state = 0;   // switch3state = 1 is active
RTC_DATA_ATTR uint8_t switch4state = 0;   // switch4state = 1 is active

RTC_DATA_ATTR uint16_t switch1on = 1;
RTC_DATA_ATTR uint16_t switch2on = 1;
RTC_DATA_ATTR uint16_t switch3on = 1;
RTC_DATA_ATTR uint16_t switch4on = 1;

RTC_DATA_ATTR uint16_t switch1del = 1;
RTC_DATA_ATTR uint16_t switch2del = 1;
RTC_DATA_ATTR uint16_t switch3del = 1;
RTC_DATA_ATTR uint16_t switch4del = 1;

RTC_DATA_ATTR uint16_t switch1off = 100;
RTC_DATA_ATTR uint16_t switch2off = 100;
RTC_DATA_ATTR uint16_t switch3off = 100;
RTC_DATA_ATTR uint16_t switch4off = 100;

RTC_DATA_ATTR uint16_t switchcntr = 0;
RTC_DATA_ATTR uint16_t switch1lim = 0;
RTC_DATA_ATTR uint16_t switch2lim = 0;
RTC_DATA_ATTR uint16_t switch3lim = 0;
RTC_DATA_ATTR uint16_t switch4lim = 0;

RTC_DATA_ATTR uint16_t ldrlim = 100;
RTC_DATA_ATTR uint16_t templim = 100;

RTC_DATA_ATTR uint8_t ldrstate = 0;    // ldrstate = 1 is active
RTC_DATA_ATTR uint8_t tempstate = 0;   // tempstate = 1 is active

RTC_DATA_ATTR uint8_t switch1Short=0;
RTC_DATA_ATTR uint8_t switch2Short=0;
RTC_DATA_ATTR uint8_t switch3Short=0;
RTC_DATA_ATTR uint8_t switch4Short=0;

RTC_DATA_ATTR uint8_t switch1Overload=0;
RTC_DATA_ATTR uint8_t switch2Overload=0;
RTC_DATA_ATTR uint8_t switch3Overload=0;
RTC_DATA_ATTR uint8_t switch4Overload=0;


//gpio_num_t PTEST = GPIO_NUM_32;    // ptest
//#define SWITCH1  14 //23 //33    // Switch output
gpio_num_t SWITCH1 = GPIO_NUM_14;    // SWITCH1
//#define SWITCH2  15 //22 //32    // Switch output
gpio_num_t SWITCH2 = GPIO_NUM_15;    // SWITCH2
//#define SWITCH3  21 //27    // Switch output
gpio_num_t SWITCH3 = GPIO_NUM_32;    // SWITCH3
//#define SWITCH4  19 //26    // Switch output
gpio_num_t SWITCH4 = GPIO_NUM_27;    // SWITCH4


/*void INIT3WIRE()
{
  pinMode(pl1pin48cs,   INPUT);
  pinMode(pl2pin47clk,  INPUT);
  pinMode(pl3pin46miso, INPUT);
  void get_3wire_string( )
{
}
*/

/*
unsigned long IRAM_ATTR micros()
{
    return (unsigned long) (esp_timer_get_time());
}

void IRAM_ATTR delayMicroseconds(uint32_t us)
{
    uint32_t m = micros();
    if(us){
        uint32_t e = (m + us);
        if(m > e){ //overflow
            while(micros() > e){
                NOP();
            }
        }
        while(micros() < e){
            NOP();
        }
    }
}
*/

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}


uint16_t makedec(uint8_t n1) {
uint16_t outDec;  
           switch(n1) { 
            case 0: outDec = 21300; break; 
            case 1: outDec = (uint16_t)ldrlim; break; 
            case 2: outDec = 21310; break; 
            case 3: outDec = (uint16_t)ldrValue; break; 
            case 4: outDec = 21320; break; 
            case 5: outDec = (uint16_t)ldrstate; break; 
            case 6: outDec = 21330; break; 
            case 7: outDec = (uint16_t)templim; break; 
            case 8: outDec = 21340; break; 
            case 9: outDec = (uint16_t)tempntcValue; break; 
            case 10: outDec = 21350;  break; 
            case 11: outDec = (uint16_t)tempstate;  break; 
            case 12: outDec = 21360;  break;
            case 13: outDec = (uint16_t)progValue1;  break;
            case 14: outDec = 21370; break; 
            case 15: outDec = (uint16_t)progValue2; break; 
            case 16: outDec = 21380; break; 
            case 17: outDec = (uint16_t)bleOnCounter; break; 
            case 18: outDec = 21390; break;
            case 19: outDec = (uint16_t)fastrunCounter; break;
            case 20: outDec = 21400; break;
            case 21: outDec = (uint16_t)switch1return; if (switch1Overload==1) {outDec = (uint16_t)(65534);}; if (switch1Short==1) {outDec = (uint16_t)(65535);}; break;
            case 22: outDec = 21410; break;
            case 23: outDec = (uint16_t)switch2return; if (switch2Overload==1) {outDec = (uint16_t)(65534);}; if (switch2Short==1) {outDec = (uint16_t)(65535);}; break;
            case 24: outDec = 21420; break;
            case 25: outDec = (uint16_t)switch3return; if (switch3Overload==1) {outDec = (uint16_t)(65534);}; if (switch3Short==1) {outDec = (uint16_t)(65535);}; break;
            case 26: outDec = 21430; break;
            case 27: outDec = (uint16_t)switch4return; if (switch4Overload==1) {outDec = (uint16_t)(65534);}; if (switch4Short==1) {outDec = (uint16_t)(65535);}; break;
            case 28: outDec = 21440; break;
            case 29: outDec = (uint16_t)switch1lim; break;
            case 30: outDec = 21450; break;
            case 31: outDec = (uint16_t)switch1del; break;
            case 32: outDec = 21460; break;
            case 33: outDec = (uint16_t)switch1on; break;
            case 34: outDec = 21470; break;
            case 35: outDec = (uint16_t)switch1off; break;
            case 36: outDec = 21480; break;
            case 37: outDec = (uint16_t)switch1state; break;
            case 38: outDec = 21490; break;
            case 39: outDec = (uint16_t)switch2lim; break;
            case 40: outDec = 21500; break;
            case 41: outDec = (uint16_t)switch2del; break;
            case 42: outDec = 21510; break;
            case 43: outDec = (uint16_t)switch2on; break;
            case 44: outDec = 21520; break;
            case 45: outDec = (uint16_t)switch2off; break;
            case 46: outDec = 21530; break;
            case 47: outDec = (uint16_t)switch2state; break;
            case 48: outDec = 21540; break;
            case 49: outDec = (uint16_t)switch3lim; break;
            case 50: outDec = 21550; break;
            case 51: outDec = (uint16_t)switch3del; break;
            case 52: outDec = 21560; break;
            case 53: outDec = (uint16_t)switch3on; break;
            case 54: outDec = 21570; break;
            case 55: outDec = (uint16_t)switch3off; break;
            case 56: outDec = 21580; break;
            case 57: outDec = (uint16_t)switch3state; break;
            case 58: outDec = 21590; break;
            case 59: outDec = (uint16_t)switch4lim; break;
            case 60: outDec = 21600; break;
            case 61: outDec = (uint16_t)switch4del; break;
            case 62: outDec = 21610; break;
            case 63: outDec = (uint16_t)switch4on; break;
            case 64: outDec = 21620; break;
            case 65: outDec = (uint16_t)switch4off; break;
            case 66: outDec = 21630; break;
            case 67: outDec = (uint16_t)switch4state; break;
            case 68: outDec = 21640; break;
            case 69: outDec = (uint16_t)switchcntr; break;
            default: break;
         }
         return outDec;   
}



class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};


// values to be changed in eeprom
void change_val(uint16_t progcode, uint16_t progvalue) {
           switch(progcode) { 
            case 21300: ldrlim = progvalue; break; 
            case 21330: templim = progvalue; break; 
            case 21450: switch1del = progvalue; break; 
            case 21460: switch1on = progvalue; break; 
            case 21470: switch1off = progvalue; break; 
            case 21500: switch2del = progvalue; break; 
            case 21510: switch2on = progvalue; break; 
            case 21520: switch2off = progvalue; break; 
            case 21550: switch3del = progvalue; break; 
            case 21560: switch3on = progvalue; break; 
            case 21570: switch3off = progvalue; break; 
            case 21600: switch4del = progvalue;  break; 
            case 21610: switch4on = progvalue;  break; 
            case 21620: switch4off = progvalue;  break;
            default: break;
         }
}


class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      //std::string rxValue = pCharacteristic->getValue();
      std::string rxValue1 = pCharacteristic->getValue();
      //rxValInt1 = (uint16_t)(std::stoul(rxValue1));
      rxValue1.copy(rxCopy, rxValue1.length());  // copy string to rxCopy
      
      if (rxValue1.length() > 0) {
        
        Serial.println("*********");
        Serial.println("Received Value: ");
        Serial.println(rxCopy);
        Serial.println();
        Serial.println("*********");
        //rx_Value = std::strtol(rxValue, NULL, 16);  // (uint16_t)(std::stoi(rxValue));
        //rx_Value = (uint16_t)(std::stoi(rxValue));  // (uint16_t)(std::stoi(rxValue));
        //rx_Value = rxValInt1;
        //rxString = rxValue1;
      }

      // break up the received string separated by ";"
      tokenCount=0;

      for (char* token = strtok(rxCopy, ";");
        token != NULL                  &&   tokenCount != max_token_count;
      //^ Loop until there are tokens  and  ^ buffer not overflown
        token = strtok(NULL, ";")) {
        tokens[tokenCount++] = token;
      //                     ^ No need to allocate memory here, if you
      //                       are going to use tokens before str goes
      //                       out of scope.
      }
      size_t i = 0;
      while (i< tokenCount) {
        change_val(atoi(tokens[i]), atoi(tokens[i+1]));
        //Serial.println(tokens[i]);         
        i=i+2;
      }
      writeEeprom(); // save values in eeprom
      readEeprom();
      
      Serial.println("*Eeprom updated*");
      //for (size_t i = 0; i != tokenCount; i++) {
      //  Serial.println(tokens[i]);
      //}

      readEeprom();
      Serial.println("*Eeprom read*");

    }
};

uint16_t analogReadCorr(byte pin){
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  double readCorr;
  if(reading < 1 || reading > 4095) return 0;
    readCorr = -0.000000012191 * pow(reading,3) + 0.000020945 * pow(reading,2) + 1.0605 * reading + 81.206;
  return (uint16_t)readCorr;
} // Added an improved polynomial, use either, comment out as required


/* ADC readings v voltage
 *  y = -0.000000000009824x3 + 0.000000016557283x2 + 0.000854596860691x + 0.065440348345433
 // Polynomial curve match, based on raw data thus:
 *   464     0.5
 *  1088     1.0
 *  1707     1.5
 *  2331     2.0
 *  2951     2.5 
 *  3775     3.0
 *  
 */
 
/* ADC readings v ADC out
 *  y = -0.000000012191x3 + 0.000020945x2 + 1.0605x + 81.206
 // Polynomial curve match, based on raw data thus:
 *   464     0.5
 *  1088     1.0
 *  1707     1.5
 *  2331     2.0
 *  2951     2.5 
 *  3775     3.0
 *  
 */


uint16_t readEepromWord(uint8_t a1,uint8_t a2) {
  uint16_t wva1;
  wva1=(uint16_t)(((EEPROM.read(a1))<<8)+EEPROM.read(a2));
  if (wva1 == 65535) {
    wva1=1;
    initwrite=1;  // write eeprom first time
    if (a1==16) {    // fill LDRLIM with sensible value
      wva1=LDRLIM;
    }
    if (a1==18) {    // fill TEMPLIM with 100C value
      wva1=TEMPLIM;
    }
  }
  return wva1;
}

void readEeprom() {
  switch1on=  readEepromWord(0,1);  
  switch2on=  readEepromWord(2,3); 
  switch3on=  readEepromWord(4,5); 
  switch4on=  readEepromWord(6,7); 
  switch1off= readEepromWord(8,9); 
  switch2off= readEepromWord(10,11); 
  switch3off= readEepromWord(12,13); 
  switch4off= readEepromWord(14,15); 
  ldrlim=     readEepromWord(16,17); 
  templim=    readEepromWord(18,19); 
  switch1del= readEepromWord(20,21); 
  switch2del= readEepromWord(22,23); 
  switch3del= readEepromWord(24,25); 
  switch4del= readEepromWord(26,27); 
}


void writeEeprom() {
    EEPROM.write(0,(uint8_t)(switch1on>>8));
    EEPROM.write(1,(uint8_t)(switch1on));
    EEPROM.write(2,(uint8_t)(switch2on>>8));
    EEPROM.write(3,(uint8_t)(switch2on));
    EEPROM.write(4,(uint8_t)(switch3on>>8));
    EEPROM.write(5,(uint8_t)(switch3on));
    EEPROM.write(6,(uint8_t)(switch4on>>8));
    EEPROM.write(7,(uint8_t)(switch4on));
    
    EEPROM.write(8,(uint8_t)(switch1off>>8));
    EEPROM.write(9,(uint8_t)(switch1off));  
    EEPROM.write(10,(uint8_t)(switch2off>>8));
    EEPROM.write(11,(uint8_t)(switch2off));  
    EEPROM.write(12,(uint8_t)(switch3off>>8));
    EEPROM.write(13,(uint8_t)(switch3off));  
    EEPROM.write(14,(uint8_t)(switch4off>>8));
    EEPROM.write(15,(uint8_t)(switch4off));  

    EEPROM.write(16,(uint8_t)(ldrlim>>8));
    EEPROM.write(17,(uint8_t)(ldrlim));

    EEPROM.write(18,(uint8_t)(templim>>8));
    EEPROM.write(19,(uint8_t)(templim));

    EEPROM.write(20,(uint8_t)(switch1del>>8));
    EEPROM.write(21,(uint8_t)(switch1del));
    EEPROM.write(22,(uint8_t)(switch2del>>8));
    EEPROM.write(23,(uint8_t)(switch2del));
    EEPROM.write(24,(uint8_t)(switch3del>>8));
    EEPROM.write(25,(uint8_t)(switch3del));
    EEPROM.write(26,(uint8_t)(switch4del>>8));
    EEPROM.write(27,(uint8_t)(switch4del));
  
    EEPROM.commit();
}


void switchOffAll() {

   rtc_gpio_set_level(SWITCH1,0);    
   switch1state=0;
   rtc_gpio_set_level(SWITCH2,0);    
   switch2state=0;
   rtc_gpio_set_level(SWITCH3,0);    
   switch3state=0;
   rtc_gpio_set_level(SWITCH4,0);    
   switch4state=0;
   switchcntr=0;
   switch1lim=switch1del;
   switch2lim=switch2del;
   switch3lim=switch3del;
   switch4lim=switch4del;
  
}

void checkinputs() {

//    connectnow = digitalRead(CONNECTBLE);
//  Serial.printf("Connect now = %d\n",connectnow);
  tempValue = analogRead(TEMPIN);
  Serial.printf("CONNECTBLE analog value = %d\n",tempValue);  
  if ((tempValue > BUTTONON) && (progstate==0)) { // button pressed
    bleOnCounter = 200;     // counter to determine length of BLE on
    progstate=10;           // enable first progstate
    fastrunCounter = 4*bleOnCounter;  // counter to keep running fast    
    delay(1000);
  }


    
    if (!deviceConnected) { digitalWrite(LED, HIGH); };
    digitalWrite(SENSOUT, HIGH);   // Turn on Sensor supply
    delay(1000);
    // tempntcValue = analogRead(TEMPIN);  // 
    // ldrValue = analogRead(LDRIN);
    tempntcValue = analogReadCorr(TEMPIN); 
    ldrValue = analogReadCorr(LDRIN);    
    Serial.printf("LDRIN analogRead value = %d\n",ldrValue);        
    Serial.printf("TEMPIN analog value = %d\n",tempntcValue);
    if (!deviceConnected) { digitalWrite(LED, LOW); };                                                                                                                                                                                                                                                                                               

    //if (!deviceConnected) {
      //progValue1 = analogRead(PROGVAL1);
      //progValue2 = analogRead(PROGVAL2);
      //progValue1 = analogReadCorr(PROGVAL1);
      //progValue2 = analogReadCorr(PROGVAL2);
      
      //Serial.printf("PROGVAL1 analog value = %d\n",progValue1);
      //Serial.printf("PROGVAL2 analog value = %d\n",progValue2);
    //}
      
    digitalWrite(SENSOUT, LOW);   // Turn the RGB LED white
    // print out the values you read:
  
    Serial.printf("\n");
    Serial.printf("SW1 DEL %d ON %d OFF %d||SW2 DEL %d ON %d OFF %d ||SW3 DEL %d ON %d OFF %d ||SW4 DEL %d ON %d OFF %d \n",switch1del,switch1on, switch1off,switch2del,switch2on, switch2off,switch3del,switch3on, switch3off,switch4del,switch4on, switch4off);
    Serial.printf("\n");



    switch1return = analogRead(SWITCH1RET);

    if (switch1state == 1) {
      //switch1return = analogRead(SWITCH1RET);
      if (switch1return>SWITCHRETSHORT) {
        Serial.printf("SWITCH1 is SHORTED\n");
      } else {
        Serial.printf("SWITCH1 is ON, SWITCH1RET is low\n");
      }
    } else {
      Serial.printf("SWITCH1RET analog value = %d\n",switch1return);       
    }

    switch2return = analogRead(SWITCH2RET);
    if (switch2state == 1) {
      if (switch2return>SWITCHRETSHORT) {
        Serial.printf("SWITCH2 is SHORTED\n");
      } else {
        Serial.printf("SWITCH2 is ON, SWITCH2RET is low\n");
      }      
    } else {
      Serial.printf("SWITCH2RET analog value = %d\n",switch2return);
    }
    
    switch3return = analogRead(SWITCH3RET);
    if (switch3state == 1) {
      if (switch3return>SWITCHRETSHORT) {
        Serial.printf("SWITCH3 is SHORTED\n");
      } else {
        Serial.printf("SWITCH3 is ON, SWITCH3RET is low\n");
      }       
    } else {    
      Serial.printf("SWITCH3RET analog value = %d\n",switch3return);
    }
    
    switch4return = analogRead(SWITCH4RET);
    if (switch4state == 1) {
      if (switch4return>SWITCHRETSHORT) {
        Serial.printf("SWITCH4 is SHORTED\n");
      } else {
        Serial.printf("SWITCH4 is ON, SWITCH4RET is low\n");
      }         
    } else {     
      Serial.printf("SWITCH4RET analog value = %d\n",switch4return);
    }
    

}

uint16_t compress1( uint16_t x) {
  // compress to improve control for low values
  // max value is SWITCHMAX
  uint32_t xcomp;
  uint32_t xdiv = 4096*4096/SWITCHMAX;
  xcomp = (((uint32_t)x)*((uint32_t)x))/xdiv;
  return ((uint16_t)xcomp);
}

uint16_t compress2( uint16_t x) {
  // compress to improve control for low values
  // max value is 4096
  uint32_t xcomp;
  uint32_t xdiv = 4096;
  xcomp = (((uint32_t)x)*((uint32_t)x))/xdiv;
  return ((uint16_t)xcomp);
}

void program_led() {

  digitalWrite(LED, HIGH);   // LED
  delay(100);  
  digitalWrite(LED, LOW);   // LED

}

void program_switch() {

  digitalWrite(LED, HIGH);   // LED
  //Serial.printf("progstate %d \n",progstate);

  digitalWrite(LED, LOW);   // LED

  /*
  
  switch1return = analogRead(SWITCH1RET);
  // print out the values you read:
  //Serial.printf("SWITCH1RET millivolts value = %d\n",analogVolts);

  if (switch1return >SWITCHRETMIN) {
    digitalWrite(SENSOUT, HIGH);   // Power sensors
    delay(100);    
    progValue1 = analogRead(PROGVAL1);
    progValue2 = analogRead(PROGVAL2);
    if (progstate==10) {
      switch1on =  (uint16_t)(1+compress1(progValue1)); 
      switch1off = (uint16_t)(1+compress1(progValue2));       
      Serial.printf("SWITCH1 ON %d OFF %d (Minutes) Press Button again to PROGRAM\n",switch1on, switch1off);
    }
    if (progstate==11) {
      switch1del =  (uint16_t)(1+compress1(progValue1)); 
      Serial.printf("SWITCH1 DEL %d (Minutes) Press Button again to PROGRAM\n",switch1del);
    }

    digitalWrite(SENSOUT, LOW);   // Power Sensor Off
  }
    switch2return = analogRead(SWITCH2RET);

 
  if (switch2return >SWITCHRETMIN) {
    digitalWrite(SENSOUT, HIGH);   // Power sensors
    delay(100);    
    progValue1 = analogRead(PROGVAL1);
    progValue2 = analogRead(PROGVAL2);
    if (progstate==10) {    
      switch2on =  (uint16_t)(1+compress1(progValue1)); 
      switch2off = (uint16_t)(1+compress1(progValue2)); 
      Serial.printf("SWITCH2 ON %d OFF %d (Minutes) Press Button again to PROGRAM\n",switch2on, switch2off);
    }
    if (progstate==11) {    
      switch2del =  (uint16_t)(1+compress1(progValue1)); 
      Serial.printf("SWITCH2 DEL %d (Minutes) Press Button again to PROGRAM\n",switch2del);
    }
    digitalWrite(SENSOUT, LOW);   // Power Sensor Off
  }
    switch3return = analogRead(SWITCH3RET);

  // print out the values you read:
  //Serial.printf("SWITCH2RET millivolts value = %d\n",analogVolts);

 
  if (switch3return >SWITCHRETMIN) {
    digitalWrite(SENSOUT, HIGH);   // Power sensors
    delay(100);    
    progValue1 = analogRead(PROGVAL1);
    progValue2 = analogRead(PROGVAL2);
    if (progstate==10) {        
      switch3on =  (uint16_t)(1+compress1(progValue1)); 
      switch3off = (uint16_t)(1+compress1(progValue2)); 
      Serial.printf("SWITCH3 ON %d OFF %d (Minutes) Press Button again to PROGRAM\n",switch3on, switch3off);
    }
    if (progstate==11) {        
      switch3del =  (uint16_t)(1+compress1(progValue1)); 
      Serial.printf("SWITCH3 DEL %d (Minutes) Press Button again to PROGRAM\n",switch3del);
    }
    digitalWrite(SENSOUT, LOW);   // Power Sensor Off
  }
 
  
    switch4return = analogRead(SWITCH4RET);

  // print out the values you read:
  //Serial.printf("SWITCH2RET millivolts value = %d\n",analogVolts);

 
  if (switch4return >SWITCHRETMIN) {
    digitalWrite(SENSOUT, HIGH);   // Power sensors
    delay(100);    
    progValue1 = analogRead(PROGVAL1);
    progValue2 = analogRead(PROGVAL2);
    if (progstate==10) {        
      switch4on =  (uint16_t)(1+compress1(progValue1)); 
      switch4off = (uint16_t)(1+compress1(progValue2)); 
      Serial.printf("SWITCH4 ON %d OFF %d (Minutes)Press Button again to PROGRAM\n",switch4on, switch4off);
    }
    if (progstate==11) {
      switch4del =  (uint16_t)(1+compress1(progValue1));       
      Serial.printf("SWITCH4 DEL %d (Minutes)Press Button again to PROGRAM\n",switch4del);
    }
    digitalWrite(SENSOUT, LOW);   // Power Sensor Off
  }
 

 
  if ((analogRead(SWITCH1RET)<SWITCHRETMIN) && (analogRead(SWITCH2RET)<SWITCHRETMIN) && (analogRead(SWITCH3RET)<SWITCHRETMIN) && (analogRead(SWITCH4RET)<SWITCHRETMIN) ) {
    digitalWrite(SENSOUT, HIGH);   // Power sensors
    delay(100);    
    progValue1 = analogRead(PROGVAL1);
    progValue2 = analogRead(PROGVAL2);
    if (progstate==10) {
      ldrlim =  (uint16_t)(compress2(progValue1)); 
      templim =  (uint16_t)(compress2(progValue2)); 
      progldrtemp = 1;
      Serial.printf("LDRLIM %d TEMPLIM %d Press Button again to PROGRAM\n",ldrlim, templim);    
    }
    digitalWrite(SENSOUT, LOW);   // Power Sensor Off
  }
 

    tempValue = analogRead(TEMPIN);
    //Serial.printf("TEMPVAL %d \n",tempValue);    
    if (tempValue > BUTTONON) { // button pressed 
      delay(1000); 
      switch(progstate) { 
         case 10: progstate=11; if (progldrtemp==1) {progstate=13; progldrtemp=0;  writeEeprom(); } break; 
         case 11: progstate=13; writeEeprom(); break; 
         default: progstate=0; break;
      }
    }

    if (progstate==13) {
      Serial.printf("PROGRAM READY\n");    
    }
    */
}

void update_switches() {

 Serial.println("LDRvalue<ldrlim: " + String(ldrValue)+ " < " + String(ldrlim) +  " || tempValue>templim: " + String(tempntcValue)+ " > " + String(templim));

 if ( tempntcValue > (templim+(1-tempstate)*HYSTERESIS) ) { // check temperature
  Serial.println("tempValue>templim: " + String(tempntcValue)+ " > " + String(templim));
  tempstate = 1;
     
  if ( ldrValue < (ldrlim-((1-ldrstate)*HYSTERESIS)))    { // check Light sensor 
   Serial.println("LDRvalue<ldrlim: " + String(ldrValue)+ " < " + String(ldrlim)  );
   ldrstate = 1;
   Serial.println("SWITCH1state: " + String(switch1state) + " || SWITCH2state: " + String(switch2state) + " || SWITCH3state: " + String(switch3state) + " || SWITCH4state: " + String(switch4state)  );
  
   if ((switch1state == 1) || ((analogRead(SWITCH1RET) >SWITCHRETMIN) && (switch1state == 0))) {
    if (switchcntr >= switch1lim) {
      if (switch1state == 1) {
        //digitalWrite(SWITCH1, LOW);   
        rtc_gpio_set_level(SWITCH1,0);
        switch1state=0;
        switch1lim=switchcntr+switch1off;
        Serial.println("SWITCH1 OFF: switchcntr: " + String(switchcntr) + " || new switch1lim: " + String(switch1lim));
      } else {
        if ( (switch2state==0) && (switch3state==0) && (switch4state==0)) {
          digitalWrite(SWITCHRST, HIGH);   // enable reset for relay
          delay(1000);
          digitalWrite(SWITCHRST, LOW);    // end reset for relay  
          switch1Short=0;
          switch1Overload=0;
          
          k=5;
          do {                  
            rtc_gpio_set_level(SWITCH1,1);   
            delayMicroseconds(1);
            rtc_gpio_set_level(SWITCH1,0);
            delayMicroseconds(1);
            k--;   
          } while (k>0);
          CheckSwitch1Return = analogRead(SWITCH1RET);  // check short
          Serial.println("SW1 RET: " + String(CheckSwitch1Return));   
          Serial.println("SW1 RET LIM: " + String((switch1return-(switch1return*SHORTLIM/100))) );    
          if (CheckSwitch1Return>(switch1return-(switch1return*SHORTLIM/100)))  {
              rtc_gpio_set_level(SWITCH1,0);
              Serial.printf("SWITCH1 is SHORTED\n");
              switch1Short=1;
           } else {
              rtc_gpio_set_level(SWITCH1,1);   
              delay(10);
              CheckSwitch1Return = analogReadCorr(SWITCH1RET);  // check short SWITCHRETSHORT
              Serial.println("ARC_Switch1Return > SWITCHRETSHORT: " + String(CheckSwitch1Return) + " > " + String(SWITCHRETSHORT) + " SWITCH1 load too high?\n");
              if (CheckSwitch1Return>SWITCHRETSHORT)  {
                rtc_gpio_set_level(SWITCH1,0);   
                switch1Overload=1;                
              } else {
                switch1state=1;
                switch1lim=switchcntr+switch1on;            
                Serial.println("SWITCH1 ON: switchcntr: " + String(switchcntr) + " || new switch1lim: " + String(switch1lim) + " || SWITCH1RETURN " + String(CheckSwitch1Return));           
              }
            }
        }
      }
    }
   }

  
   if ((switch2state == 1) || ((analogRead(SWITCH2RET) >SWITCHRETMIN) && (switch2state == 0))) {
    if (switchcntr >= switch2lim) {
      if (switch2state == 1) {
        //digitalWrite(SWITCH2, LOW);   
        rtc_gpio_set_level(SWITCH2,0);    
        switch2state=0;
        switch2lim=switchcntr+switch2off;
        Serial.println("SWITCH2 OFF: switchcntr: " + String(switchcntr) + " || new switch2lim: " + String(switch2lim));
      } else {
        if ( (switch1state==0) && (switch3state==0) && (switch4state==0)) {
          digitalWrite(SWITCHRST, HIGH);   // enable reset for relay
          delay(1000);
          digitalWrite(SWITCHRST, LOW);    // end reset for relay     
          switch2Short=0;
          switch2Overload=0;
                                        
          k=5;
          do {                  
            rtc_gpio_set_level(SWITCH2,1);   
            delayMicroseconds(1);
            rtc_gpio_set_level(SWITCH2,0);
            delayMicroseconds(1);
            k--;   
          } while (k>0);
          CheckSwitch2Return = analogRead(SWITCH2RET);  // check short
          Serial.println("SW2 RET: " + String(CheckSwitch2Return));   
          Serial.println("SW2 RET LIM: " + String((switch2return-(switch2return*SHORTLIM/100))) );    
          if (CheckSwitch2Return>(switch2return-(switch2return*SHORTLIM/100)))  {
              rtc_gpio_set_level(SWITCH2,0);
              switch2Short=1;
              Serial.printf("SWITCH2 is SHORTED\n");
           } else {
              rtc_gpio_set_level(SWITCH2,1);   
              delay(10);
              CheckSwitch2Return = analogReadCorr(SWITCH2RET);  // check short SWITCHRETSHORT
              Serial.println("ARC_Switch2Return > SWITCHRETSHORT: " + String(CheckSwitch2Return) + " > " + String(SWITCHRETSHORT) + " SWITCH2 load too high?\n");
              if (CheckSwitch2Return>SWITCHRETSHORT)  {
                switch2Overload=1;
                rtc_gpio_set_level(SWITCH2,0);   
              } else {
                switch2state=1;
                switch2lim=switchcntr+switch2on;            
                Serial.println("SWITCH2 ON: switchcntr: " + String(switchcntr) + " || new switch2lim: " + String(switch2lim) + " || SWITCH2RETURN " + String(CheckSwitch2Return));           
              }
            }
        }
      }
    }
   }

   if ((switch3state == 1) || ((analogRead(SWITCH3RET) >SWITCHRETMIN) && (switch3state == 0))) {
    if (switchcntr >= switch3lim) {
      if (switch3state == 1) {
        //digitalWrite(SWITCH3, LOW);   
        rtc_gpio_set_level(SWITCH3,0);            
        switch3state=0;
        switch3lim=switchcntr+switch3off;
        Serial.println("SWITCH3 OFF: switchcntr: " + String(switchcntr) + " || new switch3lim: " + String(switch3lim));
      } else {
        if ( (switch1state==0) && (switch2state==0) && (switch4state==0)) {
          digitalWrite(SWITCHRST, HIGH);   // enable reset for relay
          delay(1000);
          digitalWrite(SWITCHRST, LOW);    // end reset for relay             
          switch3Short=0;
          switch3Overload=0;
              
          k=5;
          do {                  
            rtc_gpio_set_level(SWITCH3,1);   
            delayMicroseconds(1);
            rtc_gpio_set_level(SWITCH3,0);
            delayMicroseconds(1);
            k--;   
          } while (k>0);
          CheckSwitch3Return = analogRead(SWITCH3RET);  // check short
          Serial.println("SW3 RET: " + String(CheckSwitch3Return));   
          Serial.println("SW3 RET LIM: " + String((switch3return-(switch3return*SHORTLIM/100))) );    
          if (CheckSwitch3Return>(switch3return-(switch3return*SHORTLIM/100)))  {
              rtc_gpio_set_level(SWITCH3,0);
              Serial.printf("SWITCH3 is SHORTED\n");
              switch3Short=1;
           } else {
              rtc_gpio_set_level(SWITCH3,1);   
              delay(10);
              CheckSwitch3Return = analogReadCorr(SWITCH3RET);  // check short SWITCHRETSHORT
              Serial.println("ARC_Switch3Return > SWITCHRETSHORT: " + String(CheckSwitch3Return) + " > " + String(SWITCHRETSHORT) + " SWITCH3 load too high?\n");
              if (CheckSwitch3Return>SWITCHRETSHORT)  {
                rtc_gpio_set_level(SWITCH3,0);   
                switch3Overload=1;
              } else {
                switch3state=1;
                switch3lim=switchcntr+switch3on;            
                Serial.println("SWITCH3 ON: switchcntr: " + String(switchcntr) + " || new switch3lim: " + String(switch3lim) + " || SWITCH3RETURN " + String(CheckSwitch3Return));           
              }
            }
        }
      }
    }
   }
 
   if ((switch4state == 1) || ((analogRead(SWITCH4RET) >SWITCHRETMIN) && (switch4state == 0))) {
    if (switchcntr >= switch4lim) {
      if (switch4state == 1) {
        //digitalWrite(SWITCH4, LOW);   
        rtc_gpio_set_level(SWITCH4,0);    
        switch4state=0;
        switch4lim=switchcntr+switch4off;
        Serial.println("SWITCH4 OFF: switchcntr: " + String(switchcntr) + " || new switch4lim: " + String(switch4lim));
      } else {
        if ( (switch1state==0) && (switch2state==0) && (switch3state==0)) {
          digitalWrite(SWITCHRST, HIGH);   // enable reset for relay
          delay(1000);
          digitalWrite(SWITCHRST, LOW);    // end reset for relay 
          switch4Short=0;
          switch4Overload=0;
                 
          k=5;
          do {                  
            rtc_gpio_set_level(SWITCH4,1);   
            delayMicroseconds(1);
            rtc_gpio_set_level(SWITCH4,0);
            delayMicroseconds(1);
            k--;   
          } while (k>0);
          CheckSwitch4Return = analogRead(SWITCH4RET);  // check short
          Serial.println("SW4 RET: " + String(CheckSwitch4Return));   
          Serial.println("SW4 RET LIM: " + String((switch4return-(switch4return*SHORTLIM/100))) );    
          if (CheckSwitch4Return>(switch4return-(switch4return*SHORTLIM/100)))  {
              rtc_gpio_set_level(SWITCH4,0);
              Serial.printf("SWITCH4 is SHORTED\n");
              switch4Short=1;
           } else {
              rtc_gpio_set_level(SWITCH4,1);   
              delay(10);
              CheckSwitch4Return = analogReadCorr(SWITCH4RET);  // check short SWITCHRETSHORT
              Serial.println("ARC_Switch4Return > SWITCHRETSHORT: " + String(CheckSwitch4Return) + " > " + String(SWITCHRETSHORT) + " SWITCH4 load too high?\n");
              if (CheckSwitch4Return>SWITCHRETSHORT)  {
                rtc_gpio_set_level(SWITCH4,0);   
                switch4Overload=1;
              } else {
                switch4state=1;
                switch4lim=switchcntr+switch4on;            
                Serial.println("SWITCH4 ON: switchcntr: " + String(switchcntr) + " || new switch4lim: " + String(switch4lim) + " || SWITCH4RETURN " + String(CheckSwitch4Return));           
              }
            }
        }
      }
    }
   }

  } else { // check on light 
   Serial.println("LDRvalue<ldrlim: " + String(ldrValue)+ " < " + String(ldrlim));
   if (!( ldrValue < ldrlim )) {
      Serial.println("Evironment too dark LDRvalue<ldrlim: " + String(ldrValue)+ " < " + String(ldrlim) );      
   }
   ldrstate = 0;

   // switch off everything

   switchOffAll();

  }
 } else { // check temperature
   Serial.println("tempValue>templim: " + String(tempntcValue)+ " > " + String(templim));
   
   if (!( tempntcValue > templim )) {
      Serial.println("Evironment too hot  tempValue>templim: " + String(tempntcValue)+ " > " + String(templim) );          
   }
   tempstate = 0;
   
   // switch off everything

   switchOffAll();
 
 }

}

void setup() {

  analogReadResolution(12);
  
  Serial.begin(115200);



  rtc_gpio_deinit((gpio_num_t)CONNECTBLE);

  //rtc_gpio_init(PTEST);
  //rtc_gpio_set_direction(PTEST, RTC_GPIO_MODE_OUTPUT_ONLY);

  rtc_gpio_init(SWITCH1);
  rtc_gpio_set_direction(SWITCH1, RTC_GPIO_MODE_OUTPUT_ONLY);

  rtc_gpio_init(SWITCH2);
  rtc_gpio_set_direction(SWITCH2, RTC_GPIO_MODE_OUTPUT_ONLY);

  rtc_gpio_init(SWITCH3);
  rtc_gpio_set_direction(SWITCH3, RTC_GPIO_MODE_OUTPUT_ONLY);

  rtc_gpio_init(SWITCH4);
  rtc_gpio_set_direction(SWITCH4, RTC_GPIO_MODE_OUTPUT_ONLY);

  ESP32CPP::GPIO::setInput((gpio_num_t)CS);
  ESP32CPP::GPIO::setInput((gpio_num_t)CLK);
  //ESP32CPP::GPIO::setInput((gpio_num_t)MISO);
  ESP32CPP::GPIO::setOutput((gpio_num_t)MOSI);

  
  //gpio_deep_sleep_hold_dis();
  //ESP32CPP::GPIO::setOutput((gpio_num_t)SWITCH1);
  //gpio_hold_dis((gpio_num_t) SWITCH1);
  //gpio_deep_sleep_hold_dis();

  ESP32CPP::GPIO::setInput((gpio_num_t)SWITCH1RET);
  //ESP32CPP::GPIO::setOutput((gpio_num_t)SWITCH2);
  //gpio_hold_dis((gpio_num_t) SWITCH2);
  //gpio_deep_sleep_hold_dis();

  ESP32CPP::GPIO::setInput((gpio_num_t)SWITCH2RET);
  //ESP32CPP::GPIO::setOutput((gpio_num_t)SWITCH3);
  //gpio_hold_dis((gpio_num_t) SWITCH3);
  //gpio_deep_sleep_hold_dis();

  ESP32CPP::GPIO::setInput((gpio_num_t)SWITCH3RET);
  //ESP32CPP::GPIO::setOutput((gpio_num_t)SWITCH4);
  //gpio_hold_dis((gpio_num_t) SWITCH4);
  
  ESP32CPP::GPIO::setInput((gpio_num_t)SWITCH4RET);
  
  ESP32CPP::GPIO::setOutput((gpio_num_t)SWITCHRST);
  ESP32CPP::GPIO::setOutput((gpio_num_t)SENSOUT);
  ESP32CPP::GPIO::setInput((gpio_num_t)LDRIN);
  ESP32CPP::GPIO::setInput((gpio_num_t)TEMPIN);
  ESP32CPP::GPIO::setInput((gpio_num_t)PROGVAL1);
  ESP32CPP::GPIO::setInput((gpio_num_t)PROGVAL2);
  ESP32CPP::GPIO::setInput((gpio_num_t)CONNECTBLE);
  ESP32CPP::GPIO::setInput((gpio_num_t)VDD3V3);  
  ESP32CPP::GPIO::setOutput((gpio_num_t)LED);

  EEPROM.begin(EEPROM_SIZE);

  readEeprom();

  if (initwrite==1) { // initially, eeprom is filled with 0xFFFF, Fill with 0x0001
    writeEeprom();
  }
  if (fastrunCounter > 1000) {  // intially fastrunCounter is0xFFFF
    fastrunCounter=0;
  }
  bootCount++;
  switchcntr++;
  Serial.println("Boot number: " + String(bootCount));
  vbatinit = analogRead(VDD3V3);  // battery voltage
  Serial.println("Initial Battery voltage: " + String(vbatinit));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  
  txString.reserve(2049);
  txString = "";
  rxString.reserve(2049);
  
  txString += "A characteristic value can be up to 20 bytes long. This is a key constraint in designing services. [...] You could also combine readings into a single characteristic, when a given sensor or actuator has multiple values associated with it.";

  // Create the BLE Device
  BLEDevice::init("Switch Box");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_READ | // external can read 
                    BLECharacteristic::PROPERTY_WRITE |
                    BLECharacteristic::PROPERTY_NOTIFY
                  );

                      
  pCharacteristic->addDescriptor(new BLE2902());

  checkinputs();

  update_switches();

  //gpio_hold_en((gpio_num_t) SWITCH1);
  //gpio_hold_en((gpio_num_t) SWITCH2);
  //gpio_hold_en((gpio_num_t) SWITCH3);   
  //gpio_hold_en((gpio_num_t) SWITCH4);  
  gpio_deep_sleep_hold_en();    
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);          // IO, sensors and ULP co-processor
  //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);     // RTC fast memory.
  //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEMRTC, ESP_PD_OPTION_OFF);  // RTC slow memory.
  //esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);             // XTAL oscillator
  //esp_sleep_pd_config(ESP_PD_DOMAIN_RC_FAST, ESP_PD_OPTION_OFF);          // Internal Fast oscillator.
  //esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_OFF);          // VDD_SDIO.
  //esp_sleep_pd_config(ESP_PD_DOMAIN_MODEM, ESP_PD_OPTION_OFF);            // MODEM, includes WiFi, Bluetooth and IEEE802.15.4.
  //esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);              // Number of domains.



  if (bleOnCounter < 1) {                                                // normal operation
    if (fastrunCounter <1) {
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
    } else {
      esp_sleep_enable_timer_wakeup(4 * uS_TO_S_FACTOR); 
      Serial.println("Setup ESP32 to sleep for every " + String(4) + " Seconds, fastruncounter: " + String(fastrunCounter));
    }
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_13,1);                         // 1 = High, 0 = Low
  } else {                                                               // programming time
    esp_sleep_enable_timer_wakeup(1 * uS_TO_S_FACTOR);
  }


  if (bleOnCounter < 1) {
    //Go to sleep now
    fastrunCounter--;
    Serial.println("Going to sleep now");
    Serial.flush();   
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
  } else {
    //delay(500); 
    pCharacteristic->setCallbacks(new MyCallbacks());
    // Start the service
    pService->start();
    // Start advertising
    pServer->getAdvertising()->start();
  }
  
}



void loop() {


    if (deviceConnected) {

        if (bleOnCounter>0) {
          delay(1000);  
          digitalWrite(LED, HIGH);   // LED double blink for connected
          delay(100);  
          digitalWrite(LED, LOW);   // LED
          delay(100);  
          digitalWrite(LED, HIGH);   // LED
          delay(100);  
          digitalWrite(LED, LOW);   // LED
          delay(100);  


          //bleOnCounter--;
          //fastrunCounter--;
          if ((bleOnCounter%50)==0) {
            Serial.println("C");          
          } else {
            Serial.print("C");
          }
        } else {
          deviceConnected=0;
          oldDeviceConnected = deviceConnected;
          delay(500); // give the bluetooth stack the chance to get things ready
          pServer->startAdvertising(); // restart advertising

          //Go to sleep now
          Serial.println("Going to sleep now");
          Serial.flush();   
          esp_deep_sleep_start();
        }

//        if (eepromRead) {
//            readEeprom();
//            eepromRead = true;
//        }
        readEeprom();
        checkinputs(); 
        
        i=0;
        do {
          txString="";
          sprintf(buffer2, "%d", makedec(i) ); 
          txString += buffer2;
          txString += ";"; 
          i++;
          sprintf(buffer2, "%d", makedec(i) ); 
          txString += buffer2;
          txString += ";"; 
          i++;
          pCharacteristic->notify();
          pCharacteristic->setValue(txString);
          delay(100);                            // bluetooth stack will go into congestion, if too many packets are sent
        } while (i<70); 

        switchOffAll();

        
	  } else {   // NO BLE device Connected
      if (bleOnCounter>0) {
        if ((bleOnCounter%50)==0) {
          Serial.println(".");          
        } else {
          Serial.print(".");
        }
        bleOnCounter--;
        fastrunCounter--;
        delay(50);
        // program_switch(); // progam by two potmeters
        program_led(); // blink led
      } else {
        //Go to sleep now
        Serial.println("Going to sleep now");
        Serial.flush();   
        esp_deep_sleep_start();              
      }
	  }
    //ESP32CPP::GPIO::write((gpio_num_t)MOSI,(bool)0);

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
		// do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}
