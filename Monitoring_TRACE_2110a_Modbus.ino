/*
 * Thís sketch is used to communicate with EPsolar Tracer 2210A
 * by Modbus protocol using a half-duplex RS485 transceiver
 * 
 * System hardware include:
 * 1/ Arduino UNO clone
 * 2/ Data logging sheild with SD card and RTC
 * 3/ RS485 transceiver
 * 
 * Created 03/04/2017
 * By Minh-Quan Dang
 * 
 * Modified 
 * 
 * 
 * The text of the Arduino reference is licensed under 
 * a Creative Commons Attribution-ShareAlike 3.0 License.
 * 
 * Reference could be found in examples of ModbusMaster, RTClib libraries
 */
//========================================================================
//===DECLARATION=========================================================
//========================================================================

//====DEFINE LIBRARY=====================================================
#include <ModbusMaster.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"

bool state = true;
int i = 1;

unsigned int pv_Volt, pv_Amps;
unsigned int batt_Volt, batt_Amps, batt_Watt;
unsigned int load_Amps, load_Watt;
unsigned int batt_Temp, sensor_Temp, SOC;
unsigned int con_KWH_D;
unsigned int gen_KWH_D;

//====Declaration for Modbus=============================================
/*!
  We're using a MAX485-compatible RS485 Transceiver.
  Rx/Tx is hooked up to the hardware serial port at 'Serial'.
  The Data Enable and Receiver Enable pins are hooked up as follows:
*/
#define MAX485_DE      3  //DE to pin3
#define MAX485_RE_NEG  2  //RE to pin2
// DI to pin TX
// RO to pin RX

// instantiate ModbusMaster object
ModbusMaster node;

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

//====Declaration for SD Card shield======================================
const int chipSelect = 10;

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below 
// if using programming port to program the Zero!
#define Serial SerialUSB
#endif

//====Declaration for RTC timer===========================================
RTC_Millis rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", 
                              "Thursday", "Friday", "Saturday"};

//====Declaration for blinking LED===========================================
int ledPin = 8;

//========================================================================
//===END DECLARATION======================================================
//========================================================================

void setup() {
  // put your setup code here, to run once:
  
// initialize digital pin LED_BUILTIN as an output.
    pinMode(ledPin, OUTPUT);
    
//----CHECK SD CARD------------------------------------------------------------
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  
//----START RTC------------------------------------------------------------
// following line sets the RTC to the date & time this sketch was compiled
    rtc.begin(DateTime(F(__DATE__), F(__TIME__)));

//----START MODBUS TRANSMITION------------------------------------------------
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // Modbus communication runs at 115200 baud
  Serial.begin(115200);

  // Modbus slave ID 1
  node.begin(1, Serial);
  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t result;
  uint16_t data[6];

switch (i){
  case 1: {
    node.readInputRegisters(0x3100, 20);
       Serial.println(" --3100--");
    pv_Volt     = node.getResponseBuffer(0x00);
    pv_Amps     = node.getResponseBuffer(0x01);

    batt_Volt   = node.getResponseBuffer(0x04);
    batt_Amps   = node.getResponseBuffer(0x05);
    batt_Watt   = node.getResponseBuffer(0x06);

    load_Amps    = node.getResponseBuffer(0x0d);
    load_Watt    = node.getResponseBuffer(0x0e);

    batt_Temp     = node.getResponseBuffer(0x10);

if (batt_Volt >0){
 digitalWrite(ledPin, HIGH);   
 delay(100);                       
 digitalWrite(ledPin, LOW);    
} else {
 digitalWrite(ledPin, HIGH);   
 delay(100);                       
 digitalWrite(ledPin, LOW); 
  delay(100);                       
  digitalWrite(ledPin, HIGH);   
 delay(100);                       
 digitalWrite(ledPin, LOW); 
  delay(100);                       
  digitalWrite(ledPin, HIGH);   
 delay(100);                       
 digitalWrite(ledPin, LOW); 
}
  }
  i++;
  break;
//-------------------------------------------------  
//------------------------------------------------- 
  case 2: {
    node.readInputRegisters(0x311a, 2);
       Serial.println(" --311a--");
    SOC           = node.getResponseBuffer(0x00);
    sensor_Temp   = node.getResponseBuffer(0x01);  
  }
  i++;
  break;
//------------------------------------------------- 
//------------------------------------------------- 
  case 3: {
    node.readInputRegisters(0x3300, 31);
       Serial.println(" --3300--");
    con_KWH_D         = node.getResponseBuffer(0x04);
    gen_KWH_D        = node.getResponseBuffer(0x0c);
  }
  i++;
  break;
//------------------------------------------------- 
  case 4:{
    i = 1;
        
       DateTime now = rtc.now();

   // make a string for assembling the data to log:
  String dataString = "";

  dataString = String(now.year(), DEC);
  dataString += ('-');
  dataString += String(now.month(), DEC);
  dataString += ('-');
  dataString += String(now.day(), DEC);
  dataString += ",";  
  dataString += String(now.hour(), DEC);
  dataString += (':');
  dataString += String(now.minute(), DEC);
  dataString += (':');
  dataString += String(now.second(), DEC);
  dataString += ",";
  dataString += String(pv_Volt);
  dataString += ",";
  dataString += String(pv_Amps);
  dataString += ",";
  dataString += String(batt_Volt);
  dataString += ",";
  dataString += String(batt_Amps);
  dataString += ",";
  dataString += String(batt_Watt);
  dataString += ",";
  dataString += String(SOC);
  dataString += ",";
  dataString += String(load_Amps);
  dataString += ",";
  dataString += String(load_Watt);
  dataString += ",";
  dataString += String(batt_Temp);
  dataString += ",";
  dataString += String(sensor_Temp);
  dataString += ",";
  dataString += String(con_KWH_D);
  dataString += ",";
  dataString += String(gen_KWH_D);
  
  String filename =  "";
  filename += String(now.year(), DEC);
  filename += String(now.month(), DEC);
  filename += String(now.day(), DEC);
  filename += ".txt";
 
 File dataFile = SD.open(filename, FILE_WRITE);
 Serial.println(filename);
  
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    Serial.println(dataString);
  }
  else {
     digitalWrite(ledPin, HIGH);   
     delay(100);                       
     digitalWrite(ledPin, LOW); 
      delay(100);                       
      digitalWrite(ledPin, HIGH);   
     delay(100);                       
     digitalWrite(ledPin, LOW); 
      delay(100);                       
      digitalWrite(ledPin, HIGH);   
     delay(100);                       
     digitalWrite(ledPin, LOW);
 }
  }
  
  delay(9700); // print data to SD card every 10s
  break;
  
}
 
  delay(200);
}
