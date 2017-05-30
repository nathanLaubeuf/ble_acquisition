#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#include "timerSetup.h"


/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

//******************************
// Sensors parameters
//******************************

int digital_VCC_Pin[] = {5};            // digital VCC set using PWM pin
int digital_GND_Pin = 11;           // digital GNDs set using PWM pin
// int voltref = 13;

int sensorPinSens[] = {A0, A1, A2, A3, A4};     // ADC sensor inputs

int measure = 0;

int freq = 50;                      // frequency

int i = 0;                          // counter
int j = 0;                          // counter`
int errors = 0;


void error(const __FlashStringHelper*err) {
  // Serial.println(err);
  while (1);
}

bool acquire = 0; // acquisition flag
String data = ""; // Data string to be sent via ble UART
bool interSend = 0; // Flag to send the datas every N acquisition cycle
bool reverse = 0; // Voltage direction

//******************************
// Benchmark
//******************************

unsigned long time;



//******************************
// Setup
//******************************
void setup() {
  // while (!Serial);  // required for Flora & Micro
  delay(500);

  analogReadResolution(12);

  // pinMode(voltref, OUTPUT);
  // digitalWrite(voltref, HIGH);
  //
  // analogReference(AR_EXTERNAL);

  Serial.println(F("----------BLE ADC acquisition----------"));
  Serial.println(F("---------------------------------------"));

  //------------------------------
  // Initialization BLE Module
  //------------------------------
  Serial.print(F("Initialising the Bluefruit LE module: "));
  if ( !ble.begin(VERBOSE_MODE) ){
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );
  if ( FACTORYRESET_ENABLE ){
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }
  /* Disable command echo from Bluefruit */
  ble.echo(false);
  
  Serial.println(F("Requesting Bluefruit info:"));
  /* Print Bluefruit information */
  ble.info();
  Serial.println(F("UART mode activated"));
  // Serial.println();
  ble.verbose(false);  // debug info is a little annoying after this point!

  //------------------------------
  // Setup sensors parameters
  //------------------------------

  //initialize digital digital_VCC_Pin
  pinMode(digital_VCC_Pin[0], OUTPUT);
  // pinMode(digital_VCC_Pin[1], OUTPUT);
  // pinMode(digital_VCC_Pin[2], OUTPUT);
  digitalWrite(digital_VCC_Pin[0], HIGH);
  // digitalWrite(digital_VCC_Pin[1], HIGH);
  // digitalWrite(digital_VCC_Pin[2], HIGH);
  pinMode(digital_GND_Pin, OUTPUT);
  digitalWrite(digital_GND_Pin, LOW);

  for(i=0;i<10;i++){
    data[i] = 0;
  }

  //------------------------------
  // Setup Timer
  //------------------------------
  startTimer(freq);
      /* Wait for connection */

  //------------------------------
  // Wait for ble connection
  //------------------------------
  while (! ble.isConnected()) {
      delay(500);
  }
  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) ){
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
    Serial.println(F("Waiting for request"));
  }
}

//******************************
// loop
//******************************
void loop() {
  // Check for incoming characters from Bluefruit
  ble.println("AT+BLEUARTRX");
  ble.readline();
  if (acquire == 1){
    if (strcmp(ble.buffer, "STOP") == 0 || errors >= 20) {
      //Disable timer interrupts
      acquire = 0;
      Serial.println(F("Acquisition Stop"));
    }
    if(strcmp(ble.buffer, "ERROR") == 0){
      errors += 4;
      Serial.println(F("Error"));
    }
    if(interSend == 1){
      //----------Benchmark------------
      time = millis();
      Serial.print(F("Start Read at : "));
      Serial.println(time);
      //-------------------------------

      interSend = 0;
      data.concat(analogRead(sensorPinSens[0]));
      data += "|";
      data.concat(analogRead(sensorPinSens[1]));
      data += "|";
      data.concat(analogRead(sensorPinSens[2]));
      data += "|";
      data.concat(analogRead(sensorPinSens[3]));
      data += "|";
      data.concat(analogRead(sensorPinSens[4]));

      //----------Benchmark------------
      time = millis();
      Serial.print(F("Stop Read at : "));
      Serial.println(time);
      //-------------------------------
      j++;
      if(j==5){
        //----------Benchmark------------
        time = millis();
        Serial.print(F("Start Sent at : "));
        Serial.println(time);
        //-------------------------------

        data += " \n";
        ble.print("AT+BLEUARTTX=");
        ble.print(data);
        // Serial.print(data);
        data = "";
        j = 0;
        if(errors != 0){
          errors -= 2;
        }
        //----------Benchmark------------
        time = millis();
        Serial.print(F("Stop Sent at : "));
        Serial.println(time);
        //-------------------------------
      }
      else {
        data += " ";
      }
    }
  }
  if (acquire == 0) {
    if (strcmp(ble.buffer, "START") == 0) {
      //Enable timer interrupts
      errors = 0;
      acquire = 1;
      Serial.println(F("Acquisition Start"));
    }
  }
  if (strcmp(ble.buffer, "OK") != 0) {
    Serial.print(F("[Recv] "));
    Serial.println(ble.buffer);
    // ble.waitForOK();
  }
}

//******************************
// Interrupt routine TIMER1
//******************************

void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    interSend = 1;
    if(reverse == 0){
      digitalWrite(digital_GND_Pin, LOW);
      digitalWrite(digital_VCC_Pin[0], HIGH);
      // digitalWrite(digital_VCC_Pin[1], HIGH);
      // digitalWrite(digital_VCC_Pin[2], HIGH);
      data.concat(reverse);
      data += "|";
      reverse = 1;
    }
    else{
      digitalWrite(digital_VCC_Pin[0], LOW);
      // digitalWrite(digital_VCC_Pin[1], LOW);
      // digitalWrite(digital_VCC_Pin[2], LOW);
      digitalWrite(digital_GND_Pin, HIGH);
      data.concat(reverse);
      data += "|";
      reverse = 0;
    }
  }
  REG_TC3_INTFLAG = TC_INTFLAG_MC0;
}
