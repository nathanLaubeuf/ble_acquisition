/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#include <SimpleFIFO.h>

#include "ATSAMD21_ADC.h"

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

int digital_VCC_Pin = 10;            // digital VCC set using PWM pin
int digital_GND_Pin = 11;           // digital GNDs set using PWM pin
// int voltref = 13;

int sensorPinSens[] = {A0, A1, A2, A3, A4, A5, 9};     // ADC sensor inputs

int freq = 90;                      // frequency

//******************************
// Runtime parameters
//******************************

// int errors = 0;

bool acquire = 0; // acquisition flag

bool interSend = 0; // Flag to send the datas every N acquisition cycle

bool reverse = 0; // Voltage direction

// ADC valueswill be written in this variable
unsigned int measure = 0;
unsigned int tosend = 0;

int i = 0;


//******************************
// Benchmark
//******************************

unsigned long time;
unsigned long timed[2] = {0, 0};

SimpleFIFO<unsigned int,100> sFIFO;

void error(const __FlashStringHelper*err) {
  // Serial.println(err);
  while (1);
}

//******************************
// Setup
//******************************
void setup() {

  // Set ADC prescaller t o16 to avoid noise
  analogPrescaler(ADC_PRESCALER_DIV16);

  Serial.println(F("----------BLE ADC acquisition----------"));
  Serial.println(F("---------------------------------------"));

  //------------------------------
  // Initialization BLE Module
  //------------------------------
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
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

  ble.verbose(false);  // debug info is a little annoying after this point!

  //------------------------------
  // Setup sensors parameters
  //------------------------------

  //initialize digital digital_VCC_Pin
  pinMode(digital_VCC_Pin, OUTPUT);
  digitalWrite(digital_VCC_Pin, HIGH);
  pinMode(digital_GND_Pin, OUTPUT);
  digitalWrite(digital_GND_Pin, LOW);

  //------------------------------
  // Setup Timer
  //------------------------------
  startTimer(freq);

  //------------------------------
  // Wait for ble connection
  //------------------------------
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

    // LED Activity command is only supported from 0.6.6
    if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
    {
      // Change Mode LED Activity
      Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
      ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    }
}


//******************************
// loop
//******************************
void loop() {

  // Timer reliablility monitoring
  // timed[1] = timed[0];
  // timed[0] = millis();
  // Serial.println(timed[0]-timed[1]);

  // Debug not stuck in interrupt
  // Serial.println(F("Loop"));

  if (acquire == 1){
    if(ble.available()){
      // Read order for app
      ble.readline();
    }

    if (strcmp(ble.buffer, "STOP") == 0) {
      acquire = 0;
      Serial.println(F("Acquisition Stop"));
      // The +++ command switches between command and data mode
      ble.println("+++");
      Serial.println("Stopped");
    }

    i=0;
    // Empties Queue while it''s not empty
    while(sFIFO.count()!= 0){
      tosend = sFIFO.dequeue();
      // Serial.println(tosend & 1023);
      ble.write(highByte(tosend));
      ble.write(lowByte(tosend));
      i++;
    }

  }
  else{
    // Wait for order to start
    ble.println("AT+BLEUARTRX");
    ble.readline();
    if (strcmp(ble.buffer, "START") == 0) {
      Serial.println(F("Acquisition Start"));
      // Set module to DATA mode
      // The +++ command switches between command and data mode
      ble.println("+++");
      //Enable timer interrupts
      acquire = 1;
    }
  }
}

//******************************
// Interrupt routine TIMER1
//******************************

void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // timed[1] = timed[0];
  // timed[0] = millis();
  // Serial.println(timed[0]-timed[1]);
  // If this interrupt is due to the compare register matching the timer count
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;

    // timed[1] = timed[0];
    // timed[0] = millis();
    // Serial.println(timed[0]-timed[1]);
    // Debug interupt executed
    // Serial.println(F("Interrupt"));
    // Timer reliablility monitoring

    if(reverse == 0){
      // Debug interupt executed
      // Serial.println(F("Interrupt 0"));
      digitalWrite(digital_GND_Pin, LOW);
      digitalWrite(digital_VCC_Pin, HIGH);
      measure = analogRead(sensorPinSens[0]) + 0;
      sFIFO.enqueue(measure);
      // Serial.print("Measured");
      // Serial.println(measure);
      measure = analogRead(sensorPinSens[1]) + (17<<10);
      sFIFO.enqueue(measure);
      measure = analogRead(sensorPinSens[2]) + (18<<10);
      sFIFO.enqueue(measure);
      measure = analogRead(sensorPinSens[3]) + (3<<10);
      sFIFO.enqueue(measure);
      measure = analogRead(sensorPinSens[4]) + (20<<10);
      sFIFO.enqueue(measure);
      // measure = analogRead(sensorPinSens[5]) + (5<<10);
      // sFIFO.enqueue(measure);
      // measure = analogRead(sensorPinSens[6]) + (6<<10);
      // sFIFO.enqueue(measure);
      reverse = 1;
    }
    else{
      // Debug interupt executed
      // Serial.println(F("Interrupt 1"));
      digitalWrite(digital_VCC_Pin, LOW);
      digitalWrite(digital_GND_Pin, HIGH);
      measure = analogRead(sensorPinSens[0]) + (48<<10);
      sFIFO.enqueue(measure);
      // Serial.print("Measured");
      // Serial.println(measure - (48<<10));
      measure = analogRead(sensorPinSens[1]) + (33<<10);
      sFIFO.enqueue(measure);
      measure = analogRead(sensorPinSens[2]) + (34<<10);
      sFIFO.enqueue(measure);
      measure = analogRead(sensorPinSens[3]) + (51<<10);
      sFIFO.enqueue(measure);
      measure = analogRead(sensorPinSens[4]) + (36<<10);
      sFIFO.enqueue(measure);
      // measure = analogRead(sensorPinSens[5]) + (53<<10);
      // sFIFO.enqueue(measure);
      // measure = analogRead(sensorPinSens[6]) + (54<<10);
      // sFIFO.enqueue(measure);
      reverse = 0;
    }
  }
}
