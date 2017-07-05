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

#include <RingBuf.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

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
    #define FACTORYRESET_ENABLE         0
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"

    #define BLUEFRUIT_HWSERIAL_NAME           Serial1
    #define BLUEFRUIT_UART_MODE_PIN         -1   // Not used with FLORA
    #define BLUEFRUIT_UART_CTS_PIN          -1   // Not used with FLORA
    #define BLUEFRUIT_UART_RTS_PIN          -1   // Not used with FLORA
/*=========================================================================*/

// Define various ADC prescaler
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


int startTimer(int freq);

//******************************
// Sensors parameters
//******************************

int digital_VCC_Pin = 3;            // digital VCC set using PWM pin
int digital_GND_Pin = 5;           // digital GNDs set using PWM pin
// int voltref = 13;

int sensorPinSens[] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11};     // ADC sensor inputs

int freq = 44;                      // frequency

//******************************
// Runtime parameters
//******************************

// int errors = 0;

bool acquire = 0; // acquisition flag

// bool interSend = 0; // Flag to send the datas every N acquisition cycle
bool reverse = 0; // Voltage direction

// ADC valueswill be written in this variable
unsigned int measure = 0;
unsigned int tosend = 0;

char c = 0;


int i = 0;

RingBuf *data = RingBuf_new(sizeof(unsigned int), 100);

//******************************
// Benchmark
//******************************

unsigned long time;
unsigned long timed[2] = {0, 0};


void error(const __FlashStringHelper*err) {
  // Serial.println(err);
  while (1);
}

//******************************
// Setup
//******************************
void setup() {

  // set up the ADC
  ADCSRA &= ~PS_128;  // remove bits set by Arduino library

  // you can choose a prescaler from above.
  // PS_16, PS_32, PS_64 or PS_128
  ADCSRA |= PS_64;    // set our own prescaler to 64

  while (!Serial);  // required for Flora & Micro
  delay(500);

  // Serial.begin(115200);

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

    Serial.println("Requesting Bluefruit info:");
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

  // for(i=0;i<10;i++){
  //   data[i] = 0;
  // }

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
  // Check for incoming characters from Bluefruit
  // Timer reliablility monitoring
  // timed[1] = timed[0];
  // timed[0] = millis();
  // Serial.println(timed[0]-timed[1]);

  // Debug not stuck in interrupt
  // Serial.println(F("Loop"));

  if (acquire == 1){
    if(ble.available()){
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
    while(i<=100 || data->isEmpty(data)){
      data->pull(data, &tosend);
      // Serial.println(tosend & 1023);
      ble.write(highByte(tosend));
      ble.write(lowByte(tosend));
      i++;
    }

  }

  if (acquire == 0) {
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

int startTimer(int freq){
  // Stop interrupts
  cli();

  // Set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 16000000/1024/freq - 1;// = (16*10^6) / (2*50*1024) - 1 (OCR1A must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  //allow interrupts
  sei();
  return 0;
}

//******************************
// Interrupt routine TIMER1
//******************************


ISR(TIMER1_COMPA_vect){
  // timed[1] = timed[0];
  // timed[0] = millis();
  // Serial.println(timed[0]-timed[1]);
  // Debug interupt executed
  // Serial.println(F("Interrupt"));
  // Timer reliablility monitoring
  // interSend = 1;
  if(reverse == 0){
    // Debug interupt executed
    // Serial.println(F("Interrupt 0"));
    digitalWrite(digital_GND_Pin, LOW);
    digitalWrite(digital_VCC_Pin, HIGH);
    measure = analogRead(sensorPinSens[0]) + 0;
    data->add(data, &measure);
    // Serial.print("Measured");
    // Serial.println(measure);
    measure = analogRead(sensorPinSens[1]) + (17<<10);
    data->add(data, &measure);
    measure = analogRead(sensorPinSens[2]) + (18<<10);
    data->add(data, &measure);
    measure = analogRead(sensorPinSens[3]) + (3<<10);
    data->add(data, &measure);
    measure = analogRead(sensorPinSens[4]) + (20<<10);
    data->add(data, &measure);
    measure = analogRead(sensorPinSens[5]) + (5<<10);
    data->add(data, &measure);
    measure = analogRead(sensorPinSens[6]) + (6<<10);
    data->add(data, &measure);
    measure = analogRead(sensorPinSens[7]) + (23<<10);
    data->add(data, &measure);
    measure = analogRead(sensorPinSens[8]) + (24<<10);
    data->add(data, &measure);
    measure = analogRead(sensorPinSens[9]) + (9<<10);
    data->add(data, &measure);
    // measure = analogRead(sensorPinSens[10]) + (10<<10);
    // data->add(data, &measure);
    // measure = analogRead(sensorPinSens[11]) + (27<<10);
    // Serial.println(measure - (27<<10));
    // data->add(data, &measure);
    reverse = 1;
  }
  else{
    // Debug interupt executed
    // Serial.println(F("Interrupt 1"));
    digitalWrite(digital_VCC_Pin, LOW);
    digitalWrite(digital_GND_Pin, HIGH);
    measure = analogRead(sensorPinSens[0]) + (48<<10);
    data->add(data, &measure);
    // Serial.print("Measured");
    // Serial.println(measure - (48<<10));
    measure = analogRead(sensorPinSens[1]) + (33<<10);
    data->add(data, &measure);
    measure = analogRead(sensorPinSens[2]) + (34<<10);
    data->add(data, &measure);
    measure = analogRead(sensorPinSens[3]) + (51<<10);
    data->add(data, &measure);
    measure = analogRead(sensorPinSens[4]) + (36<<10);
    data->add(data, &measure);
    measure = analogRead(sensorPinSens[5]) + (53<<10);
    data->add(data, &measure);
    measure = analogRead(sensorPinSens[6]) + (54<<10);
    data->add(data, &measure);
    measure = analogRead(sensorPinSens[7]) + (39<<10);
    data->add(data, &measure);
    measure = analogRead(sensorPinSens[8]) + (40<<10);
    data->add(data, &measure);
    measure = analogRead(sensorPinSens[9]) + (57<<10);
    data->add(data, &measure);
    // measure = analogRead(sensorPinSens[10]) + (58<<10);
    // data->add(data, &measure);
    // measure = analogRead(sensorPinSens[11]) + (43<<10);
    // // Serial.print("Measured");
    // // Serial.println(measure - (43<<10));
    // data->add(data, &measure);
    reverse = 0;
  }
}
