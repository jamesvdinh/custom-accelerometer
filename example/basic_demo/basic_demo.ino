//#include <Arduino_LSM9DS1.h>
/* 2/14/20
    to change the .cpp file, need to change 
    and save the .cpp file in the local folder
    for this arduino project
    works to pass parameter to adjust output data rate from a program
    tested for ODR of 125 Hz, 250 Hz
    from pin Data_Ready output as observed on oscilloscope
    default ODR is 4000 Hz and no high pass filtering
    need to have the backup file in another 
    folder for this sketch to compile, if not get errors
    using the F() macro to save memory for ASCII text
    works also on TinyDuino
    changed variable ID to DEVICE_ID in Seed_adxl357b.h and .cpp
    to avoid name conflict with another ID in SPIFlash.h
    2/17/20
    Made a backup copy for this sketch folder on 2/16/20
    basic_demobackup21620201 -> basic_demo
    disable all println statements to test interrupt on pin 2
    changed deal_cali_buf to do_cali_bUF
    
    From page 4 on datahseet for the +-40g range, scale factor multiplier
    to raw data is 78 micro-g/LSB multiply scale factor 
    with the raw data to get in unit of acceleration of gravity g
    works with a hybrid interrupt and loop combination
    
    The DATA_READY signal from ADXL357 goes low when data is read
    On oscilloscope, can observe that the DATA_READY pulse width 
    got shorter as data is read in loop()
    
    If no reading of data, the pulse width is 50% duty cycle
    Can't place read inside an interrupt since no response 
    and code too deep into i2c
    
    ODR up to 125Hz and pulse width is aobut 600 micro seconds
    ODR at 62.5 Hz pulse width 700 micro seconds
    see page 29 of data sheet
    
    2/18/20
    In ISR1 set s=true, pin 13 high and reset s in loop() and 
    after read accelerometer data then set pin 13 low 
    to indicate completion of read data 
    tested to work at 125 Hz
    
    *****input pin 2 interrupt signal to arduino from ADXL357 Data_Ready output
    *****output pin 4 for oscillocscope monitor of reading ADXL357 
    *****and write to flash memory
    *****SPI flash memory connections on TinyDuino adapter boards
    *****ADXL357 connections A4 and A5

7/10/21 compiles ok

7/9/22  compiles ok, with warnings only
    The Analog Devices ADXL357 is set to use I2C for communication
    The external flash memroy for TinyDuino is set to use SPI for communication

    Must have the Seeed Studio .h header file and the C++ file in the
    same folder as this sketch

    Use ifdef ABC block to not compile defined(ARDUINO_ARCH_AVR)
    the C preprocessor flags multiple definition of SERIAL which
    also is defined in avr code

    compiles ok, no more warnings for multiple definition of SERIAL
*/
#include <SPIFlash.h>
#include "Seeed_adxl357b.h"

#ifdef ABC
#if defined(ARDUINO_ARCH_AVR)
#pragma message(F("Defined architecture for ARDUINO_ARCH_AVR."))
#define SERIAL Serial
//#endif // 7/9/22
//#ifdef SERIAL1
#elif defined(ARDUINO_ARCH_SAM)
#pragma message(F("Defined architecture for ARDUINO_ARCH_SAM."))
#define SERIAL SerialUSB
#elif defined(ARDUINO_ARCH_SAMD)
#pragma message(F("Defined architecture for ARDUINO_ARCH_SAMD."))
#define SERIAL SerialUSB
#elif defined(ARDUINO_ARCH_STM32F4)
#pragma message(F("Defined architecture for ARDUINO_ARCH_STM32F4."))
#define SERIAL SerialUSB
#else
#pragma message(F("Not found any architecture."))
#define SERIAL Serial
#endif
#endif // ABC

// for SPI flash memory stuff
#define MAX 100 /* multiple number of memory places to store flash data this 
               // might not be same as the actual number of bytes
               // Analog Devices ADXL357 accelerometer data is 3 bytes 
               // but C programs only has 2 or 4 bytes data
               // For example, at 62.5 Hz output data rate for ADXL357,
               // and for 10 seconds of data 
               // collection => 10 sec x 62.5 Hz = 625 data points
               // each data point is 4 bytes in 
               // lenght => 4 x 625 data points = 2500 memory bytes 
               to be stored in flash
               */
#define START  0x0 // start address to store data in flash
//#define READONLY   // for testing if flash keeps data after power off

// Analog Devices ADXL357 stuff
//#define ABC // if define then do the calibration according 
//to this Seed basic_demo code
#define CALI_BUF_LEN           15  // for 16 samples
#define CALI_INTERVAL_TIME     250 // this is for a delay
#define ODR                    0x05 // see page 38 table 44 for settings 
                               /* register address 0x28 for the output data rate
                                 0x05 for 125 Hz
                                 0x06 for 62.5 Hz
                                 0x07 for 31.25 Hz
                                 0x0A for 3.906 Hz
                               */
#define DELAY1                  10 // for loop() delay in msec, for blinking LED
#define DELAY2                  1990 // for blinking LED off
#define SCALE_FACTOR_40G        (78E-6) // 40g range scale factor is 78microg/LSB
#define DELAY 10000  // delay in milliseconds

// interrupt declarations
void ISR1();  // interrupt routine for testing
volatile boolean s = true; // for interrupt rountine ISR1
volatile int i = 0; // global counter for the number of memory places to store data

// SPI flash memory declarations
const int flashCS = 5; // The chip/slave select pin is pin 5 for the TinyDuino
const bool errorCheck = true; // A boolean parameter used to check for writing errors. Turned on by default.
const bool fastRead = false; // A boolean parameter used to implement a fast read function. Defaults to false.
uint32_t address; // The specific address in memory to be written to. (Address value MUST be type uint32_t)
SPIFlash flash(flashCS); // The SPIFlash object for the chip. Passed the chip select pin in the constructor.

// Analog Deivces ADXL357 declarations 

struct accelerometer_data {
  int32_t x; // for the 3 axis data x, y and z
  int32_t y;
  int32_t z;
  float t; // for the temperature from accelerometer
};

struct accelerometer_data *d; // declare pointer for accelorometer data structure

Adxl357b  adxl357b; // an instance for the ADXL357 object

void setup(void) {
  //uint8_t value = 0;
  //float t;
  
/**************** SPI flash setup stuff *******************/
  pinMode(flashCS, OUTPUT); // Ensure chip select pin is an output. This line is currently REQUIRED for proper operation.
  flash.begin(); // Boots the flash memory
  address = START;
  unsigned int n = MAX; // multiple for the number of 4 bytes memory places

#define ERASE1  
#ifdef ERASE1   
  //flash.eraseSector(address); // Must erase memory before writing per datasheet.
  flash.eraseSection(address,n*sizeof(d->x)); // Must erase memory before writing per datasheet.
#endif // ERASE1

/**************** ADXL357 accelerometer setup stuff *********************/
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // dynamically allocate the calibration structure 
  // since calibration done then not use anymore
  d = (struct accelerometer_data*) malloc(sizeof(struct accelerometer_data));

  //SERIAL.begin(115200);
  Serial.begin(115200);
  if (adxl357b.begin()) {
    //SERIAL.println(F("Can't detect ADXL357B device ."));
    Serial.println(F("Can't detect ADXL357B device ."));
    while (1);
  }
  //SERIAL.println(F("Init OK!"));
  Serial.println(F("Init OK!"));
  /*Set full scale range to ±40g*/
  adxl357b.setAdxlRange(FOURTY_G);
  /*Switch standby mode to measurement mode.*/
  adxl357b.setPowerCtr(0);

  /* 2/14/20 set the output data rate in register 0x28 to 125Hz
    see line 121 in .cpp file */
  //adxl357b.setFilter();
  adxl357b.setOutputDataRateandFilter(ODR); // 0x05 for high pass filter off and 125 Hz ODR
  // 0x06 for high pass filter off and 62.5 Hz ODR
  delay(100);
  /*Read Uncalibration temperature.*/
  adxl357b.readTemperature(d->t);

  //SERIAL.print("Uncalibration  temp = ");
  //SERIAL.println(d->t);
  Serial.println(d->t);
  /**/
  //free(d);
  
  /********************interrupt ISR stuff******************/
  pinMode(4, OUTPUT);           // pin 4 output for oscilloscope monitor of ISR1
  delay(DELAY);
  attachInterrupt(0, ISR1, RISING);
}

// interrupt service routine to signal 
// to main loop to acquire data from ADXL357
void ISR1(){
  s =  true; // for the next low to high to pin 2 then turn LED off
  digitalWrite(4, s);
  i=i+1;  // increment this global counter to store the 
          // MAX number of data points (4 bytes each data point)
          // the switch s is used in loop() to save data or not
} // end ISR1

#ifdef RSI1
// interrupt routine for testing
void ISR1()
{  
  s = true; // for the next low to high to pin 2 then do LED
  digitalWrite(LED_BUILTIN, HIGH);
  
} // end ISR1
#endif // RSI1

#define LOOP
#ifdef LOOP
void loop(void) {

//int32_t x, y, z;

#ifndef WRITE1
// the switch s is set using pin 2 as an interrupt by the data ready 
// signal from ADXL357 accelerometer ODR output pin
//
  if (s) {
    //s=!s;
        
#ifndef WRITE2
    if (i < MAX) {
        if (adxl357b.checkDataReady()) {
            if (adxl357b.readXYZAxisResultData(d->x,d->y,d->z)) {
              //Serial.println(F("Get data failed!"));
            } // end if
            else
            {
              if(flash.writeLong(address + i*sizeof(long), (SCALE_FACTOR_40G*(d->x)), errorCheck)) {
                //if(flash.writeLong(address, x, errorCheck)) {
                digitalWrite(4, LOW);
                s=!s; // turn s off until set by data ready of accelerometer
                //digitalWrite(LED_BUILTIN, LOW); // turn the LED on with low to high transition on digital pin 2 
                } // end if
              } // end else              
           } // end if
    } // end if (i)      
#endif // WRITE2    
  } // end if(s)
  //else
    //digitalWrite(LED_BUILTIN, LOW);   // turn the LED on with low to high transition on digital pin 2

#endif // WRITE1

} // end loop
#endif // LOOP
