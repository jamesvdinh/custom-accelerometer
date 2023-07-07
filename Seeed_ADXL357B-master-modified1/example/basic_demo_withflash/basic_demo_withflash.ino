//#include <Arduino_LSM9DS1.h>
/*  added comment 7/9/22
 *  This sketch does two things 
 *  1) Acquire data from the Analog Device ADXL357 accelerometer using the I2C
 *  2) Save the acquired data to external flash memory using SPI
 *  This sketch uses the ODR signal from the accelerometer 
 *  to interrupt the microcontroller to acquire the data 
 *  
 *  2/14/20
    to change the .cpp file, need to change and save the .cpp file in the local folder
    for this arduino project
    works to pass parameter to adjust output data rate from a program
    tested for ODR of 125 Hz, 250 Hz
    from pin Data_Ready output as observed on oscilloscope
    default ODR is 4000 Hz and no high pass filtering
    need to have the backup file in another folder for this sketch to compile, if not get errors
    using the F() macro to save memory for ASCII text
    works also on TinyDuino
    changed variable ID to DEVICE_ID in Seed_adxl357b.h and .cpp
    to avoid name conflict with another ID in SPIFlash.h
    2/17/20
    Made a backup copy for this sketch folder on 2/16/20
    basic_demobackup21620201 -> basic_demo
    disable all println statements to test interrupt on pin 2
    changed deal_cali_buf to do_cali_bUF
    
    From page 4 on datahseet for the +-40g range, scale factor 
    multiplier to raw data is 78 micro-g/LSB
    multiple scale factor with the raw data to 
    get in unit of acceleratiuon of gravity g
    works with a hybrid interrupt and loop combination
    
    The DATA_READY signal from ADXL357 goes low when data is read
    On oscilloscope, can observe that the DATA_READY pulse width 
    got shorter as data is read in loop()
    If no reading of data, the pulse width is 50% duty cycle
    Can't place read inside an interrupt since no 
    response and code too deep into i2c
    ODR up to 125Hz and pulse width is aobut 600 micro seconds
    ODR at 62.5 Hz pulse width 700 micro seconds
    see page 29 of data sheet
    2/18/20
    In ISR1 set s=true, pin 13 high and reset s in loop() and 
    after read accelerometer data then set 
    pin 13 low to indicate completion of read data 
    tested to work at 125 Hz
    
    *******input pin 2 interrupt signal from ADXL357 Data_Ready output
    *******output pin 4 for oscillocscope monitor 
    *******of reading ADXL357 and write to flash memory
    *******SPI flash memory connections on Tiny adapter boards
    *******ADXL357 connections A4 and A5
    2/19/20 
    *******This sketch is now to run on the TinyDuino  
    Works in TinyDuino to change ODR and ISR ok
    Works to read data from ADXL357 and store data to flash
    Tested for 125Hz, 62.5Hz, 31.25 Hz ok
    need to use integer (long) data type to store the raw data bytes to flash
    do not multiply by the scale factor to store the raw data
    Do the multiplication in an analysis software 
    from data stored into SD card from flash
    2/24/20
    Tested with 31.25 Hz 100, 200, 300, 400, 500, 600, 625 
    and 125Hz at 650 data points saved
    works first data point bad
    
    Test procedures were
    1) upload this sketch and take triggered data from ADXL357
    2) turn off power to TinyDuino
    3) remove Tiny usb adapter
    4) turn on power for TinyDuino
    5) let Duino take accelerometer data using coin cell battery power only
    6) turn off power
    7) remove Tiny Duino from Tiny flash shield
    8) connect usb Tiny adapter
    9) upload a blank sketch to Tiny Duino
    10) connect Tiny Duino to flash shield
    11) upload sketch FlashMem_TinyShield_example_readonly
    12) verify data from flash memory
    
    works
    ************* Arduino connections ****************************
                  I2C connections
                  arduino pin A4  ->  ADXL357 pin SDA
                  arduino pin A5  ->  ADXL357 pin SCL
                              
                  Arduino external interrupt connections
                  arduino pin 2   ->  ADXL357 DATA_READY pin
                  arduino pin 4   ->  oscilloscope monitor for interrupt triggered   

    3/2/20
    testing with soldered ADXL357 to Tiny proto board
    
    download sketch using Pololu ISP adapter instead of TinyDuino USB adapter
    mosfet burned likely on TinyDuino usb adapter
    works
    
    Added comment 7/9/22
    Do not use this if TinyDuino USB adapter works
    to download arduino code to TinyDuino ok
    The below connections are for a
    Pololu AVR progrommer board used as a USB adapter
    to download arduino code to the TinyDuino microcontroller
    in the case that the TinyDuino USB adapter do not work
    
    ************* Pololu ISP connections ************************
    use pavr2gui & to get the linux device file to download sketch
    ISP has Serial to TTL
               arduino pin 0 RX ->  Pololu ISP Tx
               arduino pin 1 TX ->  Pololu ISP Rx
               arduino reset pin -> Pololu ISP DTR pin output B (default to DTR)
               need to connect a 0.1 microFarad cap. in series with DTR signal
               arduino ground pin -> Pololu ISP ground
  
  7/9/22
  Made ifdef ABC directive to not process preprocessor directive
  for Arduino architecture for SERIAL
  if not then get warning message for multiple definition of SERIAL in avr
  changed all SERIAL.println statement to Serial.println                               
*/

#include <SPI.h>
//#include <SD.h>
#include "SdFat.h"
SdFat SD;

#define SD_CS_PIN SS

#define MAX 625 /* multiple number of memory places to store flash data this 
               // might not be same as the actual number of bytes
               // Analog Devices ADXL357 accelerometer data is 3 bytes 
               // but C programs only has 2 or 4 bytes data
               // For example, at 62.5 Hz output data rate for ADXL357,
               // and for 10 seconds of data collection => 10 sec x 62.5 Hz = 625 data points
               // each data point is 4 bytes in lenght => 4 x 625 data points = 2500 memory bytes 
               to be stored in flash
               */
#define START  0x0 // start address to store data in flash
//#define READONLY   // for testing if flash keeps data after power off

// Decide what Serial port to use!
#ifdef ARDUINO_ARCH_SAMD
  #define SerialMonitor SerialUSB // Our TinyScreen+ is an SAMD board.
#else
  #define SerialMonitor Serial
#endif

// function declarations
void readflashwritetosdcard(); // read the flash memory and write the data to the SD Card

// SPI flash memory declarations
uint16_t page; // The page to be written to. (Page value MUST be type uint16_t)
uint8_t offset; // The specific location on the page. (Offset value MUST be type uint8_t)

// SD card declarations

File myFile;

/* Original Libraries */
#include <SPIFlash.h>
#include "Seeed_adxl357b.h"

#ifdef ABC // 7/9/22
#if defined(ARDUINO_ARCH_AVR)
#pragma message(F("Defined architecture for ARDUINO_ARCH_AVR."))
#define SERIAL Serial
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
// for SPI flash stuff
#define MAX 40 /* multiple number of memory places to store flash data this 
               // might not be same as the actual number of bytes
               // Analog Devices ADXL357 accelerometer data is 3 bytes 
               // but C programs only has 2 or 4 bytes data
               // For example, at 62.5 Hz output data rate for ADXL357,
               // and for 10 seconds of data collection => 10 sec x 62.5 Hz = 625 data points
               // each data point is 4 bytes in lenght => 4 x 625 data points = 2500 memory bytes 
               to be stored in flash
               */
#define START  0x0 // start address to store data in flash
//#define READONLY   // for testing if flash keeps data after power off

// for ADXL357 stuff
#define ODR                    0x0A // see page 38 table 44 for settings 
                               /* register address 0x28 for the output data rate
                                 0x05 for 125 Hz and
                                 0x06 for 62.5 Hz
                                 0x07 for 31.25 Hz
                                 0x0A for 3.906 Hz
                               */
#define SCALE_FACTOR_40G        (78E-6) // 40g range scale factor is 78 micro g/LSB
                                        // multiply the raw accelerometer data by this factor
                                        // to get resull in unit of g=9.8 meter/sec2
#define DELAY 1000  // delay in milliseconds to attach the ISR routine

// interrupt declarations
void ISR1();  // interrupt routine for testing
volatile boolean s = true; // for interrupt rountine ISR1
volatile int i = 0; // global counter for the number of memory places to store data

// SPI flash declarations
const int flashCS = 5; // The chip/slave select pin is pin 5.
const bool errorCheck = true; // A boolean parameter used to check for writing errors. Turned on by default.
const bool fastRead = false; // A boolean parameter used to implement a fast read function. Defaults to false.
uint32_t address; // The specific address in memory to be written to. (Address value MUST be type uint32_t)
SPIFlash flash(flashCS); // The SPIFlash object for the chip. Passed the chip select pin in the constructor.

// ADXL357 declarations 
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

/**************** ADXL357 setup stuff *********************/
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // dynamically allocate the calibration structure since calibration done then not use anymore
  d = (struct accelerometer_data*) malloc(sizeof(struct accelerometer_data));

  Serial.begin(9600);
  if (adxl357b.begin()) {
    Serial.println(F("Can't detect ADXL357B device . Check I2C connections"));
    while (1);
  }
  Serial.println(F("Init OK for ADXL357!"));
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

  Serial.print("Uncalibration  temp = ");
  Serial.println(d->t);

//  calibration();
  /**/
  //free(d);

  /****************** SD Card setup section 1 begin **************************/
  
  Serial.print(F("Initializing SD card..."));

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("initialization failed!"));
    return;
  }
  Serial.println(F("initialization done."));

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test1.dat", FILE_WRITE);
  
  if(myFile) Serial.println(F("open SD card file sucess"));
  //myFile.close();
  Serial.println("Writing from flash to SD card:");
//  readflashwritetosdcard();  // do read flash and write data to SD card
  
/******************end SD card setup section 1 ***************************/

  /********************interrupt ISR setup stuff******************/
  pinMode(4, OUTPUT);           // pin 4 for oscilloscope monitor of 
                                // reading ADXL357 data and writing data to flash
  delay(DELAY);                 // wait before setting up the ISR
  attachInterrupt(0, ISR1, RISING);
}

void ISR1(){
  s =  true; // switch s to loop to read data from ADXL357
  if (i<MAX) {
    digitalWrite(4, s); // signal on pin 4 at ISR
    i=i+1;  // increment this global counter to store the 
          // MAX number of data points (4 bytes each data point)
  } // end if
  else digitalWrite(4, !s); // signal on pin 4 at ISR
} // end ISR1

void readflashwritetosdcard()
{
  long x;
  Serial.println(F("Start reading from flash memory ..."));
  for(unsigned int i=0; i < MAX; i=i+1) {  
    char printBuffer[50]; // for formatting the random values, using sprintf
    sprintf(printBuffer, "%d:The 4 bytes at address %d is: ", i,(address + i*sizeof(long)));
    SerialMonitor.print(printBuffer);
    x = flash.readLong(address + i*(sizeof(long)), fastRead);    
    SerialMonitor.print(x);
    #define SAVE1
    #ifdef SAVE1
      if (myFile) {
        Serial.print(F("\tWriting to test1.dat ok\n"));
        myFile.println(x); // save data to SD card
      } else
        // if the file didn't open, print an error:
        Serial.println(F("error opening test1.dat"));
    #endif // SAVE1
  } // end for i loop
  myFile.close();
  Serial.println(F("done."));  
} // end readflashwritetosdcard

void loop(void) {
/* 
 *  The if logic blocks here goes like this
 *  wait for external signal from ADXL357 Data_Ready output to trigger interrupt ISR1
 *  Upon external trigger
 *  1) switch s set to tru
 *  2) pin 4 goes high
 *  3) counter i increments
 *  4) all the while loop() goes through until s is set true
 *  5) if counter i is less than the maximum number of data points
 *  6) read the i2c bus data from ADXL357
 *  7) store the data to flash memory 
 *  8) Save only one axis of accelerometer for now
 */
#define WRITE1
#ifdef WRITE1
  if (s) {
#define WRITE2        
#ifdef WRITE2
    if (i < MAX) { // test the global counter for MAX
        if (adxl357b.checkDataReady()) { // Read from accelerometer
            if (adxl357b.readXYZAxisResultData(d->x,d->y,d->z)) {
              Serial.println(F("Get data failed!"));
            } // end if
            else
            {
              // Save to flash
              if(flash.writeLong(address + i, (d->z), errorCheck)) {
                digitalWrite(4, LOW);
                // Write from flash to SD
                readflashwritetosdcard();
                s=!s;
                } // end if
              } // end else              
           } // end if
    } // end if (i)
    //else digitalWrite(4, LOW); // bring the output on pin 4 back to low      
#endif // WRITE2    
  } // end if(s)
  //else
    //digitalWrite(LED_BUILTIN, LOW);   // turn the LED on with low to high transition on digital pin 2

#endif // WRITE1

} // end loop
