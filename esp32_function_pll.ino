// ecoder da pau se PSRAM tiver ligada!!!!! 

#include "EEPROM.h"
#include "AiEsp32RotaryEncoder.h"
#include "Arduino.h"
#include "LedController.hpp"
#include <SPI.h>
#include <Wire.h>

//RTC_DATA_ATTR; 
//RTC_NOINIT_ATTR;
#include <TaskScheduler.h>

int buttoncount=0;
void t1Callback();
void t2Callback();
const int BUTTON_PIN = 2;
const unsigned long HOLD_TIME = 400; // hold time in milliseconds
bool buttonState = LOW;
unsigned long buttonPressTime = 0;



  TaskHandle_t Task1;
  TaskHandle_t Task2;

Task t1(10, TASK_FOREVER, &t1Callback);
Task t2(100, TASK_FOREVER, &t2Callback);
//Task t3(10, TASK_FOREVER, &t3Callback);
//Task t4(10000, TASK_FOREVER, &t4Callback);

Scheduler runner;


int Waveform;

long firstpress=0;
bool pressing=false;
const int FSYNC_PIN = 19;
const int Bootpin =0;
const unsigned long AD9833_FREQ_REGISTER = 0x4000;
const unsigned long AD9833_CONTROL_REGISTER = 0x2000;

int currentFreqReg = 0; // initialize to frequency register 0
float oldFrequency=0;
int delaytime=50; //(delaylcd)
byte modeOperation=0;
///delay between sweep steps lower is faster
//float startFreq ;//= 1000.00; // start frequency in Hz
//float endFreq;// = 10000.00; // end frequency in Hz
//float freqStep = 1.0; // frequency step size in Hz
//int waveform = SINE; // set waveform mode to SINE


//#define SLEEP_MODE      0x00C0    // Both DAC and Internal Clock
//#define DISABLE_DAC     0x0040
//#define DISABLE_INT_CLK   0x0080
//
//#define PHASE_WRITE_CMD   0xC000    // Setup for Phase write
//#define PHASE1_WRITE_REG  0x2000    // Which phase register
//#define FREQ0_WRITE_REG   0x4000    // 
//#define FREQ1_WRITE_REG   0x8000
//#define PHASE1_OUTPUT_REG 0x0400    // Output is based off REG0/REG1
//#define FREQ1_OUTPUT_REG  0x0800


const int SINE = 0x2000;                    // Define AD9833's waveform register value.
const int SQUARE = 0x2028;                  // When we update the frequency, we need to
const int TRIANGLE = 0x2002;   
//float Frequency= 5000.00; //============================================================================================SETS STARUP FREQ
uint32_t Freq_Word = 1.00;
uint32_t refFreq = 25000000;           // On-board crystal reference frequency
const unsigned long twenty_eight_bit = 268435456;
unsigned long incr = 1;
unsigned long oldIncr = 1;



class FLASHvariables 
 {                          //Class to hold various persistent variables we want to save to non-volatile SRAM
 
 public:
    //VARIABLES
float startFreq;
float endFreq;
uint8_t sweepstep; // função do pedal
float Frequency;
uint8_t sweepdelay;
uint8_t waveformindex;
uint8_t varsGood;

 //METHODS
    void save();
    void get();
    void initialize();
   
} myVars;

void FLASHvariables::save()   
{
  EEPROM.put(0,myVars);
  EEPROM.commit();
}

void FLASHvariables::get()
{
  EEPROM.begin(sizeof(myVars)); 
  EEPROM.get(0,myVars);  
}

void FLASHvariables::initialize()    
{
  //If no data can be found in flash RAM, initialize the variables held in Class Structure here.



//variaveis salvas menu 

//Variaveis 
startFreq=1000;
endFreq=10000;
sweepstep=1;
Frequency=5000;
sweepdelay=1;
waveformindex=1;

 varsGood=190;
 
  
  EEPROM.put(0,myVars);
  EEPROM.commit();
}

 //============END FLASHVARIABLES CLASS DEFINITION =======================


template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; } 
esp_chip_info_t chip_info; //Instantiate object chip_info of class esp_chip_info_t to access data on chip hardware



// Define the MAX7219 connection pins
#define DIN 15
#define CS 2
#define CLK 4

// Define the rotary encoder pins
#define ROTARY_ENCODER_A_PIN 5
#define ROTARY_ENCODER_B_PIN 17
#define ROTARY_ENCODER_BUTTON_PIN 16
#define ROTARY_ENCODER_VCC_PIN -1 /* 27 put -1 of Rotary encoder Vcc is connected directly to 3,3V; else you can use declared output pin for powering rotary encoder */
#define ROTARY_ENCODER_STEPS 4
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);


// Create a new LedController object
LedController<1, 1> lc;

bool circleValues = false;
unsigned long increaseTimer = 0;
unsigned long counter = 0;

// Define the frequency value (in kHz)
float encodervalue = 5000.00;
float digit=0;

int changeprecision=0;
static unsigned long lastTimePressed = 0;

  void selectSweep(){
Serial.println("selectsweep");

           
 }
              

void menu(){

      lc.clearMatrix();
   lc.setChar(0,7,'m',false);
   lc.setChar(0,6,'E',false);
    lc.setChar(0,5,'n',false);
     lc.setChar(0,4,'u',false);
     // lc.setChar(0,3,'R',false);
      //  lc.setChar(0,2,'E',false);
       //  lc.setChar(0,1,'q',false);
            vTaskDelay(2000);  

         
         bool menuselected = false;
  while (!menuselected) {
            vTaskDelay(10);  
            buttonState = digitalRead(Bootpin);
             if (buttonState == LOW) {
    buttonPressTime = millis();
    while (digitalRead(Bootpin) == LOW && millis() - buttonPressTime < HOLD_TIME) {
      buttoncount=1;
      // wait for button release or hold time exceeded
    }}
    // display waveform options and wait for button press
    encodervalue=rotaryEncoder.readEncoder() % 5;
    //waitButtonPress();
    rotary_loop(); //  rotary_onButtonClick();
  int menuindex = rotaryEncoder.readEncoder() % 5;
    setmenu(menuindex);
  //Serial.print("button count ");
//  Serial.println(buttoncount);
    // check button count
       if (buttoncount == 1) {
             // exit waveform selection mode
             menuselected = true;
          //    Serial.print(wave);
             // printBin(wave);

              buttoncount = 0;
             // rotaryEncoder.setEncoderValue(myVars.Frequency);   
             //  changeprecision = 0;
              vTaskDelay(100);
              
              if (menuindex==0) return selectminfreq();
              if (menuindex==1) return selectmaxfreq();
              if (menuindex==2) return selectSweepSteps();
              if (menuindex==3) return selectSweep();
              if (menuindex==4) return selectSweepDelay();
              }
  

    // set waveform based on encoder value
  

  }
 
}

void setmenu(int menuindex) {  //display dos nomes 
    vTaskDelay(10);
  switch (menuindex) {
    case 0:
      minFreq();
      break;
    case 1:
      maxFreq();
      break;
    case 2:
     SweepSteps();
      break;
      case 3:
     Sweep();
      break;
      case 4:
    SweepDelay();
    break;
    default:
      Sweep();//  Waveform=SQUARE;
      break;
  }
}

void minFreq(){
    lc.clearMatrix();

  lc.setChar(0,7,'m',false);
   lc.setChar(0,6,'i',false);
    lc.setChar(0,5,'n',false);
     lc.setChar(0,4,'F',false);
      lc.setChar(0,3,'R',false);
       lc.setChar(0,2,'E',false);
         lc.setChar(0,1,'q',false);
}

void selectSweepDelay(){
Serial.println("sweepdelay");
      lc.clearMatrix();
rotaryEncoder.setEncoderValue(myVars.sweepdelay); 

bool sweepdelayselected = false;
  while (!sweepdelayselected) {
            vTaskDelay(10);
  floatToDecimalNumber();
         buttonState = digitalRead(Bootpin);
             if (buttonState == LOW) {
    buttonPressTime = millis();
    while (digitalRead(Bootpin) == LOW && millis() - buttonPressTime < HOLD_TIME) {
      buttoncount=1;
      // wait for button release or hold time exceeded
    }}
    
    // display waveform options and wait for button press
  //  encodervalue=rotaryEncoder.readEncoder() % 4;
    //waitButtonPress();
 rotary_loop(); //  rotary_onButtonClick();
 
  //Serial.print("button count ");
//  Serial.println(buttoncount);
    // check button count
    if (buttoncount == 1) {
      // exit waveform selection mode
      sweepdelayselected = true;
  //    Serial.print(wave);
     // printBin(wave);

      buttoncount = 0;
      rotaryEncoder.setEncoderValue(myVars.Frequency);   
      changeprecision = 0;
      vTaskDelay(100);
      Serial.println("myVars.sweepdelay");
      Serial.println(myVars.sweepdelay);
       myVars.save();

      return;
    }

    // set min fre based on encoder value
  //    int menuindex = rotaryEncoder.readEncoder() % 4;
   // setmenu(menuindex);

    myVars.sweepdelay = encodervalue;


  }
  
}


void selectSweepSteps(){
Serial.println("selectseepsteps");
      lc.clearMatrix();
rotaryEncoder.setEncoderValue(myVars.sweepstep); 

bool sweepstepselected = false;
  while (!sweepstepselected) {
            vTaskDelay(10);
  floatToDecimalNumber();
         buttonState = digitalRead(Bootpin);
             if (buttonState == LOW) {
    buttonPressTime = millis();
    while (digitalRead(Bootpin) == LOW && millis() - buttonPressTime < HOLD_TIME) {
      buttoncount=1;
      // wait for button release or hold time exceeded
    }}
    
    // display waveform options and wait for button press
  //  encodervalue=rotaryEncoder.readEncoder() % 4;
    //waitButtonPress();
 rotary_loop(); //  rotary_onButtonClick();
 
  //Serial.print("button count ");
//  Serial.println(buttoncount);
    // check button count
    if (buttoncount == 1) {
      // exit waveform selection mode
      sweepstepselected = true;
  //    Serial.print(wave);
     // printBin(wave);

      buttoncount = 0;
      rotaryEncoder.setEncoderValue(myVars.Frequency);   
      changeprecision = 0;
      vTaskDelay(100);
      Serial.println("myVars.sweepstep");
      Serial.println(myVars.sweepstep);
       myVars.save();

      return;
    }

    // set min fre based on encoder value
  //    int menuindex = rotaryEncoder.readEncoder() % 4;
   // setmenu(menuindex);

    myVars.sweepstep = encodervalue;


  }
  
}


void selectmaxfreq(){
Serial.println("selectmaxfreq");
      lc.clearMatrix();
rotaryEncoder.setEncoderValue(myVars.endFreq); 

bool maxfreqselected = false;
  while (!maxfreqselected) {
            vTaskDelay(10);
  floatToDecimalNumber();
         buttonState = digitalRead(Bootpin);
             if (buttonState == LOW) {
    buttonPressTime = millis();
    while (digitalRead(Bootpin) == LOW && millis() - buttonPressTime < HOLD_TIME) {
      buttoncount=1;
      // wait for button release or hold time exceeded
    }}
    
    // display waveform options and wait for button press
  //  encodervalue=rotaryEncoder.readEncoder() % 4;
    //waitButtonPress();
 rotary_loop(); //  rotary_onButtonClick();
 
  //Serial.print("button count ");
//  Serial.println(buttoncount);
    // check button count
    if (buttoncount == 1) {
      // exit waveform selection mode
      maxfreqselected = true;
  //    Serial.print(wave);
     // printBin(wave);

      buttoncount = 0;
      rotaryEncoder.setEncoderValue(myVars.Frequency);   
      changeprecision = 0;
      vTaskDelay(100);
      Serial.println("myVars.endFreq");
      Serial.println(myVars.endFreq);
       myVars.save();

      return;
    }

    // set min fre based on encoder value
  //    int menuindex = rotaryEncoder.readEncoder() % 4;
   // setmenu(menuindex);

    myVars.endFreq = encodervalue;


  }
  
}
void selectminfreq(){
Serial.println("selectminfreq");

      lc.clearMatrix();
rotaryEncoder.setEncoderValue(myVars.startFreq); 

bool minfreqselected = false;
  while (!minfreqselected) {
            vTaskDelay(10);
  floatToDecimalNumber();
         buttonState = digitalRead(Bootpin);
             if (buttonState == LOW) {
    buttonPressTime = millis();
    while (digitalRead(Bootpin) == LOW && millis() - buttonPressTime < HOLD_TIME) {
      buttoncount=1;
      // wait for button release or hold time exceeded
    }}
    
    // display waveform options and wait for button press
  //  encodervalue=rotaryEncoder.readEncoder() % 4;
    //waitButtonPress();
 rotary_loop(); //  rotary_onButtonClick();
 
  //Serial.print("button count ");
//  Serial.println(buttoncount);
    // check button count
    if (buttoncount == 1) {
      // exit waveform selection mode
      minfreqselected = true;
  //    Serial.print(wave);
     // printBin(wave);

      buttoncount = 0;
      rotaryEncoder.setEncoderValue(myVars.Frequency);   
      changeprecision = 0;
      vTaskDelay(100);
      Serial.println("myVars.startFreq");
      Serial.println(myVars.startFreq);
       myVars.save();

      return;
    }

    // set min fre based on encoder value
  //    int menuindex = rotaryEncoder.readEncoder() % 4;
   // setmenu(menuindex);
  
myVars.startFreq = encodervalue;

  }
}

void maxFreq(){
    lc.clearMatrix();

  lc.setChar(0,7,'m',false);
   lc.setChar(0,6,'a',false);
    lc.setChar(0,5,'x',false);
     lc.setChar(0,4,'F',false);
      lc.setChar(0,3,'R',false);
        lc.setChar(0,2,'E',false);
         lc.setChar(0,1,'q',false);
 
}

void Sweep(){
    lc.clearMatrix();

  lc.setChar(0,7,'S',false);
   lc.setChar(0,6,'w',false);
    lc.setChar(0,5,'E',false);
     lc.setChar(0,4,'r',false);
      lc.setChar(0,3,'P',false);
       // lc.setChar(0,2,'E',false);
       //  lc.setChar(0,1,'Q',false);
 
}

void SweepSteps(){
    lc.clearMatrix();

  lc.setChar(0,7,'S',false);
   lc.setChar(0,6,'t',false);
    lc.setChar(0,5,'E',false);
     lc.setChar(0,4,'p',false);
      lc.setChar(0,3,'s',false);
       // lc.setChar(0,2,'E',false);
       //  lc.setChar(0,1,'Q',false);
 
}

void SweepDelay(){
    lc.clearMatrix();

  lc.setChar(0,7,'d',false);
   lc.setChar(0,6,'E',false);
    lc.setChar(0,5,'L',false);
     lc.setChar(0,4,'a',false);
      lc.setChar(0,3,'y',false);
       // lc.setChar(0,2,'E',false);
       //  lc.setChar(0,1,'Q',false);
 
}


void square(){
    lc.clearMatrix();

  lc.setChar(0,7,'S',false);
   lc.setChar(0,6,'q',false);
    lc.setChar(0,5,'U',false);
     lc.setChar(0,4,'A',false);
      lc.setChar(0,3,'R',false);
       lc.setChar(0,2,'E',false);
 
}


void triangle(){
    lc.clearMatrix();

    lc.setChar(0,7,'t',false);
   lc.setChar(0,6,'r',false);
    lc.setChar(0,5,'i',false);
     lc.setChar(0,4,'A',false);
      lc.setChar(0,3,'n',false);
       lc.setChar(0,2,'G',false);
      lc.setChar(0,1,'L',false);
       lc.setChar(0,0,'E',false);
   
}
void sine(){
    lc.clearMatrix();

     lc.setChar(0,7,'S',false);
   lc.setChar(0,6,'I',false);
    lc.setChar(0,5,'N',false);
     lc.setChar(0,4,'E',false);
    
}



void selectWaveform() {
  // enter waveform selection mode
  bool waveformSelected = false;
  while (!waveformSelected) {
            vTaskDelay(10);

    
    // display waveform options and wait for button press
    encodervalue=rotaryEncoder.readEncoder() % 3;
    //waitButtonPress();
 rotary_loop(); //  rotary_onButtonClick();
 
  //Serial.print("button count ");
//  Serial.println(buttoncount);
    // check button count
    if (buttoncount ==1) {
      // exit waveform selection mode
      waveformSelected = true;
  //    Serial.print(wave);
     // printBin(wave);

      buttoncount = 0;
    rotaryEncoder.setEncoderValue(myVars.Frequency);   
      changeprecision = 0;
      delay(100);
             myVars.save();

      return;
    }

    // set waveform based on encoder value
   myVars.waveformindex = rotaryEncoder.readEncoder() % 3;
    setWaveform(myVars.waveformindex);

  }
}

void setWaveform(int waveformIndex) {
    vTaskDelay(10);

  switch (waveformIndex) {
    
    case 0:
    Waveform=SQUARE;
         // Serial.println("SQwave selected");
      square();
      break;
    case 1:
    Waveform=SINE;
           //   Serial.println("SINE wave selected");
        sine();
      break;
    case 2:
    Waveform=TRIANGLE;
           //     Serial.println("TRIANG wave selected");
      triangle();
      break;
    default:
        Waveform=SQUARE;

      // handle invalid waveform index
      // ...
      break;
  }
}


void rotary_onButtonClick(){
  
  //ignore multiple press in that time   milliseconds
if (millis() - lastTimePressed <1000){
buttoncount++;

if ( pressing==false){
  firstpress=millis();
pressing=true;
  
}
  
  
  return;
}
  
  if (millis() - lastTimePressed >1000)  {
    buttoncount=0;


changeprecision++;
if (changeprecision == 0){
  //circleValues = true;
  //rotaryEncoder.setBoundaries(0, 10, circleValues);
  //return;
}

if (changeprecision == 1){
 
  rotaryEncoder.setEncoderValue(encodervalue * 10);
  //circleValues = true;
  //rotaryEncoder.setBoundaries(0, 10, circleValues);
}

if (changeprecision ==2){
  rotaryEncoder.setEncoderValue(encodervalue * 100);
  //circleValues = true;
  //rotaryEncoder.setBoundaries(0, 10, circleValues);
}

if (changeprecision>=3){
  changeprecision=0;
  rotaryEncoder.setEncoderValue(encodervalue);

}
   // return;
  }
  Serial.print("Precision ");
  Serial.println(changeprecision);
  
  lastTimePressed = millis();
  Serial.print("button pressed ");
  Serial.print(millis());
  Serial.println(" milliseconds after restart");
  delayMicroseconds(100000);
}
void rotary_loop(){
  //dont print anything unless value changed
  
  //  if (millis()-lastTimePressed>5000) changeprecision=0;

  if (rotaryEncoder.encoderChanged()) {
          if (changeprecision <=0 ){
           encodervalue = (float)rotaryEncoder.readEncoder(); 
           
          }
        if (changeprecision == 1){
            encodervalue = (float)rotaryEncoder.readEncoder()/10 ;
         }
        if (changeprecision == 2){
           encodervalue = (float)rotaryEncoder.readEncoder()/100;
      }  }
  
  if (rotaryEncoder.isEncoderButtonClicked()){
    rotary_onButtonClick();
  }
}

void IRAM_ATTR readEncoderISR()
{
  rotaryEncoder.readEncoder_ISR();
}


const uint8_t AD_FREQ1 = 15;    ///< Select frequency 1 register
const uint8_t AD_FREQ0 = 14;    ///< Select frequency 0 register
const uint8_t AD_PHASE = 13; 
#define SEL_FREQ0  (1<<AD_FREQ0)
#define SEL_FREQ1  (1<<AD_FREQ1)
#define SEL_PHASE0 (1<<AD_FREQ0 | 1<<AD_FREQ1 | 0<<AD_PHASE)
#define SEL_PHASE1 (1<<AD_FREQ0 | 1<<AD_FREQ1 | 1<<AD_PHASE)


#define PHASE_WRITE_CMD   0xC000    // Setup for Phase write
#define PHASE1_WRITE_REG  0x2000    // Which phase register
#define FREQ0_WRITE_REG   0x4000    // 
#define FREQ1_WRITE_REG   0x8000
#define PHASE1_OUTPUT_REG 0x0400    // Output is based off REG0/REG1
#define FREQ1_OUTPUT_REG  0x0800
void AD9833reset() {
  WriteRegister(0x100);   // Write '1' to AD9833 Control register bit D8.
  //delay(10);
}


void printBin(byte aByte) {
  for (int16_t aBit = 15; aBit >= 0; aBit--)
    Serial.write(bitRead(aByte, aBit) ? '1' : '0');
   Serial.println("");
}
uint32_t freqReg;
void AD9833setFrequency(float Frequency, int Waveform) {

//uint32_t FreqWord = (Frequency * pow(2, 28)) / refFreq;

uint32_t FreqWord = Frequency * twenty_eight_bit / refFreq;


  if (currentFreqReg == 0) {
    freqReg = 1; // use frequency register 1


uint32_t MSB = (uint32_t)((FreqWord & 0xFFFC000) >> 14);    //Only lower 14 bits are used for data
uint32_t LSB = (uint32_t)(FreqWord & 0x3FFF);

  //Set control bits 15 ande 14 to 0 and 1, respectively, for frequency register 0
  LSB |= 0x8000;
  MSB |= 0x8000;

 WriteRegister(SEL_FREQ1); // select the unused frequency register
 //Serial.print("SEL_FREQ1 ");
//printBin(SEL_FREQ1);

 WriteRegister(LSB);                  // Write lower 16 bits to AD9833 registers
 //Serial.print("LSB ");
// printBin(LSB);
 WriteRegister(MSB);                  // Write upper 16 bits to AD9833 registers.
//Serial.print("MSB ");
 //printBin(MSB);
 //WriteRegister(0xC000);               // Phase register
// WriteRegister(Waveform);             // Exit & Reset to SINE, SQUARE or TRIANGLE

 
// Update the control word to switch to the new frequency register
  uint32_t controlWord = 0x2000 | Waveform; 
  controlWord = controlWord | 0x0800; 
//  Serial.print("controlWord ");
// printBin(controlWord);
  // select the new frequency register
  WriteRegister(controlWord);

  // Update the current frequency register being used
  currentFreqReg = freqReg;
 //   Serial.print("currentFreqReg ");
  //  Serial.println(currentFreqReg);
  } else {

      long FreqWord = (Frequency * pow(2, 28)) / refFreq;

    freqReg = 0; // use frequency register 0

  uint32_t MSB = (uint32_t)((FreqWord & 0xFFFC000) >> 14);    //Only lower 14 bits are used for data
  uint32_t LSB = (uint32_t)(FreqWord & 0x3FFF);

  //Set control bits 15 ande 14 to 0 and 1, respectively, for frequency register 0
  LSB |= 0x4000;
  MSB |= 0x4000;

  WriteRegister(SEL_FREQ0); // select the unused frequency register
// Serial.print("SEL_FREQ0 ");
  // printBin(SEL_FREQ0);
  
 WriteRegister(LSB);      
 //Serial.print("LSB ");
 // printBin(LSB);// Write lower 16 bits to AD9833 registers
 WriteRegister(MSB);      
// Serial.print("MSB ");
  //printBin(MSB);// Write upper 16 bits to AD9833 registers.
// WriteRegister(0xC000);               // Phase register
//WriteRegister(Waveform);             // Exit & Reset to SINE, SQUARE or TRIANGLE

 
// Update the control word to switch to the new frequency register
  uint32_t controlWord = 0x2000 | Waveform; // select the new frequency register
 controlWord=Waveform |= 0x0400;
  //Serial.print("controlWord ");
  //printBin(controlWord);
  WriteRegister(controlWord);

  // Update the current frequency register being used
  currentFreqReg = freqReg;
 //Serial.print("currentFreqReg ");

// Serial.println(currentFreqReg);
 }}

void WriteRegister(uint32_t dat) { // used to transmit data to AD9833 DDS

   SPI.setDataMode(SPI_MODE2); // Display and AD9833 use different SPI MODES so it has to be set for the AD9833 here.

  digitalWrite(FSYNC_PIN, LOW);           // Set FSYNC low before writing to AD9833 registers
  delayMicroseconds(100);              // Give AD9833 time to get ready to receive data.

  SPI.transfer(highByte(dat));        // Each AD9833 register is 32 bits wide and each 16
  SPI.transfer(lowByte(dat));         // bits has to be transferred as 2 x 8-bit bytes.

  digitalWrite(FSYNC_PIN, HIGH);          //Write done. Set FSYNC high
}


const int DEBOUNCE_TIME = 50;  // Debounce time in milliseconds
unsigned long lastDebounceTime = 0;  // Last time the encoder was debounced


//float frequency = 0.00;
void floatToDecimalNumber()
{
  
 long integerpart = (long) encodervalue;   //Casting the float into long will cut off the decimal digits, we keep the integers (e.g.: 12587.74 becomes 12587)

//integerpart=(long)"F";
//Serial.print("integerpart part: ");
//Serial.println(integerpart);

  //multiplying the float by 100, casting it to long, then taking the modulo 100 will give us 2 digits (10^2 = 100) after the decimal point
  long decimalpart = ((long)(encodervalue * 100.00) % 100);

//  long decimalpart = (long)(100 * (encodervalue - integerpart)); //e.g.: 100 * (12587.74-12587) = 100 * 0.74 = 74
  //Possible improvement: Sometimes the decimals of the floating point number are slightly different
  //For example, this is the floating point number: 87073.37, but the extracted decimal part is 36.
  //This can happen because I do not round the number. So 87073.37 can be 87073.3650212125... which is 87073.37 when it is rounded
  //But when I do the above operation, the two digits extracted are 3 and 6. For "more accuracy", float must be rounded first.

  //Serial.print("Decimal part: ");
 // Serial.println(decimalpart);

  //At this point we know the following things:
  //1.) we manually set the number of decimal digits to 2. it will be two digits whatever we do.
  //2.) we know the value of the integer part, but we don't (yet) explicitly know the number of digits (number can be 1245.4 but also 1.29...etc.)

  //Let's count the integer part digits
  int integerDigits = 0; //number of digits in the integer part
  long tmp_integerpart = integerpart; //temporary variable, copy of the original integer parts
  long tmp2_integerpart = integerpart; //temporary variable 2, copy of the original integer parts

  while (tmp_integerpart)
  {
    tmp_integerpart = tmp_integerpart / 10;
    integerDigits++;
    //What happens inside:
    //Original float number: 16807.34, integer part: 16807 -(/10)-> 1680 -(/10)-> 168 -(/10)-> 16 -(/10)-> 1
  }

  int digitsPosition = integerDigits + 2; //+2 comes from the 2 decimal digits
 
  //Serial.print("Number of digits: "); //Total number of digits of the processed float
 // Serial.println(digitsPosition);

  //now we know the total number of digits - keep in mind that the max integer digits allowed is 6!
  //The next step is to fill up an array that stores all the digits.
  //The array is filled up in a same was as the display: decimal numbers will be 0th and 1st elements, we do this first
  long digits[digitsPosition]; //array with the needed size
  int decimalDigits = 2; //manually created number of digits
  long tmp_decimalpart = decimalpart; //copy the original decimal part value

  //I do this part "manually"
  digits[integerDigits + 1] = tmp_decimalpart % 10; //2nd decimal digit (e.g.: 634.23 -> 3)
  tmp_decimalpart /= 10;
  digits[integerDigits] = tmp_decimalpart % 10; //1st decimal digit (e.g.: 634.23 -> 2)

  //                                                                [4 3 2 1 0]
  //Example: 634.23 -> Int: 634, Dec: 23. Digits: 5. Int.Digits: 3. [3 2 4 3 6]  <----- reading direction
/*
  Serial.print("Dec 0: ");
  Serial.println(digits[integerDigits + 1]); //2nd decimal digit
  Serial.print("Dec 1: ");
  Serial.println(digits[integerDigits]); //1st decimal digit

  //We got the decimals right, now we move on to the integer digits

  Serial.print("Integer digits: "); //Number of integer digits
  Serial.println(integerDigits);
*/
  while (integerDigits--)
  {
    digits[integerDigits] = tmp2_integerpart % 10;
    tmp2_integerpart /= 10;
    //Same exercise as above with the decimal part, but I put it in a loop because we have more digits
  }

 /* Serial.println("----------------------");
  //This is just to check the contents of the array and if the number is stored properly
  for (int i = 0; i < digitsPosition; i++)
  {
    Serial.print(i);
    Serial.print(": ");
    Serial.println(digits[i]);
  }
*/
  //Finally, we print the digits on the display

  lc.clearMatrix();

  for (int j = 0; j < digitsPosition; j++)
  {
    if (j == 2)
    {
      lc.setDigit(0, j, digits[digitsPosition - j - 1], true); //we know that we have to put a decimal point here (last argument is "true")
      //0: address, j: position, value, decimal point.
      //Additional -1 is because we count the digits as first, second, third...etc, but the array starts from 0.
    }
    else
    {
      lc.setDigit(0, j, digits[digitsPosition - j - 1], false);
      //Example: number is 634.23
      //j = 0: digits[5-0-1] = digits[4] -> 0th element of the display gets the last element of the array which is the second decimal:   3 (hundreths)
      //j = 1: digits[5-1-1] = digits[3] -> 1th element of the display gets the last-1 element of the array which is the first decimal:  2 (tenths)
      //j = 2: digits[5-2-1] = digits[2] -> 2nd element of the display gets the last-2 element of the array which is the first integer:  4 (ones)
      //j = 3: digits[5-3-1] = digits[1] -> 3rd element of the display gets the last-3 element of the array which is the second integer: 3 (tens)
      //j = 4: digits[5-4-1] = digits[0] -> 4th element of the display gets the last-4 element of the array which is the third integer:  6 (hundreds)
    }
  }
}

void displaysweep(float freq){
  
 long integerpart = (long) freq;   //Casting the float into long will cut off the decimal digits, we keep the integers (e.g.: 12587.74 becomes 12587)

  //multiplying the float by 100, casting it to long, then taking the modulo 100 will give us 2 digits (10^2 = 100) after the decimal point
  long decimalpart = ((long)(freq * 100.00) % 100);

  int integerDigits = 0; //number of digits in the integer part
  long tmp_integerpart = integerpart; //temporary variable, copy of the original integer parts
  long tmp2_integerpart = integerpart; //temporary variable 2, copy of the original integer parts

  while (tmp_integerpart)
  {
    tmp_integerpart = tmp_integerpart / 10;
    integerDigits++;
    //What happens inside:
    //Original float number: 16807.34, integer part: 16807 -(/10)-> 1680 -(/10)-> 168 -(/10)-> 16 -(/10)-> 1
  }

  int digitsPosition = integerDigits + 2; //+2 comes from the 2 decimal digits
 

  long digits[digitsPosition]; //array with the needed size
  int decimalDigits = 2; //manually created number of digits
  long tmp_decimalpart = decimalpart; //copy the original decimal part value

  //I do this part "manually"
  digits[integerDigits + 1] = tmp_decimalpart % 10; //2nd decimal digit (e.g.: 634.23 -> 3)
  tmp_decimalpart /= 10;
  digits[integerDigits] = tmp_decimalpart % 10; //1st decimal digit (e.g.: 634.23 -> 2)

  while (integerDigits--)
  {
    digits[integerDigits] = tmp2_integerpart % 10;
    tmp2_integerpart /= 10;
    //Same exercise as above with the decimal part, but I put it in a loop because we have more digits
  }

  //Finally, we print the digits on the display

  lc.clearMatrix();

  for (int j = 0; j < digitsPosition; j++)
  {
    if (j == 2)
    {
      lc.setDigit(0, j, digits[digitsPosition - j - 1], true); //we know that we have to put a decimal point here (last argument is "true")
      //0: address, j: position, value, decimal point.
      //Additional -1 is because we count the digits as first, second, third...etc, but the array starts from 0.
    }
    else
    {
      lc.setDigit(0, j, digits[digitsPosition - j - 1], false);
    
    }
  }
}

void countUp()     //display////////////////////////////////////////
{
  int numberOfDigits = 0;
  unsigned long tmp_countUpNumber = 0; //variable for the counter
  unsigned long tmp2_countUpNumber = 0; //copy of the above

  tmp_countUpNumber = counter * 10; //counter (+10 every ~10 ms)
  tmp2_countUpNumber = tmp_countUpNumber; //copy of the above

  while (tmp_countUpNumber) //counting the digits
  {
    tmp_countUpNumber /= 10;
    numberOfDigits++;
    //same exercise as in the float processing part
  }

  int tmp_numberOfDigits = 0; //copy of the numberOfDigits
  int displayDigits[numberOfDigits]; //array to store the individual digits

  tmp_numberOfDigits = numberOfDigits; //copy the number of digits

  while (numberOfDigits--) //filling up the array
  {
    displayDigits[numberOfDigits] = tmp2_countUpNumber % 10;
    tmp2_countUpNumber /= 10;
    //same exercise as in the float processing part
  }

  lc.clearMatrix();

  for (int i = 0; i < tmp_numberOfDigits; i++)
  {
    lc.setDigit(0, i, displayDigits[tmp_numberOfDigits - i - 1], false);
    //same exercise as in the float processing part
  }
}


void setup(){

pinMode(0, INPUT_PULLUP);  
  
esp_chip_info(&chip_info);
 myVars.get();
  if(myVars.varsGood!=190)
    {
     // Serial<<"Failed to load persistent variables...initializing to defaults.! "<<micros();
      myVars.initialize();
    }

Serial.begin(115200);
//String TCCR5A = "0x0000";
//String TCNT5 = "0x0000";
//int OCR5A = 9; // USED TO SET TOTAL ON TIME PULSE OF OUTPUT 1 (NON INVERTED)  output pin = PB1
//int OCR5B = 9; // USED TO SET TOTAL ON TIME PULSE OF OUTPUT 1 (INVERTED)  output pin = PB1

//int TCCR5A = 178;
//int TCCR5B = 222;// CONTROL REGISTER USED TO SET PRESCALE/INPUT OF CLOCK/ PWM FUNCTIONS / 
// delay(100);
//int  ICR5= 19; // this  register variable sets the total count of pulses per gate cycle/

  
 SPI.begin();

 delay(50);

  AD9833reset();                                   // Reset AD9833 module after power-up.
  delay(50);
  pinMode(FSYNC_PIN, OUTPUT);

 AD9833setFrequency(myVars.Frequency, TRIANGLE);                  // Set the frequency and Sine Wave output

 rotaryEncoder.begin();
 rotaryEncoder.setup(readEncoderISR);
  //set boundaries and if values should cycle or not
  //in this example we will set possible values between 0 and 1000;
  circleValues = false;
  rotaryEncoder.setBoundaries(0, 9999999, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)

  /*Rotary acceleration introduced 25.2.2021.
   * in case range to select is huge, for example - select a value between 0 and 1000 and we want 785
   * without accelerateion you need long time to get to that number
   * Using acceleration, faster you turn, faster will the value raise.
   * For fine tuning slow down.
   */
  //rotaryEncoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
  rotaryEncoder.setAcceleration(encodervalue/10); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
  
  lc=LedController<1,1>(DIN,CLK,CS);

//  lc.shutdown(0, false); //Activate display
  lc.setIntensity( 5); //low brightness
  lc.clearMatrix(); //clear

    rotaryEncoder.setEncoderValue(myVars.Frequency);
setWaveform(myVars.waveformindex);



xTaskCreatePinnedToCore(
      Task1code, /* Function to implement the task */
      "Task1", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &Task1,  /* Task handle. */
      0); /* Core where the task should run */
delay(500); 

xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 


}


void Task1code( void * parameter) {
  for(;;) {
   //in loop call your custom function which will process rotary encoder values
  floatToDecimalNumber();

   // timeout do botão 
 if (pressing == true ){ 
    if ( millis() > (firstpress + 2000)){
      Serial.print(firstpress);
        if (buttoncount == 4) {
         buttoncount=0;
           modeOperation=!modeOperation;
           pressing=false;
           firstpress=0;
         } else if (buttoncount == 2) {
                 buttoncount=0;
                 pressing=false;
                 firstpress=0;
                  rotaryEncoder.setEncoderValue(0);   //zera o encoder 
                selectWaveform();   //select wave form 
        }else{
           buttoncount=0;
           firstpress=0;
           pressing=false;
}}}
  
  rotaryEncoder.setAcceleration(encodervalue/10); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration

   myVars.Frequency = encodervalue;
  
if (modeOperation == 0) {   ///frequency generator 

      if (myVars.Frequency != oldFrequency){
        AD9833setFrequency(myVars.Frequency, Waveform);
      oldFrequency=myVars.Frequency;
      }
} else if (modeOperation ==1 ){  // sweep

  //AD9833reset(); // initialize the AD9833 before the frequency sweep
for (long freq = myVars.startFreq; freq <= myVars.endFreq; freq += myVars.sweepstep) {
  myVars.Frequency=freq;
  AD9833setFrequency(freq, Waveform);
  displaysweep(freq);
  vTaskDelay(myVars.sweepdelay);//delayMicroseconds(sweepspeed); // adjust as needed to control the sweep rate

}}

buttonState = digitalRead(Bootpin);
  if (buttonState == LOW) {
    buttonPressTime = millis();
    while (digitalRead(Bootpin) == LOW && millis() - buttonPressTime < HOLD_TIME) {
   modeOperation = 0;
      // wait for button release or hold time exceeded
    }
       if (millis() - buttonPressTime >= HOLD_TIME) {
      // button held for 1 second, perform action here
      // for example, turn on an LED
      Serial.println("i read");
      menu();
      }
  }   
      vTaskDelay(10);  // Add delay if needed
  }}
//  
 void Task2code( void * parameter) {
  for(;;) {
   //in loop call your custom function which will process rotary encoder values
   rotary_loop();
        vTaskDelay(10);  // Add delay if needed

  }} 

void t1Callback(){
  
}
void t2Callback(){
  
}

void loop(){
 
}
  
