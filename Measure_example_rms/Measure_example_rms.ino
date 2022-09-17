/*
 *
 * File: Measure_example_rms.ino
 * Purpose: TrueRMS library example project
 * Version: 1.0.3
 * Date: 23-05-2019
 * last update: 30-09-2020
 * Modified by paulv to be used with timer interrupts
 * 
 * URL: https://github.com/MartinStokroos/TrueRMS
 * License: MIT License
 *
 *
 * This example illustrates the measurement of the rms value of a signal at the ADC input. To test, apply a sine- or a 
 * square wave of 50Hz/60Hz with 1V amplitude, biased on 2.5Vdc to input ADC0. The dcBias reading should be stable around 
 * the value 512 decimal (the middle of the ADC range) and the rms value of the sine wave should read about 0.71V and for a 
 * square wave about 1.00V.
 * 
 * The number of samples used to capture the input signal, must be a whole number. The sample window, expressed in 
 * number of samples, must have a length equal to at least one cycle of the input signal. If this is not the case, 
 * slow fluctuations in the rms and power readings will occure.
 * 
*/

#include <TrueRMS.h>
#include <digitalWriteFast.h> // optional: It uses digitalWriteFast only for the purpose of debugging!
                              // https://code.google.com/archive/p/digitalwritefast/downloads

#define ADC_INPUT 0     // define the used ADC input channel
#define RMS_WINDOW 40   // rms window of 40 samples, means 2 periods @50Hz
//#define RMS_WINDOW 50   // rms window of 50 samples, means 3 periods @60Hz

#define PIN_DEBUG 8 // optional: to trace activity on a scope

int samples = 40; // number of samples per measurement cycle
volatile int period_counter = 1; // keep track of sampling
volatile int Measurement_completed = 0; // signal that aquisition is complete
int adcVal; // holds the value from the ADC
float VoltRange = 4.735;  // The full scale value is set to actual V-ref but can be changed 
                          // when using an input divider

Rms readRms; // create an instance of Rms.


void setup() {
  // run once:
  Serial.begin(115200);
  pinMode(PIN_DEBUG, OUTPUT); // optional: so we can show activity on a scope
  
  // configure the RMS library for automatic base-line restoration and continuous scan mode:
  readRms.begin(VoltRange, RMS_WINDOW, ADC_10BIT, BLR_ON, CNT_SCAN);
  
  readRms.start(); //start measuring  

  // set timer0 to 1ms interrupts
  cli();  //stop interrupts while we run the register setup, just in case...
  
  TCCR0A = 0; // reset the timer from previous settings 
  TCCR0B |= (1 << WGM02)|(1 << CS01)|(1 << CS00);  
  OCR0A = 0xF9;  
  TCNT0 = 0;  
  TIMSK0 |= (1 << OCIE0A);
  sei();  //allow interrupts again
}

/* 
 * Timer interrupt routine will be called every ms
 * 50Hz has 20mS, so we'll sample the voltage samples times
 * Reading the ADC value at every interrupt and passing it on 
 * to the rms library
  */

ISR(TIMER0_COMPA_vect){//timer0 interrupt
  cli();//stop interrupts

  if (Measurement_completed == 0){
    digitalWriteFast(PIN_DEBUG, HIGH); // optional: show start on scope
    adcVal = analogRead(ADC_INPUT); // read the ADC.
    readRms.update(adcVal); // for BLR_ON or for DC(+AC) signals with BLR_OFF
    period_counter++;
    digitalWriteFast(PIN_DEBUG, LOW); // optional: show ending on scope
  }
  if (period_counter > samples){ // when we have all the cycles, reset and tell the main program
    period_counter = 1;
    Measurement_completed = 1;
  }
  sei();//allow interrupts
}


void loop() {
                                
  if (Measurement_completed == 1){
    // calculate the rms value
    readRms.publish();
    Serial.print(readRms.rmsVal,3);
    Serial.println(" Vrms");
//    Serial.print(", ");
//    Serial.println(readRms.dcBias); // show the ADC bits
    Measurement_completed = 0; // restart the measurement cycle
  }
  delay(1000); // update every second
}
