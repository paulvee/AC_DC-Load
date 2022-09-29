/*
 * prototype sketch for my AC/DC dynamic load project
 * 
 * This is unfinished code
 * 
 */
 
#include <digitalWriteFast.h>
#include <Adafruit_SSD1351.h> // 128x128 RGB OLED display
#include <Adafruit_GFX.h> // needed for display
#include <Fonts/FreeSans18pt7b.h> // display fonts used for V and A digits
#include <Fonts/FreeSans9pt7b.h> // display fonts used for the mode line
#include <SPI.h>  // communication method for the display
#include <TrueRMS.h>  // True RMS library: https://github.com/MartinStokroos/TrueRMS
#include <Ewma.h> // filter library https://github.com/jonnieZG/EWMA

#define PIN_DEBUG 8 // D8 optional: to trace activity on a scope
#define DUT_INPUT 0     // define the used ADC input channels
#define SHUNT_INPUT 2

// Rotary Encoder setup
static int enc_A = 2; // D2 No filter caps used!
static int enc_B = 3; // D3 No filter caps used!
static int enc_But = 7; // D7 No filter caps used!
volatile byte aFlag = 0; // expecting a rising edge on pinA encoder has arrived at a detent
volatile byte bFlag = 0; // expecting a rising edge on pinB encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile byte encoderPos = 0; //current value of encoder position (0-255). Change to int or uin16_t for larger values
volatile byte oldEncPos = 0; //stores the last encoder position to see if it has changed
volatile byte reading = 0; //store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

// Setup the poor man's DAC by using a PWM based version
int pwmPin = 9;      // D9 PWM connected to digital pin 9
double const DACcalibAC = 1.1; // adjust the DAC for AC current output levels
double const DACcalibDC = 0.7; // adjust the DAC for DC current output levels

// ADC read filter setup
Ewma adcFilterV(0.1); // Analog DUT voltage filter
Ewma adcFilterA(0.1); // Analog shunt voltage filter
Ewma dcFilterV(0.1);  // DC DUT voltage filter
Ewma dcFilterA(0.1);  // DC shunt voltage filter

// SSD1531 SPI declarations
// Screen dimensions
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 128 // Change this to 96 for 1.27" OLED.

// You can use any (4 or) 5 pins to drive the display
#define sclk 12 // already default, not needed - do not use pin
#define mosi 11
#define dc   4
#define cs   5
#define rst  6

// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF

// Use the hardware SPI pins to communicate with the OLED display
// (for an UNO that is sclk = 13 and sid = 11) and pin 10 must be 
// an output.
//Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, CS_PIN, DC_PIN, RST_PIN); //hardware spi
Adafruit_SSD1351 tft = Adafruit_SSD1351(cs, dc, rst); // others are defaults

// OLED display vertical line positions
int v_line = 30;    // Volt/Watt/Ohm line
int a_line = 65;    // Amp line
int stat_line = 100; // Status line
int menu_line = 118; // Menu line

// Horizontal starting positions for the 5 digits
int digit_1 = 0;
int digit_2 = 20;
int digit_3 = 45;
int digit_4 = 66;
int digit_5 = 86;

// setup the AD-DC mode switching
#define DUT_Mode 10 // use pin D10 for the toggle switch
boolean ac_mode_setup = false;
boolean dc_mode_setup = false;

// setup the True RMS functionality
#define RMS_WINDOW 40   // rms window of samples, 20 = 1 period @50Hz, 40 = 2
Rms read_dut_Rms; // create an instance of Rms for the voltage
Rms read_shunt_Rms; // and one for the shunt voltage
double dutV; // holds the DUT volt value
double dutVcalib = 10.0; // calibration for the /10 divider
double prev_dutV = 0.0; //keep track of value changes for display
double shuntV; // holds the shunt volt value
double const shuntVcalib = 0.9; // calibration for the 10x gain, match the DUT current display value
double prev_shuntV = 0.0; // keep track of value changes for display
volatile int request_adc = 0;
double const ac_bridgeLoss = 0.1; // Fudge factor compensation
double const dc_bridgeLoss = 1.3; // avg bridge diode junction drop in volt goes up to 1.48V @500mA
double const ac_halfwavefactor = 1.5; // The rms ac voltage after the bridge needs to be compensated
double const ac_dutV_factor = 13; // multiplier to create a dynamic compensation factor
double const vRef = 4.987; // the actually measured VREF voltage on the Nano board.
double const adc_conversion_factor = vRef / 1023; // for DC measurements, VREF voltage as measured

volatile int period_counter = 1; // keep track of sampling
volatile int Measurement_completed = 0; // signal that aquisition is complete
const float VoltRange = vRef; // Needed for the RMS calculation. Max ADC input voltage.

/*
 * Some Forward Function Declarations
 */
void setup_oled_ac();
void setup_oled_dc();
void send_ac_to_monitor(); // for debugging
void send_dc_to_monitor(); // for debugging
void oledSetup();
void enc_a_isr();
void enc_b_isr();


void setup() 
{
  Serial.begin(115200); // for debugging
  
  pinMode(PIN_DEBUG, OUTPUT); // optional: so we can show activity on a scope
  pinMode(DUT_Mode, INPUT_PULLUP); // Mode switch, default is AC
  
  analogReference(EXTERNAL); // use an external 5V reference
  analogRead(A0);  // force the voltage reference setting to be uswed
    
  Serial.println("start tft");
  tft.begin();
  Serial.println("start OLED setup");
  oledSetup();

  // configure RMS conversion for automatic base-line restoration and continuous scan mode:
  read_dut_Rms.begin(VoltRange, RMS_WINDOW, ADC_10BIT, BLR_ON, CNT_SCAN);
  read_dut_Rms.start(); //start measuring  

  read_shunt_Rms.begin(VoltRange, RMS_WINDOW, ADC_10BIT, BLR_ON, CNT_SCAN);
  read_shunt_Rms.start(); //start measuring 
  
  // setup the Nano registers
  cli();  //stop interrupts while we run the setup, just in case...  
  // Change the PWM base frequency from the standard 488Hz to 3.906 Hz, this helps filtering to DC.
  TCCR1B = TCCR1B & 0b11111000 | 0x02;
  pinMode(pwmPin, OUTPUT);  // sets the PWM pin as output, to act like a DAC

  // set timer0 to 1ms interrupts
  TCCR0A = 0; // reset the timer from previous settings 
  TCCR0B |= (1 << WGM02)|(1 << CS01)|(1 << CS00);  
  OCR0A = 0xF9;  
  TCNT0 = 0;  
  TIMSK0 |= (1 << OCIE0A);
  sei();  //allow interrupts again
  
  // setup the rotary encoder switches and ISR's
  pinMode(enc_A, INPUT_PULLUP); // set pinA as an input, pulled HIGH
  pinMode(enc_B, INPUT_PULLUP); // set pinB as an input, pulled HIGH
  attachInterrupt(0, enc_a_isr,RISING);
  attachInterrupt(1, enc_b_isr,RISING);
  
}


/*
 * Timer0 interrupt routine will be called every ms
 * 50Hz has 20mS, so can sample the voltage 20 times in a period to get a good coverage.
 * The actual number of samples is set by RMS_WINDOW
 * In this ISR we're reading the ADC value at every interrupt and passing it on 
 * to the rms library.
 * 
 * At this moment, the global measurement_cycle is not reset in the DC mode,
 * so the RMS measurements are only running in the AC mode.
 * The DSO will only show the sampling while we're within the window so we can
 * see exactly when we sample the waveform and how many times.
 */
ISR(TIMER0_COMPA_vect){//timer0 interrupt
  cli();//stop interrupts

  if (Measurement_completed == 0){
    digitalWriteFast(PIN_DEBUG, HIGH); // optional: show start of cycle on a scope
    read_dut_Rms.update(analogRead(DUT_INPUT)); // for BLR_ON or for DC(+AC) signals with BLR_OFF
    read_shunt_Rms.update(analogRead(SHUNT_INPUT)); // for BLR_ON or for DC(+AC) signals with BLR_OFF
    period_counter++;
  }
  if (period_counter > RMS_WINDOW){ // when we have all the cycles, reset and tell the main program
    period_counter = 1;
    Measurement_completed = 1;
  }
  digitalWriteFast(PIN_DEBUG, LOW); // optional: show ending on scope
  sei();//allow interrupts
}


void loop() {
  int temp;
  // check the mode we're in, read the switch
  int measure_mode = digitalRead(DUT_Mode); // High is AC, low is DC

  // read the rotary encoder switch value and set the PWM/DAC output accordingly
  // 0..255 is 0..5V in 20mV steps or 20mA current steps
  if(oldEncPos != encoderPos) {
    Serial.print("DAC = ");
    Serial.println(encoderPos);
    // setup the PWM with the current value
    // unfortunately, the required settings in the AC mode is slightly different from the DC mode
    if (measure_mode == HIGH){
      analogWrite(pwmPin, encoderPos * DACcalibAC); // 0..255
    }else{
      analogWrite(pwmPin, encoderPos * DACcalibDC); // 0..255
    }
    oldEncPos = encoderPos;
  } 
  
  if (measure_mode == HIGH){ // we're measuring AC
    if(ac_mode_setup == false){
      setup_oled_ac();  // prepare the OLED display
      // stay in this mode but get ready for the switch to the AC mode selection
      ac_mode_setup = true;
      dc_mode_setup = false;
    }
    if (Measurement_completed == 1){
      // calculate the rms values
      // DUT voltage
      read_dut_Rms.publish();
      float dutVraw = read_dut_Rms.rmsVal * dutVcalib; // /10 input divider factor
      Serial.println(1+dutVraw/10);
      // dutV = adcFilterV.filter(dutVraw); // low-pass filter no longer used
      dutV = (dutVraw + ac_bridgeLoss) * (ac_halfwavefactor * (1+ dutVraw/ac_dutV_factor)); //use the bridge and conversion factors to get to the real DUT voltage
      
      // current shunt
      read_shunt_Rms.publish();
      float shuntVraw = read_shunt_Rms.rmsVal;
      //shuntV = adcFilterA.filter(shuntVraw); // low-pass filter no longer used
      shuntV = shuntVraw * shuntVcalib; // use the 10x Opamp gain correction value
      
      send_ac_to_monitor(); // for debugging 
       
      // print the Amp setting
      tft.fillRect(digit_3+8, stat_line-12, 128, 20, BLACK); // Status line clear with a black rectangular
      tft.setTextColor(WHITE);
      tft.setFont(&FreeSans9pt7b);
      tft.setTextSize(1);
      tft.setCursor(digit_3+8, stat_line); // from left side, down
      tft.print(encoderPos * 20); // Using 20mA per step or click
      tft.setCursor(digit_5+10, stat_line); // from left side, down
      tft.print("mA"); 
     
      Measurement_completed = 0; // restart the measurement cycle
      }
  }else{ // We're measuring DC
    // note that in the DC mode, we do not set the global measurement_completed var so the Timer is not triggering the sampling 
    // we pick-up the ADC readings here during the normal loop execution, which is OK for DC signals.
    
    if(dc_mode_setup == false){
      setup_oled_dc();  // prepare the OLED display 
      // stay in this mode but get ready for the switch to the AC mode selection  
      dc_mode_setup = true;
      ac_mode_setup = false;
    }
    // DUT voltage
    double dutVraw = analogRead(DUT_INPUT) * adc_conversion_factor;  // adc bits to Volt conversion
    // dutV = dcFilterV.filter(dutVraw); low-pass filter, no longer used
    dutV = (dutVraw * dutVcalib) + dc_bridgeLoss; // use the /10 input divider calibration and add the bridge diode junction losses
    
    // Shunt voltage = current
    double shuntVraw = analogRead(SHUNT_INPUT) * adc_conversion_factor; // adc to Volt conversion
    // shuntV = dcFilterA.filter(shuntVraw); // low-pass filter, no longer used
    shuntV = shuntVraw * shuntVcalib; // use the 10x Opamp gain correction value

    // print the Amp setting by the rotary encoder
    tft.fillRect(digit_3+8, stat_line-12, 128, 20, BLACK); // Status line clear with a black rectangular
    tft.setTextColor(WHITE);
    tft.setFont(&FreeSans9pt7b);
    tft.setTextSize(1);
    tft.setCursor(digit_3+8, stat_line); // from left side, down
    tft.print(encoderPos * 20); // 20mA per step or click
    tft.setCursor(digit_5+10, stat_line); // from left side, down
    tft.print("mA"); 
      
    send_dc_to_monitor(); // for debugging    
  }    

  // display the voltage on the OLED display  
  // use a trick forcing only two decimals for display & also the update compare
  temp = (int (dutV * 100));
  dutV = double (temp) / 100;
           
  if (prev_dutV != dutV){ // only update changes to reduce flashing of the OLED
    prev_dutV = dutV; // reset compare value
    tft.setTextColor(BLUE);
    tft.setFont(&FreeSans18pt7b);
    tft.setTextSize(1);
    tft.fillRect(digit_1, 5, digit_4+22, 27, BLACK); // line = 5; clear digits with a black rectangular
    if (dutV > 99.99){ // prevent overflow which messes the display
      dutV = 99.99;
    }  
    if (dutV > 9.99){ // shift the display one digit to keep it clean
      tft.setCursor(digit_1, v_line);     
    }else{
      tft.setCursor(digit_2, v_line);
    }
    tft.print(String(dutV)); // send to OLED display
  }

  
  // display the current on the OLED display
  // use a trick forcing only two decimals for display & update compare
  temp = (int (shuntV * 100));
  shuntV = double (temp) / 100; 

  if (prev_shuntV != shuntV){ // new value compare; only update changes to reduce flashing of the OLED
    prev_shuntV = shuntV;  // reset compare value
    tft.setTextColor(GREEN);
    tft.setFont(&FreeSans18pt7b);
    tft.setTextSize(1);
    tft.fillRect(digit_1, 40, digit_4+22, 27, BLACK); // line = 15; clear digits with a black rectangular
    if (shuntV > 9.99){ // prevent overflow which messes the display
      shuntV = 9.99;
    }
    if (shuntV > 9.99){ // shift the display one digit to keep it clean
      tft.setCursor(digit_1, a_line);     
    }else{
      tft.setCursor(digit_2, a_line);  
    }
    tft.print(String(shuntV));  // send to OLED display
  }
  
  delay(500); // only for testing, remove for final version
}


void send_ac_to_monitor(){
  // print the ac values to the monitor
  Serial.print(dutV,4);
  Serial.print(" V_rms\t");
//    Serial.print(", ");
//    Serial.println(read_dut_Rms.dcBias); // show the ADC bits

  Serial.print(shuntV,4);
  Serial.println(" A_rms"); 
//    Serial.print(", ");
//    Serial.println(read_shunt_Rms.dcBias); // show the ADC bits
  
}

void send_dc_to_monitor(){
  // print the dc values to the monitor
  Serial.print("Vdc = ");
  Serial.print(dutV, 4);
  
  Serial.print("  Adc = ");
  Serial.println(shuntV, 4);  
}


void setup_oled_ac(){
  // clear the display fields 
  tft.fillRect(digit_1, 5, 128, 27, BLACK); // V line clear with a black rectangular
  tft.fillRect(digit_1, 40, 128, 27, BLACK); // A line

  // print the Volt suffix
  tft.setTextColor(BLUE);
  tft.setFont();
  tft.setTextSize(2);
  tft.setCursor(digit_5+10, v_line - 13);
  tft.println("V");
  tft.setTextSize(1);
  tft.setCursor(digit_5+24, v_line - 15); // this will cause a subscript level
  tft.println("rms");

  // print the Amp suffix
  tft.setTextColor(GREEN);
  tft.setFont();
  tft.setTextSize(2);
  tft.setCursor(digit_5+10, a_line - 13); 
  tft.println("A");
  tft.setTextSize(1);
  tft.setCursor(digit_5+24, a_line - 15); // this will cause a subscript level
  tft.println("rms");

  // print the mode
  tft.fillRect(digit_1, stat_line-12, 128, 20, BLACK); // V line clear with a black rectangular
  tft.setTextColor(WHITE);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setCursor(0, stat_line); // from left side, down
  tft.print("AC");  
  
}


void setup_oled_dc(){
  // clear the display fields 
  tft.fillRect(digit_1, 5, 128, 27, BLACK); // V line clear with a black rectangular
  tft.fillRect(digit_1, 40, 128, 27, BLACK); // A line

  // print the Volt suffix
  tft.setFont();
  tft.setTextSize(2);
  tft.setCursor(digit_5+10, v_line - 13);
  tft.println("V");

  // print the Amp suffix
  tft.setFont();
  tft.setTextSize(2);
  tft.setCursor(digit_5+10, a_line - 13); 
  tft.println("A");

  // print the mode
  tft.fillRect(digit_1, stat_line-12, 128, 20, BLACK); // V line clear with a black rectangular
  tft.setTextColor(WHITE);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setCursor(0, stat_line); // from left side, down
  tft.print("DC");  
   
}


/*
 * OLED display initial setup code
 */

void oledSetup() {
  Serial.println("OLED setup");
  // clear the screen
  tft.fillScreen(BLACK);

  //-------------------------------------------
  // set the outside corner indicators by using one pixel  
  tft.drawPixel(0, 0, WHITE);
  tft.drawPixel(0, 127, WHITE);
  tft.drawPixel(127, 0, WHITE);
  tft.drawPixel(127, 127, WHITE);
  Serial.println("done...");
}


/*
 * Rotary decoder ISR's for the A and B switch activities.
 */
void enc_a_isr(){
  cli(); //stop interrupts
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if(reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    if (encoderPos <= 0){
      encoderPos = 0;
    }else{
      encoderPos --;
    }
    bFlag = 0; //reset flags
    aFlag = 0; //reset flags
  }
  else if (reading == B00000100) bFlag = 1; //we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void enc_b_isr(){
  cli(); //stop interrupts
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    if (encoderPos >= 255){
      encoderPos = 255;
    }else{
      encoderPos ++;
    }
    bFlag = 0; //reset flags
    aFlag = 0; //reset flags
  }
  else if (reading == B00001000) aFlag = 1; //we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}


// -- END OF FILE --
