/*
 * prototype sketch for my AC/DC dynamic load project
 * 
 * This is unfinished code
 * This code works with the relais for the AC/DC input selection
 * 
 */
 
#include <digitalWriteFast.h>
#include <Adafruit_SSD1351.h> // 128x128 RGB OLED display
#include <Adafruit_GFX.h> // needed for display
#include <Fonts/FreeSans18pt7b.h> // display fonts used for V and A digits
#include <Fonts/FreeSans9pt7b.h> // display fonts used for the mode line
#include <SPI.h>  // communication method for the display
#include <TrueRMS.h>  // True RMS library: https://github.com/MartinStokroos/TrueRMS

#define PIN_DEBUG 8     // D8 optional: to trace rms interrupt activity on a scope
#define DUT_INPUT 0     // the ADC input for the DUT voltage
#define SHUNT_INPUT 2   // the ADC input for the DUT current

// Rotary Encoder setup
static int enc_A = 2; // D2 No filter caps used!
static int enc_B = 3; // D3 No filter caps used!
static int enc_But = 7; // D7 No filter caps used!
volatile byte aFlag = 0; // expecting a rising edge on pinA encoder has arrived at a detent
volatile byte bFlag = 0; // expecting a rising edge on pinB encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile byte encoderPos = 0; //current value of encoder position (0-255). Change to int or uin16_t for larger values
volatile byte prev_encoderPos = encoderPos; //stores the last encoder position to see if it has changed
volatile byte reading = 0; //store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

// Setup the poor man's DAC by using a PWM based version
int pwmPin = 9;      // D9 PWM output connected to digital pin 9
double const DACcalibAC = 1.0; // adjust the DAC for AC current output levels to match settings
double const DACcalibDC = 1.2195; // adjust the DAC for DC current output levels to match settings

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
int v_line = 25;    // Volt line 30
int a_line = 57;    // Amp line 65
int p_line = 62;    // Power line
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
boolean ac_mode_setup = false; // default for the startup
boolean dc_mode_setup = false;

// setup the True RMS functionality
#define RMS_WINDOW 40   // rms window of samples, 20 = 1 period @50Hz, 40 = 2
//Rms read_dut_Rms; // create an instance of Rms for the voltage
Rms read_shunt_Rms; // and one for the shunt voltage
double dutV = 0.0; // holds the DUT volt value
double prev_dutV = dutV; //keep track of value changes for display
double bridgeCal = 1.25; // bridge loss factor for AC mode
double dutVdivDC = 100.0;  // input voltage divider attenuator factor for DC
double dutVdivAC = 100.0;  // input voltage divider attenuator factor for rms
double shuntV = 0.0; // holds the shunt volt (current) value
double prev_shuntV = 1.0; // keep track of value changes for display
double const shuntVcalibAC = 1.0; // calibration of setting versus actual DUT current
double const shuntVcalibDC = 0.906; // calibration of setting versus actual DUT current
volatile int request_adc = 0;
double const dc_offset_factor = 1.0; // voltage calibration factor for low voltages
double const dc_cal_factor = 1.0376; // voltage calibration factor
double const ac_cal_factor = 1.0; // calibration factor
double const vRef = 4.99155; // the actually measured VREF voltage on the Nano board.
double const adc_conversion_factor = vRef / 1023.0; // for ADC measurements, actual VREF voltage as measured
double dutPower = 0.0; // holds the power in Watt calculation
double prev_dutPower = 1.0; //keep track of value changes for display

volatile int period_counter = 1; // keep track of rms calculation sampling
volatile int Measurement_completed = 0; // signal that aquisition is complete
const float VoltRange = vRef; // Needed for the RMS calculation. Max ADC input voltage.


/*
 * Some Forward Function Declarations
 */
void setup_oled_ac();
void setup_oled_dc();
void send_volt();
void send_current();
void send_power();
void send_encoder();
void send_ac_to_monitor(); // for debugging
void send_dc_to_monitor(); // for debugging
void oledSetup();
void enc_a_isr();
void enc_b_isr();


void setup() 
{
  Serial.begin(9600); // for debugging
  
  pinMode(PIN_DEBUG, OUTPUT); // optional: so we can show interrupt activity on a scope
  pinMode(DUT_Mode, INPUT_PULLUP); // Mode switch, default is AC
  
  analogReference(EXTERNAL); // Nano to use an external 5V reference
  analogRead(A0);  // force the voltage reference setting to be used and avoid an internal short
    
  Serial.println("start tft");
  tft.begin();
  Serial.println("start OLED setup");
  oledSetup();

  // configure RMS conversion for automatic base-line restoration and continuous scan mode:
  // We now use a hardware DUT voltage rms conversion by an LTC1966
  //read_dut_Rms.begin(VoltRange, RMS_WINDOW, ADC_10BIT, BLR_ON, CNT_SCAN);
  //read_dut_Rms.start(); //start measuring  
  // We still use the software rms for the shunt voltage (DUT current)
  read_shunt_Rms.begin(VoltRange, RMS_WINDOW, ADC_10BIT, BLR_ON, CNT_SCAN);
  read_shunt_Rms.start(); //start measuring 
  
  // setup the Nano registers for a faster PWM
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
 * Interrupt service routine to calculate the RMS value
 * Timer0 interrupt routine will be called every ms
 * 50Hz has 20mS, so we can sample the voltage 20 times in a period to get a good coverage.
 * The actual number of samples is set by RMS_WINDOW
 * In this ISR we're reading the ADC value at every interrupt and passing it on 
 * to the rms library.
 * 
 * At this moment, the global measurement_cycle is not reset in the DC mode,
 * so the RMS measurements are only running in the AC mode.
 * In order to follow what is going on, we also toggle a port so a DSO can monitor
 * The DSO will only show the sampling while we're within the window so we can
 * see exactly when we sample the waveform and how many times.
 */
ISR(TIMER0_COMPA_vect){//timer0 interrupt
  cli();//stop interrupts

  if (Measurement_completed == 0){
    digitalWriteFast(PIN_DEBUG, HIGH); // optional: show start of cycle on a scope
    /* for software RMS sampling of the DUT voltage : not used
    read_dut_Rms.update(analogRead(DUT_INPUT)); // for BLR_ON or for DC(+AC) signals with BLR_OFF
    */
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
  if(prev_encoderPos != encoderPos) {
    prev_encoderPos = encoderPos;
    // send the encoder Amp setting by the rotary encoder to the OLED display
    send_encoder();
  } 

  if (measure_mode == HIGH){ // we're measuring AC
    if(ac_mode_setup == false){
      setup_oled_ac();  // prepare the OLED display
      // stay in this mode but get ready for a switch to the AC mode selection
      ac_mode_setup = true;
      dc_mode_setup = false;
    }

    /////// =====  AC Measurements

    // output the DAC value
    analogWrite(pwmPin, encoderPos * DACcalibAC); // AC mode 0..255

    if (Measurement_completed == 1){
      // get the DUT voltage
      // The True RMS calculation for the DUT voltage is done by the LTC1966
      // The output from the LTC1966 is 0..1V DC
      double dutVraw = analogRead(DUT_INPUT); // read the LTC output
      dutVraw = dutVraw * adc_conversion_factor;  // adc bits to Volt conversion
      dutVraw = (dutVraw * dutVdivAC) + bridgeCal;  // compensate for the /100 input divider add the bridge loss factor
      dutV = dutVraw * ac_cal_factor; // add a calibration factor       

      // get the collected shunt rms voltage
      // convert the rms voltage to the DUT current
      read_shunt_Rms.publish();
      float shuntVraw = read_shunt_Rms.rmsVal;
      shuntV = shuntVraw * shuntVcalibAC * 2; // use the 10x input attenuator 

      Measurement_completed = 0; // restart the rms measurement cycle

    } // end of AC calculations
  } // end-of AC measurement mode
 
  /////// ===== DC Measurements
  if (measure_mode == LOW){  // We're measuring DC
    // note that in the DC mode, we do not set the global measurement_completed var so the Timer is not triggering the sampling 
    // we pick-up the ADC readings here during the normal loop execution, which is OK for DC signals.
    
    if(dc_mode_setup == false){
      setup_oled_dc();  // prepare the OLED display for DC settings
      // stay in this mode but get ready for the switch to the AC mode selection  
      dc_mode_setup = true;
      ac_mode_setup = false;
    }
    // output the DAC value to set the DUT current
    analogWrite(pwmPin, encoderPos * DACcalibDC); // DC mode 0..255

    // The calculation for the DUT voltage is done by the LTC1966
    // obtain the output and calculate the DC DUT voltage
    double dutVraw = analogRead(DUT_INPUT) * adc_conversion_factor;  // adc bits to Volt conversion
    dutVraw = (dutVraw * dutVdivDC) ; // compensate for the /100 input divider add the offset
    dutV = (dutVraw * dc_cal_factor)+ dc_offset_factor ; // add a calibration factor
    temp = (int (dutV * 100));   // use a trick forcing only two decimals for display
    dutV = double (temp) / 100;   
    
    // Calculate DC Shunt voltage = current
    // We read the voltage across the sense resistor
    double shuntVraw = analogRead(SHUNT_INPUT) * adc_conversion_factor; // adc to Volt conversion
    shuntV = shuntVraw * shuntVcalibDC; // add the calibration factor
    temp = (int (shuntV * 100)); // use a trick forcing only two decimals for display & update compare
    shuntV = double (temp) / 100; 

  } // end of DC mode calculations   

  // display the DUT voltage on the OLED display         
  if (prev_dutV != dutV){ // only update changes to reduce flashing of the OLED
    prev_dutV = dutV; // reset compare value
    send_volt();
  }

  // display the DUT current on the OLED display  
  if (prev_shuntV != shuntV){ // new value compare; only update changes to reduce flashing of the OLED
    prev_shuntV = shuntV;  // reset compare value
    send_current();
  }

  // calculate and display the DUT Power
  dutPower = dutV * shuntV;
  temp = (int (dutPower * 100)); // trick to limit to 2 decimals
  dutPower = double (temp) / 100;
  // display the DUT power in Watt
  if (prev_dutPower != dutPower){
    prev_dutPower = dutPower;
    send_power();
  }

  send_to_monitor(); // for debugging 

  delay(500); // only for testing, remove for final version
} // end-of loop()

void send_volt(){
  tft.setTextColor(BLUE);
  tft.setFont(&FreeSans18pt7b);
  tft.setTextSize(1);
  tft.fillRect(digit_1, 0, digit_4+22, 27, BLACK); // line = 0; clear digits with a black rectangular
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

void send_current(){
  tft.setTextColor(GREEN);
  tft.setFont(&FreeSans18pt7b);
  tft.setTextSize(1);
  tft.fillRect(digit_1, a_line-25, digit_4+22, 27, BLACK); // clear digits with a black rectangular
  if (shuntV > 9.99){ // shift the display one digit to the left
    tft.setCursor(digit_1, a_line);     
  }else{
    tft.setCursor(digit_2, a_line);  
  }
  tft.print(String(shuntV));  // send to OLED display
}

void send_power(){
  tft.fillRect(digit_1, p_line, digit_4+22, 20, BLACK);  // p-line block clear with a black rectangular
  tft.setTextColor(WHITE);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);
    if (dutPower > 9.99){ // shift the display one digit to the left
    tft.setCursor(digit_3-4, p_line + 17);     
  }else{
    tft.setCursor(digit_3+5, p_line + 17);
  }
  tft.print(String(dutPower));  // send to OLED display
}

void send_encoder(){
  tft.fillRect(digit_3+8, stat_line-12, 128, 20, BLACK); // Status line block clear with a black rectangular
  tft.setTextColor(WHITE);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setCursor(digit_3+8, stat_line); // from left side, down
  // Here we add in the DUT voltage effect to the DUT current setting
  // Multiplying and rounding the DUT voltage has the side effect that the step resolution changes with the voltage
  tft.print(encoderPos * 10 ); // nominal 10mA per click
  tft.setCursor(digit_5+10, stat_line); // from left side, down
  tft.print("mA"); 
}

void send_to_monitor(){
  // print the values to the IDE monitor
  Serial.print(dutV,4);
  Serial.print(" V_rms\t");
//    Serial.print(", ");
//    Serial.println(read_dut_Rms.dcBias); // show the ADC bits

  Serial.print(shuntV,4);
  Serial.print(" A_rms\t"); 
//    Serial.print(", ");
//    Serial.println(read_shunt_Rms.dcBias); // show the ADC bits

  Serial.print(dutPower,2);
  Serial.println(" Watt");  
}


void setup_oled_ac(){
  // clear the display fields and show the suffixes
  tft.fillRect(digit_1, v_line-24, 128, 27, BLACK); // V line clear with a black rectangular
  tft.fillRect(digit_1, a_line-24, 128, 27, BLACK); // A line
  tft.fillRect(digit_1, p_line, 128, 20, BLACK);  // P line

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

  // print the power suffix
  tft.setTextColor(WHITE);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setCursor(digit_5+10, p_line + 17); // from left side, down
  tft.print("W");  

  // print the mode
  tft.fillRect(digit_1, stat_line-12, 128, 20, BLACK); // V line clear with a black rectangular
  tft.setTextColor(WHITE);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setCursor(0, stat_line); // from left side, down
  tft.print("AC");  
  
}


void setup_oled_dc(){
  // clear the display fields and show the suffixes
  tft.fillRect(digit_1, v_line-24, 128, 27, BLACK); // V line clear with a black rectangular
  tft.fillRect(digit_1, a_line-24, 128, 27, BLACK); // A line
  tft.fillRect(digit_1, p_line, 128, 20, BLACK);  // P line

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

  // print the power suffix
  tft.setTextColor(WHITE);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setCursor(digit_5+10, p_line + 17); // from left side, down
  tft.print("W");  

  // print the mode
  tft.fillRect(digit_1, stat_line-12, 128, 20, BLACK); // stat line clear with a black rectangular
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

  //------------------- configuration help; delete when done
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
