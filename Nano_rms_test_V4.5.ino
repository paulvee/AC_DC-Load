/*
 * prototype sketch for my AC/DC dynamic load project
 * 
 * This is unfinished code
 * This code works only with the version 4 prototype PCB with the V4a modifications
 *
 * This sketch will implement the more traditional Dynamic Load implementation for DC, by eliminating the 
 * DUT voltage level effect on the current setting, something the AD633 does for the AC mode.
 *
 * By getting the multiplication of the AD633 out of the circuit in the DC input mode, the DAC is set 
 * by the encoder for a desired current setting, and stays fixed during the measurement while the hardware 
 * controlling loop takes care of changes in the DUT voltage.
 *
 * For the AC mode, the AD633 needs to be brought back into the circuit and we may need to activate the software controlling loop.
 * 
 */
 
#include <digitalWriteFast.h>
#include <Adafruit_SSD1351.h> // 128x128 RGB OLED display
#include <Adafruit_GFX.h> // needed for display
#include <Fonts/FreeSans18pt7b.h> // display fonts used for V and A digits
#include <Fonts/FreeSans9pt7b.h> // display fonts used for the mode line
#include <SPI.h>  // communication method for the display
#include <TrueRMS.h>  // True RMS library: https://github.com/MartinStokroos/TrueRMS

#define PIN_DEBUG 12    // D12 optional: to trace rms interrupt activity on a scope
#define DUT_INPUT A0    // the ADC input for the DUT voltage
#define SHUNT_INPUT A2  // the ADC input for the DUT current


// Rotary Encoder setup
static int enc_A = 2; // D2 No filter caps used!
static int enc_B = 3; // D3 No filter caps used!
static int enc_But = A3; // ADC3 - No filter caps used!
volatile byte aFlag = 0; // expecting a rising edge on pinA encoder has arrived at a detent
volatile byte bFlag = 0; // expecting a rising edge on pinB encoder has arrived at a detent (opposite direction to when aFlag is set)
int encoderPos = 0; //current value of encoder position (0-400). Change to int or uin16_t for larger values
int maxEncPos = 201; // actually 200=4A, limit the maximum setting of the encoder
int prev_encoderPos = encoderPos; //stores the last encoder position to see if it has changed
volatile byte reading = 0; //store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

// Setup the poor man's DAC by using a PWM based version
int pwmPin = 9;      // D9 PWM output (OC1A) connected to digital pin 9 with increased frequency of 4KHz
double const DACcalibAC = 1.0; // adjust the DAC for AC current output levels to match settings
double const DACcalibDC = 1.0; // adjust the DAC for DC current output levels to match settings

// faster ADC read with 76.8 KHz sample rate 
// https://yaab-arduino.blogspot.com/2015/02/fast-sampling-from-analog-input.html
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// SSD1531 SPI declarations
// Screen dimensions
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 128 // Change this to 96 for 1.27" OLED.

// You can use any (4 or) 5 pins to drive the OLED display
#define sclk 13 // OLED SCL
#define mosi 11 // OLED SDA
#define dc   4  // OLED DC
#define cs   5  // OLED CS
#define rst  6  // OLED RES

// DUT input attenuator
int attnPin = 10; // D10 is switch the input attenuator from 40V to 400V
// DUT input relais
int DUT_PWR = 7; // MOSFET switch to turn DUT power on/off port D7

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

// Horizontal starting positions for the 5 large digits
int digit_1 = 0;
int digit_2 = 20;
int digit_3 = 45;
int digit_4 = 66;
int digit_5 = 86;

// setup the AD-DC mode switching
#define DUT_Mode A5 // use pin ADC5 for the toggle switch
boolean ac_mode_setup = false; // default for the startup
boolean dc_mode_setup = false;
boolean input_mode; // ac or dc input mode

// setup the True RMS functionality
#define RMS_WINDOW 40   // rms window of samples, 20 = 1 period @50Hz, 40 = 2
//Rms read_dut_Rms; // create an instance of Rms for the voltage
Rms read_shunt_Rms; // and one for the shunt voltage
double dutV = 0.0; // holds the DUT volt value
double prev_dutV = dutV; //keep track of value changes for display
double bridgeCal = 1.25; // bridge loss factor for AC mode
double shuntV; // holds the shunt volt (current) value
double prev_shuntV = 1.0; // keep track of value changes for display
double const shuntVcalibAC = 1.0; // calibration of setting versus actual DUT current
double const shuntVcalibDC = 3.0; // calibration of setting versus actual DUT current
volatile int request_adc = 0;
double const dc_offset_factor = 1.0; // voltage calibration factor for low voltages
double const dc_cal_factor = 54; // attenuation and calibration factor for DUT voltage
double const ac_cal_factor = 1.0; // calibration factor
double const vScale = 1.0806; // measure the AREF with an actual DMM
                            // and use that voltage in mVolt instead of the Vref
double vOffset = 0.0;         // apply a low voltage and note the difference between the ADC reading and the applied voltage.
double dutPower = 0.0; // holds the power in Watt calculation
double prev_dutPower = 1.0; // keep track of value changes for display

int readAvg = 5;  // use 5 samples to average the ADC readings

volatile int period_counter = 1; // keep track of rms calculation sampling
volatile int Measurement_completed = 0; // signal that aquisition is complete
const float VoltRange = vScale; // Needed for the RMS calculation. Max ADC input voltage.

int set_current = 0;  // in the CC mode we set the current by the encoder
int currentComp; // used in the compare function
int set_power = 0;
int set_resistance = 0;
int DAC = 0;          // the value that goes to the PWM based DAC
int prev_DAC = 0;

int counter = 500; // used to limit the OLED updates, start with a display

enum {current, power, resistance} mode = current;
char *modeStrings[] = {"CC", "CP", "CR"};
char *suffixStrings[] = {"mA", "mW", "mR"};
int prev_mode = mode;

// LM35 Temperature sensor
#define LM35 A6 // output is mv per degree C
int temperature = 0;
int tempRaw = 0;

int temp; // used in the value calculations

/*
 * Forward Function Declarations
 */
void setup_oled_ac();
void setup_oled_dc();
void display_values();
void ac_measurement();
void dc_measurement();
void readADC();
void send_volt();
void send_current();
void send_power();
void send_encoder();
void send_dac();
void send_mode();
void send_temp();
void send_to_monitor();
void send_acdc_input();
void send_ac_to_monitor(); // for debugging
void send_dc_to_monitor(); // for debugging
void oledSetup();
void enc_a_isr();
void enc_b_isr();


/*
 * The initial setup code
 */
void setup() 
{
  Serial.begin(9600); // for debugging

  pinMode(PIN_DEBUG, OUTPUT); // optional: so we can show interrupt or loop duration activity on a scope
  pinMode(DUT_Mode, INPUT_PULLUP); // Mode switch, default is AC
  pinMode(DUT_PWR, OUTPUT); // DUT input on/off switch
  digitalWrite(DUT_PWR, HIGH); // set the input relais to on
  pinMode(attnPin, OUTPUT);
  digitalWrite(attnPin, LOW); // set the attenuator in the 40V setting

  pinMode(enc_But, INPUT_PULLUP); // Encoder push button

  analogReference(INTERNAL); // Nano to use the internal 1.1V reference 
  analogRead(A0);  // force the voltage reference setting to be used and avoid an internal short

  // faster ADC read with 76.8 KHz sample rate
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0); 

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
  // we use pin 9 and Timer1, a 16-bit timer, with two PWM outputs (9 and 10)
  // Timer2 is 8-bits and one PWM output
  // https://docs.arduino.cc/tutorials/generic/secrets-of-arduino-pwm/
  cli();  //stop interrupts while we run the setup, just in case...  
  // Change the PWM base frequency from the standard 488Hz to 3.906 Hz, this helps filtering to DC.
  TCCR1B = TCCR1B & 0b11111000 | 0x02;
  pinMode(pwmPin, OUTPUT);  // sets the PWM pin as output, to act like a DAC

  // set timer0 to 1ms interrupts (timer 0 is also used for millis() and delay())
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

  // setting the startup measurement mode
  mode = current;
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
    //digitalWriteFast(PIN_DEBUG, HIGH); // optional: show start of cycle on a scope
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
  //digitalWriteFast(PIN_DEBUG, LOW); // optional: show ending on scope
  sei();//allow interrupts
}

/*
 * The main loop
 */
void loop() {
  int temp; // used in the value calculations

  // check the input measurement mode we're in, read the switch
  input_mode = digitalRead(DUT_Mode); // High is DC input, low is AC input
  
  // Is a different mode selection requested?
  if (digitalRead(enc_But) == LOW){
      Serial.println("-> button pressed");
      while (digitalRead(enc_But) == LOW){ // wait for the button to be released
      Serial.println("-> button released");
      }
    // cycle through the modes
    switch (mode){
      case current:
        mode = power;
        // reset the output to avoid DUT issues
        encoderPos = 0;
        DAC = 0;
      break;
      case power:
        mode = resistance;
        // reset the output to avoid DUT issues
        encoderPos = 0;
        DAC = 0;
      break;
      case resistance:
        mode = current;
        // reset the output to avoid DUT issues
        encoderPos = 0;
        DAC = 0;
      break;
    }
    send_mode(); // to OLED display
    //Serial.print("new mode = : "); // send to serial monitor
    //Serial.println(modeStrings[mode]); 
  }


  // read the rotary encoder switch value and set the PWM/DAC output accordingly
  // 0..255 is 0..5V in 20mA/mW/mOhm steps 
  if(prev_encoderPos != encoderPos) {
    prev_encoderPos = encoderPos;
    // send the encoder setting and the mode suffix to the OLED display
    send_encoder();
  } 

  ///// ===== AC input mode
  if (input_mode == LOW){ // we're measuring AC
    if(ac_mode_setup == false){
      setup_oled_ac();  // prepare the OLED display
      // stay in this mode but get ready for a switch to the AC mode selection
      ac_mode_setup = true;
      dc_mode_setup = false;
    }
    ac_measurement();
  } 

  /////// ===== DC input Mode
  if (input_mode == HIGH){  // We're measuring DC
    // note that in the DC mode, we do not set the global measurement_completed var so the Timer is not triggering the sampling 
    // we pick-up the ADC readings here during the normal loop execution, which is OK for DC signals.
    
    if(dc_mode_setup == false){
      setup_oled_dc();  // prepare the OLED display for DC settings
      // stay in this mode but get ready for the switch to the AC mode selection  
      dc_mode_setup = true;
      ac_mode_setup = false;
    }
    dc_measurement();
  }
  //// ===== Dynamic Software Loop
  // Depending on the mode (CC, CW, CR), we use the encoder to set the mode factor
  // So in CC, we set the current (in mA) with the decoder, in the CW mode we set the power (in mWatt) 
  // and in the CR mode we set the resistance in mOhm
  // The software controlling loop determines the setting of the current based on the measured results
  switch (mode)
    {
      case current: // Constant Current Mode
        set_current = encoderPos * 2; // encoderPos is current in 20mV clicks
        //currentComp = shuntV*100; // and cast to int
        // if (set_current > currentComp){ // shuntV is in mV for mA
        //   DAC++;
        //   DAC = constrain(DAC,0,254);
        // }else{
        //   DAC--;
        //   DAC = constrain(DAC,0,254);
        // }
        break;
      case power: // Constant Power Mode
        set_power = encoderPos; // 10mW per click
        // if (set_power > int(dutPower*100)){ // 
        //   DAC++;
        //   DAC = constrain(DAC,0,254);
        // }else{
        //   DAC--;
        //   DAC = constrain(DAC,0,254);
        // }
        break;
      case resistance: // Constant Resistance Mode
        set_resistance = int(encoderPos*100); // 100mOhm per click

        // if (set_resistance > int(shuntV*1000)){ //
        //   DAC++;
        //   DAC = constrain(DAC,0,254);
        // }else{
        //   DAC--;
        //   DAC = constrain(DAC,0,254);
        // }
        break;
    }
  // looking at the reaction time of the DAC based on the current
  // Serial.print(encoderPos);
  // Serial.print("\t");
  // Serial.print(set_current);
  // Serial.print("\t");
  // Serial.print(int(shuntV*100));
  // Serial.print("\t");
  // Serial.print(shuntV);
  // Serial.print("\t");
  // Serial.print(currentComp);
  // Serial.print("\t");
  // Serial.println(DAC);

  /// ===== End of Dynamic Loop

  // Prevent the MOSFET's from going fully open when there is no DUT.
  if (dutV < 1){
    DAC = 0;
  }

  /// ===== Set the desired current by programming the DAC
  DAC = constrain(encoderPos,0,maxEncPos); // 200 max = 4A
  analogWrite(pwmPin, DAC);

  // calculate the DUT Power in Watts
  dutPower = dutV * shuntV;


  //// ===== Update the data on the OLED display
  // to limit the number of display updates, only do that every .5 seconds
  if (counter == 500){
    //OLED updates are 20-90mS
    //digitalWriteFast(PIN_DEBUG, !digitalRead(PIN_DEBUG)); // optional: show duration of SPI cycle on a scope
    display_values();
    //send_to_monitor(); // send to Arduino serial monitor for debugging 
    // Read the temperature sensor, every .5s is enough
    tempRaw = readADC(LM35, readAvg);
    temperature = tempRaw * vScale / 1024 * 100; // mv/degree C
    send_temp();
  }

  if (counter > 500){ // reset the display loop counter
    counter = 0;
  }
  counter++;

  //delay(100); // for debugging only to slow down the loop

  //// ==== This is optional so we can look at the loop duration on a DSO
  digitalWriteFast(PIN_DEBUG, !digitalRead(PIN_DEBUG));
  // current looptime is 1.8mS

} // end-of main loop()

/*
 * AC Input Measurements
 */
void ac_measurement() {
  //=====  AC Measurements << == This needs work!

  if (Measurement_completed == 1){
    // get the DUT voltage
    // The True RMS calculation for the DUT voltage is done by the LTC1966
    // The output from the LTC1966 is 0..1V DC
    double dutVraw = readADC(DUT_INPUT, readAvg); // read the LTC output, avg
    dutVraw = ((dutVraw + vOffset) * vScale) /1024;  // adc bits to Volt conversion
    dutVraw = (dutVraw * 100) + bridgeCal;  // compensate for the /100 input divider add the bridge loss factor
    dutV = dutVraw * ac_cal_factor; // add a calibration factor       

    // get the collected shunt rms voltage
    // convert the rms voltage to the DUT current
    read_shunt_Rms.publish();
    float shuntVraw = read_shunt_Rms.rmsVal;
    shuntV = shuntVraw * shuntVcalibAC; // use the 10x input attenuator 

    Measurement_completed = 0; // restart the rms measurement cycle

    // output the DAC value
    analogWrite(pwmPin, encoderPos * DACcalibAC); // AC mode 0..255
  }
}

/*
 * DC Input Measurements
 */
void dc_measurement(){
  // The calculation for the DUT voltage is done by the LTC1966
  // obtain the output and calculate the DC DUT voltage
  double dutVraw = readADC(DUT_INPUT, readAvg); // avg 
  dutVraw = dutVraw * vScale / 1024;  // adc bits to Volt conversion
  dutV = (dutVraw * dc_cal_factor); // account for input divider and add a calibration factor
  temp = (int (dutV * 100));   // use a trick forcing only two decimals
  dutV = double (temp) / 100;   
  
  // Calculate DC Shunt voltage = current
  // We read the voltage across the sense resistor
  // The voltage across the shunt is multiplied by 5 to get 100mA = 100mV
  double shuntVraw = readADC(SHUNT_INPUT, readAvg); // adc to Volt conversion, avg 
  shuntV = ((shuntVraw + vOffset) * vScale) / 1024; // adc bits to Volt conversion
  shuntV = shuntV * shuntVcalibDC; // add the calibration factors
  temp = (int (shuntV * 100)); // use a trick forcing only two decimals
  shuntV = double (temp) / 100; 

} 

/*
 * Read ADC a few times and average the result
 */
double readADC(int adc, int n) { //take and average n values to reduce noise; return average
  float raw = analogRead(adc); //dummy read
  raw = 0;
  for (int i = 0; i < n; i++) {
    raw += analogRead(adc);
  }
  float average = raw / n;
  return (average);
}

/*
 * Send the measured results to the OLED
 */
void display_values() {
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
  // display the power in Watt
  if (prev_dutPower != dutPower){
    prev_dutPower = dutPower;
    send_power();
  }
  // display the DAC value on the OLED (for testing)
  send_dac();
  // display the mode
  if (prev_mode != mode){
    prev_mode = mode;
    send_mode();
  }
}

/*
 * send the volt result to the OLED
 */
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
  tft.print(String(dutV,2)); // send to OLED display
}

/*
 * send the current result to the OLED
 */
void send_current(){
  tft.setTextColor(GREEN);
  tft.setFont(&FreeSans18pt7b);
  tft.setTextSize(1);
  tft.fillRect(digit_1, a_line-25, digit_4+22, 27, BLACK); // clear digit field with a black rectangular
  if (shuntV > 9.99){ // shift the display one digit to the left
    tft.setCursor(digit_1, a_line);     
  }else{
    tft.setCursor(digit_2, a_line);  
  }
  tft.print(String(shuntV,2));  // send to OLED display
}

/*
 * send the power result to the OLED
 */
void send_power(){
  tft.fillRect(digit_3, p_line+2, 50, 20, BLACK);  // p-line block clear with a black rectangular
  tft.setTextColor(WHITE);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);
    if (dutPower > 9.99){ // shift the display one digit to the left
    tft.setCursor(digit_3, p_line + 17);     
  }else{
    tft.setCursor(digit_3+10, p_line + 17);
  }
  tft.print(String(dutPower,1));  // send to OLED display
}

/*
 * send the encoder setting to the OLED
 * This is the encoder value converted into mA/mW/mOhm
 */
void send_encoder(){ 
  tft.fillRect(digit_3+6, stat_line-15, 45, 20, BLACK); // Status block clear with a black rectangular
  tft.setTextColor(WHITE);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setCursor(digit_3+8, stat_line); // from left side, down
  if (mode == resistance){
    tft.print(encoderPos * 200 ); // nominal 200mOhm per click    
  }else{
    tft.print(encoderPos * 20 ); // nominal 20mA/20mW per click
  }
}

/*
 * send the DAC setting to the OLED
 * This is for debugging purposes only
 */
void send_dac(){
  tft.fillRect(digit_1, stat_line+5, 40, 20, BLACK); // Status line block clear with a black rectangular
  tft.setTextColor(WHITE);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  //tft.setCursor(digit_1, stat_line+20); // from left side, down
  //tft.print("DAC");
  tft.setCursor(digit_1, stat_line+20); // from left side, down, make room for 3 digits (255)
  tft.print(DAC ); // DAC value
}

/*
 * send the temperature result to the OLED
 */
void send_temp(){
  tft.fillRect(digit_5-5, stat_line+5, 30, 20, BLACK); // Status line block clear with a black rectangular
  tft.setTextColor(WHITE);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setCursor(digit_5, stat_line+20); // from left side, down, make room for 3 digits (255)
  tft.print(temperature); // temp value
}

/*
 * Send the AC/DC Input Mode to the OLED
 */
void send_acdc_input(){ 
  tft.fillRect(digit_1, stat_line-15, 30, 20, BLACK); // field clear with a black rectangular
  tft.setTextColor(WHITE);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setCursor(0, stat_line); // from left side, down
  if (input_mode == HIGH){ 
    tft.print("DC");
  }else{
    tft.print("AC");  
  }
}

/*
 * Send the measurement mode and the suffix to the OLED
 */
void send_mode(){ 
  // print the ac/dc mode
  tft.fillRect(0, p_line+2, 30, 20, BLACK);
  tft.setTextColor(GREEN);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setCursor(0, p_line+17); // from left side, down
  tft.print(modeStrings[mode]);

  // update the encoder suffix (mA, mW, mR)
  tft.setTextColor(WHITE);
  tft.fillRect(digit_5+5, stat_line-15, 45, 20, BLACK); // block clear with a black rectangular
  tft.setCursor(digit_5+10, stat_line); // from left side, down
  tft.print(suffixStrings[mode]); 
}

/*
 * Setup the OLED for the AC measurement mode
 */
void setup_oled_ac(){ 
  // clear the display fields and show the suffixes specific for the AC mode
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
  
  draw_degree_symbol();
  send_acdc_input();
  send_mode();
  send_dac();
}

/*
 * Setup the OLED for the DC measurement mode
 */
void setup_oled_dc(){
  // clear the display fields and show the suffixes specific for the DC mode
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
  tft.setCursor(digit_5+10, p_line+17); // from left side, down
  tft.print("W");  

  // print the encoder suffix (mA, mW or mR)
  tft.setCursor(digit_5+10, stat_line); // from left side, down
  tft.print(suffixStrings[mode]); 

  draw_degree_symbol();
  send_acdc_input();
  send_mode();
  send_dac();
}

/*
 * Draw the degree symbol
 */
void draw_degree_symbol(){
  // draw the degrees symbol ° by creating a square of 3x3 pixels
  for(int h = 112 ; h <= 114; h++) {
    for(int v = stat_line+8 ; v <= stat_line+8 + 2 ; v++) {
      tft.drawPixel(h, v, WHITE);
    }
  }
  tft.drawPixel(113 , stat_line+9, BLACK); // finish by making the hole in the square
}


/*
 * Initial setup for the OLED
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
    if (encoderPos >= maxEncPos){ 
      encoderPos = maxEncPos;
    }else{
      encoderPos ++;
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
    if (encoderPos <= 0){
      encoderPos = 0;
    }else{
      encoderPos --;
    }
    bFlag = 0; //reset flags
    aFlag = 0; //reset flags
  }
  else if (reading == B00001000) aFlag = 1; //we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

/*
 * Send the values to the Arduino IDE serial monitor for debugging purposes
 */
void send_to_monitor(){
  Serial.print(dutV,2);
  Serial.print(" V_rms\t");
//    Serial.print(", ");
//    Serial.println(read_dut_Rms.dcBias); // show the ADC bits

  Serial.print(shuntV,2);
  Serial.print(" A_rms\t"); 
//    Serial.print(", ");
//    Serial.println(read_shunt_Rms.dcBias); // show the ADC bits

  Serial.print(dutPower,2);
  Serial.print(" Watt\t"); 

  Serial.print(DAC);
  Serial.print("\t");

  Serial.print(set_resistance);
  Serial.print(" mOhm\t");

  Serial.print(encoderPos);
  Serial.print(" encoderPos\t");

  Serial.print(set_current);
  Serial.println(" setCurrent");
}


// -- END OF FILE --
