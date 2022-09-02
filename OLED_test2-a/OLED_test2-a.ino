/*******
 * This is a test program for a rotary encoder with and a SSD1531 color OLED Display
 * 
 * The code is for a AC/DC dynamic load user interface.
 * 
 * There are several bits and pieces from others used in this sketch
 * Author Paul Versteeg
 * 
 * SS1531 library from Adafruit 
 * https://github.com/adafruit/Adafruit-GFX-Library
 *
 *
 * Interrupt-based Rotary Encoder Sketch
 * by Simon Merrett, based on insight from Oleg Mazurov, Nick Gammon, rt, Steve Spence
 * MyOneButton from JEFF'S ARDUINO BLOG
*/

#include <Adafruit_GFX.h>
#include <Fonts/FreeSans18pt7b.h> // used for V and A digits
#include <Fonts/FreeSans9pt7b.h> // used for the mode line
#include <Adafruit_SSD1351.h>
#include <SPI.h>
#include "MyOneButton.h" // file must be in same folder as the main .ino file

static int enc_A = 2; // No filter caps used!
static int enc_B = 3; // No filter caps used!
//static int enc_But = 7; // No filter caps used!
volatile byte aFlag = 0; // expecting a rising edge on pinA encoder has arrived at a detent
volatile byte bFlag = 0; // expecting a rising edge on pinB encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile byte encoderPos = 0; //current value of encoder position (0-255). Change to int or uin16_t for larger values
volatile byte oldEncPos = 0; //stores the last encoder position to see if it has changed
volatile byte reading = 0; //store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent


boolean down = false; // direction of decade counter

int butt_pos;        // Current state of the encoder button
                     // (LOW is pressed b/c i'm using the pullup resistors)
long millis_held;    // How long the button was held (milliseconds)
long secs_held;      // How long the button was held (seconds)
long prev_secs_held; // How long the button was held in the previous check
byte prev_butt_pos = HIGH;
unsigned long firstTime; // how long since the button was first pressed 

/*
 * Menu and Mode Declarations
 */
 enum menu{
  OpsMode,  // normal Operating Mode
  MenuMode, // menu mode, selects V or A settings, future rotary encode activity
            //   enter 1-click to move from digit to digit, exit by a long click or menu exit
  VAMode,   // V/A setting mode, can switch between V or A
            //   enter via MenuMode, exit via long click
  VSMode,   // Volt setting Mode
            //   enter via VAMode, exit via double-click
  ASMode    // Ampere setting Mode
            //   enter via VAMode, exit via double-click
 };

enum menu menuState;

// SSD1531 SPI declarations
// You can use any (4 or) 5 pins to drive the display 
#define sclk 2
#define mosi 3
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
Adafruit_SSD1351 tft = Adafruit_SSD1351(cs, dc, rst);

// OLED display vertical line positions
int v_line = 30;    // Volt/Watt/Ohm line
int a_line = 65;    // Amp line
int menu_line = 100; // Menu line
int stat_line = 118; // Status line

// used to address the individual digits in the V/I//RW lines
int pos = 4; // 5 digits, pos 1 is LSB of 88.888
// placeholders for the original values for each digit
int old_enc_pos_1 = 0; 
int old_enc_pos_2 = 0;
int old_enc_pos_3 = 0;
int old_enc_pos_4 = 0;
int old_enc_pos_5 = 0;


/*
 * Some Forward Function Declarations
 */
void fillpixelbypixel();
void enc_a_isr();
void enc_b_isr();
void digitClear();
void selectDigit();
void oledSetup();

 
#define enc_But 7 // Rotary Encoder Button connected to input pin D7

MyOneButton bp; // instantiate the button press class

// Declarations for the encoder button states
/*
 * OnClick is a single click response. 
 * Selects a menu item in the Menu Mode.
 * Moves from digit to digit in the V/A Setting mode.
 */
void OnClick(int) { // with more buttons, use (int pin) as a var to differentiate
  Serial.println("Click");
  selectDigit();
}

/*
 * OnDblClick is a double click response.
 * Used to enter the Menu Mode if in the Operations Mode.
 * Used to exit the V/A Setting Mode and goes back to the Menu Mode.
 */
void OnDblClick(int) {
  Serial.println("Double Click");
  // not used yet
}

/*
 * OnLongPress is used to terminate any setting activity and quickly jumps 
 * back to the Operating Mode.
 */
void OnLongPress(int) {
  Serial.println("Long Press");
  // not used yet
}

/*
 * OnVlongPress is not used at the moment.
 */
void OnVLongPress(int) {
  Serial.println("Very Long Press");
  // not used yet
}

  


void setup(void) {
  // setup the rotary encoder switches and ISR's
  pinMode(enc_A, INPUT_PULLUP); // set pinA as an input, pulled HIGH
  pinMode(enc_B, INPUT_PULLUP); // set pinB as an input, pulled HIGH
  attachInterrupt(0, enc_a_isr,RISING); //
  attachInterrupt(1, enc_b_isr,RISING); //
  /*
   * One Button Setup links to the service routines
   */
  bp.Configure(enc_But);
  bp.OnClick = OnClick;
  bp.OnDblClick = OnDblClick;
  bp.OnLongPress = OnLongPress;
  bp.OnVLongPress = OnVLongPress;
  
  Serial.begin(115200);
  Serial.println("Starting...");
  tft.begin();  
  oledSetup(); // setup the initial display values
    
  Serial.println("setup is done");
}


void loop(){

  bp.CheckBP(); // check for a rotary encoder button press
  
  if(oldEncPos != encoderPos) {
    Serial.println(encoderPos);
    digitClear();
    oldEncPos = encoderPos;
  }
}

/*
 * Move from digit to digit in the V/A line.
 * Moves are going up and down from 1-2-3-4-5-4-3-2-1 through the line
 * The selected digit field is made black, which is the background, to make place
 * for a new value selected by the rotary switch.
 * 
 * Later: change encoder ISR's to have limits here (encoderPos % max)
 */
void digitClear(){
  switch (pos) { // encoder rotating changes the value of the selected digit/position
    case 1:
      tft.fillRect(0, 16, 20, 26, BLACK); // define a black rectangular the size of a digit
      tft.setCursor(0, v_line);           // go to the right position of the digit
      tft.print(String(encoderPos));      // black it out
      break;
    case 2:
      tft.fillRect(20, 16, 20, 26, BLACK);
      tft.setCursor(20, v_line);
      tft.print(String(encoderPos));
      break;
    case 3:
      tft.fillRect(45, 16, 20, 26, BLACK);
      tft.setCursor(45, v_line);
      tft.print(String(encoderPos));
      break;
    case 4:
      tft.fillRect(66, 16, 20, 26, BLACK);
      tft.setCursor(66, v_line);
      tft.print(String(encoderPos));
      break;
    case 5:
      tft.fillRect(86, 16, 20, 26, BLACK);
      tft.setCursor(86, v_line);
      tft.print(String(encoderPos));
      break;
    default:
      Serial.print("case issue : ");
      Serial.println(pos);
      break;
  }
}

/*
 * Used to create special characters and symbols.
 * Not used yet.
 */
void fillpixelbypixel(uint16_t color) {
  for (uint8_t x=0; x < tft.width(); x++) {
    for (uint8_t y=0; y < tft.height(); y++) {
      tft.drawPixel(x, y, color);
    }
  }
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
    if (encoderPos >= 9){
      encoderPos = 9;
    }else{
      encoderPos ++;
    }
    bFlag = 0; //reset flags
    aFlag = 0; //reset flags
  }
  else if (reading == B00001000) aFlag = 1; //we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

/*
 * Rotary encoder single short push button click
 */
void selectDigit(){
 
  Serial.print("Button pressed, dir: ");
  Serial.println(down);
  // walk through the 5 digit positions back and forth with every push of the rotary button
  if (down) {
    pos --;
    if (pos <= 1) {
      down = false;
    }
  }else{
    pos ++;
    if (pos >= 5) {
      down = true;
    }
  }
  // debug:    Serial.println(pos);
  switch (pos) { // move from digit to digit and change the colors of the previous digit and the target digit
    case 1:
      // always coming from 2 change that back to BLUE
      tft.setCursor(20, v_line);
      tft.setTextColor(BLUE);
      old_enc_pos_2 = encoderPos; // save the 2nd position
      tft.print(String(encoderPos)); // the rotary decoded number        
      // Highlight pos 1
      tft.fillRect(0, 16, 20, 26, BLACK);
      tft.setCursor(0, v_line);
      tft.setTextColor(RED);
      encoderPos = old_enc_pos_1; // show the saved 1st pos number
      tft.print(String(encoderPos));
      break;
    case 2:
      if (down) { // coming from 3 change that back to BLUE
        tft.setCursor(45, v_line);
        tft.setTextColor(BLUE);
        old_enc_pos_3 = encoderPos; // save the 3 pos number
        tft.print(String(encoderPos));
      }else{ // coming from 1 change that back to BLUE
        tft.setCursor(0, v_line);
        tft.setTextColor(BLUE);
        old_enc_pos_1 = encoderPos; // save the 1st pos number
        tft.print(String(encoderPos));
      }
      // Highlight 2
      tft.fillRect(20, 16, 20, 26, BLACK);
      tft.setCursor(20, v_line);
      tft.setTextColor(RED);
      encoderPos = old_enc_pos_2; // show the saved 2nd number
      tft.print(String(encoderPos));
      break;
    case 3:
      if (down) { // coming from 4 change that back to BLUE
        tft.setCursor(66, v_line);
        tft.setTextColor(BLUE);
        old_enc_pos_4 = encoderPos; // show the previous 4th number
        tft.print(String(encoderPos));
      }else{ // coming from 2 change that to BLUE
        tft.setCursor(20, v_line);
        tft.setTextColor(BLUE);
        old_enc_pos_2 = encoderPos; // saved the 2nd pos number
        tft.print(String(encoderPos));
      }
      // Highlight 3
      tft.fillRect(45, 16, 20, 26, BLACK);
      tft.setCursor(45, v_line);
      tft.setTextColor(RED);
      encoderPos = old_enc_pos_3; // show the saved 3rd pos number
      tft.print(String(encoderPos));
      break;
    case 4:
      if (down) { // coming from 5 change back to BLUE
        tft.setCursor(86, v_line);
        tft.setTextColor(BLUE);
        old_enc_pos_5 = encoderPos; // save the 5th pos number
        tft.print(String(encoderPos));
      }else{ // coming from 3 change back to BLUE
        tft.setCursor(45, v_line);
        tft.setTextColor(BLUE);
        old_enc_pos_3 = encoderPos; // save the 3rd pos number
        tft.print(String(encoderPos));
      }
      // Highlight 4     
      tft.fillRect(66, 16, 20, 26, BLACK);
      tft.setCursor(66, v_line);
      tft.setTextColor(RED);
      encoderPos = old_enc_pos_4; // show the saved 4th pos number
      tft.print(String(encoderPos));
      break;
    case 5:
      // always coming from 4 change that back to BLUE
      tft.setCursor(66, v_line);
      tft.setTextColor(BLUE);
      old_enc_pos_4 = encoderPos; // save the 4th pos number
      tft.print(String(encoderPos));
      // Highlight 5
      tft.fillRect(86, 16, 20, 26, BLACK);
      tft.setCursor(86, v_line);
      tft.setTextColor(RED);
      encoderPos = old_enc_pos_5; // show the saved 5th pos number
      tft.print(String(encoderPos));
      break;
    default:
      Serial.print("case issue : ");
      Serial.println(pos);
      break;
  }
  delay(50); // probably not needed with the full code
}


/*
 * OLED display initial setup code
 */

void oledSetup() {
  // clear the screen
  tft.fillScreen(BLACK);

  //-------------------------------------------
  // Draw the Volt line
  //
  // The line will be 00.000V in blue with the 4th digit in red
    
  tft.setTextColor(BLUE);
  tft.setFont(&FreeSans18pt7b);
  tft.setTextSize(1);
  
  tft.setCursor(0, v_line);
  tft.print(String(old_enc_pos_1));
  tft.setCursor(20, v_line);
  tft.print(String(old_enc_pos_2));

  // draw a decimal point using a square of 8x8 pixels
  for(int h = 40 ; h <= 43; h++) {
    for(int v = v_line -3 ; v <= v_line; v++) {
      tft.drawPixel(h, v, BLUE);
    }
  }

  tft.setCursor(45, v_line);  
  tft.print(String(old_enc_pos_3));
  tft.setCursor(66, v_line);
  tft.setTextColor(RED);
  tft.print(String(old_enc_pos_4));
  tft.setTextColor(BLUE);
  tft.setCursor(86, v_line); 
  tft.print(String(old_enc_pos_5));
  tft.setFont();
  tft.setTextSize(2);
  tft.setCursor(108, v_line - 13);
  tft.println("V");

  //-------------------------------------------
  // draw the Amp line
  //
  // The line will be 88.888A in green with the 4th digit in red
  
  tft.setTextColor(GREEN);
  tft.setFont(&FreeSans18pt7b);
  tft.setTextSize(1);

  tft.setCursor(0, a_line);
  tft.print("8");
  tft.setCursor(20, a_line);
  tft.print("8");
  
  // draw a decimal point using a square of 8x8 pixels
  for(int h = 40 ; h <= 43; h++) {
    for(int v = a_line -3 ; v <= a_line; v++) {
      tft.drawPixel(h, v, GREEN);
    }
  }
  tft.setCursor(45, a_line);  
  tft.print("8");
  tft.setCursor(66, a_line); 
  tft.setTextColor(RED);
  tft.print("8");
  tft.setTextColor(GREEN);
  tft.setCursor(86, a_line); 
  tft.print("8");
 
  tft.setFont();
  tft.setTextSize(2);
  tft.setCursor(108, a_line - 13); 
  tft.println("A");

  //-------------------------------------------
  // draw the status line
  //
  // left justified: Fan Off 
  // right justified Temp 99C
  
  tft.setTextColor(WHITE);
  tft.setFont(); 
  tft.setTextSize(1);
  tft.setCursor(0, stat_line); // from left side, down
  tft.print("Fan Off");
  
  tft.setCursor(65, stat_line); 
  tft.print("Temp 99");
  
  // draw the degrees symbol ° by creating a square of 3x3 pixels
  for(int h = 108 ; h <= 110; h++) {
    for(int v = stat_line ; v <= stat_line + 2 ; v++) {
      tft.drawPixel(h, v, WHITE);
    }
  }
  tft.drawPixel(109 , stat_line + 1, BLACK); // now make the hole in the square
  
  tft.setCursor(113, stat_line );
  tft.print("C");

  //-------------------------------------------
  // draw the menu line
  //
  tft.setTextColor(WHITE);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setCursor(0, menu_line); // from left side, down
  tft.print("CC Mode");  
  tft.setCursor(90, menu_line); 
  tft.setTextColor(GREEN);
  tft.print("OFF");
  
  //-------------------------------------------
  // reset the font and colors for the V/A lines
  tft.setTextColor(BLUE);
  tft.setFont(&FreeSans18pt7b);
  tft.setTextSize(1);

  //-------------------------------------------
  // set the outsice corner indicators by using one pixel  
  tft.drawPixel(0, 0, WHITE);
  tft.drawPixel(0, 127, WHITE);
  tft.drawPixel(127, 0, WHITE);
  tft.drawPixel(127, 127, WHITE);
}
