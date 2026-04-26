#include "BluetoothSerial.h"
#include "ELMduino.h"
#include <LiquidCrystal.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define INC_WRAP(v,b,e) (v == e ? v = b : v++)
#define DEC_WRAP(v,b,e) (v == b ? v = e : v--)

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SDA_PIN 15
#define SCL_PIN 4
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const int redPin = 27;
const int greenPin = 14;
const int bluePin = 12;
const int buzzerPin = 13;
const int decButtonPin = 26;
const int incButtonPin = 25;
const int selButtonPin = 33;
const int rs = 32, en = 23, d4 = 22, d5 = 21, d6 = 19, d7 = 18;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

enum screen {MAIN, BUZZER, BRIGHTNESS, GEAR, TIRE, FINAL};
int screenNum = 0;

int currentMillis = 0;
int previousMillis = 0;

bool buzzerToggle = 0; // Default off
// Each one needs 3 decimal places + the integer
int gearSelected = 0; // For UI purposes
int digitSelected = 0;
int cursorType = 0; // Toggles whether we are switching screens or values
#define CURSOR_MAX 3 // 0 - screen switching, 1 - gear switching, 2 - digit selecting 3 - digit adjusting
#define DIGIT_MAX 3 // 0 - whole #, 1 - tenths, 2 - hundreths, 3 - thousandths
float gear[6] = {3.625, 2.071, 1.474, 1.038, 0.844, 0.500}; // Default values
float tireDiameter = 22.0; // Default value (in inches)


void setColor(int redValue, int greenValue,  int blueValue) {
  analogWrite(redPin, redValue);
  analogWrite(greenPin,  greenValue);
  analogWrite(bluePin, blueValue);
}

void setBuzzer(int freq) {
    if(freq > 0 && buzzerToggle)
      tone(buzzerPin, freq);
    else
      noTone(buzzerPin);
}

void printScreen()
{
  switch(screenNum)
  {
    case MAIN:
      lcd.setCursor(0, 0);
      lcd.print("Gear:           ");
      lcd.setCursor(0, 1);
      lcd.print("                ");
      break;
    case BUZZER:
      lcd.setCursor(0, 0);
      lcd.print("Buzzer Enabled: ");
      lcd.setCursor(0, 1);
      lcd.print(buzzerToggle);
      lcd.print("               ");
      lcd.setCursor(0,1);
      break;
    case BRIGHTNESS:
      lcd.setCursor(0, 0);
      lcd.print("Brightness:     ");
      lcd.setCursor(0, 1);
      lcd.print("                ");
      lcd.setCursor(0,1);
      break;
    case GEAR:
      lcd.setCursor(0, 0);
      lcd.print("Gear #");
      lcd.print(gearSelected + 1);
      lcd.print(" Ratio: ");
      lcd.setCursor(0, 1);
      char gearPrint[6];
      sprintf(gearPrint, "%.3f", gear[gearSelected]);
      lcd.print(gearPrint);
      lcd.print("           ");
      //lcd.setCursor(0,1);
      break;
    case TIRE:
      lcd.setCursor(0, 0);
      lcd.print("Tire Diameter:  ");
      lcd.setCursor(0, 1);
      lcd.print(tireDiameter);
      lcd.print("               ");
      lcd.setCursor(0,1);
      break;
    default:
      lcd.setCursor(0, 0);
      lcd.print("UNUSED          ");
      break;
  }
}

void displayNumber(int num)
{
  display.clearDisplay();
  display.setRotation(1); // Numbers will be "sideways" so that they can occupy the wider space that the OLED has
  display.setTextSize(9,16);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(num);
  display.display();
}

void setup()
{
  // Pin initialization
  pinMode(redPin,  OUTPUT);              
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(decButtonPin, INPUT_PULLUP);
  pinMode(incButtonPin, INPUT_PULLUP);
  pinMode(selButtonPin, INPUT_PULLUP);

   // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.blink();
  // Print currently selected screen.
  printScreen();

  Serial.begin(9600);
  Wire.begin(SDA_PIN, SCL_PIN);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    Serial.println("SSD1306 allocation failed");
    for(;;); // HALT
  }

  displayNumber(6);
  delay(100);
  displayNumber(5);
  delay(100);
  displayNumber(4);
  delay(100);
  displayNumber(3);
  delay(100);
  displayNumber(2);
  delay(100);
  displayNumber(1);
  delay(100);
}

void loop()
{
  int currentMillis = millis();
  
  // Button Read
  if(digitalRead(incButtonPin) == LOW){
    switch(cursorType)
    {
      case 0: // screen switching
        INC_WRAP(screenNum, 0, FINAL-1);
        printScreen(); // Don't forget to update screen anytime something displayable is changed.
        break; 
      case 1: // gear select
        if(screenNum == GEAR)
        { 
          INC_WRAP(gearSelected, 0, 5);
          printScreen(); 
          lcd.setCursor(6,0);
        }
        else if(screenNum == TIRE)
        {
          INC_WRAP(digitSelected, 0, DIGIT_MAX);
          printScreen(); 

          if(digitSelected > 1)
            lcd.setCursor(digitSelected + 1, 1);
          else
            lcd.setCursor(digitSelected, 1);
        }
        break;

      case 2: // digit select
        if(screenNum == GEAR)
        { 
          INC_WRAP(digitSelected, 0, DIGIT_MAX);
          printScreen(); 

          if(digitSelected > 0)
            lcd.setCursor(digitSelected + 1, 1);
          else
            lcd.setCursor(digitSelected, 1);
        }
        
        else if(screenNum == TIRE)
        {
          tireDiameter += 1.0 * 1.0/pow(10, (digitSelected - 1)); // - 1 because we are starting in the tens
          printScreen(); 

          if(digitSelected > 1)
            lcd.setCursor(digitSelected + 1, 1);
          else
            lcd.setCursor(digitSelected, 1);
        }
        break;

      case 3: // digit adjust
        if(screenNum == GEAR)
        { 
          gear[gearSelected] += 1.0 * 1.0/pow(10, digitSelected);
          printScreen(); 

          if(digitSelected > 0)
            lcd.setCursor(digitSelected + 1, 1);
          else
            lcd.setCursor(digitSelected, 1);
        }
        break;
      
    }
    delay(300);
  }
  else if(digitalRead(decButtonPin) == LOW){
      switch(cursorType)
      {
        case 0: // screen switching
          DEC_WRAP(screenNum, 0, FINAL-1);
          printScreen();
          break; 
        case 1: // gear select
          if(screenNum == GEAR)
          { 
            DEC_WRAP(gearSelected, 0, 5);
            printScreen(); 
            lcd.setCursor(6,0);
          }
          else if(screenNum == TIRE)
          {
            DEC_WRAP(digitSelected, 0, DIGIT_MAX);
            printScreen(); 

            if(digitSelected > 1)
              lcd.setCursor(digitSelected + 1, 1);
            else
              lcd.setCursor(digitSelected, 1);
          }
          break;

        case 2: // digit select
          if(screenNum == GEAR)
          { 
            DEC_WRAP(digitSelected, 0, DIGIT_MAX);
            printScreen(); 

            if(digitSelected > 0)
              lcd.setCursor(digitSelected + 1, 1);
            else
              lcd.setCursor(digitSelected, 1);
          }
          
          else if(screenNum == TIRE)
          {
            tireDiameter -= 1.0 * 1.0/pow(10, (digitSelected - 1)); 
            printScreen(); 

            if(digitSelected > 1)
              lcd.setCursor(digitSelected + 1, 1);
            else
              lcd.setCursor(digitSelected, 1);
          }
          break;

        case 3: // digit adjust
          if(screenNum == GEAR)
          { 
            gear[gearSelected] -= 1.0 * 1.0/pow(10, digitSelected);
            printScreen(); 

            if(digitSelected > 0)
              lcd.setCursor(digitSelected + 1, 1);
            else
              lcd.setCursor(digitSelected, 1);
          }
          break;
      }
    delay(300);
  }
  else if(digitalRead(selButtonPin) == LOW){
    switch(screenNum)
    {
      case MAIN:
        break;

      case BUZZER:
        buzzerToggle ^= 1;
        printScreen(); 
        break;

      case BRIGHTNESS:
        break;

      case GEAR:
        if(cursorType != 3 || digitSelected == DIGIT_MAX) // Select enters in the digit until the last one, then select will go back to swapping cursorType
        {
          INC_WRAP(cursorType, 0, CURSOR_MAX);
        }
        else
        {
          INC_WRAP(digitSelected, 0, DIGIT_MAX);
          if(digitSelected > 0)
            lcd.setCursor(digitSelected + 1, 1);
          else
            lcd.setCursor(digitSelected, 1);
        }
        printScreen(); 
        break;

      case TIRE:
        if(cursorType != 2 || digitSelected == DIGIT_MAX) // Select enters in the digit until the last one, then select will go back to swapping cursorType
        {
          INC_WRAP(cursorType, 0, 2); // only going up to second cursor type
        }
        else
        {
          INC_WRAP(digitSelected, 0, DIGIT_MAX);
          if(digitSelected > 1) // Assuming there will be two digits to left of decimal and two digits to right of decimal
            lcd.setCursor(digitSelected + 1, 1);
          else
            lcd.setCursor(digitSelected, 1);
        }
        printScreen(); 
        break;
      default:
        break;
    }
      
    delay(300);
  }

  // Test LED and Buzzer
  if(currentMillis - previousMillis > 3000){
    previousMillis = currentMillis;

    setColor(255, 0, 0); // Red Color

    setBuzzer(1000);
  }
  else if(currentMillis - previousMillis > 2000){
    setColor(255, 255, 0); // Yellow Color

    setBuzzer(0); // Buzzer off

  }
  else if(currentMillis - previousMillis > 1000){
    setColor(0, 255, 0); // Green Color
  }

  
}


