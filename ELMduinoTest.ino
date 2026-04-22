#include "BluetoothSerial.h"
#include "ELMduino.h"
#include <LiquidCrystal.h>

BluetoothSerial SerialBT;
#define ELM_PORT SerialBT
#define DEBUG_PORT Serial

ELM327 myELM327;

typedef enum { ENG_RPM, SPEED } obd_pid_states;
obd_pid_states obd_state = ENG_RPM;

float rpm = 0;
float mph = 0;

const int redPin = 27;
const int greenPin = 14;
const int bluePin = 12;
const int buzzerPin = 13;
const int decButtonPin = 26;
const int incButtonPin = 25;
const int selButtonPin = 33;
const int rs = 32, en = 23, d4 = 22, d5 = 21, d6 = 19, d7 = 18;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

enum screen {MAIN, BUZZER, BRIGHTNESS};
int screenNum = 0;

int currentMillis = 0;
int previousMillis = 0;

bool buzzerToggle = 1;


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
  lcd.setCursor(0, 0);

  switch(screenNum)
  {
    case MAIN:
      lcd.print("Gear:           ");
      lcd.setCursor(0, 1);
      lcd.print("                ");
      break;
    case BUZZER:
      lcd.print("Buzzer Enabled: ");
      lcd.setCursor(0, 1);
      lcd.print(buzzerToggle);
      break;
    case BRIGHTNESS:
      lcd.print("Brightness:     ");
      lcd.setCursor(0, 1);
      lcd.print("                ");
      break;
  }
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

  Serial.begin(9600);
    /**
    DEBUG_PORT.begin(115200);
    // SerialBT.setPin("1234");
    ELM_PORT.begin("ArduHUD", true);

    

    if (!ELM_PORT.connect("OBDII"))
    {
        Serial.println("Can't connect AHHH!!!!");

        DEBUG_PORT.println("Couldn't connect to OBD scanner - Phase 1");
        while (1)
            ;
    }

    if (!myELM327.begin(ELM_PORT, true, 2000))
    {
        DEBUG_PORT.println("Couldn't connect to OBD scanner - Phase 2");
        while (1)
            ;
    }

    DEBUG_PORT.println("Connected to ELM327");
    **/
}

void loop()
{
  int currentMillis = millis();

  // Print currently selected screen.
  printScreen();
  
  // Button Read
  if(digitalRead(incButtonPin) == LOW){
    screenNum == 2 ? screenNum = 0 : screenNum++;
    delay(300);
  }
  else if(digitalRead(decButtonPin) == LOW){
    screenNum == 0 ? screenNum = 2 : screenNum--;
    delay(300);
  }
  else if(digitalRead(selButtonPin) == LOW){
    buzzerToggle ^= 1;
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

  

  
  /**
  switch (obd_state)
  {
    case ENG_RPM:
    {
      rpm = myELM327.rpm();
      
      if (myELM327.nb_rx_state == ELM_SUCCESS)
      {
        DEBUG_PORT.print("rpm: ");
        DEBUG_PORT.println(rpm);
        obd_state = SPEED;
      }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
      {
        myELM327.printError();
        obd_state = SPEED;
      }
      
      break;
    }
    
    case SPEED:
    {
      mph = myELM327.mph();
      
      if (myELM327.nb_rx_state == ELM_SUCCESS)
      {
        DEBUG_PORT.print("mph: ");
        DEBUG_PORT.println(mph);
        obd_state = ENG_RPM;
      }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
      {
        myELM327.printError();
        obd_state = ENG_RPM;
      }
      
      break;
    }
  }
  **/
}


