// TODO: add mode switch when holding sel button down again, get rid of main screen, and let main screen operate during run mode
// TODO: incorporate multicolor LED indication into the gear shift algorithm

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>
#include <LiquidCrystal.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define DEBUG Serial
#define epsilon 0.0001
#define RPM_LOW 1500 //lowest the RPM should go after shifting
#define RPM_HIGH 3000 //highest RPM will be, automaticcaly shift after this
#define LOAD_HIGH 0.75 // highest the load should go after shifting
#define INC_WRAP(v,b,e) (v == e ? v = b : v++)
#define DEC_WRAP(v,b,e) (v == b ? v = e : v--)

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SDA_PIN 15
#define SCL_PIN 4

#define CURSOR_MAX 3 // 0 - screen switching, 1 - gear switching, 2 - digit selecting 3 - digit adjusting
#define DIGIT_MAX 3 // 0 - whole #, 1 - tenths, 2 - hundreths, 3 - thousandths
enum screen {MAIN, BUZZER, BRIGHTNESS, GEAR, TIRE, FINAL};
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
const int rs = 32, en = 23, d4 = 22, d5 = 21, d6 = 19, d7 = 18;
const int buzzerPin = 13;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
static BLEUUID serviceUUID("FFF0");  //Hardcoded for now
static BLEUUID charTXUUID("FFF2");   //TX is transmitting, hardcoded for now
static BLEUUID charRXUUID("FFF1");   //RX is receiving, hardcoded for now
const int redPin = 27;
const int greenPin = 14;
const int bluePin = 12;
const int decButtonPin = 26;
const int incButtonPin = 25;
const int selButtonPin = 33;
static BLEAddress obdAddress("1c:A1:35:69:8D:C5");  //also hardcoded for now
//For all of above values it is possible that they can be scanned for using the BLE libraries but idk how to do it right now
BLERemoteCharacteristic* txChar;
BLERemoteCharacteristic* rxChar;
BLEClient* client;
bool connected = false;
//variables for data
volatile float rpm = 0;
volatile float speed = 0;
volatile float load = 0;
volatile int currentGear;
volatile int lastGear = 0;
volatile float currentGearRatio;
volatile unsigned long long int mytime = 0;
volatile unsigned long long int timediff = 0;
volatile unsigned long long int prevSpeedTime = 0;
volatile float prevSpeed;
volatile float acceleration;
static float tireDiameter = 24.878;  //tire diameter in inches, will eventually have to be input by the user
volatile unsigned long long int lastGearChangeTime;
bool buzzerActive = 0;
volatile unsigned long long int buzzerStart;
int screenNum = 0;
int cursorType = 0; // Toggles whether we are switching screens or values


//variables for "semaphore" system for requesting data
// volatile bool rpmRequested = false;
// volatile bool rpmDone = false;
// volatile bool speedRequested = false;
// volatile bool speedDone = false;
// volatile bool MAFRequested = false;
// volatile bool MAFDone = false;
volatile bool pending = false;
bool buzzerToggle = 1; // Default value. 0 = off, 1 = on
int gearSelected = 0; // For UI purposes
int digitSelected = 0;
enum OBDstate {
  reqrpm,
  reqspeed,
  reqload
};
float gearRatios[6] = { 3.625, 2.071, 1.474, 1.038, 0.844, 999};  //gear ratios for gears 1-6, 6 is 0 here because it is a 5 speed and we don't want it going there
OBDstate reqstate = reqrpm;
enum SystemMode {
  RUN,       // normal operation (OBD + shifting)
  EDIT     // menu/editing mode
};

// For multicolor LED
void setColor(int redValue, int greenValue,  int blueValue) {
  analogWrite(redPin, redValue);
  analogWrite(greenPin,  greenValue);
  analogWrite(bluePin, blueValue);
}

SystemMode mode = RUN;
static unsigned long pressStart = 0; // This may need to be a global to retain the timing

void handleModeSwitch() {
  if (digitalRead(selButtonPin) == LOW) {
    if (pressStart == 0) {
      pressStart = millis();
    }

    if (millis() - pressStart > 1000) { // 1 second hold
      mode = (mode == RUN) ? EDIT : RUN;

      lcd.clear();
      printScreen();

      pressStart = 0;
      delay(300); // debounce
    }
  } else {
    pressStart = 0;
  }
}
void updatePeripherals(){
  if(!buzzerActive)
    return;
  if(millis() - buzzerStart > 300){
    setBuzzer(0);
    buzzerActive = false;

    setColor(255, 255, 255);

    return;
  }
}
void setBuzzer(int freq) {

    if(freq > 0 && buzzerToggle)
      tone(buzzerPin, freq);
    else
      noTone(buzzerPin);
}
void displayNumber(int num)
{
  display.clearDisplay();
  display.setRotation(0); // Numbers will be "sideways" so that they can occupy the wider space that the OLED has
  display.setTextSize(16,9);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(num);
  display.display();
}

float computeGearRatio(float rpm, float speed, float tireDiameter) {
  if (speed < 1) {
    return 3.625;  //hardwire first gear when speed <1mph
  }
  float gearRatio = (rpm * PI * tireDiameter) / (1056.0f * speed * 3.96f);  //formula to get gear ratio 1056 is just for unit conversions 3.96 is final drive ratio, get from vehicle spec sheet or have break in period for the device where user logs data to device
  return gearRatio;
}
int computeGear(float gearRatio, float* array, int numGears) {
  int gear = 1;
  float closest = fabsf(gearRatio - array[0]);
  for (int i = 1; i < numGears; i++) {
    float distance = fabsf(gearRatio - array[i]);
    if (distance < closest) {
      closest = distance;
      gear = i + 1;
    }
  }
  return gear;
}
// bool compareEffeciency(float rpm, float speed, float acceleration, float MAF, float* array, int currentGear){
//   float gearRatio = array[currentGear-1];
//   float gearRatioNext = array[currentGear];
//   float ratio = gearRatioNext/gearRatio;
//   float rpmNext = rpm * ratio;
//   float speedNext = speed * ratio;
//   float MAFnext = MAF * rpmNext/ rpm;
//   float accelerationNext = acceleration * ratio * rpm/rpmNext;  //this will have to be tested at some point in order to find better algorithm
//   float effeciency = speed * acceleration / MAF;
//   float effeciencyNext = speedNext * accelerationNext/MAFnext;
//   if(acceleration < 0){
//     return false;
//   }
//   if(effeciencyNext > effeciency)
//     return true;
//   else
//     return false;
// }
bool shouldShift(){
  // TODO: 5th gear lmao
  if(currentGear == 5){
    return false;
  }
  float rpmNext = rpm * gearRatios[currentGear]/gearRatios[currentGear-1]; //next/current ratio
  float loadNext = load * gearRatios[currentGear-1]/gearRatios[currentGear]; //current/next ratio
  if(rpmNext < RPM_LOW){
    return false;
  }
  if(loadNext > LOAD_HIGH){
    return false;
  }
  return true;
}
//Notify callback function to get data after it is requested using PID
void notifyCallback(
  BLERemoteCharacteristic* chr,
  uint8_t* data,
  size_t length,
  bool isNotify) {
  //Serial.print("RX: ");

  String msg = "";

  for (int i = 0; i < length; i++) {
    char c = (char)data[i];
    msg += c;
  }
  msg.trim();
  //Serial.println(msg);
  if (msg.startsWith("01")) {
    //Serial.println("Ignoring echo...");
    return;
  }

  msg.replace(">", "");
  msg.replace("\r", "");
  msg.replace("\n", "");
  //if(msg.length() >= 11){
  String reqType = msg.substring(3, 5);
  //These if else statements can be turned into a case with enums for reqType eventually, speed, rpm maf etc.
  //0C is RPM
  if (reqType == "0C") {
    int A = strtol(msg.substring(6, 8).c_str(), NULL, 16);
    int B = strtol(msg.substring(9, 11).c_str(), NULL, 16);
    rpm = ((256 * A) + B) / 4.0;
    //Serial.print("RPM: ");
    //Serial.println(rpm);
    pending = false;
    reqstate = reqspeed;

  } else if (reqType == "0D") {
    int A = strtol(msg.substring(6, 8).c_str(), NULL, 16);
    float dt = (millis() - prevSpeedTime)/1000.0f;
    // DEBUG.print("dt: ");
    // DEBUG.println(dt);
    prevSpeedTime = millis();
    speed = A / 1.609;  //speed reading from OBD-II is in kmh, dividing by 1.609 should give mph
    acceleration = (float)(speed - prevSpeed) / dt;
    prevSpeed = speed;
    //Serial.print("Speed: ");
    //Serial.println(speed);
    pending = false;
    reqstate = reqload;
  } else if (reqType == "04") {
    int A = strtol(msg.substring(6, 8).c_str(), NULL, 16);
    load = A/2.55*0.01;
    // Serial.print("Load: ");
    // Serial.println(load);
    pending = false;
    reqstate = reqrpm;
  }
  //}
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
      sprintf(gearPrint, "%.3f", gearRatios[gearSelected]);
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

bool connectToOBD() {
  DEBUG.println("Connecting to BLE OBD...");

  client = BLEDevice::createClient();

  if (!client->connect(obdAddress)) {
    DEBUG.println("BLE connection failed");
    return false;
  }

  DEBUG.println("Connected to BLE device");

  BLERemoteService* service = client->getService(serviceUUID);
  if (!service) {
    DEBUG.println("No service????");
    client->disconnect();
    return false;
  }

  txChar = service->getCharacteristic(charTXUUID);
  rxChar = service->getCharacteristic(charRXUUID);

  if (!txChar || !rxChar) {
    DEBUG.println("No tx or rx????");
    client->disconnect();
    return false;
  }


  if (rxChar->canNotify()) {
    rxChar->registerForNotify(notifyCallback);
  }

  return true;
}

// ===== Send command =====
void sendOBD(String cmd) {
  if (!txChar) {
    DEBUG.println("no tx????");
    return;
  }

  //DEBUG.print("TX: ");
  //DEBUG.println(cmd);

  cmd += "\r";
  txChar->writeValue(cmd.c_str(), cmd.length());
}

void setup() {
  pinMode(redPin,  OUTPUT);              
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(decButtonPin, INPUT_PULLUP);
  pinMode(incButtonPin, INPUT_PULLUP);
  pinMode(selButtonPin, INPUT_PULLUP);

  lcd.begin(16, 2);
  lcd.blink(); //added
  // Print currently selected screen.
  printScreen();

  Wire.begin(SDA_PIN, SCL_PIN);

  DEBUG.begin(115200);
  DEBUG.println("Starting Client application to connect to OBDII:");
  BLEDevice::init("");
  connected = connectToOBD();

  if (!connected) {
    DEBUG.println("Connection failed");

    mode = EDIT;
    //while (true) delay(1000);
  }
  sendOBD("ATE0");  // echo OFF
  DEBUG.println("Connected");

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    DEBUG.println("SSD1306 allocation failed");
    for(;;); // HALT
  }

  delay(2000);

   setColor(255,255,255);
   displayNumber(1);
}

void loop() {
  handleModeSwitch();
  if(mode == RUN){
    if (!pending) {
      switch (reqstate) {
        case reqrpm:
          sendOBD("010C");
          pending = true;
          timediff = millis() - mytime;
          mytime = millis();
          //DEBUG.print("Time between rpm requests:");
          //DEBUG.println(timediff);
          break;
        case reqspeed:
          sendOBD("010D");
          pending = true;
          break;
        case reqload:
          currentGearRatio = computeGearRatio(rpm, speed, tireDiameter); // gearRatio only updated once speed is read
          currentGear = computeGear(currentGearRatio, gearRatios, 5);
          sendOBD("0104");
          pending = true;
          break;
      }
    }

    if(currentGear != lastGear){
      if((millis()-lastGearChangeTime) > 500){
        DEBUG.println("====== SHIFTED GEAR =======");
        DEBUG.print(lastGear);
        DEBUG.print(" -> ");
        DEBUG.println(currentGear);
        lcd.setCursor(0, 0);
        lcd.print("= SHIFTED GEAR =");
        lcd.setCursor(0, 1);
        lcd.print(lastGear);
        lcd.print(" -> ");
        lcd.print(currentGear);
        lcd.print("          ");
        lastGear = currentGear;
        lastGearChangeTime = millis();
      }
    }
    // if(compareEffeciency(rpm, speed, acceleration, MAF, gearRatios, currentGear)){
    //   if(((millis() - lastGearChangeTime) > 200) && !buzzerActive){
    //     // setBuzzer(1000);
    //     buzzerActive = true;
    //     buzzerStart = millis();
    //     DEBUG.println("buzzer set off");
    //   }
    // }
    if(shouldShift()){
      if(!buzzerActive && millis() - lastGearChangeTime > 200){
        buzzerActive = true;
        setBuzzer(1000);
        DEBUG.println("Should Shift activated");
        buzzerStart = millis();
      }
      setColor(0, 255, 0);

    }
    updatePeripherals();
    //put the rest of the code here
    // lcd.setCursor(0, 0);
    // lcd.print(currentGear);
    displayNumber(currentGear);
  }
  else if(mode == EDIT){  
    //have button changing below
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
            gearRatios[gearSelected] += 1.0 * 1.0/pow(10, digitSelected);
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
              gearRatios[gearSelected] -= 1.0 * 1.0/pow(10, digitSelected);
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
  }
} 