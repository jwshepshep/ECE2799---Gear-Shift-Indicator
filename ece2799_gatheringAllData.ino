#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>
#include <LiquidCrystal.h>
#include <math.h>
#define DEBUG Serial
const int rs = 32, en = 23, d4 = 22, d5 = 21, d6 = 19, d7 = 18;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
static BLEUUID serviceUUID("FFF0");  //Hardcoded for now
static BLEUUID charTXUUID("FFF2");   //TX is transmitting, hardcoded for now
static BLEUUID charRXUUID("FFF1");   //RX is receiving, hardcoded for now

static BLEAddress obdAddress("1c:A1:35:69:8D:C5");  //also hardcoded for now
//For all of above values it is possible that they can be scanned for using the BLE libraries but idk how to do it right now
BLERemoteCharacteristic* txChar;
BLERemoteCharacteristic* rxChar;
BLEClient* client;
bool connected = false;
//variables for data
volatile float rpm = 0;
volatile float speed = 0;
volatile float MAF = 0;
volatile int currentGear;
volatile float currentGearRatio;
volatile unsigned long long int mytime = 0;
volatile unsigned long long int timediff = 0;
volatile unsigned long long int prevSpeedTime = 0;
volatile float prevSpeed;
volatile float acceleration;
static float tireDiameter = 24.878;  //tire diameter in inches, will eventually have to be input by the user
//variables for "semaphor" system for requesting data
// volatile bool rpmRequested = false;
// volatile bool rpmDone = false;
// volatile bool speedRequested = false;
// volatile bool speedDone = false;
// volatile bool MAFRequested = false;
// volatile bool MAFDone = false;
volatile bool pending = false;
enum OBDstate {
  reqrpm,
  reqspeed,
  reqMAF
};
float gearRatios[6] = { 3.625, 2.071, 1.474, 1.038, 0.844, 0 };  //gear ratios for gears 1-6, 6 is 0 here because it is a 5 speed and we don't want it going there
OBDstate reqstate = reqrpm;

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
//   float rpmnNext = l
// }
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
    float dt = millis() - prevSpeedTime;
    DEBUG.print("dt: ");
    DEBUG.println(dt);
    prevSpeedTime = millis();
    speed = A / 1.609;  //speed reading from OBD-II is in kmh, dividing by 1.609 should give mph
    acceleration = (float)(speed - prevSpeed) / dt;
    DEBUG.print("Speed: ");
    DEBUG.println(speed);
    DEBUG.print("Previous speed: ");
    DEBUG.println(prevSpeed);
    DEBUG.print("acceleration");
    DEBUG.println(acceleration);
    prevSpeed = speed;
    //Serial.print("Speed: ");
    //Serial.println(speed);
    pending = false;
    reqstate = reqMAF;
  } else if (reqType == "10") {
    int A = strtol(msg.substring(6, 8).c_str(), NULL, 16);
    int B = strtol(msg.substring(9, 11).c_str(), NULL, 16);
    MAF = ((256 * A) + B) / 100.0;
    //Serial.print("MAF: ");
    //Serial.println(MAF);
    pending = false;
    reqstate = reqrpm;
  }
  //}
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
  txChar->writeValue(cmd.c_str(), cmd.length());  // I have changed this line since testing, if it doesn't work remove the true from the end and only use 2 parameters
}

void setup() {
  DEBUG.begin(115200);
  DEBUG.println("Starting Client application to connect to OBDII:");
  BLEDevice::init("");
  lcd.begin(16, 2);
  connected = connectToOBD();

  if (!connected) {
    DEBUG.println("Connection failed");
    while (true) delay(1000);
  }
  sendOBD("ATE0");  // echo OFF
  DEBUG.println("Connected");

  delay(2000);
}

void loop() {
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
      case reqMAF:
        sendOBD("0110");
        pending = true;
        break;
    }
  }
  currentGearRatio = computeGearRatio(rpm, speed, tireDiameter);
  currentGear = computeGear(currentGearRatio, gearRatios, 5);
  //put the rest of the code here
  lcd.setCursor(0, 0);
  lcd.print(currentGear);
}