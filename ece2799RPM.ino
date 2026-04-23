#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>

#define DEBUG Serial

static BLEUUID serviceUUID("FFF0");  //Hardcoded for now
static BLEUUID charTXUUID("FFF2");  //TX is transmitting, hardcoded for now
static BLEUUID charRXUUID("FFF1");  //RX is receiving, hardcoded for now

static BLEAddress obdAddress("1c:A1:35:69:8D:C5"); //also hardcoded for now

BLERemoteCharacteristic* txChar;
BLERemoteCharacteristic* rxChar;
BLEClient* client;

bool connected = false;

void notifyCallback(
  BLERemoteCharacteristic* chr,
  uint8_t* data,
  size_t length,
  bool isNotify) {
  Serial.print("RX: ");

  String msg = "";

  for (int i = 0; i < length; i++) {
    char c = (char)data[i];
    Serial.print(c);
    msg += c;
  }

  Serial.println();
  msg.trim();

  msg.replace(">", "");
  msg.replace("\r", "");
  msg.replace("\n", "");

  if (msg.indexOf("41 0C") != -1) {
    int A = strtol(msg.substring(6, 8).c_str(), NULL, 16);
    int B = strtol(msg.substring(9, 11).c_str(), NULL, 16);

    float rpm = ((256 * A) + B) / 4.0; //will need to make this a global variable eventually, just local for now

    Serial.print("RPM: ");
    Serial.println(rpm);
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

  DEBUG.print("TX: ");
  DEBUG.println(cmd);

  cmd += "\r";
  txChar->writeValue(cmd.c_str(), cmd.length());
}

void setup() {
  DEBUG.begin(115200);
  DEBUG.println("Starting Client application to connect to OBDII:");
  BLEDevice::init("");

  connected = connectToOBD();

  if (!connected) {
    DEBUG.println("Connection failed");
    while (true) delay(1000);
  }

  DEBUG.println("Connected");

  delay(2000);
}

void loop() {
  sendOBD("010C");  // 010C is code for engine RPM (from wikipedia)
  delay(1000);
}
