#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

uint16_t cadence;
uint16_t power;
uint16_t lastcrank;
uint16_t crankrev;
uint16_t bpm;

int cmd;

const byte numChars = 32;
byte receivedChars[numChars];
boolean recvInProgress = false;
boolean newData = false;

unsigned long previousMillis = 0;
unsigned long previousMillisData = 0;
unsigned long timeold = 0;
const long interval = 1000; // Update every second
const int hallhold = 200; // Holddown for hall sensor

byte hrmeasurement[3] = {0, 0, 0};
byte cpmeasurement[8] = { 0, 0, 0, 0, 0, 0, 0, 0};
byte cpfeature[4] = { 0b1000, 0, 0, 0};
byte sensorlocation[1] = { 6 };
byte hrflags = 0b1;
byte cpflags = 0b100000;

bool _BLEClientConnected = false;

#define heartRateService BLEUUID((uint16_t)0x180D)
BLECharacteristic heartRateMeasurement(BLEUUID((uint16_t)0x2A37), BLECharacteristic::PROPERTY_NOTIFY);
//BLECharacteristic heartRateConfiguration(BLEUUID((uint16_t)0x2902), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

#define cyclingPowerService BLEUUID((uint16_t)0x1818)
BLECharacteristic cyclePowerFeature(BLEUUID((uint16_t)0x2A65), BLECharacteristic::PROPERTY_READ);
BLECharacteristic cyclePowerMeasurement(BLEUUID((uint16_t)0x2A63), BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic cyclePowerSensorLocation(BLEUUID((uint16_t)0x2A5D), BLECharacteristic::PROPERTY_READ);

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      _BLEClientConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      _BLEClientConnected = false;
    }
};

void InitBLE ()
{
  // Create BLE Device
  BLEDevice::init ("Multi Fitness BLE Sensor");

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Services
  // Create Heart Rate Service
  BLEService *pHeart = pServer->createService(heartRateService);
  pHeart->addCharacteristic( &heartRateMeasurement);

  // Create Power Service
  BLEService *pPower = pServer->createService(cyclingPowerService);
  pPower->addCharacteristic( &cyclePowerFeature);
  pPower->addCharacteristic( &cyclePowerMeasurement);
  pPower->addCharacteristic( &cyclePowerSensorLocation);

  // Add UUIDs for Services to BLE Service Advertising
  pServer->getAdvertising()->addServiceUUID(heartRateService);  
  pServer->getAdvertising()->addServiceUUID(cyclingPowerService);

  // Start p Instances
  pPower->start();
  pHeart->start();

  // Start Advertising
  pServer->getAdvertising()->start();

  // Set feature/sensorlocation
  cyclePowerFeature.setValue(cpfeature, 4);
  cyclePowerSensorLocation.setValue(sensorlocation, 1); 

}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  InitBLE();
  crankrev = 0;
  lastcrank = 0;
  bpm = 0;
  cmd = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  recvWithStartEndMarkers();
  showNewData(); //Uncomment to show wtf is going on.
  checkRPM();
  updateMeasurements();
  askForData();
}

void checkRPM() {
  unsigned long currentMillis = millis();
  //enough delay to assume crank passed
  if (((currentMillis-timeold) > hallhold) && (touchRead(T3) < 20)) {
    timeold = currentMillis;
    crankrev = crankrev+1;
    lastcrank = currentMillis*1000/1024;
  }
}

void updateMeasurements() {

  cpmeasurement[0] = cpflags & 0xff;
  cpmeasurement[1] = (cpflags >> 8) & 0xff;
  cpmeasurement[2] = power & 0xff;
  cpmeasurement[3] = (power >> 8) & 0xff;
  cpmeasurement[4] = (uint8_t)(crankrev & 0xFF);
  cpmeasurement[5] = (uint8_t)(crankrev >> 8);
  cpmeasurement[6] = (uint8_t)(lastcrank & 0xFF);
  cpmeasurement[7] = (uint8_t)(lastcrank >> 8);

  hrmeasurement[0] = hrflags & 0xff;
  hrmeasurement[1] = bpm & 0xff;
  hrmeasurement[2] = (bpm >> 8) & 0xff;

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.print("Power: ");
    Serial.println(power);
    Serial.print("Heartrate: ");
    Serial.println(bpm);
    Serial.print("Crankrevs: ");
    Serial.println(crankrev);
    heartRateMeasurement.setValue(hrmeasurement, 3);
    heartRateMeasurement.notify();
    cyclePowerMeasurement.setValue(cpmeasurement, 8);
    cyclePowerMeasurement.notify();
  }
}

void askForData() {
  unsigned long currentMillis = millis();
  if (newData == false && recvInProgress == false && (currentMillis - previousMillisData >= interval)) {
    previousMillisData = currentMillis;
    if (cmd == 0 ) {
      Serial.println("Asking for Power");
      Serial2.write(0xF1);
      Serial2.write(0xB4);
      Serial2.write(0xB4);
      Serial2.write(0xF2);
      cmd++;
    } else {
      Serial.println("Asking for HR");
      Serial2.write(0xF1);
      Serial2.write(0xB0);
      Serial2.write(0xB0);
      Serial2.write(0xF2);
      cmd = 0;
    }
  }
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    int startMarker = 0xF1;
    int endMarker = 0xF2;
    int rc;
 
    while (Serial2.available() > 0 && newData == false) {
        rc = Serial2.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
            }
            else {
                receivedChars[ndx] = rc;
                recvInProgress = false;
                ndx = 0;
//                Serial.println("Setting datatype: ");
//                Serial.println(receivedChars[2], HEX);
                if ( cmd == 0) bpm = receivedChars[4];
                else power = receivedChars[4];
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
            receivedChars[ndx] = rc;
            ndx++;
        }
    }
}

void showNewData() {
    if (newData == true) {
        Serial.println("This just in ... ");
        for (int element : receivedChars) {
            Serial.print("0x");
            Serial.print(element, HEX);
            Serial.print(",");
        }
        Serial.println();
        newData = false;
    }
}
