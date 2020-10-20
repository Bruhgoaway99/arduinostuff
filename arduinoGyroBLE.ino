#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>

int gyroX=1;
int gyroY=1;
int gyroZ=1;
float x, y, z;

BLEService customService("1101");
BLEUnsignedIntCharacteristic customXChar("2101", BLERead | BLENotify);
BLEUnsignedIntCharacteristic customYChar("2102", BLERead | BLENotify);
BLEUnsignedIntCharacteristic customZChar("2103", BLERead | BLENotify);

void setup() {
IMU.begin();
Serial.begin(9600); 
while (!Serial);

pinMode(LED_BUILTIN, OUTPUT);

if (!BLE.begin()) {
Serial.println("BLE failed to Initiate");
delay(500);
while (1);
}

BLE.setLocalName("Arduino Gyro");
BLE.setAdvertisedService(customService);
customService.addCharacteristic(customXChar);
customService.addCharacteristic(customYChar);
customService.addCharacteristic(customZChar);
BLE.addService(customService);
customXChar.writeValue(gyroX);
customYChar.writeValue(gyroY);
customYChar.writeValue(gyroZ);


BLE.advertise();

Serial.println("Bluetooth device is now active, waiting for connections...");
}


void loop() {

BLEDevice central = BLE.central();
if (central) {
Serial.print("Connected to central: ");
Serial.println(central.address());
digitalWrite(LED_BUILTIN, HIGH);
while (central.connected()) {
delay(200);
read_Gyro();

customXChar.writeValue(gyroX);
customYChar.writeValue(gyroY);
customZChar.writeValue(gyroZ);

Serial.print(gyroX);
Serial.print('\t');
Serial.print(gyroY);
Serial.print('\t');
Serial.println(gyroZ);
}
}
digitalWrite(LED_BUILTIN, LOW);
Serial.print("Disconnected from central: ");
Serial.println(central.address());
}

void read_Gyro() {

 if (IMU.gyroscopeAvailable()) {
   IMU.readGyroscope(x, y, z);
gyroX = x;
gyroY = y;
gyroZ = z;
}
}
