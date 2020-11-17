#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>
#include "MadgwickAHRS.h"

Madgwick filter;
const float sensorRate = 104.00;
float roll=0, pitch=0, heading=0;
int shot=0;

BLEService customService("1101");
BLEUnsignedIntCharacteristic customXChar("2101", BLERead | BLENotify);
BLEUnsignedIntCharacteristic customYChar("2102", BLERead | BLENotify);
BLEUnsignedIntCharacteristic customZChar("2103", BLERead | BLENotify);
BLEUnsignedIntCharacteristic customBruhChar("2104", BLERead | BLENotify);

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
filter.begin(sensorRate);
BLE.setLocalName("Arduino Gyro");
BLE.setAdvertisedService(customService);
customService.addCharacteristic(customXChar);
customService.addCharacteristic(customYChar);
customService.addCharacteristic(customZChar);
customService.addCharacteristic(customBruhChar);

BLE.addService(customService);
customXChar.writeValue(roll);
customYChar.writeValue(pitch);
customZChar.writeValue(heading);
customBruhChar.writeValue(shot);



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

 
Imu_Read();    


   
customXChar.writeValue(roll);
customYChar.writeValue(pitch);
customZChar.writeValue(heading);
customBruhChar.writeValue(shot);





}
}
digitalWrite(LED_BUILTIN, LOW);
Serial.print("Disconnected from central: ");
Serial.println(central.address());
}

void Imu_Read(){
     float xG, yG, zG;
    float xA, yA, zA;
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) { 
      IMU.readAcceleration(xA, yA, zA);
      IMU.readGyroscope(xG, yG, zG);
      filter.updateIMU(xG, yG, zG, xA, yA, zA);

      roll = filter.getRoll();
      pitch = filter.getPitch();
      heading = filter.getYaw();

      if (shot_verify(roll) == 1){
        digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(100);
      }
    
    
    Serial.print(heading);
   Serial.print(" ");
   Serial.print(pitch);
   Serial.print(" ");
   Serial.println(roll);
     
    }
}
    
    
    

bool shot_verify(float a){
  int A4;
  int angle = flexAngle4(A4);
  //Serial.println(angle);
   if(angle>40 && roll>30 && roll<130){
          shot = 1;
          
      } 
      else{
        shot =0;
      }
      return shot;
}

int STRAIGHT_RESISTANCE = 20000;
int BEND_RESISTANCE = 80000;
float VCC = 3.3;
float R_DIV = 10000;

int flexAngle(int analogPin) {

   // Read the ADC
  int flexADC = analogRead(analogPin);
  

  // Calculate the voltage that the ADC read
  float flexV = flexADC * VCC / 1023.0;

  // Calculate the resistance of the flex sensor
  float flexR = R_DIV * (VCC / flexV - 1.0);

  // Use the calculated resistance to estimate the sensor's
  // bend angle my mapping the measured resistance onto the
  // known resistances at zero and ninety degrees of bend.
  float angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE,
                   0, 90.0);
  
  return angle;
    
}
