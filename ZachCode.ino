
#include "Arduino_LSM6DS3.h"
#include "MadgwickAHRS.h"
#include "ArduinoBLE.h"

float roll, pitch, heading;
// initialize a Madgwick filter:
Madgwick filter;

// sensor's sample rate is fixed at 104 Hz:
const float sensorRate = 104.00;



void setup() {
 
  Serial.begin(9600);
  // attempt to start the IMU:
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    // stop here if you can't access the IMU:
    while (true);
  }
  // start the filter to run at the sample rate:
  filter.begin(sensorRate);
}

void loop() {

 // Basically need to find when to arduino is completely still
 int count = 0;
 // 2000 * 1 ms = 2s 
 while(count<200){
    
    float xG, yG, zG;
    
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
     
      //IMU.readAcceleration(xA, yA, zA);
      IMU.readGyroscope(xG, yG, zG);
      
      if (abs(xG)<10 && abs(yG)<10 && abs(zG)<10 ){
        count += 1;
        Serial.print("poggers");Serial.print(", ");
      } else {
        count = 0;
        Serial.print("4Head");Serial.print(", ");
      }
    }
    
    delay(10); 
 }
  
  Serial.print("finished");
  delay(10000);
 ///*
// Beginning to detect the actual shot
// Uploaded 
  bool shot = 0;
  for(int i=0; i<3500; i++){

    int angle = flexAngle(A4);
    float xG, yG, zG;
    float xA, yA, zA;
    
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) { 
      IMU.readAcceleration(xA, yA, zA);
      IMU.readGyroscope(xG, yG, zG);
      filter.updateIMU(xG, yG, zG, xA, yA, zA);

      roll = filter.getRoll();
      pitch = filter.getPitch();
      heading = filter.getYaw();

      
      if(angle>70 && roll>-15 && roll<90){
          shot = 1;
          
      } 
    }
    
  }

  if (shot = 1){
    //report shot was good
  } else;




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
