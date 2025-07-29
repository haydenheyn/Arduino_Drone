// IMPORTANT
// this is not a working example, this is just to show how to set the library
// if you need a working example please see the other example
#include "Arduino_BMI270_BMM150.h"

#include "SensorFusion.h" //SF
SF fusion;

float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;


void setup() {
  Serial.begin(115200); //serial to display data
  if(!IMU.begin()) {
    Serial.println("IMU Failed");
    while(1); // halt
  }
}

void loop() {

  // now you should read the gyroscope, accelerometer (and magnetometer if you have it also)
  // NOTE: the gyroscope data have to be in radians
  // if you have them in degree convert them with: DEG_TO_RAD example: gx * DEG_TO_RAD
  if(IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax,ay,az);
    ax*=9.8;
    ay*=9.8;
    az*=9.8;
  }
  if(IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx,gy,gz);
    gx*=DEG_TO_RAD;
    gy*=DEG_TO_RAD;
    gz*=DEG_TO_RAD;
  }
  if(IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx,my,mz);
  }

  deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
  fusion.MahonyUpdate(gx, gy, gz, -ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
  //fusion.MadgwickUpdate(gx, gy, gz, -ax, ay, az, mx, my, mz, deltat);  //else use the magwick, it is slower but more accurate

  pitch = fusion.getPitch();
  roll = fusion.getRoll();    
  yaw = fusion.getYaw();
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.print(gz); Serial.print(",");
  Serial.print(pitch); Serial.print(",");
  Serial.print(roll);Serial.print(",");
  Serial.println(yaw);
}















