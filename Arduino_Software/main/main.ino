#include "Arduino_Bz`MI270_BMM150.h"
#include <math.h>

#define G_TO_ACCELERATION 9.81

class imu {
  public: 
    float gx, gy, gz;
    float ax, ay, az, filtered_ax, filtered_ay, filtered_az, a_alpha;
    float mx, my, mz;
    float pitch, roll, yaw;

    imu() {
      gx = gy = gz = 0.0;
      ax = ay = az = 0.0;
      mx = my = mz = 0.0;
      filtered_ax = filtered_ay = filtered_az  = 0.0;
      pitch,roll,yaw = 0.0;
      a_alpha = 0.5;
    }

    void display_imu() {
      Serial.print("gx: "); Serial.print(gx); Serial.print(", ");
      Serial.print("gy: "); Serial.print(gy); Serial.print(", ");
      Serial.print("gz: "); Serial.print(gz); Serial.print(", ");
      Serial.print("ax: "); Serial.print(ax); Serial.print(", ");
      Serial.print("ay: "); Serial.print(ay); Serial.print(", ");
      Serial.print("az: "); Serial.print(az); Serial.print(", ");
      Serial.print("mx: "); Serial.print(mx); Serial.print(", ");
      Serial.print("my: "); Serial.print(my); Serial.print(", ");
      Serial.print("mz: "); Serial.println(mz);
    }
    void display_acceleration() {
        Serial.print(filtered_ax); Serial.print(",");
        Serial.print(filtered_ay); Serial.print(",");
        Serial.print(filtered_az); Serial.print(",");
       //Serial.print(ax); Serial.print(",");
       //Serial.print(ay); Serial.print(",");
       //Serial.print(az); Serial.print(",");
       Serial.print(pitch); Serial.print(",");
       Serial.print(roll); Serial.print(",");
       Serial.println(yaw); 
       
    }
};

imu sensor;  

void setup() {
  Serial.begin(115200);
  while (!Serial);  

  if (!IMU.begin()) {
    Serial.println("IMU failed to initialize");
    while (1);  
  }
}

void loop() {
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(sensor.gx, sensor.gy, sensor.gz);

  }

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(sensor.ax, sensor.ay, sensor.az);
    sensor.ax *= G_TO_ACCELERATION;
    sensor.ay *= G_TO_ACCELERATION;
    sensor.az *= G_TO_ACCELERATION;

    sensor.filtered_ax = sensor.a_alpha * sensor.ax + (1-sensor.a_alpha)*sensor.filtered_ax;
    sensor.filtered_ay = sensor.a_alpha * sensor.ay + (1-sensor.a_alpha)*sensor.filtered_ay;
    sensor.filtered_az = sensor.a_alpha * sensor.az + (1-sensor.a_alpha)*sensor.filtered_az;

    sensor.pitch = -atan2(-sensor.filtered_ax,sqrt(sensor.filtered_ay*sensor.filtered_ay + sensor.filtered_az*sensor.filtered_az)) * (180/PI);
    sensor.roll = atan2(sensor.filtered_ay,sensor.filtered_az) * (180/PI);
  }

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(sensor.mx, sensor.my, sensor.mz);
  }

  //sensor.display_imu();  
  sensor.display_acceleration();
  delay(100);
}
