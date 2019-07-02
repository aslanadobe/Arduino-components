#include "Wire.h"
#include"I2Cdev.h"
#include"MPU9150.h"

MPU9150 accelGyroMag;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

bool xy_calibrated, z_calibrated, z_ready = false;

float x_max, x_min, x_offset;
float y_max, y_min, y_offset;
float z_max, z_min, z_offset;

int x_init, y_init, z_init;

uint32_t timer;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(38400);

  Serial.println("Initializing I2C devices...");
  accelGyroMag.initialize();

  Serial.println("Testing device connections...");
  Serial.println(accelGyroMag.testConnection() ? "MPU9150 connection successful" : "MPU9150 connection failed");

  accelGyroMag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  x_init = mx;
  y_init = my;

  delay(1000);

  Serial.println("Please be ready to calibrate the magnetometer");
  timer = millis();

}

void loop() {
  // put your main code here, to run repeatedly:
  calibrateMag();
  if ( xy_calibrated and z_calibrated) {
    x_offset = ( x_max + x_min) / 2;
    y_offset = ( y_max + y_min) / 2;
    z_offset = ( z_max + z_min) / 2;
    Serial.print("The offset of x/y/z is\t");
    Serial.print(x_offset);  Serial.print("\t");
    Serial.print(y_offset);  Serial.print("\t");
    Serial.print(z_offset);  Serial.print("\n");
  }
}
void calibrateMag() {
  accelGyroMag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  if (not xy_calibrated) {
    Serial.println("Please start rotating");
    if (mx > x_max) x_max = mx;
    if (mx < x_min) x_min = mx;
    if (my > y_max) y_max = my;
    if (my < y_min) y_min = my;
    if (((mx - x_init) <= 10) && ((my - y_init) <= 10) && ((millis() - timer) >= 5000)) {
      xy_calibrated = true;
      Serial.println("XY-axis calibration completed");
      Serial.print("X_max:\t"); Serial.print(x_max);
      Serial.print("X_min:\t"); Serial.print(x_min); Serial.print("\t");
      Serial.print("Y_max:\t"); Serial.print(y_max);
      Serial.print("Y_min:\t"); Serial.print(y_min);  Serial.print("\n");
      Serial.println("Please be ready to rotate around z-axis");
      z_init = mz;

    }
  }
  if ( xy_calibrated and (not z_calibrated)) {
    if (not z_ready) {
      Serial.println("Please rotate to make Z-axis horizonal");
      if ((z_init - mz) > 25) {
        z_ready = true;
        z_init = mz;
        timer = millis();
        Serial.println("Now Z-axis is ready to calibrate");
      }
    }
    else {
      Serial.println("Please start rotating");
      if ( mz > z_max)  z_max = mz;
      if ( mz < z_min)  z_min = mz;
      if ((( mz - z_init) <= 10) and ((millis() - timer) > 5000)) {
        z_calibrated = true;
        Serial.println("Z-axis calibration completed");
        Serial.print("Z_max:\t"); Serial.print(z_max);
        Serial.print("Z_min:\t"); Serial.print(z_min); Serial.print("\n");
        Serial.println("Calibration completed");
      }
    }
  }

}
