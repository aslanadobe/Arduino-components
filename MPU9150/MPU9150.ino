#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9150.h"
#include "Kalman.h"

#define RESTRICT_PITCH

/* Compass Offset */
#define X_OFFSET -32
#define Y_OFFSET -16
#define Z_OFFSET -122

/* Create Kalman instances for roll, pitch, yaw */
Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanZ;

MPU9150 accelGyroMag;

/* IMU DATA */
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
int16_t magX, magY, magZ;

double BX, BY, BZ; //mag used to calculate yaw

double roll, pitch, yaw;

double gyroAngleX, gyroAngleY, gyroAngleZ; //Angle calculate using the gyro only
double compAngleX, compAngleY, compAngleZ; //Calculated angle using a complementary filter
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter

uint32_t timer;



void setup() {
  Wire.begin();
  Serial.begin(38400);

  Serial.println("Initializing I2C devices...");
  accelGyroMag.initialize();

  //Verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelGyroMag.testConnection() ? "MPU9150 connection successful" : "MPU9150 connection failed");

  delay(100); //Wait for the sensor to stabilize

  /* Set Kalman and gyro starting angle */
  accelGyroMag.getMotion9(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ, &magX, &magY, &magZ);
  updateRollPitch();
  updateYaw();

  kalmanX.setAngle(roll); // First set roll starting angle
  gyroAngleX = roll;
  compAngleX = roll;

  kalmanY.setAngle(pitch); // Then pitch
  gyroAngleY = pitch;
  compAngleY = pitch;

  kalmanZ.setAngle(yaw); // And finally yaw
  gyroAngleZ = yaw;
  compAngleZ = yaw;

  Serial.print("The starting angle:\t");
  Serial.print("roll:\t");
  Serial.print(roll); Serial.print("\t");
  Serial.print(gyroAngleX); Serial.print("\t");
  Serial.print(compAngleX); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");

  Serial.print("pitch:\t");
  Serial.print(pitch); Serial.print("\t");
  Serial.print(gyroAngleY); Serial.print("\t");
  Serial.print(compAngleY); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");


  Serial.print("yaw:\t");
  Serial.print(yaw); Serial.print("\t");
  Serial.print(gyroAngleZ); Serial.print("\t");
  Serial.print(compAngleZ); Serial.print("\t");
  Serial.print(kalAngleZ); Serial.print("\t");

  Serial.print("\n");

  timer = micros(); // Initialize the timer
}

void loop() {
  // Get IMU Data
  accelGyroMag.getMotion9(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ, &magX, &magY, &magZ);

  double dt = (double) (micros() - timer) / 1000000;   //Calculate delta time
  timer = micros();

  /* Roll Pitch Estimation */
  updateRollPitch();
  double gyroXrate = (double)gyroX / 131.0; //Convert to deg/s
  double gyroYrate = (double)gyroY / 131.0; //Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroAngleX = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroAngleY = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  /* Yaw Estimation */
  updateYaw();
  double gyroZrate = gyroZ / 131.0; // Convert to deg/s
  // This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
  if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
    kalmanZ.setAngle(yaw);
    compAngleZ = yaw;
    kalAngleZ = yaw;
    gyroAngleZ = yaw;
  } else
    kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter

  /* Estimate angles using gyro only */
  gyroAngleX += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroAngleY += gyroYrate * dt;
  gyroAngleZ += gyroZrate * dt;

  /* Estimate angles using complimentary filter */
  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

  // Reset the gyro angles when they has drifted too much
  if (gyroAngleX < -180 || gyroAngleX > 180)
    gyroAngleX = kalAngleX;
  if (gyroAngleY < -180 || gyroAngleY > 180)
    gyroAngleY = kalAngleY;
  if (gyroAngleZ < -180 || gyroAngleZ > 180)
    gyroAngleZ = kalAngleZ;

  Serial.print("roll:\t");
  Serial.print(roll); Serial.print("\t");
  Serial.print(gyroAngleX); Serial.print("\t");
  Serial.print(compAngleX); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");

  //Serial.print("pitch:\t");
  //Serial.print(pitch); Serial.print("\t");
  //Serial.print(gyroAngleY); Serial.print("\t");
  //Serial.print(compAngleY); Serial.print("\t");
  //Serial.print(kalAngleY); Serial.print("\t");


  Serial.print("yaw:\t");
  Serial.print(yaw); Serial.print("\t");
  Serial.print(gyroAngleZ); Serial.print("\t");
  Serial.print(compAngleZ); Serial.print("\t");
  Serial.print(kalAngleZ); Serial.print("\t");

  Serial.print("\n");



}
void updateRollPitch() {
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll  = atan2((double) accY, (double) accZ) * RAD_TO_DEG;
  pitch = atan((double) - accX / sqrt((double) accY * (double) accY + (double) accZ * (double) accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll  = atan((double) accY / sqrt((double) accX * (double)accX + (double) accZ * (double)accZ)) * RAD_TO_DEG;
  pitch = atan2((double) - accX, (double)accZ) * RAD_TO_DEG;
#endif
}

void updateYaw() {
  BX = magY - Y_OFFSET;
  BY = magX - X_OFFSET;
  BZ = (magZ - Z_OFFSET) * -1;

  double rollAngle = kalAngleX * DEG_TO_RAD;
  double pitchAngle = kalAngleY * DEG_TO_RAD;
  double Bfy = BZ * sin(rollAngle) - BY * cos(rollAngle);
  double Bfx = BX * cos(pitchAngle) + BY * sin(pitchAngle) * sin(rollAngle) + BZ * sin(pitchAngle) * cos(rollAngle);
  yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;


}
