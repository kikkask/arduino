// Code for the Bravo rocket electronics to be used in LASC.
#include <Wire.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

// Global variables
float RateRoll, RatePitch, RateYaw;
float AngleRoll, AnglePitch;
float AccX, AccY, AccZ;
float AccZInertial;
unsigned long LoopTimer;
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5;
int16_t dig_P6, dig_P7, dig_P8, dig_P9;
float AltitudeBarometer, AltitudeBarometerStartUp;
int RateCalibrationNumber;
float AltitudeKalman, VelocityVerticalKalman;

// Kalman filter matrices
BLA::Matrix<2,2> F_matrix;
BLA::Matrix<2,1> G_matrix;
BLA::Matrix<2,2> P_matrix;
BLA::Matrix<2,2> Q_matrix;
BLA::Matrix<2,1> S_vector;
BLA::Matrix<1,2> H_matrix;
BLA::Matrix<2,2> I_matrix;
BLA::Matrix<1,1> Acc_matrix;
BLA::Matrix<2,1> K_vector;
BLA::Matrix<1,1> R_matrix;
BLA::Matrix<1,1> L_matrix;
BLA::Matrix<1,1> M_matrix;

// Kalman filter function
void kalman_2d(void){
  Acc_matrix(0,0) = AccZInertial;
  S_vector = F_matrix * S_vector + G_matrix * Acc_matrix;
  P_matrix = F_matrix * P_matrix * (~F_matrix) + Q_matrix;
  L_matrix = H_matrix * P_matrix * (~H_matrix) + R_matrix;

  // Since L_matrix is a 1x1 matrix, invert it manually
  float invL = 1.0f / L_matrix(0,0);

  // Compute Kalman gain K_vector
  K_vector = (P_matrix * (~H_matrix)) * invL;

  M_matrix(0,0) = AltitudeBarometer;

  // Update state estimate S_vector
  S_vector = S_vector + K_vector * (M_matrix - H_matrix * S_vector);

  AltitudeKalman = S_vector(0,0);
  VelocityVerticalKalman = S_vector(1,0);

  // Update estimate covariance P_matrix
  P_matrix = (I_matrix - K_vector * H_matrix) * P_matrix;
}

// Barometer reading function
void barometer_signals(void){
  // [Barometer reading code remains unchanged]
  // Ensure you replace the placeholder comments with your actual barometer code.
  // ...
}

// Gyro and accelerometer reading function
void gyro_signals(void) {
  // [Gyro reading code remains unchanged]
  // Ensure you replace the placeholder comments with your actual gyro code.
  // ...
}

void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // Initialize I2C with ESP32 default SDA and SCL pins
  Wire.begin(21, 22);
  Wire.setClock(400000);
  delay(250);

  // Initialize MPU6050
  // [MPU6050 initialization code remains unchanged]
  // ...

  // Initialize Barometer
  // [Barometer initialization code remains unchanged]
  // ...

  // Read calibration data from barometer
  // [Calibration code remains unchanged]
  // ...

  // Calibrate barometer
  // [Calibration loop remains unchanged]
  // ...

  // Initialize Kalman filter matrices by assigning elements individually
  F_matrix(0,0) = 1.0f;
  F_matrix(0,1) = 0.004f;
  F_matrix(1,0) = 0.0f;
  F_matrix(1,1) = 1.0f;

  G_matrix(0,0) = 0.5f * 0.004f * 0.004f;
  G_matrix(1,0) = 0.004f;

  H_matrix(0,0) = 1.0f;
  H_matrix(0,1) = 0.0f;

  I_matrix(0,0) = 1.0f;
  I_matrix(0,1) = 0.0f;
  I_matrix(1,0) = 0.0f;
  I_matrix(1,1) = 1.0f;

  // Compute Q_matrix
  Q_matrix = (G_matrix * (~G_matrix)) * 100.0f; // Ensure scalar multiplication is with float

  R_matrix(0,0) = 900.0f; // 30 * 30

  // Initialize P_matrix and S_vector to zero matrices
  P_matrix.Fill(0.0f);
  S_vector.Fill(0.0f);

  LoopTimer = micros();
}

void loop() {
  gyro_signals();
  AccZInertial = -sin(AnglePitch * (PI / 180.0f)) * AccX +
                  cos(AnglePitch * (PI / 180.0f)) * sin(AngleRoll * (PI / 180.0f)) * AccY +
                  cos(AnglePitch * (PI / 180.0f)) * cos(AngleRoll * (PI / 180.0f)) * AccZ;
  AccZInertial = (AccZInertial - 1.0f) * 9.81f * 100.0f;

  barometer_signals();
  AltitudeBarometer -= AltitudeBarometerStartUp;
  kalman_2d();

  Serial.print("Altitude [cm]: ");
  Serial.print(AltitudeKalman);
  Serial.print(" Vertical velocity [cm/s]: ");
  Serial.println(VelocityVerticalKalman);

  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}
