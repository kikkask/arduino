// Code for the Bravo rocket electronics to be used in LASC for ESP32 DEVKIT V1.
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
BLA::Matrix<2,2> F; BLA::Matrix<2,1> G;
BLA::Matrix<2,2> P; BLA::Matrix<2,2> Q;
BLA::Matrix<2,1> S; BLA::Matrix<1,2> H;
BLA::Matrix<2,2> I; BLA::Matrix<1,1> Acc;
BLA::Matrix<2,1> K; BLA::Matrix<1,1> R;
BLA::Matrix<1,1> L; BLA::Matrix<1,1> M;

// Kalman filter function
void kalman_2d(void){
  Acc = {AccZInertial};
  S = F * S + G * Acc;
  P = F * P * ~F + Q;
  L = H * P * ~H + R;
  K = P * ~H * Invert(L);
  M = {AltitudeBarometer};
  S = S + K * (M - H * S);
  AltitudeKalman = S(0,0);
  VelocityVerticalKalman = S(1,0);
  P = (I - K * H) * P;
}

// Barometer reading function
void barometer_signals(void){
  Wire.beginTransmission(0x76);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(0x76, 6);

  uint8_t press_msb = Wire.read();
  uint8_t press_lsb = Wire.read();
  uint8_t press_xlsb = Wire.read();
  uint8_t temp_msb = Wire.read();
  uint8_t temp_lsb = Wire.read();
  uint8_t temp_xlsb = Wire.read();

  uint32_t adc_P = ((uint32_t)press_msb << 12) | ((uint32_t)press_lsb << 4) | ((uint32_t)press_xlsb >> 4);
  uint32_t adc_T = ((uint32_t)temp_msb << 12) | ((uint32_t)temp_lsb << 4) | ((uint32_t)temp_xlsb >> 4);

  int32_t var1, var2, t_fine;
  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;

  int64_t var1_p, var2_p, p;
  var1_p = ((int64_t)t_fine) - 128000;
  var2_p = var1_p * var1_p * (int64_t)dig_P6;
  var2_p = var2_p + ((var1_p * (int64_t)dig_P5) << 17);
  var2_p = var2_p + (((int64_t)dig_P4) << 35);
  var1_p = ((var1_p * var1_p * (int64_t)dig_P3) >> 8) + ((var1_p * (int64_t)dig_P2) << 12);
  var1_p = (((((int64_t)1) << 47) + var1_p)) * ((int64_t)dig_P1) >> 33;
  
  if (var1_p == 0) {
    p = 0; // Avoid division by zero
  } else {
    p = 1048576 - adc_P;
    p = (((p << 31) - var2_p) * 3125) / var1_p;
    var1_p = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2_p = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1_p + var2_p) >> 8) + (((int64_t)dig_P7) << 4);
  }
  
  double pressure = (double)p / 25600.0;
  AltitudeBarometer = 44330 * (1 - pow(pressure / 1013.25, 1 / 5.255)) * 100;
}

// Gyro and accelerometer reading function
void gyro_signals(void) {
  // Read accelerometer data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);

  uint8_t AccXHigh = Wire.read();
  uint8_t AccXLow = Wire.read();
  uint8_t AccYHigh = Wire.read();
  uint8_t AccYLow = Wire.read();
  uint8_t AccZHigh = Wire.read();
  uint8_t AccZLow = Wire.read();

  int16_t AccXLSB = (int16_t)(AccXHigh << 8 | AccXLow);
  int16_t AccYLSB = (int16_t)(AccYHigh << 8 | AccYLow);
  int16_t AccZLSB = (int16_t)(AccZHigh << 8 | AccZLow);

  // Read gyro data
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);

  uint8_t GyroXHigh = Wire.read();
  uint8_t GyroXLow = Wire.read();
  uint8_t GyroYHigh = Wire.read();
  uint8_t GyroYLow = Wire.read();
  uint8_t GyroZHigh = Wire.read();
  uint8_t GyroZLow = Wire.read();

  int16_t GyroX = (int16_t)(GyroXHigh << 8 | GyroXLow);
  int16_t GyroY = (int16_t)(GyroYHigh << 8 | GyroYLow);
  int16_t GyroZ = (int16_t)(GyroZHigh << 8 | GyroZLow);

  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  // Apply accelerometer calibration offsets here
  AccX = (float)AccXLSB / 4096 - 0.05;
  AccY = (float)AccYLSB / 4096 + 0.01;
  AccZ = (float)AccZLSB / 4096 - 0.11;

  AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * (180 / PI);
  AnglePitch = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * (180 / PI);
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
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // Initialize Barometer
  Wire.beginTransmission(0x76);
  Wire.write(0xF4);
  Wire.write(0x57);
  Wire.endTransmission();

  Wire.beginTransmission(0x76);
  Wire.write(0xF5);
  Wire.write(0x14);
  Wire.endTransmission();

  // Read calibration data from barometer
  uint8_t data[24];
  Wire.beginTransmission(0x76);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(0x76, 24);
  for (uint8_t i = 0; i < 24 && Wire.available(); i++) {
    data[i] = Wire.read();
  }

  dig_T1 = (data[1] << 8) | data[0];
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6];
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11] << 8) | data[10];
  dig_P4 = (data[13] << 8) | data[12];
  dig_P5 = (data[15] << 8) | data[14];
  dig_P6 = (data[17] << 8) | data[16];
  dig_P7 = (data[19] << 8) | data[18];
  dig_P8 = (data[21] << 8) | data[20];
  dig_P9 = (data[23] << 8) | data[22];
  delay(250);

  // Calibrate barometer
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    barometer_signals();
    AltitudeBarometerStartUp += AltitudeBarometer;
    delay(1);
  }
  AltitudeBarometerStartUp /= 2000;

  // Initialize Kalman filter matrices
  F = {1, 0.004,
       0, 1};
  G = {0.5 * 0.004 * 0.004,
       0.004};
  H = {1, 0};
  I = {1, 0,
 
