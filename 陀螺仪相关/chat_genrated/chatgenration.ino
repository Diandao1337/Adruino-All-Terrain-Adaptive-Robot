#include <Wire.h>
#include <Servo.h>

// MPU6050相关
#define MPU_ADDRESS 0x68
#define ACCEL_SCALE 16384.0
#define GYRO_SCALE 131.0
#define PITCH_OFFSET 2.0 //校准偏移量

int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;
float pitch;

//串口通信相关
#define SERIAL_BAUDRATE 9600
int input_angle = 0;

//PID控制相关
#define KP 10
#define KD 1
#define KI 5
#define PID_INTERVAL 10

double error = 0;
double last_error = 0;
double integral_error = 0;
double output = 0;

//舵机控制相关
Servo servo;
#define SERVO_PIN 3
#define SERVO_MIN_ANGLE -90
#define SERVO_MAX_ANGLE 90
#define SERVO_MIN_PULSE_WIDTH 544
#define SERVO_MAX_PULSE_WIDTH 2400

//设置MPU6050
void setupMPU() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

//读取MPU6050数据
void readMPUData() {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 14, true);  // request a total of 14 registers

  //读取加速度计数据
  accel_x = Wire.read() << 8 | Wire.read();
  accel_y = Wire.read() << 8 | Wire.read();
  accel_z = Wire.read() << 8 | Wire.read();

  //读取陀螺仪数据
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();

  //计算pitch角度
  float accel_angle = atan2(accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / PI;
  float gyro_rate = gyro_y / GYRO_SCALE;
  pitch = 0.98 * (pitch + gyro_rate * PID_INTERVAL / 1000.0) + 0.02 * accel_angle + PITCH_OFFSET;
}

//设置串口通信
void setupSerial() {
  Serial.begin(SERIAL_BAUDRATE);
}

//读取串口输入
void readSerialInput() {
  if (Serial.available()) {
    input_angle = Serial.parseInt();
  }
}

//计算PID控制值
void calculatePID() {
  //计算误差
  error = input_angle - pitch;

  //计算积分误差
  integral_error += error * PID_INTERVAL;

  //计算微分误差
  double derivative_error = (error - last_error) / PID_INTERVAL;

  //计算控制量
  output = KP * error + KD * derivative_error + KI * integral_error;
  //限制输出控制量范围
if (output > SERVO_MAX_ANGLE) {
output = SERVO_MAX_ANGLE;
} else if (output < SERVO_MIN_ANGLE) {
output = SERVO_MIN_ANGLE;
}

//记录上一次误差
last_error = error;
}
int pulse_width;
//控制舵机角度
void controlServo() {
pulse_width = map(output, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
servo.writeMicroseconds(pulse_width);
}

// int ii=0;

//程序初始化
void setup() {
setupMPU();
setupSerial();
servo.attach(SERVO_PIN);
}

//主程序循环
void loop() {
readMPUData();
Serial.println(pulse_width);
readSerialInput();
calculatePID();
controlServo();
delay(PID_INTERVAL);
}
