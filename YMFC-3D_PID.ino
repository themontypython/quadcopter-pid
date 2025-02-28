#include <Wire.h>
#include <Servo.h>

// ESC objects
Servo esc1, esc2, esc3, esc4;

// Gyro variables
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
int cal_int;
int16_t x, y, z;
float roll_rate, pitch_rate, yaw_rate;

// PID variables
float pid_roll_setpoint = 0;
float pid_pitch_setpoint = 0;
float pid_yaw_setpoint = 0;

float pid_p_gain_roll = 1.3;    // Increase for more response
float pid_i_gain_roll = 0.04;
float pid_d_gain_roll = 18.0;

float pid_p_gain_pitch = pid_p_gain_roll;
float pid_i_gain_pitch = pid_i_gain_roll;
float pid_d_gain_pitch = pid_d_gain_roll;

float pid_p_gain_yaw = 4.0;
float pid_i_gain_yaw = 0.02;
float pid_d_gain_yaw = 0.0;

// Receiver variables
volatile unsigned long timer_3;
volatile int receiver_input_channel_3;  // Throttle
volatile unsigned long current_time;

// PID calculations
float pid_error_temp;
float pid_i_mem_roll, pid_roll_output, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_output_yaw, pid_last_yaw_d_error;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // Initialize ESCs
  esc1.attach(4);
  esc2.attach(5);
  esc3.attach(6);
  esc4.attach(7);
  
  // Set ESCs to minimum throttle
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  
  // Throttle pin
  PCICR |= (1 << PCIE0);    // Set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT2);  // Pin 10 (throttle)
  
  // Initialize gyro
  Wire.beginTransmission(0x6B);
  Wire.write(0x20);
  Wire.write(0x0F);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x6B);
  Wire.write(0x23);
  Wire.write(0x30);
  Wire.endTransmission();
  
  Serial.println("Calibrating gyro...");
  for(cal_int = 0; cal_int < 2000; cal_int++) {
    read_gyro_values();
    gyro_x_cal += x;
    gyro_y_cal += y;
    gyro_z_cal += z;
    if(cal_int % 100 == 0) Serial.print(".");
    delay(3);
  }
  
  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;
  
  Serial.println("\nCalibration done!");
}

void read_gyro_values() {
  Wire.beginTransmission(0x6B);
  Wire.write(0x28|0x80);
  Wire.endTransmission(false);
  Wire.requestFrom(0x6B, 6);
  
  while(Wire.available() < 6);
  
  y = Wire.read() | (Wire.read() << 8);
  x = Wire.read() | (Wire.read() << 8);
  z = Wire.read() | (Wire.read() << 8);
}

ISR(PCINT0_vect){
  current_time = micros();
  // Channel 3 (throttle)
  if(PINB & B00000100){                            // Is input 10 high?
    if(timer_3 == 0) timer_3 = current_time;       // Start timer
  }
  else if(timer_3 != 0){                           // Input 10 is low
    receiver_input_channel_3 = current_time - timer_3; // Calculate time
    timer_3 = 0;                                   // Reset timer
  }
}

void calculate_pid() {
  // Roll calculations
  pid_error_temp = roll_rate;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > 400) pid_i_mem_roll = 400;
  else if(pid_i_mem_roll < -400) pid_i_mem_roll = -400;
  pid_roll_output = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_roll_output > 400) pid_roll_output = 400;
  else if(pid_roll_output < -400) pid_roll_output = -400;
  pid_last_roll_d_error = pid_error_temp;

  // Pitch calculations
  pid_error_temp = pitch_rate;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > 400) pid_i_mem_pitch = 400;
  else if(pid_i_mem_pitch < -400) pid_i_mem_pitch = -400;
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > 400) pid_output_pitch = 400;
  else if(pid_output_pitch < -400) pid_output_pitch = -400;
  pid_last_pitch_d_error = pid_error_temp;

  // Yaw calculations
  pid_error_temp = yaw_rate;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > 400) pid_i_mem_yaw = 400;
  else if(pid_i_mem_yaw < -400) pid_i_mem_yaw = -400;
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > 400) pid_output_yaw = 400;
  else if(pid_output_yaw < -400) pid_output_yaw = -400;
  pid_last_yaw_d_error = pid_error_temp;
}

void loop() {
  // Read gyro
  read_gyro_values();
  
  // Calculate rates
  roll_rate = (x - gyro_x_cal) * 0.07;
  pitch_rate = (y - gyro_y_cal) * 0.07;
  yaw_rate = (z - gyro_z_cal) * 0.07;
  
  // Calculate PID
  calculate_pid();
  
  // Get throttle
  int throttle = receiver_input_channel_3;
  if(throttle > 1800) throttle = 1800;
  
  // Apply PID to motors
  esc1.writeMicroseconds(throttle - pid_roll_output - pid_output_pitch + pid_output_yaw); //FR
  esc2.writeMicroseconds(throttle + pid_roll_output - pid_output_pitch - pid_output_yaw); //FL
  esc3.writeMicroseconds(throttle + pid_roll_output + pid_output_pitch + pid_output_yaw); //BR
  esc4.writeMicroseconds(throttle - pid_roll_output + pid_output_pitch - pid_output_yaw); //BL
  
  // Debug output
  Serial.print("Roll:"); Serial.print(roll_rate);
  Serial.print(" Pitch:"); Serial.print(pitch_rate);
  Serial.print(" Yaw:"); Serial.print(yaw_rate);
  Serial.print(" Throttle:"); Serial.println(throttle);
  
  delay(10);
}

