#include <Wire.h> //LCD
#include <LiquidCrystal_I2C.h> //LCD

//LCD screen instance
LiquidCrystal_I2C LCD(0x27, 16, 2);

//True when device is used in a task
boolean deviceBusy  = false;

//This variable holds the task number to be executed. 0 is no tasks
int currentTask = 0;

//Wheel PID controller variables
bool motor_running = false;
int motor_pin = 9;
int pwm_pulse = 0;
float set_speed = 0;
float current_speed = 0;
float error_speed = 0;
float error_speed_previous = 0;
float error_speed_sum = 0;
int kp = 1;
int ki = 1;
int kd = 1;



void setup() {
  Serial.begin(9600);
  LCD.init();
  LCD.backlight();

  //Standby LCD message
  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("Device Ready");
  LCD.setCursor(0, 1);
  LCD.print("Waiting for Task");
  delay(250);
}

void loop() {
  if (!deviceBusy) {
    deviceReadyMessage();
  }

  if (currentTask == 1) {

  }

  if (currentTask == 2) {

  }

  if (currentTask == 3) {

  }

  if (currentTask == 4) {

  }

}

void deviceReadyMessage() {
  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("Device Ready");
  LCD.setCursor(0, 1);
  LCD.print("Waiting for Task");
  delay(500);
}

void pid() {

  if (motor_running) {
    error_speed = set_speed - current_speed;

    pwm_pulse = (error_speed * kp) + (error_speed_sum * ki) + ((error_speed - error_speed_previous) * kd);

    error_speed_previous = error_speed;  //save last (previous) error
    error_speed_sum += error_speed; //sum of error
    if (error_speed_sum > 4000) error_speed_sum = 4000;
    if (error_speed_sum < -4000) error_speed_sum = -4000;
  }
  else {
    error_speed = 0;
    error_speed_previous = 0;
    error_speed_sum = 0;
    pwm_pulse = 0;
  }

  //Send new pwm singnal. Lowest 70 as motor stalls below 70
  if (pwm_pulse <255 & pwm_pulse >70) {
    analogWrite(motor_pin, pwm_pulse);
  }
  else {
    if (pwm_pulse > 255) {
      analogWrite(motor_pin, 255);
    }
    else {
      analogWrite(motor_pin, 70);
    }
  }
}
