#include <Wire.h> //LCD
#include <LiquidCrystal_I2C.h> //LCD
#include <ros.h> //rosserial
#include <std_msgs/Int16.h> //rosserial
#include <std_msgs/String.h> //rosserial
#include <Servo.h>

void updateTaskNumber(std_msgs::Int16& data);
void updateServoAngle(std_msgs::Int16& data);
void updateDisplayNumber(std_msgs::Int16& data);
void updateWheelSpeed(std_msgs::Int16& data);

ros::NodeHandle nh;


//Topic to receive task number
ros::Subscriber<std_msgs::Int16> task("taskNumber", &updateTaskNumber );
int currentTask = 0;

// Task 1 : Numbers from 0 to 9
ros::Subscriber<std_msgs::Int16> lcd_message_number("lcd_msg", &updateDisplayNumber);
int displayNumber = 0;

// Task 2 :
ros::Subscriber<std_msgs::Int16> servo_angle("servo_angle", &updateServoAngle);
Servo myservo;
int servoAngle = 0;

// Task 3 :
std_msgs::Int16 distance_msg;
ros::Publisher sensor_distance("distance", &distance_msg);
int trigPin = 7;
int echoPin = 8;
float duration, distance;

//Task 4 :
ros::Subscriber<std_msgs::Int16> wheelSpeed("wheel_speed", &updateWheelSpeed);
bool motor_running = false;
int motor_pin = 6;
int pwm_pulse = 150;
int set_speed = 2;
float current_speed = 0;
float error_speed = 0;
float error_speed_previous = 0;
float error_speed_sum = 0;
int kp = 1;
int ki = 1;
int kd = 1;


//LCD screen instance
LiquidCrystal_I2C LCD(0x27, 16, 2);

//True when device is used in a task
boolean deviceBusy  = false;

//IR sensor
int IRSensor = 3;
int rotations = 0;
int rotation_previous = 0;

//DC motor
int motor2pin1 = 4;
int motor2pin2 = 5;
int now_time = 0;
int start_time = 0;

void setup() {
  Serial.begin(57600);
  //Initializing LCS\D
  LCD.init();
  LCD.backlight();

  //Initializing Servo
  myservo.attach(9);

  pinMode(13, OUTPUT);

  //Ultrasonic Sensor Pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(IRSensor), button_ISR, RISING);

  //Initializing ROS Node Handle
  nh.initNode();
  nh.subscribe(task);
  nh.subscribe(lcd_message_number);
  nh.subscribe(servo_angle);
  nh.subscribe(wheelSpeed);
  nh.advertise(sensor_distance);

  //Standby LCD message
  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("Device Ready");
  LCD.setCursor(0, 1);
  LCD.print("Waiting for Task");
  delay(250);
}

void loop(){

nh.spinOnce();
delay(50);
}


void deviceReadyMessage() {
  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("Device Ready");
  LCD.setCursor(0, 1);
  LCD.print("Waiting for Task");
  delay(500);
}

void updateTaskNumber(std_msgs::Int16& data) {
  currentTask = data.data;
  if (currentTask == 3) {
    ultrasonicDistance();
  }

}

void updateServoAngle(std_msgs::Int16& data) {
  servoAngle = data.data;
  myservo.write(servoAngle);
  delay(15);
}

void updateDisplayNumber(std_msgs::Int16& data) {
  displayNumber = data.data;
  receivingData();
}

void updateWheelSpeed(std_msgs::Int16& data) {
  //set_speed = data.data;
  motor_running = true;
  DCMotor();
}

void button_ISR() {
  rotations+=1;
}
void DCMotor() {
  analogWrite(motor_pin, pwm_pulse); //ENB pin
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  now_time = millis();
  if (now_time - start_time > 1000) {
    //pid();
    start_time = now_time;
  }
  current_speed = rotations - rotation_previous;
  rotation_previous = rotations;
}

void ultrasonicDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration * .0343) / 2;
  distance_msg.data = distance;
  sensor_distance.publish( &distance_msg );

  delay(100);
}

void receivingData() {

  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("Topic: /lcd_msg");
  LCD.setCursor(0, 1);
  LCD.print("Received: ");
  LCD.setCursor(11, 1);
  LCD.print(displayNumber);
  delay(50);

}


void pid() {

  if (motor_running) {
    error_speed = set_speed - current_speed;

    pwm_pulse = (error_speed * kp) + (error_speed_sum * ki) + ((error_speed - error_speed_previous) * kd);

    error_speed_previous = error_speed;  //save last (previous) error
    error_speed_sum += error_speed; //sum of error
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
    //Setting maximum and minimum values.
    if (pwm_pulse > 255) {
      analogWrite(motor_pin, 255);
    }
    else {
      analogWrite(motor_pin, 70);
    }
  }
}
