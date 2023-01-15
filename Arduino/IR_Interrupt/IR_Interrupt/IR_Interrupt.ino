const int ledPin = 12;
int IRSensor = 3;
unsigned long interupt_count = 0;
int rotations = 0;

int motor2pin1 = 4;
int motor2pin2 = 5;

unsigned long start_time = 0;
unsigned long now_time = 0;

int rotation_previous = 0;
int wheel_speed = 0;

boolean motor_running = true;
int motor_pin = 9;
float pwm_pulse = 70.0;
float pwm_pulse_correction = 0.0;
float set_speed = 16.0;
float current_speed = 0;
float error_speed = 0;
float error_speed_previous = 0;
float error_speed_sum = 0;
float kp = 0.6;
float ki = 0.055;
float kd = 0.0;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(IRSensor, INPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(IRSensor), button_ISR, RISING);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  pinMode(9, OUTPUT);//PWM for motor
}

void loop() {
  analogWrite(motor_pin, pwm_pulse); //ENB pin
  //Serial.println(pwm_pulse); //ENB pin
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  now_time = millis();
  if (now_time - start_time > 1000) {
    //wheel_speed = rotations - rotation_previous;
    //Serial.println(wheel_speed);
    //rotation_previous = rotations;
    pid();
    start_time = now_time;
  }
  current_speed = rotations - rotation_previous;
  rotation_previous = rotations;
  //Serial.print("current Speed : ");
  //Serial.println(current_speed); 
}

void button_ISR() {
  rotations+=1;
  //interupt_count += 1;
  //if (interupt_count % 2 == 0) {
  //  rotations += 1;
  //}
}
void pid() {

  if (motor_running) {
    error_speed = set_speed - current_speed;
    
    pwm_pulse_correction = (error_speed * kp) + (error_speed_sum * ki) + ((error_speed - error_speed_previous) * kd);
    //Serial.println(error_speed);
    Serial.println(pwm_pulse_correction);

    pwm_pulse = pwm_pulse+pwm_pulse_correction;
    //Serial.println(pwm_pulse);
    error_speed_previous = error_speed;  //save last (previous) error
    error_speed_sum += error_speed; //sum of error
    //if (error_speed_sum > 4000) error_speed_sum = 4000;
    //if (error_speed_sum < -4000) error_speed_sum = -4000;
  }
  else {
    error_speed = 0;
    error_speed_previous = 0;
    error_speed_sum = 0;
    pwm_pulse = 0;
  }

  //Send new pwm singnal. Lowest 70 as motor stalls below 70
  /*if (pwm_pulse <255 & pwm_pulse >70) {
    analogWrite(motor_pin, pwm_pulse);
  }
  else {
    if (pwm_pulse > 255) {
      analogWrite(motor_pin, 255);
    }
    else {
      analogWrite(motor_pin, 70);
    }
  }*/
}
