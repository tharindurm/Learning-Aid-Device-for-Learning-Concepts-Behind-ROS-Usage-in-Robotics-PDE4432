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
  analogWrite(10, 0); //ENB pin
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  now_time = millis();
  if (now_time - start_time > 1000) {
    wheel_speed = rotations - rotation_previous;
    Serial.println(wheel_speed);
    rotation_previous = rotations;
    start_time = now_time;
  }

}

void button_ISR() {
  interupt_count += 1;
  if (interupt_count % 4 == 0) {
    rotations += 1;
  }
}
