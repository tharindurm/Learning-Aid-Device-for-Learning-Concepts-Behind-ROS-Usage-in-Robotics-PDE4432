#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

unsigned long currentTime = 0;
unsigned long startTime = 0;

boolean m1 = false;
boolean m2 = false;
boolean m3 = false;
boolean m4 = false;

void setup()
{
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
}

void receivingDataAnimation() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Receiving");
  delay(250);

  lcd.setCursor(0, 1);
  lcd.print(">>>");
  delay(250);

  lcd.setCursor(0, 1);
  lcd.print(">>> >>>");
  delay(250);

  lcd.setCursor(0, 1);
  lcd.print(">>> >>> >>>");
  delay(250);

  lcd.setCursor(0, 1);
  lcd.print(">>> >>> >>> >>>");
  delay(250);

}
void loop()
{
  Serial.println("Got msg");
  receivingDataAnimation();
  Serial.println("Loop");
}
