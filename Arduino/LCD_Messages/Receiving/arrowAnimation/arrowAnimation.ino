#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

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

void sendingDataAnimation() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("         Sending");
  delay(250);

  lcd.setCursor(0, 1);
  lcd.print("            <<<");
  delay(250);

  lcd.setCursor(0, 1);
  lcd.print("        <<< <<<");
  delay(250);

  lcd.setCursor(0, 1);
  lcd.print("    <<< <<< <<<");
  delay(250);

  lcd.setCursor(0, 1);
  lcd.print("<<< <<< <<< <<<");
  delay(250);
}

void loop()
{
  Serial.println("Got msg");
  receivingDataAnimation();
  delay(500);
  sendingDataAnimation();
  delay(500);
  Serial.println("Loop");
}
