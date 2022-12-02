#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup()
{
  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
}


void loop()
{

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Receiving");
  delay(500);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Receiving");
  lcd.setCursor(0, 1);
  lcd.print(">>>");
  delay(500);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Receiving");
  lcd.setCursor(0, 1);
  lcd.print(">>> >>>");
  delay(500);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Receiving");
  lcd.setCursor(0, 1);
  lcd.print(">>> >>> >>>");
  delay(500);


  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Receiving");
  lcd.setCursor(0, 1);
  lcd.print(">>> >>> >>> >>>");
  delay(500);
}
