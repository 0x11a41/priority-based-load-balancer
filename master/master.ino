#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "driver/gpio.h" // Ensures GPIO definitions are explicitly loaded

#define SDA_PIN 21
#define SCL_PIN 19
#define I2C_ADDR 0x27

#define BTN1_PIN 4
#define BTN2_PIN 5
#define BTN3_PIN 18
#define BUZZER_PIN 13

#define PWM_CHANNEL 0
#define PWM_FREQ    2700
#define PWM_RES     8

LiquidCrystal_I2C lcd(I2C_ADDR, 16, 2);

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  lcd.init();
  lcd.backlight();

  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(BTN3_PIN, INPUT_PULLUP);

  // Correct drive capability constant for ESP32
  pinMode(BUZZER_PIN, OUTPUT);
  gpio_set_drive_capability((gpio_num_t)BUZZER_PIN, GPIO_DRIVE_CAP_3);

  // Set up ESP32 hardware PWM
  ledcAttachChannel(BUZZER_PIN, PWM_FREQ, PWM_RES, PWM_CHANNEL);

  lcd.setCursor(0, 0);
  lcd.print("Buzzer Pin Test ");
}

void loop() {
  if (digitalRead(BTN3_PIN) == LOW) {
    lcd.setCursor(0, 1);
    lcd.print("BUZZER ACTIVE!  ");
    ledcWrite(BUZZER_PIN, 128); 
  } else {
    ledcWrite(BUZZER_PIN, 0); 
    
    if (digitalRead(BTN1_PIN) == LOW) {
      lcd.setCursor(0, 1);
      lcd.print("Button 1 Pressed");
    } else if (digitalRead(BTN2_PIN) == LOW) {
      lcd.setCursor(0, 1);
      lcd.print("Button 2 Pressed");
    }
  }

  delay(50);
}