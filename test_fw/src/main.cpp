#include <Arduino.h>
#include <U8g2lib.h>

#include "cube_init.h"

// i2c bitbanged
static constexpr int SDA_PIN = PA7;
static constexpr int SCL_PIN = PA6;
// buttons (high active)
static constexpr int BUT_MINUS = PA4;
static constexpr int BUT_PLUS = PA5;
static constexpr int BUT_SET = PB1;
// heating output
static constexpr int TIPHEAT_DRV = PA3;
// adc inputs
static constexpr int VIN_MEAS = PA1;
static constexpr int TIPTEMP_MEAS = PA2;

U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, /* clock=*/SCL_PIN, /* data=*/SDA_PIN, /* reset=*/U8X8_PIN_NONE);

void setup()
{
  // lowlevel stm32cube setup
  MX_GPIO_Init();

  // adc setup
  pinMode(VIN_MEAS, INPUT_ANALOG);
  pinMode(TIPTEMP_MEAS, INPUT_ANALOG);
  analogReadResolution(12);
  MX_ADC_Config();

  // turn heat off
  digitalWrite(TIPHEAT_DRV, 0);

  // stock firmware does what looks like a display reset, but it isn't connected on my board
  digitalWrite(PA9, 0);
  delay(200);
  digitalWrite(PA9, 1);
  delay(200);

  u8g2.begin();
  u8g2.setFont(u8g2_font_5x8_mr);

  // splash screen
  u8g2.firstPage();
  do
  {
    u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 2, u8g2.getDisplayHeight() / 2, U8G2_BTN_HCENTER | U8G2_BTN_BW1, 0, 2, 2, "PEN SOLDER V3");
  } while (u8g2.nextPage());
  delay(1000);
}

void loop(void)
{
  static uint32_t last_showtime = 0;
  static char tmp[64];

  const uint32_t vin = analogRead(VIN_MEAS);
  const uint32_t tiptemp = analogRead(TIPTEMP_MEAS);

  const uint32_t vin_mv = vin * 3300 / 4096 * 110000 / 10000; // 3.3V ADC ref., 12 bit ADC, compensate for 1/11 voltage divider
  const uint32_t tt_uv = tiptemp * 3300 / 4096 * 1000 / 221;  // ... to uV, 221x gain from OpAmp

  const bool buttonSet = digitalRead(BUT_SET);
  const bool buttonMinus = digitalRead(BUT_MINUS);
  const bool buttonPlus = digitalRead(BUT_PLUS);

  const uint32_t start = micros();
  u8g2.firstPage();
  do
  {
    // first row
    u8g2.setCursor(0, 8);
    if (buttonSet)
    {
      u8g2.print("(SET)");
    }
    else
    {
      u8g2.print(" SET ");
    }
    if (buttonMinus)
    {
      u8g2.print("(-)");
    }
    else
    {
      u8g2.print(" - ");
    }
    if (buttonPlus)
    {
      u8g2.print("(+)");
    }
    else
    {
      u8g2.print(" + ");
    }

    // second row
    u8g2.setCursor(0, 16);
    snprintf(tmp, sizeof(tmp), "vin: %04d %7d mV", vin, vin_mv);
    u8g2.print(tmp);

    // third row
    u8g2.setCursor(0, 24);
    snprintf(tmp, sizeof(tmp), "tip: %04d %7d uV", tiptemp, tt_uv);
    u8g2.print(tmp);

    // 4th row
    u8g2.setCursor(0, 32);
    u8g2.print("micros: ");
    u8g2.print(last_showtime);
  } while (u8g2.nextPage());

  last_showtime = micros() - start;
  delay(100);
}
