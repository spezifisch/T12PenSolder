#include <Arduino.h>
#include <BitBang_I2C.h>

#include "cube_init.h"
#include "SSD1306Ascii.h"
#include "SSD1306AsciiBBI2c.h"

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

BBI2C bbi2c;
SSD1306AsciiBBI2c oled;

void setup()
{
  MX_GPIO_Init();
  MX_ADC_Init();

  analogReadResolution(12);

  // turn heat off
  digitalWrite(TIPHEAT_DRV, 0);

  // display init
  memset(&bbi2c, 0, sizeof(bbi2c));
  bbi2c.bWire = false;
  bbi2c.iSDA = SDA_PIN;
  bbi2c.iSCL = SCL_PIN;
  I2CInit(&bbi2c, 1000000L);

  if (true)
  {
    // stock firmware does what looks like a display reset, but it isn't connected
    digitalWrite(PA9, 0);
    delay(200);
    digitalWrite(PA9, 1);
  } else {
    // for some reason the display needs a bit of delay to turn of reliably
    delay(200);
  }

  oled.begin(&SolderingIron128x64, &bbi2c, 0x3c);
}

void loop()
{
  oled.setFont(Adafruit5x7);

  uint32_t m = micros();

  oled.clear();

  // first row
  if (digitalRead(BUT_SET)) {
    oled.print("(SET)");
  } else {
    oled.print(" SET ");
  }
  if (digitalRead(BUT_MINUS)) {
    oled.print("(-)");
  } else {
    oled.print(" - ");
  }
  if (digitalRead(BUT_PLUS)) {
    oled.print("(+)");
  } else {
    oled.print(" + ");
  }
  oled.println();
  
  // second row
  uint32_t vin = analogRead(VIN_MEAS);
  uint32_t tiptemp = analogRead(TIPTEMP_MEAS);
  uint32_t vin_mv = vin * 3300 / 4096 * 110000 / 10000; // 3.3V ADC ref., 12 bit ADC, compensate for 1/11 voltage divider
  uint32_t tt_uv = tiptemp * 3300 / 4096 * 1000 / 221; // ... to uV, 221x gain from OpAmp
  oled.printf("vin: %04d %7d mV\n", vin, vin_mv);
  oled.printf("tip: %04d %7d uV\n", tiptemp, tt_uv);

  // third row
  oled.print("micros: ");
  oled.print(micros() - m);

  delay(1000);
}
