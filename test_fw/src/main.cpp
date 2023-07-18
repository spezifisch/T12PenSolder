#include <Arduino.h>
#include <U8g2lib.h>

#include "config.h"
#include "cube_init.h"
#include "debounce.h"
#include "solderingtip.h"

// display
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, /* clock=*/SCL_PIN, /* data=*/SDA_PIN, /* reset=*/U8X8_PIN_NONE);

void setup()
{
    // lowlevel stm32cube setup
    MX_GPIO_Init();

    // turn heat off
    digitalWrite(TIPHEAT_DRV, 0);
    solderingTip.safeMode();

    // stock firmware does what looks like a display reset, but it isn't connected on my board
    // digitalWrite(PA9, 0);
    // delay(200);
    // digitalWrite(PA9, 1);
    delay(500); // keep some delay to be sure that display is up

    // adc setup
    pinMode(VIN_MEAS, INPUT_ANALOG);
    pinMode(TIPTEMP_MEAS, INPUT_ANALOG);
    analogReadResolution(12);
    // set sampling time to same as stock firmware
    MX_ADC_Config();

    // PWM setup
    solderingTip.setup();

    // display setup
    u8g2.begin();

    // splash screen
    u8g2.setFont(u8g2_font_5x8_mr);
    u8g2.firstPage();
    do
    {
        u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 2, u8g2.getDisplayHeight() / 2, U8G2_BTN_HCENTER | U8G2_BTN_BW1, 0, 2, 2, "PEN SOLDER V3");
        u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 2, 28, U8G2_BTN_HCENTER, 0, 0, 0, "github.com/spezifisch");
    } while (u8g2.nextPage());
    delay(1000);
}

void loop(void)
{
    static char tmp[64];

    // timekeeping
    static constexpr uint32_t UPDATE_PERIOD_ms = 100;                               // display update period
    static uint32_t last_millis = UPDATE_PERIOD_ms * (millis() / UPDATE_PERIOD_ms); // last display update
    static uint32_t last_showtime = 0;                                              // last display buffer transfer duration

    // buttons
    static constexpr uint32_t BUTTON_DEBOUNCE_COUNT = 2; // button state must be stable for this many display updates
    static Debounce<BUTTON_DEBOUNCE_COUNT> buttonSetDebounce, buttonMinusDebounce, buttonPlusDebounce;

    // runtime settings
    static bool heatOn = false;                                          // soldering tip enabled
    static uint32_t selectedTemperature_degC = DEFAULT_TEMPERATURE_degC; // target temp., change with -/+

    // 10 Hz UI update
    const uint32_t now = millis();
    if (now - last_millis < UPDATE_PERIOD_ms)
    {
        return;
    }
    last_millis = now;

    // loop benchmark
    const uint32_t start = micros();

    // read and handle buttons
    const bool buttonSet = buttonSetDebounce.measure(digitalRead(BUT_SET));
    const bool buttonMinus = buttonMinusDebounce.measure(digitalRead(BUT_MINUS));
    const bool buttonPlus = buttonPlusDebounce.measure(digitalRead(BUT_PLUS));

    if (buttonSet)
    {
        heatOn = !heatOn;
        solderingTip.setTargetTemperature(heatOn ? selectedTemperature_degC : 0);
    }

    if (buttonMinus)
    {
        selectedTemperature_degC -= 10;
        if (selectedTemperature_degC < 150)
        {
            selectedTemperature_degC = 150;
        }

        solderingTip.setTargetTemperature(heatOn ? selectedTemperature_degC : 0);
        buttonMinusDebounce.reset(); // repeat when holding button
    }

    if (buttonPlus)
    {
        selectedTemperature_degC += 10;
        if (selectedTemperature_degC > 450)
        {
            selectedTemperature_degC = 450;
        }

        solderingTip.setTargetTemperature(heatOn ? selectedTemperature_degC : 0);
        buttonPlusDebounce.reset(); // repeat
    }

    // get tip state
    const uint32_t vin_raw = solderingTip.getVinRaw();
    const uint32_t vin_mv = solderingTip.getVinmV();

    const uint32_t tt_raw = solderingTip.getTipTempRaw();
    const uint32_t tt_uv = solderingTip.getTipTempuV();
    const int32_t tt_degC = solderingTip.getTipTempDegC();

    const int32_t t_pwm = solderingTip.getPWM();
    const int32_t t_outputWatts = solderingTip.getOutputWatts();

    // display output
    u8g2.firstPage();
    do
    {
        u8g2.setFont(u8g2_font_5x8_mr);

        // first row
        u8g2.setCursor(4, 8);
        snprintf(tmp, sizeof(tmp), "Vin:%04d %5dmV T%07d", vin_raw, vin_mv, now % 10000000);
        u8g2.print(tmp);

        u8g2.setCursor(4, 16);
        snprintf(tmp, sizeof(tmp), "Tip:%04d %5duV D%2d%%L%2d", tt_raw, tt_uv, t_pwm, last_showtime / 1000);
        u8g2.print(tmp);

        // 3rd row
        u8g2.setCursor(45, 24);
        u8g2.print("Target");

        // 4th row
        u8g2.setCursor(45, 32);
        snprintf(tmp, sizeof(tmp), "%3dC", selectedTemperature_degC);
        u8g2.print(tmp);

        // big
        u8g2.setCursor(4, 32);
        u8g2.setFont(u8g2_font_spleen12x24_mf);
        if (heatOn)
        {
            snprintf(tmp, sizeof(tmp), "%2dW", t_outputWatts);
            u8g2.print(tmp);
        }
        else
        {
            u8g2.print("OFF");
        }

        //
        u8g2.setCursor(80, 32);
        snprintf(tmp, sizeof(tmp), "%3dC", tt_degC);
        u8g2.print(tmp);
    } while (u8g2.nextPage());

    last_showtime = micros() - start;
}
