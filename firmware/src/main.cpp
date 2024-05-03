/*
 * Copyright 2023 spezifisch <spezifisch23@proton.me> https://github.com/spezifisch
 *
 * This file is part of T12PenSolder.
 *
 * T12PenSolder is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, version 3 of the
 * License.
 *
 * T12PenSolder is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with Foobar. If not, see
 * <https://www.gnu.org/licenses/>.
 */

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

    // V7 needs different display settings, see https://github.com/spezifisch/T12PenSolder/issues/1
#if PENSOLDER_V == 7
    u8g2.sendF("ca", 0x0a8, 0x02f);
#endif

    // splash screen
    u8g2.setFont(u8g2_font_5x8_mr);
    u8g2.firstPage();
    do
    {
        u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 2, u8g2.getDisplayHeight() / 2, U8G2_BTN_HCENTER | U8G2_BTN_BW1, 0, 2, 2, "PEN SOLDER V3 1.0");
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
    static constexpr uint32_t BUTTON_TEMPERATURE_STEP_K = 10; // temperature increment on button press
    static constexpr uint32_t BUTTON_DEBOUNCE_COUNT = 2;      // button state must be stable for this many display updates
    static Debounce<BUTTON_DEBOUNCE_COUNT> buttonSetDebounce, buttonMinusDebounce, buttonPlusDebounce;

    // runtime settings
    static bool heatOn = false;                                          // soldering tip enabled
    static uint32_t selectedTemperature_degC = DEFAULT_TEMPERATURE_degC; // target temp., change with -/+
    static uint32_t idle_time_ms = 0;

    // 10 Hz UI update
    const uint32_t now = millis();
    if (now - last_millis < UPDATE_PERIOD_ms)
    {
        return;
    }
    // update idle time counter
    if (heatOn && idle_time_ms < DEFAULT_STANDBY_TIME_ms)
    {
        idle_time_ms += now - last_millis;
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
        selectedTemperature_degC -= BUTTON_TEMPERATURE_STEP_K;
        if (selectedTemperature_degC < SolderingTip::MIN_TARGET_TEMPERATURE_degC)
        {
            selectedTemperature_degC = SolderingTip::MIN_TARGET_TEMPERATURE_degC;
        }
        if (heatOn)
        {
            solderingTip.setTargetTemperature(selectedTemperature_degC);
        }

        buttonMinusDebounce.reset(); // repeat when holding button
    }

    if (buttonPlus)
    {
        selectedTemperature_degC += BUTTON_TEMPERATURE_STEP_K;
        if (selectedTemperature_degC > SolderingTip::MAX_TARGET_TEMPERATURE_degC)
        {
            selectedTemperature_degC = SolderingTip::MAX_TARGET_TEMPERATURE_degC;
        }
        if (heatOn)
        {
            solderingTip.setTargetTemperature(selectedTemperature_degC);
        }

        buttonPlusDebounce.reset(); // repeat
    }

    // reset idle time on any button press
    if (buttonSet || buttonMinus || buttonPlus)
    {
        idle_time_ms = 0;
    }

    // turn off tip after standby counter elapsed
    const bool inStandby = idle_time_ms >= DEFAULT_STANDBY_TIME_ms;
    if (inStandby)
    {
        solderingTip.setTargetTemperature(0);
        solderingTip.safeMode(); // for good measure
        heatOn = false;
    }

    // get tip state
    const uint32_t vin_raw = solderingTip.getVinRaw();
    const uint32_t vin_mv_dec = solderingTip.getVinmV() % 1000;
    const uint32_t vin_v = solderingTip.getVinmV() / 1000;

    const uint32_t tt_raw = solderingTip.getTipTempRaw();
    const uint32_t tt_uv = solderingTip.getTipTempuV();
    const int32_t tt_degC = solderingTip.getTipTempDegC();
    const int32_t tt_missing = solderingTip.getTipMissing();

    const int32_t t_pwm = solderingTip.getPWM();
    const int32_t t_output_w = solderingTip.getOutputW();

    // UI info
    const uint32_t timestamp = (now % 10000000) / 100;
    const uint32_t showtime_ms = last_showtime / 1000;
    uint32_t standby_counter_s = 0;
    if (DEFAULT_STANDBY_TIME_ms > idle_time_ms)
    {
        standby_counter_s = (DEFAULT_STANDBY_TIME_ms - idle_time_ms) / 1000;
    }

    // display output
    u8g2.firstPage();
    do
    {
        // small...
        u8g2.setFont(u8g2_font_5x8_mr);

        // first row
        u8g2.setCursor(4, 6);
        snprintf(tmp, sizeof(tmp), "     A%04d T%05dL%2d %04d", vin_raw, timestamp, showtime_ms, tt_raw);
        u8g2.print(tmp);

        // 2nd row
        u8g2.setCursor(4, 15);
        snprintf(tmp, sizeof(tmp), "     .%03dV D%02dS%02d %5duV", vin_mv_dec, t_pwm, standby_counter_s, tt_uv);
        u8g2.print(tmp);

        // 3rd row
        u8g2.setCursor(45, 24);
        u8g2.print("Target");

        // 4th row
        u8g2.setCursor(45, 32);
        snprintf(tmp, sizeof(tmp), "%3dC", selectedTemperature_degC);
        u8g2.print(tmp);

        // big...
        u8g2.setFont(u8g2_font_spleen12x24_mf);

        // Vin: xx(V)
        u8g2.setCursor(4, 15);
        snprintf(tmp, sizeof(tmp), "%2d", vin_v);
        u8g2.print(tmp);

        // power: xxW/SBY/OFF
        u8g2.setCursor(4, 32);
        if (heatOn)
        {
            snprintf(tmp, sizeof(tmp), "%2dW", t_output_w);
            u8g2.print(tmp);
        }
        else if (inStandby)
        {
            u8g2.print("SBY");
        }
        else
        {
            u8g2.print("OFF");
        }

        // tip: xxxC/nTIP
        u8g2.setCursor(80, 32);
        if (tt_missing)
        {
            u8g2.print("nTIP");
        }
        else
        {
            snprintf(tmp, sizeof(tmp), "%3dC", tt_degC);
            u8g2.print(tmp);
        }
    } while (u8g2.nextPage());

    last_showtime = micros() - start;
}
