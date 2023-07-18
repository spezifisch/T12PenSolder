/*
 * Copyright 2023 spezifisch <spezifisch23@proton.me> https://github.com/spezifisch
 * Copyright 2020 Ralim, IronOS https://github.com/Ralim/IronOS
 *
 * Partly based on IronOS v2.21, GPLv3: https://github.com/Ralim/IronOS/blob/v2.21/LICENSE
 * Original source files:
 * - https://github.com/Ralim/IronOS/blob/v2.21/source/Core/Threads/PIDThread.cpp
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

#include "config.h"
#include "ironos_hakko.h"
#include "ironos_pid.h"

#include "solderingtip.h"

SolderingTip solderingTip;

static void Update_IT_callback(void)
{
    // Update event correspond to Rising edge of PWM when configured in PWM1 mode
    // timer reset
    solderingTip.pwmUpdateCallback();
}

static void Switch_callback(void)
{
    // Compare match event correspond to falling edge of PWM when configured in PWM1 mode
    // timer channel 1 OC
    solderingTip.pwmSwitchCallback();
}

static void Measure_callback(void)
{
    // timer channel 2 OC
    solderingTip.pwmMeasureCallback();
}

static void Controller_callback(void)
{
    // timer channel 3 OC
    solderingTip.pwmPIDControllerCallback();
}

void SolderingTip::safeMode()
{
    tipOff();
    targetTemperature_degC = 0;
    output_W = 0;
    pwmDuty_percent = 0;
    tipOff();
}

void SolderingTip::setup()
{
    safeMode();

    // configure PWM heater output
    TipHeatTimer = new HardwareTimer(TIM3);
    TipHeatTimer->setOverflow(100000, MICROSEC_FORMAT); // reset every 100 ms, 10 Hz

    // our 3 timer channels
    TipHeatTimer->setMode(THTSwitchChannel, TIMER_DISABLED, -1);     // on compare cb: switch output off
    TipHeatTimer->setMode(THTMeasureChannel, TIMER_DISABLED, -1);    // on cb: measure tip temperature
    TipHeatTimer->setMode(THTControllerChannel, TIMER_DISABLED, -1); // on cb: update PID controller
    TipHeatTimer->setCaptureCompare(THTSwitchChannel, 0, PERCENT_COMPARE_FORMAT);
    TipHeatTimer->setCaptureCompare(THTMeasureChannel, 10, PERCENT_COMPARE_FORMAT);    // switch% + 10%
    TipHeatTimer->setCaptureCompare(THTControllerChannel, 95, PERCENT_COMPARE_FORMAT); // always after 95ms (5ms should be more than enough time for PID update)

    TipHeatTimer->attachInterrupt(Update_IT_callback);
    TipHeatTimer->attachInterrupt(THTSwitchChannel, Switch_callback);
    TipHeatTimer->attachInterrupt(THTMeasureChannel, Measure_callback);
    TipHeatTimer->attachInterrupt(THTControllerChannel, Controller_callback);

    TipHeatTimer->resume();
}

void SolderingTip::setTargetTemperature(uint32_t target_degC)
{
    if (target_degC > MAX_TARGET_TEMPERATURE_degC)
    {
        target_degC = MAX_TARGET_TEMPERATURE_degC;
    }
    else if (target_degC < MIN_TARGET_TEMPERATURE_degC)
    {
        target_degC = 0;
    }

    targetTemperature_degC = target_degC;
}

// call from timer ISR with 10 Hz
void SolderingTip::pwmUpdateCallback()
{
    const bool turnedOn = targetTemperature_degC > 0;
    const bool dutyCycleSanity = pwmDuty_percent >= 0 && pwmDuty_percent <= MAX_DUTY_PERCENT;

    if (turnedOn && dutyCycleSanity && !tipTemp_adcLimitReached)
    {
        pidRunning = true;
        if (pwmDuty_percent > 0)
        {
            tipOn();
        }
        TipHeatTimer->setCaptureCompare(THTSwitchChannel, pwmDuty_percent, PERCENT_COMPARE_FORMAT);
        TipHeatTimer->setCaptureCompare(THTMeasureChannel, pwmDuty_percent + 10, PERCENT_COMPARE_FORMAT);
    }
    else
    {
        pidRunning = false;
        pwmDuty_percent = 0;
        output_W = 0;
        tipOff();
    }
}

void SolderingTip::pwmSwitchCallback()
{
    tipOff();
}

void SolderingTip::pwmMeasureCallback()
{
    // measure Vin and tip temperature
    vin_raw = constrain(analogRead(VIN_MEAS), 0, 4095);
    tipTemp_raw = constrain(analogRead(TIPTEMP_MEAS), 0, 4095);

    // convert to V
    vin_mV = constrain(vin_raw * 3300 / 4095 * 110000 / 10000, 0, 30000);     // 3.3V ADC ref., 12 bit ADC, compensate for 1/11 voltage divider
    tipTemp_uV = constrain(tipTemp_raw * 3300 / 4095 * 1000 / 221, 0, 15000); // ... to uV, 221x gain from OpAmp

    // convert to °C
    tipTemp_degC = IronOS::convertuVToDegC(tipTemp_uV); // IronOS T12 curve

    // tipTemp_degC = (tipTemp_raw * 204 - 73840) / 1000; // custom curve from manual measurements and linear regression, pretty similar above 250°C

    // IronOS: Secondary safety check to forcefully disable header when within ADC noise of top of ADC
    tipTemp_adcLimitReached = tipTemp_raw >= 4095 - 32;
}

void SolderingTip::pwmPIDControllerCallback()
{
    // PID controller based on IronOS' PIDThread
    const uint32_t now = millis();

    const uint32_t deltaT_ms = now - lastPIDTime;
    if ((deltaT_ms == 0 || deltaT_ms > 500) && !init)
    {
        // something's hanging
        targetTemperature_degC = 0;
        return;
    }
    const uint32_t rate_Hz = 1000 / deltaT_ms;
    lastPIDTime = now;

    if (!pidRunning || init)
    {
        // disabled by UI or first call
        init = false;
        return;
    }

    // PID input
    int32_t error_K = targetTemperature_degC - tipTemp_degC;

    // calculate PID output
    int32_t outputX10Watts = powerStore.update(TIP_THERMAL_MASS * error_K, // the required power
                                               TIP_THERMAL_MASS,           // Inertia, smaller numbers increase dominance of the previous value
                                               2,                          // gain
                                               rate_Hz,                    // PID cycle frequency
                                               HARDWARE_MAX_WATTAGE_X10);

    // calculate PWM output
    if (outputX10Watts > 0)
    {
        // how much power is available?
        const uint32_t vinX10 = getVinmV() / 100;
        uint32_t availableWattsX10 = (vinX10 * vinX10) / TIP_RESISTANCE_X10OHM;
        // apply duty cycle limit to get max. average power
        availableWattsX10 = availableWattsX10 * MAX_DUTY_PERCENT;
        availableWattsX10 /= 100;

        // calculate duty cycle
        uint32_t newDuty = (MAX_DUTY_PERCENT * outputX10Watts) / availableWattsX10;
        // sanity check: don't exceed it because we need time for measurement
        newDuty = constrain(newDuty, 0, MAX_DUTY_PERCENT);
        // this is our output. it gets applied in the next pwmUpdateCallback() call.
        pwmDuty_percent = newDuty;

        // for display
        output_W = (pwmDuty_percent * availableWattsX10) / MAX_DUTY_PERCENT / 10;
    }
    else
    {
        // we're too hot
        pwmDuty_percent = 0;
        output_W = 0;
    }
}

void SolderingTip::tipOn()
{
    HAL_GPIO_WritePin(TIPHEAT_GPIO, TIPHEAT_PIN_MASK, GPIO_PIN_SET);
}

void SolderingTip::tipOff()
{
    HAL_GPIO_WritePin(TIPHEAT_GPIO, TIPHEAT_PIN_MASK, GPIO_PIN_RESET);
}
