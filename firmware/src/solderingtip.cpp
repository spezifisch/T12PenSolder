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
    solderingTip.pwmUpdateCallback();
}

static void Switch_callback(void)
{
    // Compare match event correspond to falling edge of PWM when configured in PWM1 mode
    solderingTip.pwmSwitchCallback();
}

static void Measure_callback(void)
{
    solderingTip.pwmMeasureCallback();
}

static void Controller_callback(void)
{
    solderingTip.pwmPIDControllerCallback();
}

void SolderingTip::safeMode()
{
    tipOff();
    targetTemperature_degC = 0;
    outputWatts = 0;
    pwmDutyPercent = 0;
    tipOff();
}

void SolderingTip::setup()
{
    safeMode();

    // configure PWM heater output
    TipHeatTimer = new HardwareTimer(TIM3);
    TipHeatTimer->setOverflow(100000, MICROSEC_FORMAT);              // 100 ms, 10 Hz
    TipHeatTimer->setMode(THTSwitchChannel, TIMER_DISABLED, -1);     // on compare cb: switch output off
    TipHeatTimer->setMode(THTMeasureChannel, TIMER_DISABLED, -1);    // measure tip temperature
    TipHeatTimer->setMode(THTControllerChannel, TIMER_DISABLED, -1); // update PID controller
    TipHeatTimer->setCaptureCompare(THTSwitchChannel, 0, PERCENT_COMPARE_FORMAT);
    TipHeatTimer->setCaptureCompare(THTMeasureChannel, 10, PERCENT_COMPARE_FORMAT);    // switch% + 10%
    TipHeatTimer->setCaptureCompare(THTControllerChannel, 95, PERCENT_COMPARE_FORMAT); // always after 95ms
    TipHeatTimer->attachInterrupt(Update_IT_callback);
    TipHeatTimer->attachInterrupt(THTSwitchChannel, Switch_callback);
    TipHeatTimer->attachInterrupt(THTMeasureChannel, Measure_callback);
    TipHeatTimer->attachInterrupt(THTControllerChannel, Controller_callback);
    TipHeatTimer->resume();
}

void SolderingTip::setTargetTemperature(uint32_t target_degC)
{
    if (target_degC > 450)
    {
        target_degC = 450;
    }
    else if (target_degC < 150)
    {
        target_degC = 0;
    }

    targetTemperature_degC = target_degC;
}

// call from timer ISR with 10 Hz
void SolderingTip::pwmUpdateCallback()
{
    const bool turnedOn = targetTemperature_degC > 0;
    const bool dutyCycleSanity = pwmDutyPercent >= 0 && pwmDutyPercent <= MAX_DUTY_PERCENT;
    // IronOS: Secondary safety check to forcefully disable header when within ADC noise of top of ADC
    const bool adcLimitReached = getTipTempRaw() > (0x7FFF - 32);

    if (turnedOn && dutyCycleSanity && !adcLimitReached)
    {
        pidRunning = true;
        if (pwmDutyPercent > 0)
        {
            tipOn();
        }
        TipHeatTimer->setCaptureCompare(THTSwitchChannel, pwmDutyPercent, PERCENT_COMPARE_FORMAT);
        TipHeatTimer->setCaptureCompare(THTMeasureChannel, pwmDutyPercent + 10, PERCENT_COMPARE_FORMAT);
    }
    else
    {
        pidRunning = false;
        pwmDutyPercent = 0;
        outputWatts = 0;
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
    tipTemp_degC = IronOS::convertuVToDegC(tipTemp_uV); // IronOS curve
                                                        // tipTemp_degC = (tipTemp_raw * 204 - 73840) / 1000; // custom curve
}

void SolderingTip::pwmPIDControllerCallback()
{
    // PID controller or rather I², based on IronOS' PIDThread:
    // https://github.com/Ralim/IronOS/blob/v2.21/source/Core/Threads/PIDThread.cpp
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
        pwmDutyPercent = newDuty;

        // for display
        outputWatts = (pwmDutyPercent * availableWattsX10) / MAX_DUTY_PERCENT / 10;
    }
    else
    {
        // we're too hot
        pwmDutyPercent = 0;
        outputWatts = 0;
    }
}

void SolderingTip::tipOn()
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
}

void SolderingTip::tipOff()
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
}
