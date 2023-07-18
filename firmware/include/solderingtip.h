#ifndef SOLDERINGTIP_H
#define SOLDERINGTIP_H

#include <Arduino.h>

#include "ironos_pid.h"

class SolderingTip
{
public:
    SolderingTip() = default;

    // timer configuration, call from your setup()
    void setup();

    // can always be called to turn the tip off and set target to 0°C
    void safeMode();
    void setTargetTemperature(uint32_t target_degC);

    // internal timer callbacks
    void pwmUpdateCallback();
    void pwmSwitchCallback();
    void pwmMeasureCallback();
    void pwmPIDControllerCallback();

    // getters for UI...
    unsigned getTipTempRaw()
    {
        return tipTemp_raw;
    }

    unsigned getTipTempuV()
    {
        return tipTemp_uV;
    }

    int getTipTempDegC()
    {
        return tipTemp_degC;
    }

    unsigned getVinRaw()
    {
        return vin_raw;
    }

    unsigned getVinmV()
    {
        return vin_mV;
    }

    unsigned getPWM()
    {
        return pwmDutyPercent;
    }

    unsigned getOutputWatts()
    {
        return outputWatts;
    }

protected:
    void tipOn();
    void tipOff();

    // an STM32 timer with 3+ channels, e.g. TIM3 on F030
    HardwareTimer *TipHeatTimer;
    static constexpr uint32_t THTSwitchChannel = 1;
    static constexpr uint32_t THTMeasureChannel = 2;
    static constexpr uint32_t THTControllerChannel = 3;

    // output
    uint32_t pwmDutyPercent = 0;

    // measurements
    uint32_t tipTemp_raw = 0;
    uint32_t tipTemp_uV = 0;
    int32_t tipTemp_degC = 0;

    uint32_t vin_raw = 0;
    int32_t vin_mV = 0;

    // control
    bool pidRunning = false;
    uint32_t targetTemperature_degC = 0;

    uint32_t lastPIDTime = 0;
    IronOS::Integrator<int32_t> powerStore = {0};
    uint32_t outputWatts = 0;

    bool init = true;

    // Our PWM runs with 10Hz. We need time between switching off the heat and measuring so the
    // MOSFETs are off and heat distribution in the tip has settled a bit.
    // - PID calculation at 95ms
    // - Measurement worst case at 90ms
    // -> Duty cycle limit 80%
    static constexpr uint32_t MAX_DUTY_PERCENT = 80;
    // 8.3 Ohm measured with multimeter. IronOS only differentiates between 6 and 8 Ohm tips, so take 8.
    static constexpr uint32_t TIP_RESISTANCE_X10OHM = 80;
    // according to advertising :)
    static constexpr uint32_t HARDWARE_MAX_WATTAGE_X10 = 650;
    // default gain in IronOS for 8 Ohm tips
    static constexpr uint32_t TIP_THERMAL_MASS = 65; // X10 watts to raise 1 deg C in 1 second
};

extern SolderingTip solderingTip;

#endif /* SOLDERINGTIP_H */
