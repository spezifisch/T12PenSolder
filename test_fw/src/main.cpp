#include <Arduino.h>
#include <U8g2lib.h>

#include "cube_init.h"
#include "ironos_pid.h"

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

static constexpr bool HEAT_ARMED = true;

// an STM32 timer with 3+ channels
HardwareTimer *TipHeatTimer;

static constexpr uint32_t THTSwitchChannel = 1;
static constexpr uint32_t THTMeasureChannel = 2;
static constexpr uint32_t THTControllerChannel = 3;

// HACK just for comparison
uint32_t IronOS_convertuVToDegC(uint32_t tipuVDelta);

U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, /* clock=*/SCL_PIN, /* data=*/SDA_PIN, /* reset=*/U8X8_PIN_NONE);

class SolderingTip
{
public:
  SolderingTip() = default;

  void safeMode()
  {
    tipOff();
    targetTemperature_degC = 0;
    outputWatts = 0;
    pwmDutyPercent = 0;
    tipOff();
  }

  void setTargetTemperature(uint32_t target_degC)
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
  void pwmUpdateCallback()
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

  void
  pwmSwitchCallback()
  {
    tipOff();
  }

  void pwmMeasureCallback()
  {
    // measure Vin and tip temperature
    vin_raw = constrain(analogRead(VIN_MEAS), 0, 4095);
    tipTemp_raw = constrain(analogRead(TIPTEMP_MEAS), 0, 4095);

    // convert to V
    vin_mV = constrain(vin_raw * 3300 / 4095 * 110000 / 10000, 0, 30000);     // 3.3V ADC ref., 12 bit ADC, compensate for 1/11 voltage divider
    tipTemp_uV = constrain(tipTemp_raw * 3300 / 4095 * 1000 / 221, 0, 15000); // ... to uV, 221x gain from OpAmp

    // convert to °C
    tipTemp_degCX1000 = tipTemp_raw * 204 - 73840;
  }

  void pwmPIDControllerCallback()
  {
    // PID, largely ripped from IronOS
    const uint32_t now = millis();

    const uint32_t deltaT_ms = now - lastPIDTime;
    if ((deltaT_ms == 0 || deltaT_ms > 500) && !init)
    {
      // something's hanging
      targetTemperature_degC = 0;
      return;
    }
    const uint32_t rate = 1000 / deltaT_ms;
    lastPIDTime = now;

    if (!pidRunning)
    {
      // disabled by UI
      return;
    }
    init = false;

    int32_t error_K = targetTemperature_degC - getTipTempDegC();

    int32_t outputX10Watts = powerStore.update(TIP_THERMAL_MASS * error_K, // the required power
                                               TIP_THERMAL_MASS,           // Inertia, smaller numbers increase dominance of the previous value
                                               2,                          // gain
                                               rate,                       // PID cycle frequency
                                               HARDWARE_MAX_WATTAGE_X10);

    // output
    if (outputX10Watts > 0)
    {
      const uint32_t v = getVinmV() / 100;
      uint32_t availableWattsX10 = (v * v) / TIP_RESISTANCE_X10OHM;
      availableWattsX10 = availableWattsX10 * MAX_DUTY_PERCENT;
      availableWattsX10 /= 100;

      uint32_t newDuty = (MAX_DUTY_PERCENT * outputX10Watts) / availableWattsX10;
      pwmDutyPercent = constrain(newDuty, 0, MAX_DUTY_PERCENT);

      outputWatts = (pwmDutyPercent * availableWattsX10) / MAX_DUTY_PERCENT / 10;
    }
    else
    {
      // we're too hot
      pwmDutyPercent = 0;
      outputWatts = 0;
    }
  }

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
    return tipTemp_degCX1000 / 1000;
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
  void tipOn()
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
  }

  void tipOff()
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
  }

  // output
  uint32_t pwmDutyPercent = 0;
  static constexpr uint32_t MAX_DUTY_PERCENT = 80;

  // measurements
  uint32_t tipTemp_raw = 0;
  uint32_t tipTemp_uV = 0;
  int32_t tipTemp_degCX1000 = 0;

  uint32_t vin_raw = 0;
  int32_t vin_mV = 0;

  // control
  bool pidRunning = false;
  uint32_t targetTemperature_degC = 0;

  uint32_t lastPIDTime = 0;
  IronOS::Integrator<int32_t> powerStore = {0};
  uint32_t outputWatts = 0;

  bool init = true;

  static constexpr uint32_t TIP_RESISTANCE_X10OHM = 80; // 8.3 Ohm measured
  static constexpr uint32_t HARDWARE_MAX_WATTAGE_X10 = 650;
  static constexpr uint32_t TIP_THERMAL_MASS = 65; // X10 watts to raise 1 deg C in 1 second
};
SolderingTip solderingTip;

void Update_IT_callback(void)
{
  // Update event correspond to Rising edge of PWM when configured in PWM1 mode
  solderingTip.pwmUpdateCallback();
}

void Switch_callback(void)
{
  // Compare match event correspond to falling edge of PWM when configured in PWM1 mode
  solderingTip.pwmSwitchCallback();
}

void Measure_callback(void)
{
  solderingTip.pwmMeasureCallback();
}

void Controller_callback(void)
{
  solderingTip.pwmPIDControllerCallback();
}

void setup()
{
  // lowlevel stm32cube setup
  MX_GPIO_Init();

  // turn heat off
  digitalWrite(TIPHEAT_DRV, 0);
  solderingTip.safeMode();

  // stock firmware does what looks like a display reset, but it isn't connected on my board
  digitalWrite(PA9, 0);
  delay(200);
  digitalWrite(PA9, 1);
  delay(200);

  // adc setup
  pinMode(VIN_MEAS, INPUT_ANALOG);
  pinMode(TIPTEMP_MEAS, INPUT_ANALOG);
  analogReadResolution(12);
  // set sampling time to same as stock firmware
  MX_ADC_Config();

  // configure PWM heater output
  TipHeatTimer = new HardwareTimer(TIM3);
  TipHeatTimer->setOverflow(100000, MICROSEC_FORMAT);              // 100 ms, 10 Hz
  TipHeatTimer->setMode(THTSwitchChannel, TIMER_DISABLED, -1);     // on compare cb: switch output off
  TipHeatTimer->setMode(THTMeasureChannel, TIMER_DISABLED, -1);    // measure tip temperature
  TipHeatTimer->setMode(THTControllerChannel, TIMER_DISABLED, -1); // update PID controller
  TipHeatTimer->setCaptureCompare(THTSwitchChannel, 0, PERCENT_COMPARE_FORMAT);
  TipHeatTimer->setCaptureCompare(THTMeasureChannel, 10, PERCENT_COMPARE_FORMAT);    // switch% + 10%
  TipHeatTimer->setCaptureCompare(THTControllerChannel, 90, PERCENT_COMPARE_FORMAT); // always after 90ms
  TipHeatTimer->attachInterrupt(Update_IT_callback);
  TipHeatTimer->attachInterrupt(THTSwitchChannel, Switch_callback);
  TipHeatTimer->attachInterrupt(THTMeasureChannel, Measure_callback);
  TipHeatTimer->attachInterrupt(THTControllerChannel, Controller_callback);
  TipHeatTimer->resume();

  // display setup
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
  static constexpr uint32_t UPDATE_PERIOD_ms = 100;                               // display update period
  static uint32_t last_millis = UPDATE_PERIOD_ms * (millis() / UPDATE_PERIOD_ms); // last display update
  static uint32_t last_showtime = 0;                                              // last display buffer transfer duration

  static constexpr uint32_t BUTTON_DEBOUNCE_COUNT = 2;                                          // button state must be stable for this many display updates
  static uint32_t buttonSetPressCount = 0, buttonMinusPressCount = 0, buttonPlusPressCount = 0; // debounce counters

  static bool heatOn = false;                     // soldering tip enabled
  static uint32_t selectedTemperature_degC = 320; // target temp., change with -/+

  static char tmp[64];

  // 10 Hz UI update
  const uint32_t now = millis();
  if (now - last_millis < UPDATE_PERIOD_ms)
  {
    return;
  }
  last_millis = now;

  // loop benchmark
  const uint32_t start = micros();

  // read buttons
  const bool buttonSet = digitalRead(BUT_SET);
  const bool buttonMinus = digitalRead(BUT_MINUS);
  const bool buttonPlus = digitalRead(BUT_PLUS);

  if (buttonSet)
  {
    if (buttonSetPressCount < BUTTON_DEBOUNCE_COUNT)
    {
      buttonSetPressCount++;

      if (buttonSetPressCount == BUTTON_DEBOUNCE_COUNT)
      {
        heatOn = !heatOn;

        solderingTip.setTargetTemperature(heatOn ? selectedTemperature_degC : 0);
      }
    }
  }
  else
  {
    buttonSetPressCount = 0;
  }

  if (buttonMinus)
  {
    if (buttonMinusPressCount < BUTTON_DEBOUNCE_COUNT)
    {
      buttonMinusPressCount++;

      if (buttonMinusPressCount == BUTTON_DEBOUNCE_COUNT)
      {
        selectedTemperature_degC -= 10;
        if (selectedTemperature_degC < 150)
        {
          selectedTemperature_degC = 150;
        }

        solderingTip.setTargetTemperature(heatOn ? selectedTemperature_degC : 0);
        buttonMinusPressCount = 0; // repeat
      }
    }
  }
  else
  {
    buttonMinusPressCount = 0;
  }

  if (buttonPlus)
  {
    if (buttonPlusPressCount < BUTTON_DEBOUNCE_COUNT)
    {
      buttonPlusPressCount++;

      if (buttonPlusPressCount == BUTTON_DEBOUNCE_COUNT)
      {
        selectedTemperature_degC += 10;
        if (selectedTemperature_degC > 450)
        {
          selectedTemperature_degC = 450;
        }

        solderingTip.setTargetTemperature(heatOn ? selectedTemperature_degC : 0);
        buttonPlusPressCount = 0; // repeat
      }
    }
  }
  else
  {
    buttonPlusPressCount = 0;
  }

  // get tip state
  const uint32_t vin_raw = solderingTip.getVinRaw();
  const uint32_t vin_mv = solderingTip.getVinmV();

  const int32_t tt_raw = solderingTip.getTipTempRaw();
  const int32_t tt_degC = solderingTip.getTipTempDegC();
  const uint32_t tt_iosDegC = IronOS_convertuVToDegC(solderingTip.getTipTempuV());

  const int32_t t_pwm = solderingTip.getPWM();
  const int32_t t_outputWatts = solderingTip.getOutputWatts();

  // display output
  u8g2.firstPage();
  do
  {
    u8g2.setFont(u8g2_font_5x8_mr);

    // first row
    u8g2.setCursor(0, 8);
    snprintf(tmp, sizeof(tmp), "Vin:%04d %5dmV T%07d", vin_raw, vin_mv, now % 10000000);
    u8g2.print(tmp);

    u8g2.setCursor(0, 16);
    snprintf(tmp, sizeof(tmp), "Tip:%2dW Du%2d%% Tgt%3dC L%2d", t_outputWatts, t_pwm, selectedTemperature_degC, last_showtime / 1000);
    u8g2.print(tmp);

    // third row
    u8g2.setCursor(0, 24);
    u8g2.print(heatOn ? "ON!" : "off");

    // 4th row
    u8g2.setCursor(0, 32);
    snprintf(tmp, sizeof(tmp), "IOS%03d", tt_iosDegC);
    u8g2.print(tmp);

    //
    u8g2.setFont(u8g2_font_7x14B_mf);
    u8g2.setCursor(48, 30);
    snprintf(tmp, sizeof(tmp), "%3dC", tt_degC);
    u8g2.print(tmp);

    u8g2.setCursor(96, 30);
    snprintf(tmp, sizeof(tmp), "%04d", tt_raw);
    u8g2.print(tmp);
  } while (u8g2.nextPage());

  last_showtime = micros() - start;
}
