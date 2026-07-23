/**
  ******************************************************************************
  * @file    lighting.c
  * @brief   Реализация управления светодиодной балкой (выход L1 = DRV1).
  *
  *          Полярность моста DRV1 — как у катушек на ведомом узле:
  *          CH1 (IN_A) — регулируемое плечо, сравнение PWM_PERIOD = закрыто,
  *          0 = полный ток; CH2 (IN_B) — второе плечо, PWM_PERIOD = открыто,
  *          0 = разрыв цепи (используется защитой от перетока).
  *
  *          Мигание указателей поворота: пока команда от ведущего активна,
  *          реле переключается каждые TURN_BLINK_TOGGLE_MS (~1.5 Гц).
  ******************************************************************************
  */

#include "lighting.h"
#include "config.h"
#include "bsp.h"
#include "io.h"

/* Стробоскоп: активность и отметка начала отсчёта фазы вспышек. */
static uint8_t  strobe_active = 0;
static uint32_t strobe_tick   = 0;

/* Авария по току: балка погашена до возврата потенциометра в ноль. */
static uint8_t  bar_fault = 0;

/* Указатели поворота: команды ведущего (пишутся из приёма CAN) и общая
 * фаза мигания. Активен всегда максимум один указатель (оба рычага —
 * стоп-сигнал, спереди оба гаснут), поэтому фаза общая.                     */
static volatile uint8_t turn_left_active  = 0;
static volatile uint8_t turn_right_active = 0;
static uint8_t  blink_on   = 0;
static uint32_t blink_tick = 0;

uint32_t Lighting_CalcPeriod(uint8_t value)
{
  /* Умножение до деления: PWM_PERIOD не кратен 100, иначе при value = 0
   * остаток давал бы паразитную засветку ~1 %.                              */
  uint32_t percentage = 100 - value;
  return (PWM_PERIOD * percentage) / 100;
}

/* Чтение потенциометра с реверсом шкалы: движок в крайнем положении
 * (максимум яркости, стробоскоп) даёт минимум АЦП, поэтому значение
 * инвертируется в прямую шкалу 0..POT_ADC_MAX.                              */
static uint32_t Lighting_ReadPot(void)
{
  uint32_t adc = ADS_RES_BUFFER[POT_ADC_IDX];
  return (adc >= POT_ADC_MAX) ? 0 : (POT_ADC_MAX - adc);
}

/* Положение потенциометра (прямая шкала) -> яркость 0..100 %. */
static uint8_t Lighting_PotToPercent(uint32_t adc)
{
  if (adc < POT_ADC_DEADBAND) {
    return 0;
  }
  uint32_t percent = (adc * 100) / POT_ADC_MAX;
  return (percent > 100) ? 100 : (uint8_t)percent;
}

/* -------------------------------------------------------------------------- */
void Lighting_Init(void)
{
  /* Габаритные огни — горят постоянно с момента старта. */
  IO_RelayOn(RELAY_MARKER_1);
  IO_RelayOn(RELAY_MARKER_2);

  /* Исходное состояние моста DRV1: плечо A закрыто (балка погашена),
   * плечо B открыто — готово к регулировке током по CH1.                    */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWM_PERIOD);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, PWM_PERIOD);

  /* DRV2 не используется: оба плеча закрыты, EN остаются низкими.           */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_PERIOD);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

  /* Разрешение драйвера DRV1. */
  HAL_GPIO_WritePin(GPIOA, DRV1_EN_A_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, DRV1_EN_B_Pin, GPIO_PIN_SET);

  strobe_tick = HAL_GetTick();
}

/* -------------------------------------------------------------------------- */
void Lighting_SetTurnLeft(uint8_t on)
{
  if (on && !turn_left_active) {
    /* Немедленное включение на фронте команды, фаза мигания с нуля. */
    blink_on   = 1;
    blink_tick = HAL_GetTick();
    turn_left_active = 1;
    IO_RelayOn(RELAY_TURN_LEFT);
  } else if (!on && turn_left_active) {
    turn_left_active = 0;
    IO_RelayOff(RELAY_TURN_LEFT);
  }
}

void Lighting_SetTurnRight(uint8_t on)
{
  if (on && !turn_right_active) {
    blink_on   = 1;
    blink_tick = HAL_GetTick();
    turn_right_active = 1;
    IO_RelayOn(RELAY_TURN_RIGHT);
  } else if (!on && turn_right_active) {
    turn_right_active = 0;
    IO_RelayOff(RELAY_TURN_RIGHT);
  }
}

/* Мигание активных указателей: переключение реле каждые TURN_BLINK_TOGGLE_MS. */
static void Lighting_UpdateTurns(uint32_t now)
{
  if (!turn_left_active && !turn_right_active) {
    return;
  }

  if ((now - blink_tick) >= TURN_BLINK_TOGGLE_MS) {
    blink_tick = now;
    blink_on   = !blink_on;

    if (turn_left_active) {
      if (blink_on) { IO_RelayOn(RELAY_TURN_LEFT); } else { IO_RelayOff(RELAY_TURN_LEFT); }
    }
    if (turn_right_active) {
      if (blink_on) { IO_RelayOn(RELAY_TURN_RIGHT); } else { IO_RelayOff(RELAY_TURN_RIGHT); }
    }
  }
}

/* Балка: выбор режима (яркость / стробоскоп) и обновление ШИМ. */
static void Lighting_UpdateBar(uint32_t now)
{
  uint32_t adc = Lighting_ReadPot();

  if (bar_fault) {
    /* Сброс аварии — только возвратом ручки в ноль: оператор подтверждает,
     * что заметил отключение, и балка не вспыхнет на полной яркости.        */
    if (adc >= POT_ADC_DEADBAND) {
      return;
    }
    bar_fault = 0;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, PWM_PERIOD);
  }

  /* Вход/выход стробоскопа на краю диапазона — с гистерезисом. */
  if (!strobe_active && adc >= STROBE_ON_ADC) {
    strobe_active = 1;
    strobe_tick   = now;
  } else if (strobe_active && adc < STROBE_OFF_ADC) {
    strobe_active = 0;
  }

  if (strobe_active) {
    /* Вспышка STROBE_FLASH_MS в начале каждого периода STROBE_PERIOD_MS. */
    uint32_t phase = (now - strobe_tick) % STROBE_PERIOD_MS;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,
                          (phase < STROBE_FLASH_MS) ? 0 : PWM_PERIOD);
  } else {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,
                          Lighting_CalcPeriod(Lighting_PotToPercent(adc)));
  }
}

/* -------------------------------------------------------------------------- */
void Lighting_Update(void)
{
  uint32_t now = HAL_GetTick();

  Lighting_UpdateTurns(now);
  Lighting_UpdateBar(now);
}

/* -------------------------------------------------------------------------- */
void Lighting_CheckOvercurrent(void)
{
  /* Отметка начала непрерывного перетока; 0 — перетока нет. */
  static uint32_t oc_since = 0;

  uint32_t i_bar = ADS_RES_BUFFER[DRV1_CURRENT_IDX];

  if (bar_fault) {
    return;
  }

  if (i_bar > BAR_OVERCURRENT_ADC_HI || i_bar < BAR_OVERCURRENT_ADC_LO) {
    uint32_t now = HAL_GetTick();
    if (oc_since == 0) {
      oc_since = (now != 0) ? now : 1;
      return;
    }
    if ((now - oc_since) < BAR_OVERCURRENT_TRIP_MS) {
      return;  /* выдержка: возможно, одиночный выброс или пусковой ток */
    }
    /* Переток держится дольше выдержки — разомкнуть оба плеча DRV1:
     * CH1 = PWM_PERIOD (A закрыто), CH2 = 0 (B закрыто).                    */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWM_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    bar_fault = 1;
    oc_since  = 0;
  } else {
    oc_since = 0;
  }
}
