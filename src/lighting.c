/**
  ******************************************************************************
  * @file    lighting.c
  * @brief   Реализация управления балкой (L1 = DRV1) и габаритами (L2 = DRV2).
  *
  *          Полярность мостов — как у катушек на ведомом узле: сравнение 0 на
  *          канале IN_x открывает верхний ключ плеча (полный ток), сравнение
  *          PWM_PERIOD сажает выход плеча на землю (нагрузка обесточена).
  *
  *          Балка включена между обоими плечами DRV1: CH1 (IN_A) —
  *          регулируемое плечо, CH2 (IN_B) — второе плечо моста.
  *
  *          Габариты питаются одним плечом A драйвера DRV2 (CH3), возврат тока
  *          — по общему минусу фонаря на массу платы; плечо B не используется
  *          и остаётся запрещённым (см. config.h).
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

/* Авария по току габаритов: снимается только перезапуском узла — органа
 * управления у габаритов нет, а самовосстановление в КЗ дало бы циклический
 * перезапуск нагрузки.                                                       */
static uint8_t  marker_fault = 0;

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
  /* Исходное состояние моста DRV1: плечо A закрыто (балка погашена),
   * плечо B открыто — готово к регулировке током по CH1.                    */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWM_PERIOD);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, PWM_PERIOD);

  /* Габариты: плечо A драйвера DRV2 на заданной яркости, горят постоянно
   * с момента старта. Плечо B не используется — его канал держим в нуле,
   * а сам ключ запрещён (DRV2_EN_B остаётся низким после MX_GPIO_Init).     */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,
                        Lighting_CalcPeriod(MARKER_BRIGHTNESS_PERCENT));
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

  /* Разрешение драйверов: DRV1 — оба плеча (балка включена в мост),
   * DRV2 — только плечо A (габариты возвращают ток на массу платы).         */
  HAL_GPIO_WritePin(GPIOA, DRV1_EN_A_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, DRV1_EN_B_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, DRV2_EN_A_Pin, GPIO_PIN_SET);

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
    HAL_GPIO_WritePin(GPIOA, DRV1_EN_A_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, DRV1_EN_B_Pin, GPIO_PIN_SET);
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

/* Выдержка перетока: 1, когда @p over держится дольше OVERCURRENT_TRIP_MS.
 * Отметку начала перетока хранит вызывающая сторона — у каждого канала своя. */
static uint8_t Lighting_TripDelayElapsed(uint32_t *since, uint8_t over)
{
  if (!over) {
    *since = 0;
    return 0;
  }

  uint32_t now = HAL_GetTick();
  if (*since == 0) {
    *since = (now != 0) ? now : 1;  /* 0 зарезервирован под "перетока нет" */
    return 0;
  }
  return ((now - *since) >= OVERCURRENT_TRIP_MS) ? 1 : 0;
}

/* -------------------------------------------------------------------------- */
void Lighting_CheckOvercurrent(void)
{
  static uint32_t bar_oc_since    = 0;
  static uint32_t marker_oc_since = 0;

  /* --- Балка (DRV1) --- */
  if (!bar_fault) {
    uint32_t i = ADS_RES_BUFFER[DRV1_CURRENT_IDX];
    uint8_t  over = (i > BAR_OVERCURRENT_ADC_HI || i < BAR_OVERCURRENT_ADC_LO);

    if (Lighting_TripDelayElapsed(&bar_oc_since, over)) {
      /* Обесточить балку: оба плеча моста на землю — на нагрузке нулевая
       * разность потенциалов, затем запретить сами ключи DRV1.              */
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWM_PERIOD);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, PWM_PERIOD);
      HAL_GPIO_WritePin(GPIOA, DRV1_EN_A_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, DRV1_EN_B_Pin, GPIO_PIN_RESET);
      bar_fault    = 1;
      bar_oc_since = 0;
    }
  }

  /* --- Габариты (DRV2, плечо A) --- */
  if (!marker_fault) {
    uint32_t i = ADS_RES_BUFFER[DRV2_CURRENT_IDX];
    uint8_t  over = (i > MARKER_OVERCURRENT_ADC_HI || i < MARKER_OVERCURRENT_ADC_LO);

    if (Lighting_TripDelayElapsed(&marker_oc_since, over)) {
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_PERIOD);
      HAL_GPIO_WritePin(GPIOB, DRV2_EN_A_Pin, GPIO_PIN_RESET);
      marker_fault    = 1;
      marker_oc_since = 0;
    }
  }
}
