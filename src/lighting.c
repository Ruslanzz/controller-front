/**
  ******************************************************************************
  * @file    lighting.c
  * @brief   Реализация управления балкой (L1 = DRV1) и габаритами (L2 = DRV2).
  *
  *          Выходной каскад DRV — пара независимых ключей: верхний (P-канал)
  *          коммутирует V_BAT, нижний (N-канал) сажает вторую клемму на землю
  *          через датчик тока. Полярность управления — как у катушек на
  *          ведомом узле: сравнение 0 на канале открывает ключ (полный ток),
  *          сравнение PWM_PERIOD его закрывает.
  *
  *          Балка включена между обеими клеммами L1: CH1 — верхний ключ
  *          (регулировка), CH2 — нижний. Габариты — на верхнем ключе L2 (CH3),
  *          возврат тока на массу платы, нижний ключ (CH4) закрыт.
  *
  *          Разрешение драйверов (DRV*_EN_*) выставляется один раз при старте
  *          и больше не трогается: снятие разрешения не гасит выход, а
  *          залипает его на V_BAT (см. предупреждение в config.h).
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

/* Записать яркость габаритов в CH3. Регистр сравнения трогаем только при
 * смене значения, чтобы не дёргать его в каждом проходе главного цикла.     */
static uint32_t marker_compare = 0xFFFFFFFFu;  /* заведомо недостижимое */

static void Lighting_SetMarkerPercent(uint8_t percent)
{
  uint32_t compare = Lighting_CalcPeriod(percent);

  if (compare != marker_compare) {
    marker_compare = compare;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, compare);
  }
}

/* -------------------------------------------------------------------------- */
void Lighting_Init(void)
{
  /* Исходное состояние моста DRV1: плечо A закрыто (балка погашена),
   * плечо B открыто — готово к регулировке током по CH1.                    */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWM_PERIOD);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, PWM_PERIOD);

  /* Габариты: CH3 — верхний ключ DRV2, он и задаёт яркость. Нижний ключ
   * (CH4) держим закрытым: габариты возвращают ток на массу платы.          */
  Lighting_SetMarkerPercent(MARKER_BRIGHTNESS_PERCENT);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

  /* Разрешение обоих плеч у обоих драйверов. */
  HAL_GPIO_WritePin(GPIOA, DRV1_EN_A_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, DRV1_EN_B_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, DRV2_EN_A_Pin|DRV2_EN_B_Pin, GPIO_PIN_SET);

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

/* Габариты: пока активен любой из указателей поворота, приглушаются до
 * MARKER_DIM_PERCENT от штатной яркости, чтобы не забивать вспышки.         */
static void Lighting_UpdateMarkers(void)
{
  uint8_t percent = MARKER_BRIGHTNESS_PERCENT;

  if (turn_left_active || turn_right_active) {
    percent = (uint8_t)((MARKER_BRIGHTNESS_PERCENT * MARKER_DIM_PERCENT) / 100);
  }

  Lighting_SetMarkerPercent(percent);
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
  Lighting_UpdateMarkers();
  Lighting_UpdateBar(now);
}

/* -------------------------------------------------------------------------- */
void Lighting_CheckOvercurrent(void)
{
  static uint32_t window_start = 0;  /* начало текущего окна наблюдения    */
  static uint32_t last_sample  = 0;  /* когда последний раз брали выборку  */
  static uint8_t  hits         = 0;  /* превышений в текущем окне          */

  if (bar_fault) {
    return;
  }

  /* Опрос по времени: главный цикл крутится куда быстрее, чем АЦП обновляет
   * канал (полный цикл сканирования 8 каналов ~224 мкс), и без ограничения
   * одно и то же значение попало бы в счётчик десятки раз.                 */
  uint32_t now = HAL_GetTick();
  if ((now - last_sample) < OVERCURRENT_SAMPLE_MS) {
    return;
  }
  last_sample = now;

  if ((now - window_start) >= OVERCURRENT_WINDOW_MS) {
    window_start = now;
    hits         = 0;
  }

  /* Ток рублен ШИМ верхнего плеча, поэтому считаем именно превышения, а не
   * их непрерывную длительность: в паузах показание падает почти до нуля.
   * Порог только верхний — ток в этой цепи однонаправленный, и низкое
   * показание означает неисправный датчик, а не переток (см. config.h).   */
  if (ADS_RES_BUFFER[DRV1_CURRENT_IDX] <= BAR_OVERCURRENT_ADC_HI) {
    return;
  }

  if (++hits < OVERCURRENT_TRIP_HITS) {
    return;
  }

  /* Закрыть верхний ключ балки. Разрешение драйвера при этом НЕ снимаем:
   * без него затвор верхнего ключа притягивается вниз и выход залипает на
   * V_BAT — защита включила бы балку вместо того, чтобы погасить.          */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWM_PERIOD);
  bar_fault = 1;
  hits      = 0;
}
