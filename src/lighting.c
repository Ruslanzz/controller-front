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

/* Авария по току на габаритах: канал погашен до перезагрузки узла. Сбрасывать
 * его нечем — у габаритов нет органа управления, возвратом которого оператор
 * подтвердил бы, что заметил отключение (у балки эту роль играет ручка).    */
static uint8_t  marker_fault = 0;

/* ===== Привязка каналов к железу ==========================================
 * L1 — балка: TIM4 CH1 = IN_A (верхнее плечо, регулировка), CH2 = IN_B
 * (нижнее, постоянно открыто), ток на ADC CH5 (PA5).
 * L2 — габариты: CH3 = IN_A, CH4 = IN_B (закрыт: ток возвращается на массу
 * платы мимо нижнего ключа), ток на ADC CH6 (PA6).                          */
typedef struct {
  uint32_t ch_a;       /* канал TIM4: верхнее плечо (задаёт ток)             */
  uint8_t  adc_idx;    /* индекс датчика тока в ADS_RES_BUFFER               */
  uint32_t oc_peak_ma; /* заданный порог канала, мА                          */
} DrvHw;

static const DrvHw drv_hw[DRV_COUNT] = {
  { TIM_CHANNEL_1, DRV1_CURRENT_IDX, BAR_OC_PEAK_MA    },
  { TIM_CHANNEL_3, DRV2_CURRENT_IDX, MARKER_OC_PEAK_MA }
};

/* ===== Состояние измерения и защиты по току =============================== */
typedef struct {
  uint32_t zero_adc;    /* ноль датчика: измеренный или номинальный          */
  uint8_t  zero_valid;
  uint32_t ceiling_ma;  /* потолок измерения при этом нуле, мА               */
  uint32_t oc_peak_ma;  /* действующий порог, мА                             */
  uint8_t  oc_enabled;  /* 0 — порог не помещается под потолок               */
  uint32_t ma_meas;     /* последнее измеренное значение, мА (телеметрия)    */
  uint8_t  saturated;   /* показание упирается в потолок АЦП                 */
  uint32_t hits[OVERCURRENT_TRIP_HITS];  /* кольцо отметок превышений        */
  uint8_t  head;
  uint8_t  count;
  uint16_t trips;
} DrvCurrent;

static DrvCurrent drv_i[DRV_COUNT];

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

/* --------------------------------------------------------------------------
 * Исходное безопасное состояние. Вызывать ДО запуска ШИМ: иначе между стартом
 * ШИМ и первой записью на плечи уйдут значения сравнения по умолчанию, а ноль
 * на верхнем плече означает нагрузку, включённую на полную.
 * -------------------------------------------------------------------------- */
void Lighting_Init(void)
{
  /* Исходное состояние моста DRV1: плечо A закрыто (балка погашена),
   * плечо B открыто — готово к регулировке током по CH1.                    */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWM_PERIOD);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, PWM_PERIOD);

  /* Габариты: CH3 — верхний ключ DRV2, он и задаёт яркость. Нижний ключ
   * (CH4) держим закрытым: габариты возвращают ток на массу платы.
   *
   * До первого разрешения драйверов габариты держим ПОГАШЕННЫМИ, а рабочую
   * яркость выставляем в Lighting_Update: между записью и подачей EN канал
   * всё равно ничем не управляется.                                         */
  marker_compare = PWM_PERIOD;
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_PERIOD);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

  for (uint8_t i = 0; i < DRV_COUNT; i++) {
    drv_i[i].zero_adc   = DRV_ZERO_NOMINAL_ADC;
    drv_i[i].zero_valid = 0;
    drv_i[i].ceiling_ma = 0;
    drv_i[i].oc_peak_ma = drv_hw[i].oc_peak_ma;
    drv_i[i].oc_enabled = 0;
    drv_i[i].ma_meas    = 0;
    drv_i[i].saturated  = 0;
    drv_i[i].head       = 0;
    drv_i[i].count      = 0;
    drv_i[i].trips      = 0;
  }

  strobe_tick = HAL_GetTick();
}

/* Поднять разрешение обоих драйверов. Вызывать один раз, ПОСЛЕ запуска ШИМ и
 * Lighting_Init: снятие EN не гасит нагрузку, а залипает выход на V_BAT
 * (см. config.h), поэтому обратной операции здесь нет вообще.                */
void Lighting_EnableDrivers(void)
{
  HAL_GPIO_WritePin(GPIOA, DRV1_EN_A_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, DRV1_EN_B_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, DRV2_EN_A_Pin|DRV2_EN_B_Pin, GPIO_PIN_SET);
}

/* Отсчёты АЦП от нуля датчика -> миллиамперы. Масштаб выводится из
 * ИЗМЕРЕННОГО нуля (датчик ратиометричный), поэтому пересчёт верен и при
 * питании датчика 5 В, и при 3.3 В, и при делителе на выходе — вывод в
 * config.h.                                                                  */
static uint32_t Drv_AdcToMa(uint8_t i, uint32_t adc_delta)
{
  if (drv_i[i].zero_adc == 0) {
    return 0;
  }
  return (adc_delta * (uint32_t)DRV_ADC_FULL_SCALE_MA) / drv_i[i].zero_adc;
}

void Lighting_CalibrateCurrentSensors(void)
{
  uint32_t sum[DRV_COUNT] = {0};

  /* Верхние плечи закрыты (свет погашен), EN уже поднят — тока через датчики
   * нет, они показывают собственный ноль. Ждать «снятия EN» здесь нельзя:
   * снятый EN означает открытый верхний ключ, то есть включённый свет.       */
  HAL_Delay(50);
  for (uint8_t s = 0; s < ACS_CAL_SAMPLES; s++) {
    for (uint8_t i = 0; i < DRV_COUNT; i++) {
      sum[i] += ADS_RES_BUFFER[drv_hw[i].adc_idx];
    }
    HAL_Delay(2);
  }

  for (uint8_t i = 0; i < DRV_COUNT; i++) {
    uint32_t zero = sum[i] / ACS_CAL_SAMPLES;

    if (zero >= ACS_CAL_MIN_ADC && zero <= ACS_CAL_MAX_ADC) {
      drv_i[i].zero_adc   = zero;
      drv_i[i].zero_valid = 1;
    } else {
      /* Датчик неисправен или не подключён: берём номинальный ноль и
       * отключаем защиту канала. Держать порог на сигнале, который мы сами
       * объявили недостоверным, значит гасить исправный свет по неисправности
       * измерения — а погасшая ночью балка опаснее незащищённого КЗ, которое
       * ловит предохранитель.                                                */
      drv_i[i].zero_adc   = DRV_ZERO_NOMINAL_ADC;
      drv_i[i].zero_valid = 0;
    }

    /* Потолок измерения: выше 4095 отсчётов АЦП не видит ничего. Если порог не
     * помещается под него с запасом, защита канала честно отключается —
     * мёртвый порог хуже честно выключенного (факт виден в телеметрии).      */
    drv_i[i].ceiling_ma = (drv_i[i].zero_adc < 4095u)
                            ? Drv_AdcToMa(i, 4095u - drv_i[i].zero_adc) : 0u;

    drv_i[i].oc_peak_ma = drv_hw[i].oc_peak_ma;
    drv_i[i].oc_enabled = (uint8_t)(drv_i[i].zero_valid &&
                                    drv_i[i].ceiling_ma >=
                                      (drv_hw[i].oc_peak_ma +
                                       (uint32_t)DRV_OC_MIN_HEADROOM_MA));
  }
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

  if (marker_fault) {
    /* Канал отключён защитой по току и остаётся погашенным: сбрасывать аварию
     * нечем (органа управления у габаритов нет), а самовозврат означал бы
     * повторное включение в замыкание раз за разом.                         */
    Lighting_SetMarkerPercent(0);
    return;
  }

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

/* --------------------------------------------------------------------------
 * Защита по току обоих каналов.
 *
 * Ток рублен ШИМ верхнего плеча, поэтому считаются именно ПРЕВЫШЕНИЯ В ОКНЕ,
 * а не их непрерывная длительность: в паузах показание падает почти до нуля,
 * и условие «держится непрерывно N мс» при яркости ниже полной не выполнялось
 * бы никогда. Порог только ВЕРХНИЙ — ток однонаправленный, и низкое показание
 * означает неисправный датчик, а не переток (см. config.h).
 *
 * Кольцо из OVERCURRENT_TRIP_HITS отметок вместо прежнего окна с обнулением по
 * границе: скользящее окно не теряет счёт на стыке двух соседних окон.
 * -------------------------------------------------------------------------- */
static uint8_t Drv_RegisterOverCurrent(uint8_t i, uint32_t now)
{
  drv_i[i].hits[drv_i[i].head] = now;
  drv_i[i].head = (uint8_t)((drv_i[i].head + 1) % OVERCURRENT_TRIP_HITS);
  if (drv_i[i].count < OVERCURRENT_TRIP_HITS) {
    drv_i[i].count++;
  }

  if (drv_i[i].count < OVERCURRENT_TRIP_HITS) {
    return 0;
  }
  return ((now - drv_i[i].hits[drv_i[i].head]) <= OVERCURRENT_WINDOW_MS) ? 1 : 0;
}

void Lighting_CheckOvercurrent(void)
{
  static uint32_t last_sample = 0;  /* когда последний раз брали выборку  */

  /* Опрос по времени: главный цикл крутится куда быстрее, чем АЦП обновляет
   * канал (полный цикл сканирования 8 каналов ~224 мкс), и без ограничения
   * одно и то же значение попало бы в счётчик десятки раз.                 */
  uint32_t now = HAL_GetTick();
  if ((now - last_sample) < OVERCURRENT_SAMPLE_MS) {
    return;
  }
  last_sample = now;

  for (uint8_t i = 0; i < DRV_COUNT; i++) {
    uint32_t raw_u = ADS_RES_BUFFER[drv_hw[i].adc_idx];
    int32_t  diff  = (int32_t)raw_u - (int32_t)drv_i[i].zero_adc;

    /* Показание упёрлось в потолок АЦП: истинный ток больше измеренного. */
    drv_i[i].saturated = (uint8_t)(raw_u >= 4090u);
    drv_i[i].ma_meas   = (diff > 0) ? Drv_AdcToMa(i, (uint32_t)diff) : 0u;

    uint8_t tripped = (i == 0) ? bar_fault : marker_fault;

    if (tripped || !drv_i[i].oc_enabled) {
      drv_i[i].head  = 0;
      drv_i[i].count = 0;
      continue;
    }

    if (drv_i[i].ma_meas <= drv_i[i].oc_peak_ma ||
        !Drv_RegisterOverCurrent(i, now)) {
      continue;
    }

    /* Закрыть верхний ключ канала. Разрешение драйвера при этом НЕ снимаем:
     * без него затвор верхнего ключа притягивается вниз и выход залипает на
     * V_BAT — защита включила бы нагрузку вместо того, чтобы её погасить.   */
    __HAL_TIM_SET_COMPARE(&htim4, drv_hw[i].ch_a, PWM_PERIOD);

    if (i == 0) {
      bar_fault = 1;
    } else {
      marker_fault   = 1;
      /* Держим кэш сравнения согласованным с тем, что реально записано, иначе
       * Lighting_SetMarkerPercent(0) сочтёт запись лишней и не выполнит её. */
      marker_compare = PWM_PERIOD;
    }

    if (drv_i[i].trips < 0xFFFF) {
      drv_i[i].trips++;
    }
    drv_i[i].head  = 0;
    drv_i[i].count = 0;
  }
}

/* ========================================================================== */
void Lighting_GetCurrentStatus(uint8_t index, DrvCurrentStatus *out)
{
  if (index >= DRV_COUNT || out == 0) {
    return;
  }

  out->ma_meas    = (uint16_t)((drv_i[index].ma_meas > 0xFFFFu)
                               ? 0xFFFFu : drv_i[index].ma_meas);
  out->zero_adc   = (uint16_t)drv_i[index].zero_adc;
  out->ceiling_ma = (uint16_t)((drv_i[index].ceiling_ma > 0xFFFFu)
                               ? 0xFFFFu : drv_i[index].ceiling_ma);
  out->oc_peak_ma = (uint16_t)(drv_i[index].oc_enabled
                               ? drv_i[index].oc_peak_ma : 0u);
  out->zero_valid = drv_i[index].zero_valid;
  out->oc_enabled = drv_i[index].oc_enabled;
  out->saturated  = drv_i[index].saturated;
  out->fault      = (uint8_t)((index == 0) ? bar_fault : marker_fault);
  out->trips      = drv_i[index].trips;
}
