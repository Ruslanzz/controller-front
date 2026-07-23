/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Точка входа и главный цикл — узел ПЕРЕДНЕЙ светотехники.
  *
  *  Узел включает габаритные огни при старте, управляет реле передних
  *  указателей поворота по CAN-командам ведущего (device_id 0x01) и
  *  светодиодной балкой на выходе L1 (DRV1): яркость 0..100 % задаётся
  *  потенциометром LA42DWQ-22 на входе ADC_IN_1 (управление реверсное),
  *  в крайнем положении потенциометра включается стробоскоп. Периодически
  *  шлёт телеметрию своих дискретных входов.
  *
  *  Модули: bsp, io, lighting, can_bus.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "bsp.h"
#include "config.h"
#include "io.h"
#include "lighting.h"
#include "can_bus.h"

/**
  * @brief  The application entry point.
  */
int main(void)
{
  BSP_Init();

  HAL_ADCEx_Calibration_Start(&hadc1);

  /* Вывести трансивер CAN из режима ожидания и запустить шину с приёмом. */
  HAL_GPIO_WritePin(GPIOA, CAN_STB_Pin, GPIO_PIN_RESET);
  CanBus_Start();

  /* Запуск ШИМ балки и исходное состояние моста DRV1 (балка погашена). */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  Lighting_Init();

  /* TIM1: база + канал CH3 — периодический тик отправки телеметрии. */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);

  /* Запуск непрерывного измерения АЦП по DMA (8 каналов): потенциометр
   * яркости (EXT_ADC_1), датчики тока (ACS724) и температуры. */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADS_RES_BUFFER, 8);

  /* Общее реле питания. */
  IO_RelayOn(RELAY_POWER);

  while (1)
  {
    CanBus_CheckTimeout();          /* потеря связи -> поворотники гаснут      */
    Lighting_CheckOvercurrent();    /* переток балки -> аварийное отключение   */
    Lighting_Update();              /* мигание поворотников + балка/стробоскоп */
  }
}

/* ==========================================================================
 * Колбэки и обработчики прерываний прикладного уровня.
 * ========================================================================== */

/** Прерывание сравнения TIM1. */
void TIM1_CC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}

/** Диспетчер событий Output Compare TIM1 по каналам. */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM1) {
    return;
  }

  switch (htim->Channel) {
    case HAL_TIM_ACTIVE_CHANNEL_3:
      /* Периодическая отправка телеметрии и перепланирование тика. */
      CanBus_TxTask();
      BSP_TimerAdvanceCompare(&htim1, TIM_CHANNEL_3, tim1_ch3_pulse);
      break;

    default:
      /* CH1/CH2/CH4 передним узлом не используются. */
      break;
  }
}
