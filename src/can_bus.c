/**
  ******************************************************************************
  * @file    can_bus.c
  * @brief   Обмен по шине CAN — узел передней светотехники.
  *
  *          Приём (FIFO0) кадров ведущего (device_id 0x01):
  *            BASE_COMP — рычаги ведущего → реле указателей поворота.
  *          Расширенные (29-бит) кадры — трафик VESC, игнорируются.
  *
  *          Передача: периодический кадр BASE_COMP с состоянием собственных
  *          дискретных входов (телеметрия для диагностики).
  ******************************************************************************
  */

#include "can_bus.h"
#include "config.h"
#include "bsp.h"
#include "io.h"

/* Заголовки и буферы передачи/приёма. */
static CAN_TxHeaderTypeDef TxHeader_Std;
static uint8_t  TxData_Std[8];

volatile CAN_Debug_t can_debug = {0};
volatile uint32_t    can_last_control_tick = 0;

/* ========================================================================== */
void CanBus_Start(void)
{
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

uint32_t CanBus_GenerateStdId(uint8_t dev_id, uint8_t base_index,
                              uint8_t parameter_index)
{
  return ((uint32_t)dev_id << 8) | (uint32_t)(base_index + parameter_index);
}

void CanBus_SendStd(uint32_t std_id, uint8_t *data, uint8_t length)
{
  TxHeader_Std.StdId = std_id;
  TxHeader_Std.ExtId = 0x00;
  TxHeader_Std.IDE   = CAN_ID_STD;
  TxHeader_Std.RTR   = CAN_RTR_DATA;
  TxHeader_Std.DLC   = length;

  for (uint8_t i = 0; i < length; i++) {
    TxData_Std[i] = data[i];
  }
  for (uint8_t i = length; i < 8; i++) {
    TxData_Std[i] = 0x00;
  }

  uint32_t mailbox;
  HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(&hcan, &TxHeader_Std, TxData_Std, &mailbox);

  switch (result) {
    case HAL_OK:
      can_debug.hal_ok++;
      break;
    case HAL_ERROR:
      can_debug.hal_error++;
      can_debug.last_error_code = result;
      can_debug.last_error_mailbox = mailbox;
      break;
    case HAL_BUSY:
      can_debug.hal_busy++;
      can_debug.last_error_code = result;
      break;
    case HAL_TIMEOUT:
      can_debug.hal_timeout++;
      can_debug.last_error_code = result;
      break;
  }
}

/* --------------------------------------------------------------------------
 * Периодическая отправка своего состояния (только кадр comp — у переднего
 * узла нет ни VESC, ни селектора, ни собственных команд управления).
 * -------------------------------------------------------------------------- */
void CanBus_TxTask(void)
{
  uint8_t data_comp[8];
  for (uint8_t i = 0; i < 8; i++) {
    data_comp[i] = IO_ReadPin(comp[i]);
  }
  CanBus_SendStd(CanBus_GenerateStdId(device_id, BASE_COMP, COMP_COUNT), data_comp, 8);
}

/* --------------------------------------------------------------------------
 * Приём кадров ведущего.
 * -------------------------------------------------------------------------- */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];

  if (HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
    return;
  }
  if (RxHeader.IDE == CAN_ID_EXT) {
    return;  /* расширенные кадры — обмен ведущего с VESC, не наш трафик */
  }

  uint32_t stdid           = RxHeader.StdId;
  uint8_t  std_device_id   = (stdid >> 8) & 0xFF;
  uint8_t  parameter_index = stdid & 0xFF;

  if (std_device_id != MASTER_DEVICE_ID) {
    return;
  }

  /* Рычаги ведущего -> передние указатели поворота: comp[0] — левый,
   * comp[1] — правый (та же семантика кадра, что у заднего узла). Оба нажаты
   * — стоп-сигнал: его показывает задний узел, спереди поворотники гаснут.   */
  if (parameter_index == BASE_COMP + COMP_COUNT) {
    uint8_t left_turn  = RxData[0];
    uint8_t right_turn = RxData[1];

    if (left_turn && right_turn) {
      IO_RelayOff(RELAY_TURN_LEFT);
      IO_RelayOff(RELAY_TURN_RIGHT);
    } else {
      if (left_turn)  { IO_RelayOn(RELAY_TURN_LEFT);  } else { IO_RelayOff(RELAY_TURN_LEFT);  }
      if (right_turn) { IO_RelayOn(RELAY_TURN_RIGHT); } else { IO_RelayOff(RELAY_TURN_RIGHT); }
    }
    can_last_control_tick = HAL_GetTick();
  }
}

/* --------------------------------------------------------------------------
 * Защита от потери связи с ведущим.
 * -------------------------------------------------------------------------- */
void CanBus_CheckTimeout(void)
{
  if ((HAL_GetTick() - can_last_control_tick) > CAN_CONTROL_TIMEOUT_MS) {
    IO_RelayOff(RELAY_TURN_LEFT);
    IO_RelayOff(RELAY_TURN_RIGHT);
  }
}
