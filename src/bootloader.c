/**
  ******************************************************************************
  * @file    bootloader.c
  * @brief   Реализация сопряжения с CAN-загрузчиком Katapult.
  ******************************************************************************
  */

#include "bootloader.h"
#include "main.h"
#include <stdint.h>

#ifdef USE_KATAPULT_BOOTLOADER

/* Адрес, с которого начинается приложение (сразу за 8 КБ загрузчика).
 * Должен совпадать с ORIGIN секции FLASH в ldscript/stm32f103c8_katapult.ld
 * и с параметром "8KiB bootloader" в menuconfig Katapult.               */
#define APP_FLASH_BASE   0x08002000UL

/* Начало flash — там лежит таблица векторов Katapult. */
#define BOOT_FLASH_BASE  0x08000000UL

/* Сигнатура запроса загрузчика (REQUEST_CANBOOT из src/canboot.h Katapult).
 * Значение зашито в загрузчике; при обновлении Katapult сверить заново.  */
#define KATAPULT_REQUEST_CODE  0x5984E3FA6CA1589BULL

void Bootloader_RelocateVectors(void)
{
  SCB->VTOR = APP_FLASH_BASE;
  __DSB();
}

void Bootloader_Request(void)
{
  /* Первое слово таблицы векторов загрузчика — его начальный указатель
   * стека; ровно по этому адресу Katapult ищет сигнатуру запроса (он
   * резервирует под неё последние 8 байт ОЗУ). Адрес читаем из таблицы, а
   * не зашиваем: так он останется верным при смене настроек Katapult.   */
  const uint32_t *boot_vectors = (const uint32_t *)BOOT_FLASH_BASE;
  volatile uint64_t *request   = (volatile uint64_t *)(uintptr_t)boot_vectors[0];

  __disable_irq();
  *request = KATAPULT_REQUEST_CODE;
  __DSB();

  NVIC_SystemReset();  /* не возвращается */
}

#else  /* сборка без загрузчика: прошивка стартует с 0x08000000 */

void Bootloader_RelocateVectors(void)
{
}

void Bootloader_Request(void)
{
}

#endif /* USE_KATAPULT_BOOTLOADER */
