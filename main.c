/*
	Copyright 2020 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC BMS bootloader.

	The VESC BMS bootloader is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC BMS bootloader is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "ch.h"
#include "hal.h"
#include "stm32l4xx_hal_conf.h"
#include "buffer.h"
#include "crc.h"

/*
 * Settings
 */
#define FLASH_PAGE_MAIN_APP			0
#define FLASH_PAGE_NEW_APP			256
#define FLASH_PAGE_BOOTLOADER		316
#define FLASH_BANK_MAIN_APP			FLASH_BANK_1
#define FLASH_BANK_NEW_APP			FLASH_BANK_2
#define FLASH_BANK_BOOTLOADER		FLASH_BANK_2
#define FLASH_ADDRESS_MAIN_APP		0x08000000
#define FLASH_ADDRESS_NEW_APP		0x08020000
#define FLASH_ADDRESS_BOOTLOADER	0x0803E000
#define FLASH_PAGES_MAIN_APP		60
#define FLASH_PAGES_BOOTLOADER		4
#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE				2048
#endif
#define MAX_SIZE_MAIN_APP			(FLASH_PAGES_MAIN_APP * FLASH_PAGE_SIZE)

#define LED_GREEN					0
#define LED_RED						1
#define LINE_LED_RED				PAL_LINE(GPIOA, 9)
#define LINE_LED_GREEN				PAL_LINE(GPIOA, 10)
#define LED_OFF(led)				//palClearLine(led)
#define LED_ON(led)					//palSetLine(led)
#define LED_TOGGLE(led)				//palToggleLine(led)

// Private functions
static void exit_bootloader(void);
static int16_t erase_app(void);
static int16_t write_app_data(uint32_t offset, uint8_t *data, uint32_t len);
void blink_led(int led, int blinks);

static void exit_bootloader(void) {
	NVIC_SystemReset();
}

static int16_t erase_app(void) {
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

	FLASH_EraseInitTypeDef eType;
	eType.TypeErase = FLASH_TYPEERASE_PAGES;
	eType.Banks = FLASH_BANK_MAIN_APP;
	eType.Page = FLASH_PAGE_MAIN_APP;
	eType.NbPages = FLASH_PAGES_MAIN_APP;

	uint32_t res = 0;
	return HAL_FLASHEx_Erase(&eType, &res);
}

static int16_t write_app_data(uint32_t offset, uint8_t *data, uint32_t len) {
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

	for (uint32_t i = 0;i < len;i += 8) {
		uint64_t dword =
				((uint64_t) data[i + 7]) << 56 |
				((uint64_t) data[i + 6]) << 48 |
				((uint64_t) data[i + 5]) << 40 |
				((uint64_t) data[i + 4]) << 32 |
				((uint64_t) data[i + 3]) << 24 |
				((uint64_t) data[i + 2]) << 16 |
				((uint64_t) data[i + 1]) << 8 |
				((uint64_t) data[i + 0]) << 0;

		int16_t res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_ADDRESS_MAIN_APP + offset + i, dword);
		if (res != HAL_OK) {
			return res;
		}
	}

	return HAL_OK;
}

static int16_t write_new_app(uint8_t *addr) {
	int32_t ind = 0;
	uint32_t size = buffer_get_uint32(addr, &ind);
	uint16_t crc_app = buffer_get_uint16(addr, &ind);

	if (size > MAX_SIZE_MAIN_APP) {
		return -1;
	}

	if (size == 0) {
		return -2;
	}

	uint16_t crc_calc = crc16(addr + ind, size);
	if (crc_calc != crc_app) {
		return -3;
	}

	int16_t res = erase_app();
	if (res != HAL_OK) {
		return res;
	}

	// Add 7 in case data is not a multiple of 8 bytes
	size += 7;
	if (size > MAX_SIZE_MAIN_APP) {
		size = MAX_SIZE_MAIN_APP;
	}

	// Pad to multiple of 8 bytes
	while ((size % 8) != 0) {
		size++;
	}

	return write_app_data(0, addr + ind, size);
}

void blink_led(int led, int blinks) {
	if (led == LED_GREEN) {
		for (int i = 0;i < blinks;i++) {
			LED_ON(LINE_LED_GREEN);
			chThdSleepMilliseconds(200);
			LED_OFF(LINE_LED_GREEN);
			chThdSleepMilliseconds(200);
		}
	} else if (led == LED_RED) {
		for (int i = 0;i < blinks;i++) {
			LED_ON(LINE_LED_RED);
			chThdSleepMilliseconds(200);
			LED_OFF(LINE_LED_RED);
			chThdSleepMilliseconds(200);
		}
	}
}

int main(void) {
	halInit();
	chSysInit();

	palSetLineMode(LINE_LED_RED, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_LED_GREEN, PAL_MODE_OUTPUT_PUSHPULL);

	LED_OFF(LINE_LED_RED);
	LED_OFF(LINE_LED_GREEN);

	// Up to 5 retries
	for (int i = 0;i < 5;i++) {
		int16_t res = write_new_app((uint8_t *)FLASH_ADDRESS_NEW_APP);

		if (res == -1) {
			blink_led(LED_RED, 3);
			exit_bootloader();
		} else if (res == -2) {
			blink_led(LED_RED, 4);
			exit_bootloader();
		} else if (res == -3) {
			blink_led(LED_RED, 5);
			exit_bootloader();
		} else if (res == HAL_OK) {
			blink_led(LED_GREEN, 3);
			exit_bootloader();
		}

		blink_led(LED_RED, 1);
	}

	// Erasing or writing to flash failed. A programmer is needed
	// to upload the firmware now.
	chThdSleepMilliseconds(2000);
	blink_led(LED_RED, 8);
	exit_bootloader();

	for (;;) {
		chThdSleepMilliseconds(1000);
		LED_TOGGLE(LINE_LED_RED);
	}
}
