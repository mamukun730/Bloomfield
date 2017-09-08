/*
 * system.hpp
 *
 *  Created on: 2017/09/03
 *      Author: mmkn
 */

#ifndef INCLUDE_SYSTEM_HPP_
#define INCLUDE_SYSTEM_HPP_

#include "flash/r_flash_api_rx_if.h"

#ifdef __cplusplus
extern "C" void R_INIT_StopModule();
extern "C" void R_INIT_NonExistentPort();
extern "C" void R_INIT_Clock();

extern "C" void R_FlashDataAreaAccess(uint16_t read_en_mask, uint16_t write_en_mask);
extern "C" uint8_t R_FlashDataAreaBlankCheck(uint32_t address, uint8_t size);
extern "C" uint8_t R_FlashErase(uint32_t block);
extern "C" uint8_t R_FlashEraseRange(uint32_t start_addr, uint32_t bytes);
extern "C" uint8_t R_FlashWrite(uint32_t flash_addr, uint32_t buffer_addr, uint16_t bytes);

extern "C" void Timer_CMT0();
extern "C" void Timer_CMT1();
#endif

#define RSPI_CS_GYRO PORTC.PODR.BIT.B4
#define RSPI_CS_LED PORT2.PODR.BIT.B7

namespace System {
	class SetUp {
		public:
			static void Clock();
			static void IO();
			static void Functions();
			static void StartCheck();
	};

	class ADC {
		public:
			static void Init();
			static void ChSelect(unsigned int ch);
			static void StartConvert();
			static float BatteryVoltage();
	};

	class Timer {
		public:
			enum Status {
				Disable = 0, Enable = 1
			};

			static void Init();
			static void Switch(unsigned char ch, unsigned char enable);
			static void CH0();
			static void CH1();
			static void wait_ms(uint32_t time);

			static uint32_t GetTimerElapsed();

		private:
			static bool wait_enable;
			volatile static uint32_t wait_count;
			volatile static uint32_t timer;
			static void wait_add();
	};

	class Flash {
		public:
			static void Init();
			static bool EraseAll();
			static bool EraseBlock(unsigned char block);
			static bool WriteWallData();
			static bool ReadWallData();
	};

	class RSPI {
		public:
			enum Type {
				Gyro, LED
			};

			enum RW {
				Write = 0, Read = 1
			};

			static void Init();
			static uint8_t ReadData(uint8_t id, uint8_t address);
			static void WriteData(uint8_t id, uint8_t address, uint8_t data);
			static void SelectDevice(uint8_t id, bool enable);
	};

	class SCI {
		public:
			static void Init();
			static void SendChar(char senddata[]);
			static void SendChar(char senddata);
	};

	class Interface {
		public:
			static void InitLED();
			static void SetLEDColor(uint8_t id, uint8_t red, uint8_t green, uint8_t blue);
	};
}

#endif /* INCLUDE_SYSTEM_HPP_ */
