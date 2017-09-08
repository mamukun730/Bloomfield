/*
 * system.cpp
 *
 *  Created on: 2017/09/03
 *      Author: mmkn
 */

#include "common.hpp"
#include "system.hpp"

#include "clock/r_init_clock.h"
#include "clock/r_init_non_existent_port.h"
#include "clock/r_init_stop_module.h"

namespace System {
	void SetUp::Clock() {
		/* ---- Disable maskable interrupts ---- */
		clrpsw_i();

		/* ---- Stopping the peripherals which start operations  ---- */
		R_INIT_StopModule();

		/* ---- Initialization of the non-existent ports ---- */
		R_INIT_NonExistentPort();

		/* ---- Initialization of the clock ---- */
		R_INIT_Clock();

		setpsw_i();
	}

	void SetUp::IO() {
		// PORTx.PDR:	ポート方向
		PORT0.PMR.BYTE = 0x00;
		PORT1.PMR.BYTE = 0x00;
		PORT2.PMR.BYTE = 0x00;
		PORT3.PMR.BYTE = 0x00;
		PORT4.PMR.BYTE = 0x00;
		PORT5.PMR.BYTE = 0x00;
		PORTA.PMR.BYTE = 0x00;
		PORTB.PMR.BYTE = 0x00;
		PORTC.PMR.BYTE = 0x00;
		PORTD.PMR.BYTE = 0x00;
		PORTE.PMR.BYTE = 0x00;

		MPC.PWPR.BIT.B0WI = 0;			// MPC書き込みプロテクト [p.720]
		MPC.PWPR.BIT.PFSWE = 1;

		// MPC.PxxPFS:	マルチファンクションピンコントローラ
		PORT0.PDR.BYTE = 0xFF;

		PORT1.PDR.BYTE = 0x0F;
		MPC.P14PFS.BIT.PSEL = 0x04;		// TCLKA [p.725]
		MPC.P15PFS.BIT.PSEL = 0x04;		// TCLKB
		MPC.P16PFS.BIT.PSEL = 0x04;		// TCLKC
		MPC.P17PFS.BIT.PSEL = 0x04;		// TCLKD

		PORT2.PDR.BYTE = 0xFF;
		MPC.P26PFS.BIT.PSEL = 0x0A;		// TXD1 [p.727]

		PORT3.PDR.BYTE = 0x3E;
		MPC.P30PFS.BIT.PSEL = 0x0A;		// RXD1 [p.729]

		PORT4.PDR.BYTE = 0xA0;
		MPC.P40PFS.BIT.ASEL = 1;
		MPC.P41PFS.BIT.ASEL = 1;
		MPC.P42PFS.BIT.ASEL = 1;
		MPC.P43PFS.BIT.ASEL = 1;
		MPC.P44PFS.BIT.ASEL = 1;
		MPC.P46PFS.BIT.ASEL = 1;

		PORT5.PDR.BYTE = 0xFF;
		PORT6.PDR.BYTE = 0xFF;
		PORT7.PDR.BYTE = 0xFF;
		PORT8.PDR.BYTE = 0xFF;
		PORT9.PDR.BYTE = 0xFF;
		PORTA.PDR.BYTE = 0xFC;
		PORTB.PDR.BYTE = 0xBF;

		PORTC.PDR.BYTE = 0x7B;			// SSLA0 GPIOで代用
		MPC.PC5PFS.BIT.PSEL = 0x0D;		// RSPCKA [p.744]
		MPC.PC6PFS.BIT.PSEL = 0x0D;		// MOSIA
		MPC.PC7PFS.BIT.PSEL = 0x0D;		// MISOA

		PORTD.PDR.BYTE = 0xFF;

		PORTE.PDR.BYTE = 0xFC;
		MPC.PE0PFS.BIT.ASEL = 1;
		MPC.PE1PFS.BIT.ASEL = 1;
		MPC.PE5PFS.BIT.PSEL = 0x02;		// MTIOC2B [p.747]

		PORTF.PDR.BYTE = 0xFF;
		PORTG.PDR.BYTE = 0xFF;
		PORTJ.PDR.BYTE = 0xFF;

		MPC.PWPR.BIT.PFSWE = 0;
		MPC.PWPR.BIT.B0WI = 1;

		// PORTx.PMR:	汎用入出力[0]? 周辺機能[1]?
		PORT0.PMR.BYTE = 0x00;
		PORT1.PMR.BYTE = 0xF0;			// 0B 1111 0000
		PORT2.PMR.BYTE = 0x40;			// 0B 0100 0000
		PORT3.PMR.BYTE = 0x01;			// 0B 0000 0001
		PORT4.PMR.BYTE = 0x00;			// 0B 0000 0000
		PORT5.PMR.BYTE = 0x00;
		PORT6.PMR.BYTE = 0x00;
		PORT7.PMR.BYTE = 0x00;
		PORT8.PMR.BYTE = 0x00;
		PORT9.PMR.BYTE = 0x00;
		PORTA.PMR.BYTE = 0x00;
		PORTB.PMR.BYTE = 0x00;
		PORTC.PMR.BYTE = 0xE0;			// 0B 1110 0000
		PORTD.PMR.BYTE = 0x00;
		PORTE.PMR.BYTE = 0x20;			// 0B 0010 0000
		PORTF.PMR.BYTE = 0x00;
		PORTG.PMR.BYTE = 0x00;
		PORTJ.PMR.BYTE = 0x00;
	}

	void SetUp::Functions() {
		ADC::Init();
		Timer::Init();
		Flash::Init();
		RSPI::Init();
		SCI::Init();

		PWM::Buzzer::Init();
		PWM::Motor::Init();
	}

	void SetUp::StartCheck() {
		char senddata[128];
		uint8_t address = 0;
		bool debug = true, operation = true;
		float batt_voltage = 0.0;

		sprintf(senddata, "\n*** StartUp Check ***\n\n");
		SCI::SendChar(senddata);

		batt_voltage = ADC::BatteryVoltage();
		sprintf(senddata, "Battery:\t\t%f [V]\n", batt_voltage);
		SCI::SendChar(senddata);

		if (batt_voltage < BATT_VOLTAGE_ERROR) {
			sprintf(senddata, "!! Low Battery Voltage !!\n");
			SCI::SendChar(senddata);

			while(1) {
				System::Interface::SetLEDColor(0, 255, 0, 0);
				System::Timer::wait_ms(500);
				System::Interface::SetLEDColor(0, 0, 0, 0);
				System::Timer::wait_ms(500);
			}
		} else {
			address = System::RSPI::ReadData(System::RSPI::Gyro, RSPI_ADDRESS_GYRO_DEVICEID);

			sprintf(senddata, "Gyro Device ID:\t\t0x%x\n", address);
			SCI::SendChar(senddata);

			if (address != RSPI_DATA_GYRO_DEVICEID) {
				sprintf(senddata, "!! Gyro Communicate Error !!\n");
				SCI::SendChar(senddata);

				while (1) {
					System::Interface::SetLEDColor(0, 255, 255, 0);
					System::Timer::wait_ms(500);
					System::Interface::SetLEDColor(0, 0, 0, 0);
					System::Timer::wait_ms(500);
				}
			}
		}
	}

// ADConverter
	void ADC::Init() {
		SYSTEM.PRCR.WORD = 0xA502;
		SYSTEM.MSTPCRA.BIT.MSTPA17 = 0;	// スタンバイ解除
		SYSTEM.PRCR.WORD = 0xA500;

		S12AD.ADCSR.BIT.ADST = 0;		// AD変換停止
		S12AD.ADCSR.BIT.ADCS = 0;		// シングルスキャン
		S12AD.ADCSR.BIT.CKS = 3;
		S12AD.ADEXICR.WORD = 0x0000;
		S12AD.ADANS0.BIT.ANS0 = 0x00;	// 対象ch [0-7ch]
	}

	void ADC::ChSelect(unsigned int ch) {
		S12AD.ADANS0.BIT.ANS0 = ch;
	}

	void ADC::StartConvert() {
		S12AD.ADCSR.BIT.ADST = 1;

		for (volatile unsigned int i = 0; i < 10; i++);
		while (S12AD.ADCSR.BIT.ADST == 1);

		S12AD.ADCSR.BIT.ADST = 0;
	}

	float ADC::BatteryVoltage() {
		unsigned int addata = 0;
		ADC::ChSelect(0x01);
		ADC::StartConvert();

		addata = S12AD.ADDR0;

		return (((float)addata * 3.3) / (4095.0 * 0.6500));
	}

// CMTimer
	bool Timer::wait_enable = false;
	volatile uint32_t Timer::wait_count = 0;
	volatile uint32_t Timer::timer = 0;

	void Timer::Init() {
		SYSTEM.PRCR.WORD = 0xA502;
		//SYSTEM.MSTPCRA.BIT.MSTPA14 = 0;	// CMT2,CMT3
		SYSTEM.MSTPCRA.BIT.MSTPA15 = 0;		// CMT0,CMT1
		SYSTEM.PRCR.WORD = 0xA500;

		CMT.CMSTR0.WORD = 0x00;
		CMT.CMSTR1.WORD = 0x00;

		CMT0.CMCR.BIT.CKS = 2;				// PCLK / 128
		CMT0.CMCR.BIT.CMIE = 1;
		CMT0.CMCOR = ((SYSTEM_CLOCK / (128 * 1000)) * CMT0_INTERVAL) - 1;

		CMT1.CMCR.BIT.CKS = 0;				// PCLK / 8
		CMT1.CMCR.BIT.CMIE = 1;
		CMT1.CMCOR = ((SYSTEM_CLOCK / (8 * 1000 * 1000)) * CMT1_INTERVAL) - 1;

		IPR(CMT0, CMI0) = 12;
		IEN(CMT0, CMI0) = 1;

		IPR(CMT1, CMI1) = 1;
		IEN(CMT1, CMI1) = 1;
	}

	void Timer::Switch(unsigned char ch, unsigned char enable) {
		switch(ch) {
			case 0:
				CMT.CMSTR0.BIT.STR0 = enable;
				break;

			case 1:
				CMT.CMSTR0.BIT.STR1 = enable;
				break;

			default:
				CMT.CMSTR0.BIT.STR0 = 0;
				CMT.CMSTR0.BIT.STR1 = 0;
				break;
		}
	}

	void Timer::wait_ms(uint32_t time) {
		Timer::wait_count = 0;
		Timer::wait_enable = true;

		while(Timer::wait_count < time);

		Timer::wait_count = 0;
		Timer::wait_enable = false;
	}

	void Timer::wait_add() {
		if (Timer::wait_enable) {
			Timer::wait_count++;
		}
	}

	uint32_t Timer::GetTimerElapsed() {
		return timer;
	}

	void Timer::CH0() {
//		static volatile unsigned int cnt = 0, log_cnt = 0;
//
		wait_add();
//		Mystat::Status::CheckMouseVelocity();
//
//		if (Mystat::Status::CheckExecuteFlag()) {
//			ADC::Sensor::SetValue(Mystat::Status::GetTravelDir());
//			Mystat::Status::UpdateStatusValue();
//			Mystat::Status::DetectWallEdge();
//			PWM::Motor::SetDuty();
//			PWM::Fan::SetDuty();
//
//			if (RECORD_LOG && (log_cnt < (LOG_SIZE - 1))) {
//				log_actual1[log_cnt] = Mystat::Status::GetAVelocity();
//				log_target1[log_cnt] = Mystat::Status::GetActualAVelocity();
////				log_actual2[log_cnt] = ADC::Sensor::GetValue(ADC::Sensor::RS);
////				log_target2[log_cnt] = ADC::Sensor::GetValue(ADC::Sensor::LS);
//				log_cnt++;
//			} else {
//				log_cnt = 0;
//				RECORD_LOG = false;
//			}
//
			timer++;
//		} else {
//			Mystat::Status::SetBatteryVoltage();
//
//			if (Interface::Switch::GetWaitingFlag()) {
//				if ((ADC::Sensor::GetValue(ADC::Sensor::LF) > WALL_THRESHOLD_F_LF) && (ADC::Sensor::GetValue(ADC::Sensor::RF) > WALL_THRESHOLD_F_RF)) {
//					Interface::LED::SetColor(Interface::LED::Blue, Interface::LED::Right);
//					cnt++;
//
//					if (cnt > 2000) {
//						cnt = 0;
//						Interface::Switch::SetWaitingFlag(false);
//					}
//				} else {
//					Interface::LED::SetColor(Interface::LED::Red, Interface::LED::Right);
//					cnt = 0;
//				}
//
//				ADC::Sensor::SetValue(false);
//			} else {
//				cnt = 0;
//
//				for (volatile unsigned int i = 0; i < 1000; i++);
//			}
//
//			PWM::Fan::SetDuty();
//			timer = 0;
//		}
	}

	void Timer::CH1() {
//		Interface::LED::CtrlDuty();
	}

// Flash
	void Flash::Init() {
		R_FlashDataAreaAccess(0xFFFF, 0xFFFF);
	}

/*	bool Flash::EraseAll() {
		uint32_t loop, ret;
		bool result = true;

		for(loop = BLOCK_DB0; loop < BLOCK_DB15; loop++) {
		    ret = R_FlashErase(loop);
		    if (FLASH_SUCCESS != ret) {
		        result = false;
		        break;
		    }
		}

		return result;
	}

	bool Flash::EraseBlock(unsigned char block) {
		uint32_t ret, start_addr;
		bool result = true;

		if ((block >= BLOCK_DB0) && (block <= BLOCK_DB15)) {
			start_addr = g_flash_BlockAddresses[block];
	//	    ret = R_FlashErase(block);
			ret = R_FlashEraseRange(start_addr, 2048);
		    if (FLASH_SUCCESS != ret) {
		        result = false;
		    }
		} else {
			result = false;
		}

		return result;
	}

	bool Flash::WriteWallData() {
		unsigned char wall[16][16], ret;
		uint32_t flash_addr = g_flash_BlockAddresses[BLOCK_DB0];
		uint32_t buffer_addr = (uint32_t)wall;
		uint16_t bytes = sizeof(wall);
		bool result = true;

		for (unsigned char j = 0; j < MAPSIZE_Y; j++) {
		    for (unsigned char i = 0; i < MAPSIZE_X; i++) {
		    	wall[i][j] = Mystat::Map::ReadWallData(i, j);
		    }
		}

		ret = R_FlashWrite(flash_addr, buffer_addr, bytes);
		if (FLASH_SUCCESS != ret) {
		    result = false;
		}

		return result;
	}

	bool Flash::ReadWallData() {
		bool result = true;
		unsigned char wall;
		uint32_t address;

		address = g_flash_BlockAddresses[BLOCK_DB0];

		for (unsigned char j = 0; j < MAPSIZE_Y; j++) {
		    for (unsigned char i = 0; i < MAPSIZE_X; i++) {
		    	wall = *(unsigned char *)(address + (MAPSIZE_X * i) + j);
		    	Mystat::Map::WriteWallData(i, j, wall);
		    }
		}

		return result;
	}*/

// RSPI
	void RSPI::Init() {
		SYSTEM.PRCR.WORD = 0xA502;
		MSTP(RSPI0) = 0;
		SYSTEM.PRCR.WORD = 0xA500;

		RSPI0.SPCR.BYTE = 0x00;
		RSPI0.SPPCR.BYTE = 0x00;
		RSPI0.SPBR = 5;					// 1.0Mbps
		RSPI0.SPDCR.BYTE = 0x00;		// 1フレーム送信, ワードアクセス
		RSPI0.SSLND.BYTE = 0x00;  		// スレーブセレクトネゲート1RSPCK遅延(SPCMDn.SLNDEN=0なら無効)
		RSPI0.SPND.BYTE = 0x00;  		// 次アクセス1RSPCK＋2PCLK遅延(SPCMDn.SPNDEN=0なら無効)
		RSPI0.SPCR2.BYTE = 0x00; 		// 送信・受信パリティなし，アイドル割り込み禁止
		RSPI0.SSLP.BYTE = 0x00;  		// スレーブセレクトSSL0～SSL3信号は0アクティブ
		RSPI0.SPSR.BYTE = 0xA0;  		// ステータスレジスタクリア
		RSPI0.SPSCR.BYTE = 0x00; 		// シーケンス長1でSPCMD0のみ参照
		RSPI0.SPCMD0.WORD = 0x48B;		// 0B 0000 0100 1000 1011

		RSPI0.SPCR.BYTE = 0x08;			// SPI動作, マスタモード
	}

	uint8_t RSPI::ReadData(uint8_t id, uint8_t address) {
		uint8_t transmit_data = (address | (RSPI::Read << 7)), dummy = 0;

		// Enable RSPI Function
		RSPI0.SPCR.BIT.SPE = 1;
		RSPI0.SPSR.BYTE = 0xA0;
		RSPI::SelectDevice(id, true);
		RSPI0.SPDR.WORD.H = transmit_data;

		// Wait Transfer End
		RSPI0.SPSR.BYTE; /* TNRXA147AJ */
		while(RSPI0.SPSR.BIT.IDLNF);

		// Start 2nd Transfer
		dummy = RSPI0.SPDR.WORD.H;
		RSPI0.SPDR.WORD.H = 0x00; // dummy

		// Wait Transfer End
		RSPI0.SPSR.BYTE;
		while(RSPI0.SPSR.BIT.IDLNF);

		// Read 2nd Data
		transmit_data = RSPI0.SPDR.WORD.H;
		RSPI0.SPCR.BIT.SPE = 0;
		RSPI::SelectDevice(id, false);

		return transmit_data;
	}

	void RSPI::WriteData(uint8_t id, uint8_t address, uint8_t data) {
		uint8_t transmit_data = (address | (RSPI::Write << 7)), dummy = 0;

		// Enable RSPI Function
		RSPI0.SPCR.BIT.SPE = 1;
		RSPI0.SPSR.BYTE = 0xA0;
		RSPI::SelectDevice(id, true);
		RSPI0.SPDR.WORD.H = transmit_data;

		// Wait Transfer End
		RSPI0.SPSR.BYTE; /* TNRXA147AJ */
		while(RSPI0.SPSR.BIT.IDLNF);

		// Start 2nd Transfer
		dummy = RSPI0.SPDR.WORD.H;
		RSPI0.SPDR.WORD.H = data;

		// Wait Transfer End
		RSPI0.SPSR.BYTE;
		while(RSPI0.SPSR.BIT.IDLNF);

		// Discard 2nd Data
		dummy = RSPI0.SPDR.WORD.H;
		RSPI0.SPCR.BIT.SPE = 0;
		RSPI::SelectDevice(id, false);
	}

	void RSPI::SelectDevice(uint8_t id, bool enable) {
		RSPI_CS_GYRO = 1;
		RSPI_CS_LED = 1;

		if (enable) {
			switch (id) {
				case RSPI::Gyro:
					RSPI_CS_GYRO = 0;
					break;

				case RSPI::LED:
					RSPI_CS_LED = 0;
					break;
			}
		}
	}

	void SCI::Init() {
		SYSTEM.PRCR.WORD = 0xA502;
		SYSTEM.MSTPCRB.BIT.MSTPB30 = 0;	// SCI1
		SYSTEM.PRCR.WORD = 0xA500;

		// SCI1
		SCI1.SCR.BYTE = 0x00;
		SCI1.SMR.BIT.CKS = 0;	// PCLK / 1
		SCI1.SMR.BIT.CM = 0;	// 調歩同期
		SCI1.SMR.BIT.CHR = 0;	// 8bit
		SCI1.SMR.BIT.PE = 0;	// パリティ無し
		SCI1.SMR.BIT.STOP = 0;	// 1bit
		SCI1.BRR = 38;			// 38400bps

		SCI1.SCR.BYTE |= 0x30;	// TE/RE enable
	}

	void SCI::SendChar(char senddata[]) {
		for (unsigned int cnt_sd = 0; senddata[cnt_sd] != '\0'; cnt_sd++) {
			while(!SCI1.SSR.BIT.TEND);
			SCI1.TDR = senddata[cnt_sd];
		}
	}

	void SCI::SendChar(char senddata) {
		while(!SCI1.SSR.BIT.TEND);
		SCI1.TDR = senddata;
	}

	void Interface::InitLED() {
		// CS_Run Enable
		System::RSPI::WriteData(System::RSPI::LED, RSPI_ADDRESS_LED_CONFIG, RSPI_DATA_LED_ENABLECS);

		// Execute CS_Run
		RSPI_CS_LED = 0;
		System::Timer::wait_ms(20);
		RSPI_CS_LED = 1;
	}

	void Interface::SetLEDColor(uint8_t id, uint8_t red, uint8_t green, uint8_t blue) {
		uint8_t data_red = 255 - red, data_green = 255 - green, data_blue = 255 - blue;

		// Output:0x00 => Low-logic output
		// Output:0x01 => High-logic output
		if (data_red < 2) {
			data_red = 2;
		}

		if (data_green < 2) {
			data_green = 2;
		}

		if (data_blue < 2) {
			data_blue = 2;
		}

		switch (id) {
			case 0:
				RSPI::WriteData(RSPI::LED, RSPI_ADDRESS_LED_RED1, data_red);
				RSPI::WriteData(RSPI::LED, RSPI_ADDRESS_LED_GREEN1, data_green);
				RSPI::WriteData(RSPI::LED, RSPI_ADDRESS_LED_BLUE1, data_blue);
				break;

			case 1:
				RSPI::WriteData(RSPI::LED, RSPI_ADDRESS_LED_RED2, data_red);
				RSPI::WriteData(RSPI::LED, RSPI_ADDRESS_LED_GREEN2, data_green);
				RSPI::WriteData(RSPI::LED, RSPI_ADDRESS_LED_BLUE2, data_blue);
				break;
		}
	}

/*
	void Check::StartUp() {
		char senddata[128];
		unsigned char address = 0;
		bool debug = true, operation = true;
		float batt_voltage = 0.0;

		Interrupt::CMTimer::Switch(0, Interrupt::CMTimer::Status::Disable);

		sprintf(senddata, "\n*** StartUp Check ***\n\n");
		Comm::SCI::SendChar(senddata);
		memset(senddata, 0, 128);

		batt_voltage = ADC::Get::BatteryVoltage();
		sprintf(senddata, "Battery:\t\t%f [V]\n", batt_voltage);
		Comm::SCI::SendChar(senddata);
		memset(senddata, 0, 128);

		if (batt_voltage < BATT_VOLTAGE_ERROR) {
			sprintf(senddata, "!! Low Battery Voltage !!\n");
			Comm::SCI::SendChar(senddata);

			while (1) {
				Interrupt::CMTimer::Switch(0, Interrupt::CMTimer::Status::Enable);

				Interface::LED::SetColor(Interface::LED::LED_Color::Red, Interface::LED::Side::Both);
				Interrupt::CMTimer::wait_ms(500);
				Interface::LED::SetColor(Interface::LED::LED_Color::None, Interface::LED::Side::Both);
				Interrupt::CMTimer::wait_ms(500);
			}
		} else if (batt_voltage < BATT_VOLTAGE_WARNING) {
			Interface::LED::SetColor(Interface::LED::LED_Color::Yellow, Interface::LED::Side::Left);
		} else {
			Interface::LED::SetColor(Interface::LED::LED_Color::Green, Interface::LED::Side::Left);
		}

		address = Comm::RSPI::ReadData(ADDRESS_WHO_AM_I);

		sprintf(senddata, "Gyro Device ID:\t\t%d\n", address);
		Comm::SCI::SendChar(senddata);
		memset(senddata, 0, 128);

		if (address != DATA_WHO_AM_I) {
			sprintf(senddata, "!! Gyro Communicate Error !!\n");
			Comm::SCI::SendChar(senddata);

			while (1) {
				Interrupt::CMTimer::Switch(0, Interrupt::CMTimer::Status::Enable);

				Interface::LED::SetColor(Interface::LED::LED_Color::Yellow, Interface::LED::Side::Both);
				Interrupt::CMTimer::wait_ms(500);
				Interface::LED::SetColor(Interface::LED::LED_Color::None, Interface::LED::Side::Both);
				Interrupt::CMTimer::wait_ms(500);
			}
		}

		sprintf(SCI_SendChar, "Gyro PWR_MGMT1:\t\t%d", Comm::RSPI::ReadData(ADDRESS_PWR_MGMT1));
		Comm::SCI::SendChar(SCI_SendChar);

		Comm::RSPI::WriteData(ADDRESS_PWR_MGMT1, DATA_PWR_MGMT1);
		Comm::RSPI::WriteData(ADDRESS_PWR_MGMT2, DATA_PWR_MGMT2);
		Comm::RSPI::WriteData(ADDRESS_USER_CTRL, DATA_USER_CTRL);
		Comm::RSPI::WriteData(ADDRESS_SIGNAL_PATH_RESET, DATA_SIGNAL_PATH_RESET);
		Comm::RSPI::WriteData(ADDRESS_CONFIG, DATA_CONFIG);
		Comm::RSPI::WriteData(ADDRESS_GYRO_CONFIG, DATA_GYRO_CONFIG);

		sprintf(SCI_SendChar, " => %d\n", Comm::RSPI::ReadData(ADDRESS_PWR_MGMT1));
		Comm::SCI::SendChar(SCI_SendChar);

		sprintf(SCI_SendChar, "Gyro PWR_MGMT2:\t\t%d\n", Comm::RSPI::ReadData(ADDRESS_PWR_MGMT2));
		Comm::SCI::SendChar(SCI_SendChar);

		sprintf(SCI_SendChar, "Gyro Config:\t\t%d\n", Comm::RSPI::ReadData(ADDRESS_GYRO_CONFIG));
		Comm::SCI::SendChar(SCI_SendChar);

		Comm::RSPI::Init_HighSpeedRead();

		// Start X, Y
		sprintf(SCI_SendChar, "Start:\t\t\t%d, %d\n", START_X, START_Y);
		Comm::SCI::SendChar();

		// Goal X, Y
		sprintf(SCI_SendChar, "Goal:\t\t\t%d, %d", GOAL_X, GOAL_Y);
		Comm::SCI::SendChar();

		// FullSizeGoal Enable?
		if (ENABLE_FULLGOAL) {
			sprintf(SCI_SendChar, " [FullSize Section]\n", GOAL_X, GOAL_Y);
		} else {
			sprintf(SCI_SendChar, " [Single Section]\n", GOAL_X, GOAL_Y);
		}

		Comm::SCI::SendChar();

		// CtrlTarget
		sprintf(SCI_SendChar, "CtrlTarget:\t\t%d, %d / %d, %d\n", WALL_CTRL_TARGET_F_LS, WALL_CTRL_TARGET_F_RS, WALL_CTRL_TARGET_R_LS, WALL_CTRL_TARGET_R_RS);
		Comm::SCI::SendChar();

		// CtrlThreshold
		sprintf(SCI_SendChar, "CtrlThreshold:\t\t%d, %d / %d, %d\n", WALL_CTRL_THRESHOLD_F_LS, WALL_CTRL_THRESHOLD_F_RS, WALL_CTRL_THRESHOLD_R_LS, WALL_CTRL_THRESHOLD_R_RS);
		Comm::SCI::SendChar();

		// WallThreshold
		sprintf(SCI_SendChar, "WallThreshold:\t\t%d, %d, %d, %d / %d, %d, %d, %d\n", WALL_THRESHOLD_F_LS, WALL_THRESHOLD_F_LF, WALL_THRESHOLD_F_RF, WALL_THRESHOLD_F_RS, WALL_THRESHOLD_R_LS, WALL_THRESHOLD_R_LF, WALL_THRESHOLD_R_RF, WALL_THRESHOLD_R_RS);
		Comm::SCI::SendChar();

		// WalloffThreshold
		sprintf(SCI_SendChar, "EdgeDetectThreshold:\t%d, %d / %d, %d\n", WALL_EDGE_THRESHOLD_F_LS, WALL_EDGE_THRESHOLD_F_RS, WALL_EDGE_THRESHOLD_R_LS, WALL_EDGE_THRESHOLD_R_RS);
		Comm::SCI::SendChar();

		// Map
		if (R_FlashDataAreaBlankCheck(BLOCK_DB0, BLANK_CHECK_ENTIRE_BLOCK) == FLASH_NOT_BLANK) {
			operation = Flash::ReadWallData();
			Mystat::Map::CalcStep(GOAL_X, GOAL_Y, false);

			sprintf(SCI_SendChar, "MapInfoOnFlash:\t\tYes\n");
			Comm::SCI::SendChar();
		} else {
			Mystat::Map::Init();

			sprintf(SCI_SendChar, "MapInfoOnFlash:\t\tNo\n");
			Comm::SCI::SendChar();
		}

		// InfoOutput End
		sprintf(SCI_SendChar, "\n");
		Comm::SCI::SendChar();

		// If CheckMode == 1
		// Sensor Data Loop
		if (SW_NEXT == 0) {
			Interface::LED::SetColor(Interface::LED::Purple, Interface::LED::Both);
			Interrupt::CMTimer::Switch(0, Interrupt::CMTimer::Status::Enable);
			Interrupt::CMTimer::wait_ms(2000);
			Interface::LED::SetColor(Interface::LED::None, Interface::LED::Both);

			while (debug) {
				Mystat::Status::SetExecuteFlag(false);
				Interface::Switch::SetWaitingFlag(true);
				Interface::Switch::SelectMode();
				Interface::LED::SetColor(Interface::LED::None, Interface::LED::Both);

				switch (Interface::Switch::GetExecuteMode()) {
					case 1:
					// GyroReference
					Comm::RSPI::SetGyroReference();
					sprintf(senddata, "Gyro Reference:\t\t%f\n", Comm::RSPI::GetGyroReference());
					Comm::SCI::SendChar(senddata);
					memset(senddata, 0, 128);

					while(1) {
						ADC::Sensor::SetValue(false);
						sprintf(senddata, "[Front]%d, %d, %d, %d\t\t", ADC::Sensor::GetValue(ADC::Sensor::SensorNum::LS), ADC::Sensor::GetValue(ADC::Sensor::SensorNum::LF), ADC::Sensor::GetValue(ADC::Sensor::SensorNum::RF), ADC::Sensor::GetValue(ADC::Sensor::SensorNum::RS));
						Comm::SCI::SendChar(senddata);
						memset(senddata, 0, 128);

						ADC::Sensor::SetValue(true);
						sprintf(senddata, "[Rear]%d, %d, %d, %d\t", ADC::Sensor::GetValue(ADC::Sensor::SensorNum::LS), ADC::Sensor::GetValue(ADC::Sensor::SensorNum::LF), ADC::Sensor::GetValue(ADC::Sensor::SensorNum::RF), ADC::Sensor::GetValue(ADC::Sensor::SensorNum::RS));
						Comm::SCI::SendChar(senddata);
						memset(senddata, 0, 128);

						sprintf(SCI_SendChar, " / Gyro: %f\n", Comm::RSPI::GetAngulerVelocity());
						Comm::SCI::SendChar(SCI_SendChar);

						Interrupt::CMTimer::wait_ms(100);

						if (SW_PREV == 0) {
							debug = false;
							break;
						}
					}

					break;

					case 2:
						Mystat::Map::SendData();
						break;

					case 3:
						if (Flash::EraseBlock(BLOCK_DB0)) {
							Mystat::Map::Init();
							Interface::LED::SetColor(Interface::LED::Green, Interface::LED::Both);
						} else {
							Interface::LED::SetColor(Interface::LED::Red, Interface::LED::Both);
						}

						Interrupt::CMTimer::wait_ms(1000);
						Interface::LED::SetColor(Interface::LED::None, Interface::LED::Both);
						break;

					default:
						debug = false;
						break;
				}

				Interrupt::CMTimer::wait_ms(1000);
			}
		}

		Interrupt::CMTimer::Switch(0, Interrupt::CMTimer::Status::Enable);
	}

	void Etc::StartAction() {
		unsigned char led = 0;
		unsigned int melody_num = ((ADC::Sensor::GetValue(ADC::Sensor::LS) + ADC::Sensor::GetValue(ADC::Sensor::LF) + ADC::Sensor::GetValue(ADC::Sensor::RF) + ADC::Sensor::GetValue(ADC::Sensor::RS)) % 7);

		if (Interface::Switch::GetExecuteMode() == MODE_NUMBER) {
			PWM::Buzzer::Melody_TE32_1();
		} else {
			switch (melody_num) {
				case 0:
					PWM::Buzzer::Melody_TE3_1();
					break;

				case 1:
					PWM::Buzzer::Melody_GK8_1();
					break;

				case 2:
					PWM::Buzzer::Melody_TE16_1();
					break;

				case 3:
					PWM::Buzzer::Melody_Shibuya_Otogi();
					break;

				case 4:
					PWM::Buzzer::Melody_TE26_1();
					break;

				case 5:
					PWM::Buzzer::Melody_TE10_1();
					break;

				case 6:
					PWM::Buzzer::Melody_Namiki();
					break;

				default:
					break;
			}
		}

//		led = Interface::LED::GetColor(Interface::LED::Left);
//		Interface::LED::SetColor(Interface::LED::None, Interface::LED::Right);
//
//		for (unsigned char cnt = 0; cnt < 2; cnt++) {
//			Interface::LED::SetColor(Interface::LED::None, Interface::LED::Left);
//			Interrupt::CMTimer::wait_ms(250);
//
//			Interface::LED::SetColor(led, Interface::LED::Left);
//			Interrupt::CMTimer::wait_ms(250);
//		}
	}
	*/
}

void Timer_CMT0() {
	System::Timer::CH0();
}

void Timer_CMT1() {
	System::Timer::CH1();
}
