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
		MPC.P40PFS.BIT.ASEL = 1;		// AN000
		MPC.P41PFS.BIT.ASEL = 1;		// AN001
		MPC.P42PFS.BIT.ASEL = 1;		// AN002
		MPC.P43PFS.BIT.ASEL = 1;		// AN003
		MPC.P44PFS.BIT.ASEL = 1;		// AN004
		MPC.P46PFS.BIT.ASEL = 1;		// AN006

		PORT5.PDR.BYTE = 0xFF;
		PORT6.PDR.BYTE = 0xFF;
		PORT7.PDR.BYTE = 0xFF;
		PORT8.PDR.BYTE = 0xFF;
		PORT9.PDR.BYTE = 0xFF;
		PORTA.PDR.BYTE = 0xFC;

		PORTB.PDR.BYTE = 0xBF;
		MPC.PB1PFS.BIT.PSEL = 0x01;		// MTIOC0C [p.740]
		MPC.PB3PFS.BIT.PSEL = 0x01;		// MTIOC0A

		PORTC.PDR.BYTE = 0x7B;			// SSLA0 GPIOで代用
		MPC.PC5PFS.BIT.PSEL = 0x0D;		// RSPCKA [p.744]
		MPC.PC6PFS.BIT.PSEL = 0x0D;		// MOSIA
		MPC.PC7PFS.BIT.PSEL = 0x0D;		// MISOA

		PORTD.PDR.BYTE = 0xFF;

		PORTE.PDR.BYTE = 0xFC;
		MPC.PE0PFS.BIT.ASEL = 1;		// AN008
		MPC.PE1PFS.BIT.ASEL = 1;		// AN009
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
		PORTB.PMR.BYTE = 0x0A;			// OB 0000 1010
		PORTC.PMR.BYTE = 0xE0;			// 0B 1110 0000
		PORTD.PMR.BYTE = 0x00;
		PORTE.PMR.BYTE = 0x20;			// 0B 0010 0000
		PORTF.PMR.BYTE = 0x00;
		PORTG.PMR.BYTE = 0x00;
		PORTJ.PMR.BYTE = 0x00;
	}

	void SetUp::Functions() {
		// Interface LEDは別にInit呼び出し(CMT利用)
		ADC::Init();
		Timer::Init();
		Flash::Init();
		RSPI::Init();
		SCI::Init();

		PWM::Buzzer::Init();
		PWM::Motor::Init();

		Interface::InitEncoder();
	}

	void SetUp::StartCheck() {
		char senddata[128];
		uint8_t address = 0;
		bool debug = true, operation = true;
		float batt_voltage = 0.0;

		sprintf(senddata, "\n*** StartUp Check ***\n\n");
		SCI::SendChar(senddata);

		batt_voltage = ADC::GetBatteryVoltage();
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

			sprintf(senddata, "Gyro Device ID:\t\t0x%2x\n", address);
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

			System::RSPI::WriteData(System::RSPI::Gyro, RSPI_ADDRESS_GYRO_PWR_MGMT1, RSPI_DATA_GYRO_PWR_MGMT1);
			System::RSPI::WriteData(System::RSPI::Gyro, RSPI_ADDRESS_GYRO_PWR_MGMT2, RSPI_DATA_GYRO_PWR_MGMT2);
			System::RSPI::WriteData(System::RSPI::Gyro, RSPI_ADDRESS_GYRO_SMPRT_DIV, RSPI_DATA_GYRO_SMPRT_DIV);
			System::RSPI::WriteData(System::RSPI::Gyro, RSPI_ADDRESS_GYRO_CONFIG, RSPI_DATA_GYRO_CONFIG);
			System::RSPI::WriteData(System::RSPI::Gyro, RSPI_ADDRESS_GYRO_GYROCONFIG, RSPI_DATA_GYRO_GYROCONFIG);

			sprintf(senddata, "Gyro PWR MGMT1:\t\t0x%2x\n", System::RSPI::ReadData(System::RSPI::Gyro, RSPI_ADDRESS_GYRO_PWR_MGMT1));
			SCI::SendChar(senddata);

			sprintf(senddata, "Gyro PWR MGMT2:\t\t0x%2x\n", System::RSPI::ReadData(System::RSPI::Gyro, RSPI_ADDRESS_GYRO_PWR_MGMT2));
			SCI::SendChar(senddata);
		}

		sprintf(senddata, "\n");
		SCI::SendChar(senddata);
	}

	void SetUp::CreateClass() {
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

	void ADC::ChSelect(uint16_t ch) {
		S12AD.ADANS0.BIT.ANS0 = ch;
	}

	void ADC::StartConvert() {
		S12AD.ADCSR.BIT.ADST = 1;

		for (volatile uint8_t i = 0; i < 10; i++);
		while (S12AD.ADCSR.BIT.ADST == 1);

		S12AD.ADCSR.BIT.ADST = 0;
	}

	float ADC::GetBatteryVoltage() {
		uint16_t addata = 0;
		ADC::ChSelect(0x01);
		ADC::StartConvert();

		addata = S12AD.ADDR0;

		return (((float)addata * 3.3) / (4095.0 * 0.6500));
	}

	void ADC::SetSensorValue() {
		uint16_t sensor_on[7], sensor_off[7];

		// Phase 1
		SENSOR_LED_LS = 1;
		SENSOR_LED_LC = 0;
		SENSOR_LED_LF = 0;
		SENSOR_LED_F = 0;
		SENSOR_LED_RF = 1;
		SENSOR_LED_RC = 0;
		SENSOR_LED_RS = 0;
		ADC::ChSelect(ADC_WALLSENSOR_P1);
		for (volatile uint16_t cnt = 0; cnt < SENSOR_LED_WAIT; cnt++);
		ADC::StartConvert();

		sensor_on[0]	= ADDR_SENSOR0;
		sensor_off[2]	= ADDR_SENSOR2;
		sensor_on[4]	= ADDR_SENSOR4;
		sensor_off[6]	= ADDR_SENSOR6;

		// Phase 2
		SENSOR_LED_LS = 0;
		SENSOR_LED_LC = 1;
		SENSOR_LED_LF = 0;
		SENSOR_LED_F = 0;
		SENSOR_LED_RF = 0;
		SENSOR_LED_RC = 1;
		SENSOR_LED_RS = 0;
		ADC::ChSelect(ADC_WALLSENSOR_P2);
		for (volatile uint16_t cnt = 0; cnt < SENSOR_LED_WAIT; cnt++);
		ADC::StartConvert();

		sensor_on[1]	= ADDR_SENSOR1;
		sensor_off[3]	= ADDR_SENSOR3;
		sensor_on[5]	= ADDR_SENSOR5;

		// Phase 3
		SENSOR_LED_LS = 0;
		SENSOR_LED_LC = 0;
		SENSOR_LED_LF = 1;
		SENSOR_LED_F = 0;
		SENSOR_LED_RF = 0;
		SENSOR_LED_RC = 0;
		SENSOR_LED_RS = 1;
		ADC::ChSelect(ADC_WALLSENSOR_P1);
		for (volatile uint16_t cnt = 0; cnt < SENSOR_LED_WAIT; cnt++);
		ADC::StartConvert();

		sensor_off[0]	= ADDR_SENSOR0;
		sensor_on[2]	= ADDR_SENSOR2;
		sensor_off[4]	= ADDR_SENSOR4;
		sensor_on[6]	= ADDR_SENSOR6;

		// Phase 4
		SENSOR_LED_LS = 0;
		SENSOR_LED_LC = 0;
		SENSOR_LED_LF = 0;
		SENSOR_LED_F = 1;
		SENSOR_LED_RF = 0;
		SENSOR_LED_RC = 0;
		SENSOR_LED_RS = 0;
		ADC::ChSelect(ADC_WALLSENSOR_P2);
		for (volatile uint16_t cnt = 0; cnt < SENSOR_LED_WAIT; cnt++);
		ADC::StartConvert();

		sensor_off[1]	= ADDR_SENSOR1;
		sensor_on[3]	= ADDR_SENSOR3;
		sensor_off[5]	= ADDR_SENSOR5;

		SENSOR_LED_LS = 0;
		SENSOR_LED_LC = 0;
		SENSOR_LED_LF = 0;
		SENSOR_LED_F = 0;
		SENSOR_LED_RF = 0;
		SENSOR_LED_RC = 0;
		SENSOR_LED_RS = 0;

		Status::Sensor::SetValue(sensor_on, sensor_off);
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
		log_cnt = 0;

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
		wait_add();
		ADC::SetSensorValue();

		if (ExecuteFlag.GetValue()) {
			Interface::SetEncoderValue();
			Status::Calc::RenewActualVelocity(false);
			Status::Calc::RenewTargetVelocity(false);
			Status::Calc::RenewVelocityDiff(false);
			Status::Calc::RenewAccelTarget(false);
			Status::Calc::RenewDegree(false);
			Status::Calc::RenewDistance(false);
			PWM::Motor::SetDuty();

			if ((log_cnt < LOGSIZE) && (TPUA.TSTR.BIT.CST1 == 1) && (TPUA.TSTR.BIT.CST2 == 1)) {
				logdata1[log_cnt] = A_Velocity.GetValue(false);
				logdata2[log_cnt] = A_Velocity.GetValue(true);
				logdata3[log_cnt] = A_VelocityDiff.GetValue();

				log_cnt++;
			}

			timer++;	// ウルトラマン
		} else {
			//Interface::StartWithFrontSensor();
		}
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

// Interface
	int8_t Interface::mode = 1;
	bool Interface::waiting = true;
	int16_t Interface::encoder_r = 0, Interface::encoder_l = 0, Interface::sensor_cnt = 0;

	void Interface::InitLED() {
		// CS_Run Enable
		System::RSPI::WriteData(System::RSPI::LED, RSPI_ADDRESS_LED_CONFIG, RSPI_DATA_LED_ENABLECS);

		// Execute CS_Run
		RSPI_CS_LED = 0;
		System::Timer::wait_ms(20);
		RSPI_CS_LED = 1;
	}

	void Interface::InitEncoder() {
		SYSTEM.PRCR.WORD = 0xA502;
		MSTP(TPU0) = 0;
		SYSTEM.PRCR.WORD = 0xA500;

		TPUA.TSTR.BIT.CST1 = 0;
		TPUA.TSTR.BIT.CST2 = 0;
		TPU1.TMDR.BIT.MD = 4;
		TPU2.TMDR.BIT.MD = 4;
		TPU1.TCNT = ENCODER_INIT_VAL;
		TPU2.TCNT = ENCODER_INIT_VAL;
	}

	void Interface::Encoder_Enable() {
		TPUA.TSTR.BIT.CST1 = 1;
		TPUA.TSTR.BIT.CST2 = 1;
	}

	void Interface::Encoder_Disable() {
		TPUA.TSTR.BIT.CST1 = 0;
		TPUA.TSTR.BIT.CST2 = 0;
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

	void Interface::SetEncoderValue() {
		if ((TPUA.TSTR.BIT.CST1 == 1) && (TPUA.TSTR.BIT.CST2 == 1)) {
			encoder_l = ENCODER_INIT_VAL - TPU1.TCNT;
			encoder_r = TPU2.TCNT - ENCODER_INIT_VAL;

			TPU1.TCNT = ENCODER_INIT_VAL;
			TPU2.TCNT = ENCODER_INIT_VAL;
		}
	}

	int16_t Interface::GetEncoderValue(uint8_t side) {
		int16_t enc_value = 0;

		switch (side) {
			case Interface::Left:
				enc_value = encoder_l;
				break;

			case Interface::Right:
				enc_value = encoder_r;
				break;
		}

		return enc_value;
	}

	void Interface::ModeSelect() {
		volatile uint8_t cnt = 0;
		bool locked = true, add = false;

		//Interface::SetLEDColor(0, 255, 0, 0);

		while (waiting) {
			if ((SW_NEXT == 0) && locked) {
				cnt++;

				if (cnt > 50) {
					while(SW_NEXT == 0);
					locked = false;
					add = true;
					cnt = 0;
				}
			} else if ((SW_PREV == 0) && locked) {
				cnt++;

				if (cnt > 50) {
					while(SW_PREV == 0);
					locked = false;
					add = false;
					cnt = 0;
				}
			} else {
				cnt = 0;
			}

			if (!locked) {
				if (add) {
					mode++;
					locked = true;
					add = false;

					PWM::Buzzer::Enable();
					PWM::Buzzer::SetDuty(1000.0);
					Timer::wait_ms(100);
					PWM::Buzzer::Disable();
				} else {
					mode--;
					locked = true;

					PWM::Buzzer::Enable();
					PWM::Buzzer::SetDuty(750.0);
					Timer::wait_ms(100);
					PWM::Buzzer::Disable();
				}

				if (mode < 1) {
					mode = MODE_NUMBER;
				} else if (mode > MODE_NUMBER) {
					mode = 1;
				}

				//Interface::SetLEDColor(0, 255, 0, 0);
			}

			switch (mode) {
				case 0:
					Interface::SetLEDColor(0, 255, 0, 0);
					break;

				case 1:
					Interface::SetLEDColor(0, 0, 255, 0);
					break;

				default:
					Interface::SetLEDColor(0, 0, 0, 255);
					break;
			}
		}

		PWM::Buzzer::Enable();
		PWM::Buzzer::SetDuty(1000.0);
		Timer::wait_ms(1000);
		PWM::Buzzer::Disable();

		waiting = true;
	}

	void Interface::StartWithFrontSensor() {
		if (ExecuteFlag.GetValue()) {
			return;
		}

		if (Status::Sensor::CheckWallExist(SIDE_FORWARD) > SENSOR_WALL_EXIST_F) {
			sensor_cnt++;
			Interface::SetLEDColor(1, 0, 255, 0);
		} else {
			sensor_cnt = 0;
			Interface::SetLEDColor(1, 255, 0, 0);
		}

		if (sensor_cnt > WAITING_SENSORSTART) {
			waiting = false;
		}
	}
}

void Timer_CMT0() {
	System::Timer::CH0();
}

void Timer_CMT1() {
	System::Timer::CH1();
}
