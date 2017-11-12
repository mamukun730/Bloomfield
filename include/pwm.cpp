/*
 * pwm.cpp
 *
 *  Created on: 2017/09/04
 *      Author: mmkn
 */

#include "common.hpp"
#include "pwm.hpp"

namespace PWM {
	struct _slalom_param Motor::slalom_param[NUMBER_SLALOM_PARAM + 1] = {
/*
			const volatile float angle_velocity;
			const volatile float angle_accel;
			const volatile float distance_before_left;
			const volatile float distance_before_right;
			const volatile float distance_after_left;
			const volatile float distance_after_right;
			const volatile float turn_angle;
			const volatile unsigned char clothoid_angle;
			const volatile unsigned char wall_correction;

			wall_correction:
			bit	3		2		1		0
				Sl_IN	Sl_OUT	St_IN	St_OUT
 */
			// Slip = 16,000?
			{ 550.0394, 6050.8686,  8.000,  8.000,10.500,13.000, 90.000, 25, 0x03},		// 000: 90度小,  240mm/s, r=25

			// Slip = 60,000?
			{ 381.9719,	2431.7084, 20.000, 20.000, 2.000, 2.000, 90.000, 30, 0x03},		// 001: 90度大,  300mm/s, r=45
			{ 390.6530, 2543.4967, 10.000, 10.000, 2.000, 2.000,180.000, 30, 0x03},		// 002: 180度大, 300mm/s, r=43
			{ 429.7183, 4616.4464, 13.000, 13.000, 0.500, 0.500, 45.000, 20, 0x02},		// 003: 45度In,  300mm/s, r=40
			{ 429.7183, 4616.4464, 31.000, 31.000, 2.000, 2.000, 45.000, 20, 0x08},		// 004: 45度Out, 300mm/s, r=40
			{ 429.7183, 3693.1571, 13.000, 13.000, 0.500, 0.500,135.000, 25, 0x02},		// 005: 135度In, 300mm/s, r=40
			{ 429.7183, 3693.1571,  8.500,  8.500, 2.000, 2.000,135.000, 25, 0x08},		// 006: 135度Out,300mm/s, r=40
			{ 429.7183, 3693.1571,  4.000,  4.000, 0.500, 0.500, 90.000, 25, 0x08},		// 007: 90度,    300mm/s, r=40

			// Slip = 60,000?
			{ 458.3662,	3501.6601, 12.000, 12.000, 2.000, 2.000, 90.000, 30, 0x03},		// 008: 90度大,  400mm/s, r=50
			{ 532.9840, 4734.5323,  5.000,  5.000, 2.000, 2.000,180.000, 30, 0x03},		// 009: 180度大, 400mm/s, r=43
			{ 572.9578, 8207.0158, 14.000, 14.000, 0.500, 0.500, 45.000, 20, 0x02},		// 010: 45度In,  400mm/s, r=40
			{ 572.9578, 8207.0158, 31.000, 31.000, 2.000, 2.000, 45.000, 20, 0x08},		// 011: 45度Out, 400mm/s, r=40
			{ 572.9578, 8207.0158, 20.000, 20.000, 0.500, 0.500,135.000, 20, 0x02},		// 012: 135度In, 400mm/s, r=40
			{ 572.9578, 8207.0158, 12.000, 12.000, 2.000, 2.000,135.000, 20, 0x08},		// 013: 135度Out,400mm/s, r=40
			{ 572.9578, 8207.0158,  6.500,  6.500, 0.500, 0.500, 90.000, 20, 0x08},		// 014: 90度,    400mm/s, r=40
	};

	void Buzzer::Init() {
		SYSTEM.PRCR.WORD = 0xA502;
		MSTP(MTU) = 0;
		SYSTEM.PRCR.WORD = 0xA500;

		PORTE.PMR.BIT.B5 = 1;

		// ch2: Buzzer
		// ch4: Motor
		MTU.TSTR.BIT.CST2 = 0;
		MTU2.TCNT = 0x00;
		MTU2.TCR.BIT.TPSC = 2;			// PCLK / 16
		MTU2.TCR.BIT.CKEG = 0;
		MTU2.TCR.BIT.CCLR = 1;			// Reset: TGRA
		MTU2.TMDR.BIT.MD = 3;
		MTU2.TIOR.BIT.IOA = 0;			// TGRA: Low-High
		MTU2.TIOR.BIT.IOB = 2;			// TGRB: Low-Low
		MTU2.TGRB = 65535;
		MTU2.TGRA = MTU2.TGRB;
	}

	void Buzzer::Enable() {
		Buzzer::Init();
		MTU.TSTR.BIT.CST2 = 1;
	}

	void Buzzer::Disable() {
		Buzzer::SetDuty(0.0);
		MTU.TSTR.BIT.CST2 = 0;

		PORTE.PMR.BIT.B5 = 0;
		PORTE.PODR.BIT.B5 = 0;
	}

	void Buzzer::SetDuty(float hz) {
		MTU2.TCNT = 0x00;

		if (hz > 0.0) {
			MTU2.TGRA = (int)((float)SYSTEM_CLOCK / (16.0 * hz));
			MTU2.TGRB = (int)(MTU2.TGRA / 2);
		} else {
			MTU2.TGRA = 65535;
			MTU2.TGRB = MTU2.TGRA;
		}
	}

	void Buzzer::Melody_M_Coin() {
		Buzzer::Enable();
		Buzzer::SetDuty(O4_Si);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O5_Mi);
		System::Timer::wait_ms(200);
		Buzzer::Disable();
	}

	void Buzzer::Melody_M_Dead() {
		Buzzer::Enable();
		Buzzer::SetDuty(O4_Si);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O4_Fa);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O4_Fa);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(10);
		Buzzer::SetDuty(O4_Fa);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O4_Mi);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O4_Re);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O4_Do);
		System::Timer::wait_ms(100);
		Buzzer::Disable();
	}

	void Buzzer::Melody_M_UpMush() {
		Buzzer::Enable();
		Buzzer::SetDuty(O4_Mi);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O4_Sol);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O5_Mi);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O5_Do);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O5_Re);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O5_Sol);
		System::Timer::wait_ms(100);
		Buzzer::Disable();
	}

	void Buzzer::Melody_TX1() {
		Buzzer::Enable();
		Buzzer::SetDuty(O4_Ra);
		System::Timer::wait_ms(113);	// 16: ((108/60)/16)*1000=112.5
		Buzzer::SetDuty(O4_Fa);
		System::Timer::wait_ms(113);
		Buzzer::SetDuty(O4_Ra);
		System::Timer::wait_ms(113);
		Buzzer::SetDuty(O4_Fa);
		System::Timer::wait_ms(113);
		Buzzer::SetDuty(O4_Ra);
		System::Timer::wait_ms(113);
		Buzzer::SetDuty(O4_Fa);
		System::Timer::wait_ms(113);
		Buzzer::SetDuty(O5_Do);
		System::Timer::wait_ms(225);	// 8: ((108/60)/16)*1000=112.5

		Buzzer::SetDuty(O4_Si);
		System::Timer::wait_ms(113);	// 16: ((108/60)/16)*1000=112.5
		Buzzer::SetDuty(O4_Sol);
		System::Timer::wait_ms(113);
		Buzzer::SetDuty(O4_Si);
		System::Timer::wait_ms(113);
		Buzzer::SetDuty(O4_Sol);
		System::Timer::wait_ms(113);
		Buzzer::SetDuty(O4_Si);
		System::Timer::wait_ms(113);
		Buzzer::SetDuty(O4_Sol);
		System::Timer::wait_ms(113);
		Buzzer::SetDuty(O5_Do);
		System::Timer::wait_ms(675);	// 8+4: ((108/60)/4)*1000*1.5=112.5
		Buzzer::Disable();
	}

	void Buzzer::Melody_TX3() {
		Buzzer::Enable();
		Buzzer::SetDuty(O5_Do);
		System::Timer::wait_ms(120);	// 16: (60/125)*0.25*1000=120
		Buzzer::SetDuty(O5_Mi);
		System::Timer::wait_ms(120);
		Buzzer::SetDuty(O5_Sol);
		System::Timer::wait_ms(120);
		Buzzer::SetDuty(O5_Mi);
		System::Timer::wait_ms(120);
		Buzzer::SetDuty(O5_Do);
		System::Timer::wait_ms(120);
		Buzzer::SetDuty(O5_Mi);
		System::Timer::wait_ms(120);
		Buzzer::SetDuty(O5_Sol);
		System::Timer::wait_ms(120);
		Buzzer::SetDuty(O5_Mi);
		System::Timer::wait_ms(120);
		Buzzer::SetDuty(O5_Do);
		System::Timer::wait_ms(480);	// 4: (60/125)*1*1000=480
		Buzzer::Disable();
	}

	void Buzzer::Melody_GK1_1() {
		Buzzer::Enable();

		Buzzer::SetDuty(O5_Ra);
		System::Timer::wait_ms(150);	// 16: (60/100)*0.25*1000=150
		Buzzer::SetDuty(O5_Sol);
		System::Timer::wait_ms(150);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(150);
		Buzzer::SetDuty(O5_Mi);
		System::Timer::wait_ms(150);
		Buzzer::SetDuty(O5_Re);
		System::Timer::wait_ms(150);
		Buzzer::SetDuty(O5_Mi);
		System::Timer::wait_ms(150);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(150);
		Buzzer::SetDuty(O5_Re);
		System::Timer::wait_ms(150);

		Buzzer::SetDuty(O5_Mi);
		System::Timer::wait_ms(300);	// 8: (60/100)*0.5*1000=150
		Buzzer::SetDuty(O5_Sol);
		System::Timer::wait_ms(300);
		Buzzer::SetDuty(O6_Do);
		System::Timer::wait_ms(1200);	// 2: (60/100)*2*1000=150

		Buzzer::Disable();
	}

	void Buzzer::Melody_GK8_1() {
		Buzzer::Enable();

		Buzzer::SetDuty(O4_Do);
		System::Timer::wait_ms(125);	// 16: (60/120)*0.25*1000=120
		Buzzer::SetDuty(O4_Sol);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O5_Do);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(125);

		Buzzer::SetDuty(O5_Re);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O5_Mi);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O5_Do);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Sol);
		System::Timer::wait_ms(125);

		Buzzer::SetDuty(O5_Re);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Si);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O5_Do);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Sol);
		System::Timer::wait_ms(125);

		Buzzer::SetDuty(O4_Fa);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O5_Do);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Mi);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Re);
		System::Timer::wait_ms(125);

		Buzzer::SetDuty(O4_Do);
		System::Timer::wait_ms(500);	// 4: (60/120)*1*1000=400
		Buzzer::Disable();
	}

	// 6/8
	void Buzzer::Melody_TE3_1() {
		Buzzer::Enable();

		Buzzer::SetDuty(O4_Ra);
		System::Timer::wait_ms(160);	// 8: (60/140)*(3/4)*0.5*1000=160
		Buzzer::SetDuty(O5_Do);
		System::Timer::wait_ms(160);
		Buzzer::SetDuty(O5_Mi);
		System::Timer::wait_ms(160);
		Buzzer::SetDuty(O5_Re);
		System::Timer::wait_ms(160);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(160);
		Buzzer::SetDuty(O5_Ra);
		System::Timer::wait_ms(160);

		Buzzer::SetDuty(O5_Do);
		System::Timer::wait_ms(80);	// 16: (60/140)*(3/4)*0.25*1000=80
		Buzzer::SetDuty(O5_Mi);
		System::Timer::wait_ms(80);
		Buzzer::SetDuty(O5_Sol);
		System::Timer::wait_ms(80);
		Buzzer::SetDuty(O6_Do);
		System::Timer::wait_ms(321);

		Buzzer::Disable();
	}

	void Buzzer::Melody_TE10_1() {
		Buzzer::Enable();
/*
		// 4b: Re Mi Ra Si
		Buzzer::SetDuty(O3_Sol);
		System::Timer::wait_ms(125);	// 8: (60/240)*0.5*1000=125
		Buzzer::SetDuty(O4_Do);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Fa);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Sol);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O3_Sol);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Do);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Fa);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Sol);
		System::Timer::wait_ms(125);

		Buzzer::SetDuty(O4_Do);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Re_Sharp);		// Mi_Flat
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Sol);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Ra_Sharp);		// Si_Flat
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Do);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Re_Sharp);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Sol);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Ra_Sharp);
		System::Timer::wait_ms(125);
*/
		Buzzer::SetDuty(O4_Do_Sharp);		// Re_Flat
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Sol);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Ra_Sharp);		// Si_Flat
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O5_Do_Sharp);		// Re_Flat
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Do_Sharp);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Sol);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Ra_Sharp);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O5_Do_Sharp);
		System::Timer::wait_ms(125);

		Buzzer::SetDuty(O2_Sol_Sharp);		// Ra_Flat
		System::Timer::wait_ms(83);	// 8(3): (60/240)*0.5*(2/3)*1000=83
		Buzzer::SetDuty(O3_Do);
		System::Timer::wait_ms(83);
		Buzzer::SetDuty(O3_Re_Sharp);		// Mi_Flat
		System::Timer::wait_ms(83);

		Buzzer::SetDuty(O3_Sol_Sharp);		// Ra_Flat
		System::Timer::wait_ms(83);
		Buzzer::SetDuty(O4_Do);
		System::Timer::wait_ms(83);
		Buzzer::SetDuty(O4_Re_Sharp);		// Mi_Flat
		System::Timer::wait_ms(83);

		Buzzer::SetDuty(O4_Sol_Sharp);		// Ra_Flat
		System::Timer::wait_ms(83);
		Buzzer::SetDuty(O5_Do);
		System::Timer::wait_ms(83);
		Buzzer::SetDuty(O5_Re_Sharp);		// Mi_Flat
		System::Timer::wait_ms(332);

		Buzzer::Disable();
	}

	void Buzzer::Melody_TE16_1() {
		Buzzer::Enable();

		Buzzer::SetDuty(O4_Sol);
		System::Timer::wait_ms(231);	// 8: (60/130)*0.5*1000=800
		Buzzer::SetDuty(O5_Do);
		System::Timer::wait_ms(115);	// 16: (60/130)*0.25*1000=400
		Buzzer::SetDuty(O4_Si);
		System::Timer::wait_ms(115);
		Buzzer::SetDuty(O5_Do);
		System::Timer::wait_ms(231);
		Buzzer::SetDuty(O5_Re);
		System::Timer::wait_ms(346);	// 8+16

		Buzzer::SetDuty(O5_Sol);
		System::Timer::wait_ms(115);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(115);
		Buzzer::SetDuty(O5_Re);
		System::Timer::wait_ms(115);
		Buzzer::SetDuty(O4_Si);
		System::Timer::wait_ms(115);
		Buzzer::SetDuty(O5_Re);
		System::Timer::wait_ms(115);
		Buzzer::SetDuty(O4_Si);
		System::Timer::wait_ms(115);
		Buzzer::SetDuty(O4_Sol);
		System::Timer::wait_ms(115);
		Buzzer::SetDuty(O5_Do);
		System::Timer::wait_ms(1000);	// 1: (60/130)*4*1000 = 1,846

		Buzzer::Disable();
	}

	void Buzzer::Melody_TE26_1() {
		Buzzer::Enable();

		Buzzer::SetDuty(O5_Mi);
		System::Timer::wait_ms(125);	// 16: (60/120)*0.25*1000=125
		Buzzer::SetDuty(O5_Re_Sharp);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O5_Do_Sharp);
		System::Timer::wait_ms(125);

		Buzzer::SetDuty(O4_Ra_Sharp);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O4_Si);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O5_Do_Sharp);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O5_Re);
		System::Timer::wait_ms(125);
		Buzzer::SetDuty(O5_Sol_Sharp);
		System::Timer::wait_ms(250);	// 8: (60/120)*0.5*1000=250
		Buzzer::SetDuty(O5_Ra_Sharp);
		System::Timer::wait_ms(250);
		Buzzer::SetDuty(O5_Si);
		System::Timer::wait_ms(1000);	// 2: (60/120)*2*1000=1000

		Buzzer::Disable();
	}

	// 6/8
	void Buzzer::Melody_TE32_1() {
		Buzzer::Enable();

		Buzzer::SetDuty(O5_Ra);
		System::Timer::wait_ms(180);	// 8: (60/250)*(3/4)*1000=204
		Buzzer::SetDuty(O5_Ra_Sharp);
		System::Timer::wait_ms(180);
		Buzzer::SetDuty(O6_Do);
		System::Timer::wait_ms(180);
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(1);
		Buzzer::SetDuty(O6_Do);
		System::Timer::wait_ms(180);
		Buzzer::SetDuty(O5_Ra_Sharp);
		System::Timer::wait_ms(180);
		Buzzer::SetDuty(O5_Ra);
		System::Timer::wait_ms(180);

		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(180);
		Buzzer::SetDuty(O5_Ra);
		System::Timer::wait_ms(180);
		Buzzer::SetDuty(O6_Do);
		System::Timer::wait_ms(180);
		Buzzer::SetDuty(O5_Sol);
		System::Timer::wait_ms(180);
		Buzzer::SetDuty(O6_Do);
		System::Timer::wait_ms(180);
		Buzzer::SetDuty(O6_Mi);
		System::Timer::wait_ms(180);

		Buzzer::SetDuty(O6_Fa);
		System::Timer::wait_ms(360);	// 4: (60/250)*(3/2)*1000 = 360

		Buzzer::Disable();
	}

	void Buzzer::Melody_Namiki() {
		Buzzer::Enable();

/*		Buzzer::SetDuty(O5_Mi);
		System::Timer::wait_ms(169);	// 8: (60/178)*0.5*1000=169
		Buzzer::SetDuty(O5_Re_Sharp);
		System::Timer::wait_ms(169);
		Buzzer::SetDuty(O5_Mi);
		System::Timer::wait_ms(169);
		Buzzer::SetDuty(O4_Si);
		System::Timer::wait_ms(169);
		Buzzer::SetDuty(O5_Do);
		System::Timer::wait_ms(169);
		Buzzer::SetDuty(O4_Si);
		System::Timer::wait_ms(169);
		Buzzer::SetDuty(O5_Do);
		System::Timer::wait_ms(169);
		Buzzer::SetDuty(O4_Sol_Sharp);
		System::Timer::wait_ms(169);

		Buzzer::SetDuty(O4_Ra);
		System::Timer::wait_ms(169);
		Buzzer::SetDuty(O4_Mi);
		System::Timer::wait_ms(169);
		Buzzer::SetDuty(O4_Si);
		System::Timer::wait_ms(169);
		Buzzer::SetDuty(O4_Mi);
		System::Timer::wait_ms(169);
		Buzzer::SetDuty(O5_Do);
		System::Timer::wait_ms(169);
		Buzzer::SetDuty(O4_Mi);
		System::Timer::wait_ms(169);
		Buzzer::SetDuty(O5_Mi);
		System::Timer::wait_ms(169);
		Buzzer::SetDuty(O4_Sol);
		System::Timer::wait_ms(169);
*/
		Buzzer::SetDuty(O5_Re);
		System::Timer::wait_ms(169);
		Buzzer::SetDuty(O5_Sol);
		System::Timer::wait_ms(169);
		Buzzer::SetDuty(O5_Re);
		System::Timer::wait_ms(169);
		Buzzer::SetDuty(O4_Sol);
		System::Timer::wait_ms(169);
		Buzzer::SetDuty(O4_Ra);
		System::Timer::wait_ms(84);	// 16: (60/178)*0.25*1000=42
		Buzzer::SetDuty(O4_Si);
		System::Timer::wait_ms(84);
		Buzzer::SetDuty(O4_Ra);
		System::Timer::wait_ms(84);
		Buzzer::SetDuty(O4_Si);
		System::Timer::wait_ms(84);
		Buzzer::SetDuty(O4_Ra);
		System::Timer::wait_ms(169);
		Buzzer::SetDuty(O4_Sol);
		System::Timer::wait_ms(169);
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(1);
		Buzzer::SetDuty(O4_Sol);
		System::Timer::wait_ms(674);	// 2: (60/178)*2*100=674

		Buzzer::Disable();
	}

	// 3/4
	void Buzzer::Melody_Shibuya_Otogi() {
		Buzzer::Enable();

		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(100);	// 8: (60/225)*0.5*3/4*1000=100
		Buzzer::SetDuty(O5_Sol_Sharp);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O5_Re);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O4_Si);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O5_Re);
		System::Timer::wait_ms(100);

		Buzzer::SetDuty(O5_Do);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O4_Fa);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O4_Ra);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O5_Do);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O4_Ra_Sharp);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O4_Sol);
		System::Timer::wait_ms(100);

		Buzzer::SetDuty(O4_Mi);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O4_Fa);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O4_Sol_Sharp);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O4_Ra);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O4_Ra_Sharp);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(O5_Do);
		System::Timer::wait_ms(100);

		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(533);	// 2: (60/225)*2*1000=533

		Buzzer::Disable();
	}

	void Buzzer::Melody_OsakaLoop() {
		Buzzer::Enable();

		Buzzer::SetDuty(O4_Mi);
		System::Timer::wait_ms(115);	// 8: (60/260)*0.5*1000=115
		Buzzer::SetDuty(O4_Fa_Sharp);
		System::Timer::wait_ms(115);
		Buzzer::SetDuty(O4_Ra);
		System::Timer::wait_ms(115);
		Buzzer::SetDuty(O4_Mi);
		System::Timer::wait_ms(115);
		Buzzer::SetDuty(O4_Fa_Sharp);
		System::Timer::wait_ms(115);
		Buzzer::SetDuty(O4_Ra);
		System::Timer::wait_ms(115);
		Buzzer::SetDuty(O4_Mi);
		System::Timer::wait_ms(115);
		Buzzer::SetDuty(O4_Fa_Sharp);
		System::Timer::wait_ms(115);

		Buzzer::SetDuty(O4_Ra);
		System::Timer::wait_ms(115);
		Buzzer::SetDuty(O4_Mi);
		System::Timer::wait_ms(115);
		Buzzer::SetDuty(O4_Fa_Sharp);
		System::Timer::wait_ms(115);
		Buzzer::SetDuty(O4_Ra);
		System::Timer::wait_ms(115);
		Buzzer::SetDuty(O4_Re);
		System::Timer::wait_ms(115);
		Buzzer::SetDuty(O4_Mi);
		System::Timer::wait_ms(115);
		Buzzer::SetDuty(O4_Fa_Sharp);
		System::Timer::wait_ms(115);
		Buzzer::SetDuty(O4_Ra);
		System::Timer::wait_ms(115);

		Buzzer::SetDuty(O5_Re);
		System::Timer::wait_ms(231);	// 4: (60/260)*1*1000 = 360

		Buzzer::Disable();
	}

	void Buzzer::Melody_Oedo_Door() {
		// 5b: Re Mi Sol Ra Si
		Buzzer::Enable();

		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(126);	// 8: (60/120)*0.25*1000=115
		Buzzer::SetDuty(O5_Sol_Sharp);		// Ra_Flat
		System::Timer::wait_ms(126);
		Buzzer::SetDuty(O6_Do_Sharp);		// Re_Flat
		System::Timer::wait_ms(126);
		Buzzer::SetDuty(O5_Sol_Sharp);		// Ra_Flat
		System::Timer::wait_ms(126);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(126);
		Buzzer::SetDuty(O5_Sol_Sharp);		// Ra_Flat
		System::Timer::wait_ms(126);
		Buzzer::SetDuty(O6_Do_Sharp);	// Re_Flat
		System::Timer::wait_ms(750);	// 4(dot): (60/120)*(1+0.5)*1000=750

		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(126);	// 8: (60/120)*0.25*1000=115
		Buzzer::SetDuty(O5_Sol_Sharp);		// Ra_Flat
		System::Timer::wait_ms(126);
		Buzzer::SetDuty(O6_Do_Sharp);		// Re_Flat
		System::Timer::wait_ms(126);
		Buzzer::SetDuty(O5_Sol_Sharp);		// Ra_Flat
		System::Timer::wait_ms(126);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(126);
		Buzzer::SetDuty(O5_Sol_Sharp);		// Ra_Flat
		System::Timer::wait_ms(126);
		Buzzer::SetDuty(O6_Do_Sharp);	// Re_Flat
		System::Timer::wait_ms(750);	// 4(dot): (60/120)*(1+0.5)*1000=750

		Buzzer::Disable();
	}

	void Buzzer::Melody_FoxMovie() {
		Buzzer::Enable();

		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(375);	// 8(dot): (60/120)*(0.5+0.25)*1000=375
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(1);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(63);		// 32: (60/120)*(4/32)*1000=62.5
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(1);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(63);
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(1);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(667);	// 4+8(3): (60/120)*(1+0.5*2/3)*1000=750
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(1);
		Buzzer::SetDuty(O5_Fa_Sharp);	// Sol_Flat
		System::Timer::wait_ms(167);	// 8(3): (60/120)*0.5*(2/3)*1000=750
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(1);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(167);
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(1);

		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(375);
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(1);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(63);
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(1);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(167);
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(1);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(167);
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(1);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(167);
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(1);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(167);
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(1);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(167);
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(1);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(167);
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(1);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(167);
		Buzzer::SetDuty(O5_Re);
		System::Timer::wait_ms(167);
		Buzzer::SetDuty(O5_Re_Sharp);	// Mi_Flat
		System::Timer::wait_ms(167);

		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(375);
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(1);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(63);
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(1);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(63);
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(1);
		Buzzer::SetDuty(O5_Fa);
		System::Timer::wait_ms(667);

		Buzzer::Disable();
	}

	void Buzzer::Melody_ProtectionRadio() {
		Buzzer::Enable();

		Buzzer::SetDuty(1500.0);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(100);

		Buzzer::SetDuty(1500.0);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(100);

		Buzzer::SetDuty(1500.0);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(100);

		Buzzer::SetDuty(1500.0);
		System::Timer::wait_ms(100);
		Buzzer::SetDuty(0.0);
		System::Timer::wait_ms(100);

		Buzzer::SetDuty(1500.0);
		System::Timer::wait_ms(100);

		Buzzer::Disable();
	}

	void Motor::Init() {
		MTU.TSTR.BIT.CST0 = 0;

		MTU0.TCNT = 0x00;
		MTU0.TCR.BIT.TPSC = 0;			// PCLK / 1
		MTU0.TCR.BIT.CKEG = 0;			// 立ち上がりカウント
		MTU0.TCR.BIT.CCLR = 6;			// Reset: TGRD
		MTU0.TMDR.BIT.MD = 3;			// PWM2
		MTU0.TIORH.BIT.IOA = 2;			// TGRA: Low-High
		MTU0.TIORH.BIT.IOB = 0;			// TGRB
		MTU0.TIORL.BIT.IOC = 2;			// TGRC: Low-High
		MTU0.TIORL.BIT.IOD = 0;			// TGRD

		MTU0.TGRA = MOTOR_OPCYCLE;
		MTU0.TGRB = MOTOR_OPCYCLE;
		MTU0.TGRC = MOTOR_OPCYCLE;
		MTU0.TGRD = MOTOR_OPCYCLE;

		MOTOR_CTRL_L = 0;
		MOTOR_CTRL_R = 0;
}

	void Motor::SetDriveDir(unsigned char dir) {
		switch (dir) {
			case Motor::Forward:
				MOTOR_CTRL_L = 0;
				MOTOR_CTRL_R = 1;
				break;

			case Motor::Back:
				MOTOR_CTRL_L = 1;
				MOTOR_CTRL_R = 0;
				break;

			case Motor::Turn:
				MOTOR_CTRL_L = 1;
				MOTOR_CTRL_R = 1;
				break;

			default:
				MOTOR_CTRL_L = 0;
				MOTOR_CTRL_R = 0;
				break;
		}
	}

	void Motor::SetDuty() {
		static float v_last_diff = 0.0, av_last_diff = 0.0;
		float a_velocity_wall = 0.0;
		float v_actual = 0.0, v_target = 0.0;
		float av_actual = 0.0, av_target = 0.0;
		float duty_r = 0.0, duty_l = 0.0, torque_r = 0.0, torque_l = 0.0;
		float ff_r = 0.0, ff_l = 0.0, fb_r = 0.0, fb_l = 0.0, battery_v = 0.0, gain_p_wall = GAIN_P_WALL;

		static uint16_t cnt = 0;

		if (WallPFlag.GetValue()) {
			if (Velocity.GetValue(false) > SEARCH_SPEED) {
				gain_p_wall = GAIN_P_WALL_SH;
			}

			a_velocity_wall = -1.0 * GAIN_P_WALL * Status::Calc::WallControlQuantity();

			if (fabsf(a_velocity_wall) < CTRL_WALL_LIMIT) {
				A_Velocity.SetValue(false, a_velocity_wall);
			} else {
				if (a_velocity_wall > 0.0) {
					A_Velocity.SetValue(false, CTRL_WALL_LIMIT);
				} else {
					A_Velocity.SetValue(false, -CTRL_WALL_LIMIT);
				}
			}

			Degree.SetValue(0.0);
		}

		v_actual = Velocity.GetValue(true);
		v_target = Velocity.GetValue(false);

		av_actual = A_Velocity.GetValue(true);
		av_target = A_Velocity.GetValue(false);

		fb_r = (GAIN_P_ENCODER * (v_target - v_actual) + (GAIN_I_ENCODER * VelocityDiff.GetValue()) + (GAIN_D_ENCODER * ((v_target - v_actual) - v_last_diff)));
		fb_l = (GAIN_P_ENCODER * (v_target - v_actual) + (GAIN_I_ENCODER * VelocityDiff.GetValue()) + (GAIN_D_ENCODER * ((v_target - v_actual) - v_last_diff)));

		fb_r += (GAIN_P_GYRO * (av_target - av_actual) + (GAIN_I_GYRO * A_VelocityDiff.GetValue()) + (GAIN_D_GYRO * ((av_target - av_actual) - av_last_diff)));
		fb_l -= (GAIN_P_GYRO * (av_target - av_actual) + (GAIN_I_GYRO * A_VelocityDiff.GetValue()) + (GAIN_D_GYRO * ((av_target - av_actual) - av_last_diff)));

//		ff_r = (0.5 * BODY_WEIGHT * (Accel.GetValue(false) / 1000.0)) * ((0.5 * (WHEEL_D / 1000.0)) / GEAR_RATIO);
//		ff_l = (0.5 * BODY_WEIGHT * (Accel.GetValue(false) / 1000.0)) * ((0.5 * (WHEEL_D / 1000.0)) / GEAR_RATIO);
//
//		ff_r += ((BODY_MOMENT_INERTIA * A_Accel.GetValue(false) * DEGREE_RAD) / BODY_WIDTH) * ((0.5 * (WHEEL_D / 1000.0)) / GEAR_RATIO);
//		ff_l -= ((BODY_MOMENT_INERTIA * A_Accel.GetValue(false) * DEGREE_RAD) / BODY_WIDTH) * ((0.5 * (WHEEL_D / 1000.0)) / GEAR_RATIO);

		v_last_diff = v_target - v_actual;
		av_last_diff = av_target - av_actual;

		MTU0.TCNT = 0x00;

		if ((ff_r + fb_r) > 0.0) {
			MOTOR_CTRL_R = 1;
		} else {
			MOTOR_CTRL_R = 0;
		}

		if ((ff_l + fb_l) > 0.0) {
			MOTOR_CTRL_L = 1;
		} else {
			MOTOR_CTRL_L = 0;
		}

		fb_r = fabsf(ff_r + fb_r);
		fb_l = fabsf(ff_l + fb_l);

		if ((MOTOR_OPCYCLE - (uint16_t)fb_r) < MOTOR_DUTY_MAX) {
			MTU0.TGRC = MOTOR_DUTY_MAX;
//		} else if (((MOTOR_OPCYCLE - (uint16_t)fb_r) > MOTOR_DUTY_MIN) && (fb_r > 0.0)) {
//			MTU4.TGRB = MOTOR_DUTY_MIN;
		} else {
			MTU0.TGRC = (MOTOR_OPCYCLE - (uint16_t)fb_r);
		}

		if ((MOTOR_OPCYCLE - (uint16_t)fb_l) < MOTOR_DUTY_MAX) {
			MTU0.TGRA = MOTOR_DUTY_MAX;
//		} else if (((MOTOR_OPCYCLE - (uint16_t)fb_l) > MOTOR_DUTY_MIN) && (fb_l > 0.0)) {
//			MTU4.TGRD = MOTOR_DUTY_MIN;
		} else {
			MTU0.TGRA = (MOTOR_OPCYCLE - (uint16_t)fb_l);
		}
	}

	void Motor::Enable() {
		MTU0.TGRA = MOTOR_OPCYCLE;
		MTU0.TGRB = MOTOR_OPCYCLE;
		MTU0.TGRC = MOTOR_OPCYCLE;
		MTU0.TGRD = MOTOR_OPCYCLE;

		PORTB.PMR.BIT.B1 = 1;
		PORTB.PMR.BIT.B3 = 1;

		Status::Reset();
		System::Interface::Encoder_Enable();
		MTU.TSTR.BIT.CST0 = 1;
	}

	void Motor::Disable() {
		System::Interface::Encoder_Disable();

		MTU0.TGRA = MOTOR_OPCYCLE;
		MTU0.TGRB = MOTOR_OPCYCLE;
		MTU0.TGRC = MOTOR_OPCYCLE;
		MTU0.TGRD = MOTOR_OPCYCLE;

		MOTOR_CTRL_L = 0;
		MOTOR_CTRL_R = 0;

		MTU.TSTR.BIT.CST0 = 0;

		PORTB.PMR.BIT.B1 = 0;
		PORTB.PMR.BIT.B3 = 0;
		PORTB.PODR.BIT.B1 = 0;
		PORTB.PODR.BIT.B3 = 0;

	}

	void Motor::CtrlAvoidObstacle(bool exit_area) {
		unsigned int k = 1;
		float a_vel = 0.0;

		/*
		 TODO: 出口区画では次に曲がる方向を考慮する? 前壁誤検知対策の強化
		 */

		if (exit_area) {
			k = 1;
		}

//		if ((ADC::Sensor::GetValue(ADC::Sensor::LF) > WALL_CTRL_THRESHOLD_F_LF_SL * k) && (ADC::Sensor::GetValue(ADC::Sensor::RF) < WALL_CTRL_THRESHOLD_F_RF_SL)) {
		if (Status::Sensor::GetValue(Status::Sensor::LC, false) > (SENSOR_CTRL_THRESHOLD_LC * k)) {
			a_vel = GAIN_P_WALL_SLANT * (float)(Status::Sensor::GetValue(Status::Sensor::LC, false) - SENSOR_CTRL_THRESHOLD_LC);
			A_Velocity.SetValue(false, -a_vel);
//		} else if ((ADC::Sensor::GetValue(ADC::Sensor::RF) > WALL_CTRL_THRESHOLD_F_RF_SL * k) && (ADC::Sensor::GetValue(ADC::Sensor::LF) < WALL_CTRL_THRESHOLD_F_LF_SL)) {
		} else if (Status::Sensor::GetValue(Status::Sensor::RC, false) > (SENSOR_CTRL_THRESHOLD_RC * k)) {
			a_vel = GAIN_P_WALL_SLANT * (float)(Status::Sensor::GetValue(Status::Sensor::RC, false) - SENSOR_CTRL_THRESHOLD_RC);
			A_Velocity.SetValue(false, a_vel);
		} else {
			A_Velocity.SetValue(false, 0.0);
		}
	}

	void Motor::AccelDecel(float velocity, float accel, bool half_block) {
		float deceleration_distance = 0.0;

		WallPFlag.SetValue(true);

		if (!half_block && (Velocity.GetValue(false) < SEARCH_SPEED) && (accel > 0.0)) {
			WallEdgeFlag.SetValue(false);
		} else {
			WallEdgeFlag.SetValue(true);
		}

		if (half_block) {
			Distance.SetValue(SECTION_STRAIGHT / 2.0);
		}

		if (accel < 0.0) {
			deceleration_distance = ((velocity * velocity - Velocity.GetValue(false) * Velocity.GetValue(false)) / (2.0 * accel));
			while ((Distance.GetValue() < (SECTION_STRAIGHT - deceleration_distance)) && ExecuteFlag.GetValue());
		}

		TargetVelocity.SetValue(velocity);
		Accel.SetValue(false, accel);

		while ((Distance.GetValue() < SECTION_STRAIGHT) && ExecuteFlag.GetValue()) {
			if ((accel < 0.0) && (Velocity.GetValue(false) <= 5.0)) {
				Velocity.SetValue(false, 5.0);
				Accel.SetValue(false, 0.0);
			}
		}

		if (TargetVelocity.GetValue() <= 1.0) {
			WallPFlag.SetValue(false);
			Status::Reset();
		}

		Distance.SetValue(0.0);
	}

	void Motor::AccelRun (unsigned char section, bool slant, float accel_target, float decel_target, float decel_length, float accel) {
		float section_length;

		WallEdgeFlag.SetValue(false);
		TargetVelocity.SetValue(accel_target);
		Accel.SetValue(false, accel);

		if (slant) {
			section_length = SECTION_SLANT;
			GyroCtrlFlag.SetValue(true);
			WallPFlag.SetValue(false);
			A_Accel.SetValue(false, 0.0);
			A_Velocity.SetValue(false, 0.0);
		} else {
			section_length = SECTION_STRAIGHT / 2.00;
			GyroCtrlFlag.SetValue(true);
			WallPFlag.SetValue(true);
		}

		while ((Distance.GetValue() < (section_length * (float)section) - decel_length) && ExecuteFlag.GetValue()) {
			if (slant) {
				Motor::CtrlAvoidObstacle(false);
			}
		}

		TargetVelocity.SetValue(decel_target);
		Accel.SetValue(false, -accel);
		A_Velocity.SetValue(false, 0.0);

		while ((Distance.GetValue() < (section_length * (float)section)) && ExecuteFlag.GetValue()) {
			if (slant) {
				if (Distance.GetValue() < (section_length * ((float)section - 0.35))) {
					Motor::CtrlAvoidObstacle(false);
				} else {
					break;
				}
			}

			if (Velocity.GetValue(false) <= 5.0) {
				Velocity.SetValue(false, 5.0);
				Accel.SetValue(false, 0.0);
			}
		}

		Distance.SetValue(0.0);
	}

	void Motor::Run(uint8_t block, bool half_block) {
		volatile uint8_t cnt = 0;

		WallPFlag.SetValue(true);
		WallEdgeFlag.SetValue(true);

		if (block > 0) {
			while (cnt < block) {
				while (((Distance.GetValue() < SECTION_STRAIGHT) && (Velocity.GetValue(false) > 0.0)) && ExecuteFlag.GetValue());
				Distance.SetValue(0.0);
				cnt++;
			}
		} else {
			Distance.SetValue(SECTION_STRAIGHT / 2.0);
			while (((Distance.GetValue() < SECTION_STRAIGHT) && (Velocity.GetValue(false) > 0.0)) && ExecuteFlag.GetValue());
			Distance.SetValue(0.0);
		}
	}

	void Motor::Slalom(int8_t dir) {
			uint8_t led_dir = 0;

//			if (dir > 0) {
//				led_dir = Interface::LED::Left;
//			} else {
//				led_dir = Interface::LED::Right;
//			}

			WallPFlag.SetValue(true);
			WallEdgeFlag.SetValue(false);

			Distance.SetValue(SECTION_STRAIGHT / 2.0);
			System::Interface::SetLEDColor(0, 0, 255, 0);

			if (dir > 0) {
				while ((Distance.GetValue() <= ((SECTION_STRAIGHT / 2.0) + slalom_param[0].distance_before_left)) && ExecuteFlag.GetValue());
			} else {
				while ((Distance.GetValue() <= ((SECTION_STRAIGHT / 2.0) + slalom_param[0].distance_before_right)) && ExecuteFlag.GetValue());
			}

			WallPFlag.SetValue(false);
			System::Interface::SetLEDColor(0, 0, 0, 0);

			A_Velocity.SetValue(false, 0.0);
			A_Accel.SetValue(false, (float)dir * slalom_param[0].angle_accel);
			Degree.SetValue(0.0);
//			Interface::LED::SetColor(Interface::LED::Purple, led_dir);

			if (dir > 0) {
				while ((A_Velocity.GetValue(false) <= (float)slalom_param[0].angle_velocity) && ExecuteFlag.GetValue()) {
					Distance.SetValue(0.0);
				}
			} else if (dir < 0) {
				while ((A_Velocity.GetValue(false) >= -(float)slalom_param[0].angle_velocity) && ExecuteFlag.GetValue()) {
					Distance.SetValue(0.0);
				}
			}

			A_Accel.SetValue(false, 0.0);
//			Interface::LED::SetColor(Interface::LED::None, led_dir);

			if (dir > 0) {
				while ((Degree.GetValue() <= (float)(slalom_param[0].turn_angle - slalom_param[0].clothoid_angle)) && ExecuteFlag.GetValue()) {
					Distance.SetValue(0.0);
				}
			} else if (dir < 0) {
				while ((Degree.GetValue() >= (float)(-slalom_param[0].turn_angle + slalom_param[0].clothoid_angle)) && ExecuteFlag.GetValue()) {
					Distance.SetValue(0.0);
				}
			}

			A_Accel.SetValue(false, -(float)dir * slalom_param[0].angle_accel);
//			Interface::LED::SetColor(Interface::LED::SBlue, led_dir);

			if (dir > 0) {
				while ((Degree.GetValue() <= (float)(slalom_param[0].turn_angle)) && (A_Velocity.GetValue(false) > 0.0) && ExecuteFlag.GetValue()) {
					Distance.SetValue(0.0);
				}
			} else if (dir < 0) {
				while ((Degree.GetValue() >= (float)(-slalom_param[0].turn_angle)) && (A_Velocity.GetValue(false) < 0.0) && ExecuteFlag.GetValue()) {
					Distance.SetValue(0.0);
				}
			}

			Distance.SetValue(0.0);
			A_Velocity.SetValue(false, 0.0);
			A_Accel.SetValue(false, 0.0);
			System::Interface::SetLEDColor(0, 0, 0, 255);

			WallPFlag.SetValue(false);

			if (dir > 0) {
				Distance.SetValue(SECTION_STRAIGHT - slalom_param[0].distance_after_left);
			} else {
				Distance.SetValue(SECTION_STRAIGHT - slalom_param[0].distance_after_right);
			}

			while ((Distance.GetValue() < SECTION_STRAIGHT) && ExecuteFlag.GetValue());

			Distance.SetValue(0.0);
			System::Interface::SetLEDColor(0, 0, 0, 0);
//			Interface::LED::SetColor(Interface::LED::None, led_dir);
		}

	void Motor::Slalom(unsigned char parameter, signed char dir) {
		bool walledge_detect = false, checked_walledge = false;
		unsigned char led_dir = 0;
		unsigned int phase_cnt = 0;
		float degree_start = 0.0, section_length = 0.0, phase = 0.0, target_av = 0.0, target_a_acc = 0.0;

		// TODO: 壁切れ状態/しきい値再チェック

		WallEdgeFlag.SetValue(false);
		WallPFlag.SetValue(true);
//		A_Velocity.SetValue(false, 0.0);

		if ((slalom_param[parameter].wall_correction & 0x02) == 0x02) {
//			WallPFlag.SetValue(true);

			if (dir == SLALOM_RIGHT) {
				System::Interface::SetLEDColor(1, 0, 255, 0);
				while ((Status::Sensor::GetValue(Status::Sensor::RC, false) < WALL_EDGE_THRESHOLD_F_RC) && ExecuteFlag.GetValue());
				System::Interface::SetLEDColor(0, 0, 255, 0);
				System::Interface::SetLEDColor(1, 0, 0, 0);

				while (ExecuteFlag.GetValue()) {
					if ((Status::Sensor::GetValue(Status::Sensor::RS, false) > WALL_EDGE_THRESHOLD_SH_RS)
							&& (Status::Sensor::GetValue(Status::Sensor::RC, false) < WALL_EDGE_THRESHOLD2_F_RC)) {
						Distance.SetValue(POSITION_EDGE_DETECT_SH_R);
						break;
					}

					if ((Status::Sensor::GetValue(Status::Sensor::RS, false) < WALL_EDGE_THRESHOLD_SH_RS)
							&& (Status::Sensor::GetValue(Status::Sensor::RC, false) < POLE_EDGE_THRESHOLD_F_RC)) {
						Distance.SetValue(POSITION_POLE_DETECT_SH_R);
						System::Interface::SetLEDColor(1, 255, 255, 255);
						break;
					}
				}

				System::Interface::SetLEDColor(0, 0, 0, 0);

				while ((Distance.GetValue() <= ((SECTION_STRAIGHT / 2.0) + slalom_param[parameter].distance_before_right)) && ExecuteFlag.GetValue());
				System::Interface::SetLEDColor(1, 0, 0, 0);
			} else if (dir == SLALOM_LEFT) {
				System::Interface::SetLEDColor(1, 255, 0, 0);
				while ((Status::Sensor::GetValue(Status::Sensor::LC, false) < WALL_EDGE_THRESHOLD_F_LC) && ExecuteFlag.GetValue());
				System::Interface::SetLEDColor(0, 255, 0, 0);
				System::Interface::SetLEDColor(1, 0, 0, 0);

				while (ExecuteFlag.GetValue()) {
					if ((Status::Sensor::GetValue(Status::Sensor::LS, false) > WALL_EDGE_THRESHOLD_SH_LS)
							&& (Status::Sensor::GetValue(Status::Sensor::LC, false) < WALL_EDGE_THRESHOLD2_F_LC)) {
						Distance.SetValue(POSITION_EDGE_DETECT_SH_L);
						break;
					}

					if ((Status::Sensor::GetValue(Status::Sensor::LS, false) < WALL_EDGE_THRESHOLD_SH_LS)
							&& (Status::Sensor::GetValue(Status::Sensor::LC, false) < POLE_EDGE_THRESHOLD_F_LC)) {
						Distance.SetValue(POSITION_POLE_DETECT_SH_L);
						System::Interface::SetLEDColor(1, 255, 255, 255);
						break;
					}
				}

				System::Interface::SetLEDColor(0, 0, 0, 0);

				while ((Distance.GetValue() <= ((SECTION_STRAIGHT / 2.0) + slalom_param[parameter].distance_before_left)) && ExecuteFlag.GetValue());
				System::Interface::SetLEDColor(1, 0, 0, 0);
			}
		} else if ((slalom_param[parameter].wall_correction & 0x08) == 0x08) {
			WallPFlag.SetValue(false);
			System::Interface::SetLEDColor(0, 255, 255, 0);

			while (!checked_walledge && ExecuteFlag.GetValue()) {
//				Motor::CtrlAvoidObstacle(true);
				checked_walledge = Status::DetectWallEdge_Slant(dir, false);
			}

			(void)Status::DetectWallEdge_Slant(0, true);
			System::Interface::SetLEDColor(0, 0, 0, 0);

			if (dir == SLALOM_RIGHT) {
				while ((Distance.GetValue() <= (SECTION_SLANT + slalom_param[parameter].distance_before_right)) && ExecuteFlag.GetValue()) {
					//				Motor::CtrlAvoidObstacle(true);
				}
			} else if (dir == SLALOM_LEFT) {
				while ((Distance.GetValue() <= (SECTION_SLANT + slalom_param[parameter].distance_before_left)) && ExecuteFlag.GetValue()) {
					//				Motor::CtrlAvoidObstacle(true);
				}
			}
		} else {
			WallPFlag.SetValue(false);

			if (dir == SLALOM_RIGHT) {
				while ((Distance.GetValue() <= ((SECTION_STRAIGHT / 2.0) + slalom_param[parameter].distance_before_right)) && ExecuteFlag.GetValue()) {
//					if (!Mystat::Status::CheckCtrlMode(CTRL_WALL_P)) {
//						Motor::CtrlAvoidObstacle(true);
//					}
				}
			} else if (dir == SLALOM_LEFT) {
				while ((Distance.GetValue() <= ((SECTION_STRAIGHT / 2.0) + slalom_param[parameter].distance_before_left)) && ExecuteFlag.GetValue()) {
//					if (!Mystat::Status::CheckCtrlMode(CTRL_WALL_P)) {
//						Motor::CtrlAvoidObstacle(true);
//					}
				}
			}
		}

//		Mystat::Status::ResetValue(false);
		WallPFlag.SetValue(false);
		System::Interface::SetLEDColor(0, 0, 0, 0);
		degree_start = Degree.GetValue();

//		Interface::LED::SetColor(Interface::LED::Purple, led_dir);

		phase_cnt = 0;
		phase = ((M_PI * slalom_param[parameter].angle_accel) / slalom_param[parameter].angle_velocity) * 0.001;

		if (dir > 0) {
			while ((A_Velocity.GetValue(false) <= (float)slalom_param[parameter].angle_velocity) && ExecuteFlag.GetValue()) {
				Distance.SetValue(0.0);

				target_av = (slalom_param[parameter].angle_velocity / 2.0) * (mysin_rad((phase * (float)phase_cnt) - (M_PI / 2)) + 1);
				target_a_acc = (slalom_param[parameter].angle_velocity / 2.0) * phase * mycos_rad((phase * (float)phase_cnt) - (M_PI / 2)) * 1000.0;
				A_Velocity.SetValue(false, target_av);
				A_Accel.SetValue(false, target_a_acc);
				phase_cnt++;
				System::Timer::wait_ms(1);

				if ((phase * (float)phase_cnt) >= M_PI) {
					break;
				}
			}
		} else if (dir < 0) {
			while ((A_Velocity.GetValue(false) >= -(float)slalom_param[parameter].angle_velocity) && ExecuteFlag.GetValue()) {
				Distance.SetValue(0.0);

				target_av = (slalom_param[parameter].angle_velocity / 2.0) * (mysin_rad((phase * (float)phase_cnt) - (M_PI / 2)) + 1);
				target_a_acc = (slalom_param[parameter].angle_velocity / 2.0) * phase * mycos_rad((phase * (float)phase_cnt) - (M_PI / 2)) * 1000.0;
				A_Velocity.SetValue(false, -target_av);
				A_Accel.SetValue(false, -target_a_acc);
				phase_cnt++;
				System::Timer::wait_ms(1);

				if ((phase * (float)phase_cnt) >= M_PI) {
					break;
				}
			}
		}

		phase_cnt = 0;

		A_Velocity.SetValue(false, (float)dir * slalom_param[parameter].angle_velocity);
		A_Accel.SetValue(false, 0.0);
//		Interface::LED::SetColor(Interface::LED::None, led_dir);

		if (dir > 0) {
			while ((Degree.GetValue() <= (degree_start + slalom_param[parameter].turn_angle - slalom_param[parameter].clothoid_angle - (A_Velocity.GetValue(false) * 0.001))) && ExecuteFlag.GetValue()) {
				Distance.SetValue(0.0);
			}
		} else if (dir < 0) {
			while ((Degree.GetValue() >= (degree_start - slalom_param[parameter].turn_angle + slalom_param[parameter].clothoid_angle - (A_Velocity.GetValue(false) * 0.001))) && ExecuteFlag.GetValue()) {
				Distance.SetValue(0.0);
			}
		}

//		Interface::LED::SetColor(Interface::LED::SBlue, led_dir);

		if (dir > 0) {
			while ((Degree.GetValue() <= (degree_start + slalom_param[parameter].turn_angle)) && (A_Velocity.GetValue(false) > 0.0) && ExecuteFlag.GetValue()) {
				Distance.SetValue(0.0);

				target_av = (slalom_param[parameter].angle_velocity / 2.0) * (mysin_rad((M_PI / 2) - (phase * (float)phase_cnt)) + 1);
				target_a_acc = (slalom_param[parameter].angle_velocity / 2.0) * phase * mycos_rad((M_PI / 2) - (phase * (float)phase_cnt)) * 1000.0;
				A_Velocity.SetValue(false, target_av);
				A_Accel.SetValue(false, -target_a_acc);
				phase_cnt++;
				System::Timer::wait_ms(1);

				if ((phase * (float)phase_cnt) >= M_PI) {
					break;
				}
			}
		} else if (dir < 0) {
			while ((Degree.GetValue() >= (degree_start - slalom_param[parameter].turn_angle)) && (A_Velocity.GetValue(false) < 0.0) && ExecuteFlag.GetValue()) {
				Distance.SetValue(0.0);

				target_av = (slalom_param[parameter].angle_velocity / 2.0) * (mysin_rad((M_PI / 2) - (phase * (float)phase_cnt)) + 1);
				target_a_acc = (slalom_param[parameter].angle_velocity / 2.0) * phase * mycos_rad((M_PI / 2) - (phase * (float)phase_cnt)) * 1000.0;
				A_Velocity.SetValue(false, -target_av);
				A_Accel.SetValue(false, target_a_acc);
				phase_cnt++;
				System::Timer::wait_ms(1);

				if ((phase * (float)phase_cnt) >= M_PI) {
					break;
				}
			}
		}

//		Mystat::Status::ResetValue(false);
		A_Velocity.SetValue(false, 0.0);
		A_Accel.SetValue(false, 0.0);

		if (dir > 0) {
			Degree.SetValue(degree_start + slalom_param[parameter].turn_angle);
		} else if (dir < 0) {
			Degree.SetValue(degree_start - slalom_param[parameter].turn_angle);
		}

//		Interface::LED::SetColor(Interface::LED::Red, led_dir);

		if ((slalom_param[parameter].wall_correction & 0x01) == 0x01) {
//			WallPFlag.SetValue(true);
			section_length = SECTION_STRAIGHT;
		} else {
			if ((slalom_param[parameter].wall_correction & 0x04) == 0x04) {
				if (dir == SLALOM_RIGHT) {
					System::Interface::SetLEDColor(0, 0, 255, 0);
				} else if (dir == SLALOM_LEFT) {
					System::Interface::SetLEDColor(0, 255, 0, 0);
				}
			}

			WallPFlag.SetValue(false);
			section_length = SECTION_SLANT;
		}

		if (dir > 0) {
			Distance.SetValue(section_length - slalom_param[parameter].distance_after_left);
		} else {
			Distance.SetValue(section_length - slalom_param[parameter].distance_after_right);
		}

		while (((Distance.GetValue() < section_length) || !checked_walledge) && ExecuteFlag.GetValue()) {
			if ((slalom_param[parameter].wall_correction & 0x04) == 0x04) {
//				Motor::CtrlAvoidObstacle(true);

				if (!checked_walledge) {
					checked_walledge = Status::DetectWallEdge_Slant(dir * -1, false);
				}
			} else {
				checked_walledge = true;
			}
		}

//		Mystat::Status::ResetValue(false);
		(void)Status::DetectWallEdge_Slant(0, true);
//		Interface::LED::SetColor(Interface::LED::None, Interface::LED::Both);
		System::Interface::SetLEDColor(0, 0, 0, 0);
		Distance.SetValue(0.0);
	}

	void Motor::Turning(bool opposite, int8_t dir) {
		float accel_distance = ((TURN_ANGLE_SPEED * TURN_ANGLE_SPEED) / (TURN_ANGLE_ACCEL * 2.0));
		float turn_angle = 0.0;
		int8_t turn_dir = dir;

		if (opposite) {
			turn_angle = TURN_ANGLE_OPPOSITE;
			turn_dir = 1;
		} else {
			turn_angle = TURN_ANGLE_RIGHT;
		}

		Status::Reset();
		A_Accel.SetValue(false, (float)turn_dir * TURN_ANGLE_ACCEL);
		WallPFlag.SetValue(false);
//		Mystat::Status::EnableWallEdgeDetect(false);

		if (turn_dir > 0) {
			while ((A_Velocity.GetValue(false) < TURN_ANGLE_SPEED) && ExecuteFlag.GetValue());
		} else {
			while ((A_Velocity.GetValue(false) > -TURN_ANGLE_SPEED) && ExecuteFlag.GetValue());
		}

		A_Accel.SetValue(false, 0.0);

		if (turn_dir > 0) {
			while ((Degree.GetValue() < (turn_angle - accel_distance)) && ExecuteFlag.GetValue());
		} else {
			while ((Degree.GetValue() > -(turn_angle - accel_distance)) && ExecuteFlag.GetValue());
		}

		A_Accel.SetValue(false, -(float)turn_dir * TURN_ANGLE_ACCEL);

		if (turn_dir > 0) {
			while ((Degree.GetValue() <= turn_angle) && (A_Velocity.GetValue(false) > 0.0) && ExecuteFlag.GetValue()) {
				if (A_Velocity.GetValue(false) < 2.5) {
					A_Velocity.SetValue(false, 2.5);
					A_Accel.SetValue(false, 0.0);
				}
			}
		} else {
			while ((Degree.GetValue() >= -turn_angle) && (A_Velocity.GetValue(false) < 0.0) && ExecuteFlag.GetValue()) {
				if (A_Velocity.GetValue(false) > -2.5) {
					A_Velocity.SetValue(false, -2.5);
					A_Accel.SetValue(false, 0.0);
				}
			}
		}

		Status::Reset();
//		Mystat::Status::AddSectionDistance(-10.0);
	}

	void Motor::BackAtBlindAllay() {
		WallPFlag.SetValue(false);

		TargetVelocity.SetValue(-SEARCH_SPEED / 2.0);
		Accel.SetValue(false, -SEARCH_ACCEL);

		System::Timer::wait_ms(300);
		GyroCtrlFlag.SetValue(false);
		System::Timer::wait_ms(200);

		TargetVelocity.SetValue(0.0);
		Accel.SetValue(false, SEARCH_ACCEL);

		while(Velocity.GetValue(false) < 0.0);

		Accel.SetValue(false, 0.0);

		Status::Reset();
		Distance.SetValue(POSITION_BACKWALL_CORRECTION);
	}

	void Motor::TestDetectEdge(bool slant, int8_t side) {
		Status::Calc::SetGyroReference();
		Status::Reset();
		ExecuteFlag.SetValue(true);
		PWM::Motor::Enable();

		if (slant) {
			PWM::Motor::AccelDecel(300.0, 3000.0, true);
			PWM::Motor::Slalom(3, SLALOM_RIGHT);
			while(Status::Sensor::GetValue(Status::Sensor::LC, false) < POLE_EDGE_THRESHOLD_SL_START);
			while(Status::Sensor::GetValue(Status::Sensor::LC, false) > POLE_EDGE_THRESHOLD_SL_END);
			System::Interface::SetLEDColor(0, 255, 255, 0);
			Distance.SetValue(POSITION_EDGE_DETECT_SL_L);
			while(Distance.GetValue() < SECTION_SLANT);
			Distance.SetValue(0.0);

			PWM::Motor::AccelRun(1, true, 300.0, 0.0, 38.4, 3000.0);
		} else {
			Distance.SetValue(POSITION_BACKWALL_CORRECTION);
			PWM::Motor::AccelDecel(SEARCH_SPEED, SEARCH_ACCEL, false);
//			PWM::Motor::Run(1, false);
			WallEdgeFlag.SetValue(false);

			while(1) {
				if (side > 0) {
					if ((Status::Sensor::GetValue(Status::Sensor::LS, false) > WALL_EDGE_THRESHOLD_SE_LS)
							&& (Status::Sensor::GetValue(Status::Sensor::LC, false) < WALL_EDGE_THRESHOLD2_F_LC)) {
						System::Interface::SetLEDColor(0, 0, 0, 255);
						Distance.SetValue(POSITION_EDGE_DETECT_F_L);
						break;
					}

					if ((Status::Sensor::GetValue(Status::Sensor::LS, false) < WALL_EDGE_THRESHOLD_SE_LS)
							&& (Status::Sensor::GetValue(Status::Sensor::LC, false) < POLE_EDGE_THRESHOLD_F_LC)) {
						System::Interface::SetLEDColor(0, 255, 0, 0);
						Distance.SetValue(POSITION_POLE_DETECT_F_L);
						break;
					}
				} else {
					if ((Status::Sensor::GetValue(Status::Sensor::RS, false) > WALL_EDGE_THRESHOLD_SE_RS)
							&& (Status::Sensor::GetValue(Status::Sensor::RC, false) < WALL_EDGE_THRESHOLD2_F_RC)) {
						System::Interface::SetLEDColor(0, 0, 0, 255);
						Distance.SetValue(POSITION_EDGE_DETECT_F_R);
						break;
					}

					if ((Status::Sensor::GetValue(Status::Sensor::RS, false) < WALL_EDGE_THRESHOLD_SE_RS)
							&& (Status::Sensor::GetValue(Status::Sensor::RC, false) < POLE_EDGE_THRESHOLD_F_RC)) {
						System::Interface::SetLEDColor(0, 255, 0, 0);
						Distance.SetValue(POSITION_POLE_DETECT_F_R);
						break;
					}
				}
			}

			while(Distance.GetValue() < SECTION_STRAIGHT);
			Distance.SetValue(0.0);

			PWM::Motor::AccelDecel(0.0, -SEARCH_ACCEL, true);
		}

		System::Timer::wait_ms(1000);
		System::Interface::SetLEDColor(0, 0, 0, 0);
		PWM::Motor::Disable();
		ExecuteFlag.SetValue(false);
	}

	void Motor::TestSlalom(bool use_fan) {
		char senddata[127];
//		RECORD_LOG = true;

		Status::Calc::SetGyroReference();
		Status::Reset();
		ExecuteFlag.SetValue(true);
		PWM::Motor::Enable();

		Distance.SetValue(POSITION_BACKWALL_CORRECTION);
		PWM::Motor::AccelDecel(400.0, SEARCH_ACCEL, false);
//		PWM::Motor::Run(1, false);

		PWM::Motor::Slalom(12, SLALOM_LEFT);
//		PWM::Motor::Slalom(7, SLALOM_RIGHT);
//		PWM::Motor::Slalom(SLALOM_RIGHT);
//		PWM::Motor::Run(1, false);
//		PWM::Motor::Slalom(SLALOM_RIGHT);
//		PWM::Motor::Slalom(SLALOM_LEFT);
//		PWM::Motor::Run(1, false);
//		PWM::Motor::Slalom(SLALOM_LEFT);
		PWM::Motor::AccelDecel(0.0, -SEARCH_ACCEL, false);
		System::Timer::wait_ms(1000);

		PWM::Motor::Disable();
		ExecuteFlag.SetValue(false);

		while(SW_NEXT == 1);
		System::Interface::SetLEDColor(0, 0, 255, 0);
		for (uint16_t cnt = 0; cnt < LOGSIZE; cnt++) {
			sprintf(senddata, "%d, %d, %d, %d, %f, %f, %f, %f\n",
					log_sen_ls[cnt], log_sen_lc[cnt], log_sen_rc[cnt], log_sen_rs[cnt],
					log_v_target[cnt], log_v_actual[cnt], log_a_v_target[cnt], log_a_v_actual[cnt]);
			System::SCI::SendChar(senddata);
		}

		ExecuteFlag.SetValue(false);

		System::Timer::wait_ms(1000);
		System::Interface::SetLEDColor(0, 0, 0, 0);

//		Mystat::Status::SendLogData();
	}

	void Motor::TestWheelDiameter() {
		Status::Calc::SetGyroReference();
		Status::Reset();
		ExecuteFlag.SetValue(true);
		PWM::Motor::Enable();

		PWM::Motor::AccelDecel(SEARCH_SPEED, SEARCH_ACCEL, true);
		PWM::Motor::Run(14, false);
		PWM::Motor::AccelDecel(0.0, -SEARCH_ACCEL, true);

		System::Timer::wait_ms(1000);
		PWM::Motor::Disable();
		ExecuteFlag.SetValue(false);
	}
}

/*

	ExecuteFlag.SetValue(true);
	PWM::Motor::Enable();

	Accel.SetValue(false, 2000.0F);
	while(Velocity.GetValue(false) < 360.0F);
	Velocity.SetValue(false, 360.0F);
	Accel.SetValue(false, -2000.0F);
	while(Velocity.GetValue(false) > 240.0F);
	Velocity.SetValue(false, 240.0F);
	Accel.SetValue(false, 0.0F);
	System::Timer::wait_ms(1000);

	PWM::Motor::Disable();
	ExecuteFlag.SetValue(false);

 */
