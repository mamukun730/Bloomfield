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
			const volatile float distance_before;
			const volatile float distance_after_left;
			const volatile float distance_after_right;
			const volatile float turn_angle;
			const volatile unsigned char clothoid_angle;
			const volatile unsigned char wall_correction;

			wall_correction:
			bit	3		2		1		0
				Sl_IN	Sl_OUT	St_IN	St_OUT
 */
			{ 509.2958, 5187.6446,  8.000,  8.000, 8.000,  90.000, 25, 0x03 },		// 000: 90度小, 240mm/s, r=27
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
		float ff_r = 0.0, ff_l = 0.0, fb_r = 0.0, fb_l = 0.0, battery_v = 0.0;

		static uint16_t cnt = 0;

		if (WallPFlag.GetValue()) {
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

			//A_VelocityDiff.SetValue(0.0);
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

		v_last_diff = v_target - v_actual;
		av_last_diff = av_target - av_actual;

		MTU0.TCNT = 0x00;

		if (fb_r > 0.0) {
			MOTOR_CTRL_R = 1;
		} else {
			MOTOR_CTRL_R = 0;
		}

		if (fb_l > 0.0) {
			MOTOR_CTRL_L = 1;
		} else {
			MOTOR_CTRL_L = 0;
		}

		fb_r = fabsf(fb_r);
		fb_l = fabsf(fb_l);

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

	void Motor::AccelDecel(float velocity, float accel, bool half_block) {
		float deceleration_distance = 0.0;

		WallPFlag.SetValue(true);

		if (half_block) {
//			if ((velocity < 1.0) && Status::Sensor::CheckWallExist(SIDE_FORWARD)) {
//				Distance.SetValue(35.0);
//			} else {
				Distance.SetValue(SECTION_STRAIGHT / 2.0);
//			}
		}

		if (accel < 0.0) {
			WallEdgeFlag.SetValue(false);

			deceleration_distance = ((velocity * velocity - Velocity.GetValue(false) * Velocity.GetValue(false)) / (2.0 * accel));
			while ((Distance.GetValue() < (SECTION_STRAIGHT - deceleration_distance)) && ExecuteFlag.GetValue());
		} else {
			WallEdgeFlag.SetValue(true);
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

			while ((Distance.GetValue() <= ((SECTION_STRAIGHT / 2.0) + slalom_param[0].distance_before)) && ExecuteFlag.GetValue());

			WallPFlag.SetValue(false);

			A_Velocity.SetValue(false, 0.0);
			A_Accel.SetValue(false, (float)dir * slalom_param[0].angle_accel);
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

//			Interface::LED::SetColor(Interface::LED::Red, led_dir);

			WallPFlag.SetValue(false);

			if (dir > 0) {
				Distance.SetValue(SECTION_STRAIGHT - slalom_param[0].distance_after_left);
			} else {
				Distance.SetValue(SECTION_STRAIGHT - slalom_param[0].distance_after_right);
			}

			while ((Distance.GetValue() < SECTION_STRAIGHT) && ExecuteFlag.GetValue());

			Distance.SetValue(0.0);
//			Interface::LED::SetColor(Interface::LED::None, led_dir);
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

	void Motor::TestDetectEdge(bool slant) {
		Status::Calc::SetGyroReference();
		Status::Reset();
		ExecuteFlag.SetValue(true);
		PWM::Motor::Enable();

		if (slant) {
//			PWM::Motor::Accel(720.0, 8000.0, true);
//			PWM::Motor::Slalom(3, SLALOM_RIGHT);
//			while(ADC::Sensor::GetValue(ADC::Sensor::LS) < WALL_EDGE_THRESHOLD_F_LS_SL);
//			while(ADC::Sensor::GetValue(ADC::Sensor::LS) > WALL_EDGE_THRESHOLD2_F_LS_SL);
//			Interface::LED::SetColor(Interface::LED::Yellow, Interface::LED::Left);
//			Mystat::Status::SetSectionDistance(false, POSITION_EDGE_DETECT_F_LS_SL);
//			while(Mystat::Status::GetSectionDistance() < SECTION_SLANT);
//			Mystat::Status::SetSectionDistance(true, 0.0);
//
//			Interface::LED::SetColor(Interface::LED::None, Interface::LED::Left);
//			PWM::Motor::AccelRun(1, true, 720.0, 0.0, 32.4, 8000.0);
		} else {
			PWM::Motor::AccelDecel(SEARCH_SPEED, SEARCH_ACCEL, true);
			while(Status::Sensor::GetValue(Status::Sensor::LS, false) < WALL_EDGE_THRESHOLD_F_LS);
			while(Status::Sensor::GetValue(Status::Sensor::LC, false) > WALL_EDGE_THRESHOLD2_F_LC);
			Distance.SetValue(POSITION_EDGE_DETECT_F_L);
			while(Distance.GetValue() < SECTION_STRAIGHT);
			Distance.SetValue(0.0);

			PWM::Motor::AccelDecel(0.0, -SEARCH_SPEED, true);
		}

		System::Timer::wait_ms(1000);
		PWM::Motor::Disable();
		ExecuteFlag.SetValue(false);
	}

	void Motor::TestSlalom(bool use_fan) {
//		RECORD_LOG = true;

		Status::Calc::SetGyroReference();
		Status::Reset();
		ExecuteFlag.SetValue(true);
		PWM::Motor::Enable();

		PWM::Motor::AccelDecel(SEARCH_SPEED, SEARCH_ACCEL, true);
		PWM::Motor::Run(2, false);

//		PWM::Motor::Slalom(SLALOM_RIGHT);
		PWM::Motor::Slalom(SLALOM_LEFT);

		PWM::Motor::AccelDecel(0.0, -SEARCH_SPEED, true);

		System::Timer::wait_ms(1000);
		PWM::Motor::Disable();
		ExecuteFlag.SetValue(false);

//		Mystat::Status::SendLogData();
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
