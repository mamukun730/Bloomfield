/*
 * pwm.cpp
 *
 *  Created on: 2017/09/04
 *      Author: mmkn
 */

#include "common.hpp"
#include "pwm.hpp"

namespace PWM {
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
		MTU.TSTR.BIT.CST4 = 0;

		IPR(MTU4, TGIA4) = 15;
		IEN(MTU4, TGIA4) = 1;

		IPR(MTU4, TGIB4) = 14;
		IEN(MTU4, TGIB4) = 1;

		IPR(MTU4, TGID4) = 13;
		IEN(MTU4, TGID4) = 1;

		MTU4.TCNT = 0x00;
		MTU4.TCR.BIT.TPSC = 0;			// PCLK / 1
		MTU4.TCR.BIT.CKEG = 0;			// 立ち上がりカウント
		MTU4.TCR.BIT.CCLR = 1;			// Reset: TGRA
		MTU4.TMDR.BIT.MD = 2;

		MTU4.TIER.BIT.TGIEA = 1;
		MTU4.TIER.BIT.TGIEB = 1;
		MTU4.TIER.BIT.TGIED = 1;

		MTU4.TGRA = MOTOR_OPCYCLE;
		MTU4.TGRB = MOTOR_OPCYCLE;
		MTU4.TGRC = MOTOR_OPCYCLE;
		MTU4.TGRD = MOTOR_OPCYCLE;

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

	void Motor::Enable() {
		MTU4.TGRA = MOTOR_OPCYCLE;
		MTU4.TGRB = MOTOR_OPCYCLE;
		MTU4.TGRC = MOTOR_OPCYCLE;
		MTU4.TGRD = MOTOR_OPCYCLE;

		PORT3.PODR.BIT.B1 = 0;
		PORT5.PODR.BIT.B4 = 0;

//		Interface::Encoder::Enable();
		MTU.TSTR.BIT.CST4 = 1;
	}

	void Motor::Disable() {
//		Interface::Encoder::Disable();

		MTU4.TGRA = MOTOR_OPCYCLE;
		MTU4.TGRB = MOTOR_OPCYCLE;
		MTU4.TGRC = MOTOR_OPCYCLE;
		MTU4.TGRD = MOTOR_OPCYCLE;

		MOTOR_CTRL_L = 0;
		MOTOR_CTRL_R = 0;

		PORT3.PODR.BIT.B1 = 0;
		PORT5.PODR.BIT.B4 = 0;

		MTU.TSTR.BIT.CST4 = 0;
	}
}

void Timer_MTU4A() {
	PORT3.PODR.BIT.B1 = 0;
	PORT5.PODR.BIT.B4 = 0;
}

void Timer_MTU4B() {
	PORT3.PODR.BIT.B1 = 1;
}

void Timer_MTU4D() {
	PORT5.PODR.BIT.B4 = 1;
}
