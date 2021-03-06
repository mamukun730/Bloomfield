/*
 * pwm.hpp
 *
 *  Created on: 2017/09/04
 *      Author: mmkn
 */

#ifndef INCLUDE_PWM_HPP_
#define INCLUDE_PWM_HPP_

#ifdef __cplusplus
//extern "C" void Timer_MTU4A();
//extern "C" void Timer_MTU4B();
//extern "C" void Timer_MTU4D();
#endif

#define MOTOR_CTRL_L PORT5.PODR.BIT.B4
#define MOTOR_CTRL_R PORT3.PODR.BIT.B1

// Hz
// http://hpcgi3.nifty.com/prismwave/wiki/wiki.cgi?p=%B2%BB%B3%AC%A4%C8%BC%FE%C7%C8%BF%F4
// http://www.yk.rim.or.jp/~kamide/music/notes.html
const static float O1_Ra		= 110.00;
const static float O1_Ra_Sharp	= 116.54;
const static float O1_Si		= 123.47;

const static float O2_Do		= 130.81;
const static float O2_Do_Sharp	= 138.59;
const static float O2_Re		= 146.83;
const static float O2_Re_Sharp	= 155.56;
const static float O2_Mi		= 164.81;
const static float O2_Fa		= 174.61;
const static float O2_Fa_Sharp	= 184.99;
const static float O2_Sol		= 195.99;
const static float O2_Sol_Sharp	= 207.65;
const static float O2_Ra		= 220.00;
const static float O2_Ra_Sharp	= 233.08;
const static float O2_Si		= 246.94;

const static float O3_Do		= 261.62;
const static float O3_Do_Sharp	= 277.18;
const static float O3_Re		= 293.66;
const static float O3_Re_Sharp	= 311.12;
const static float O3_Mi		= 329.62;
const static float O3_Fa		= 349.22;
const static float O3_Fa_Sharp	= 369.99;
const static float O3_Sol		= 391.99;
const static float O3_Sol_Sharp	= 415.30;
const static float O3_Ra		= 440.00;
const static float O3_Ra_Sharp	= 466.16;
const static float O3_Si		= 493.88;

const static float O4_Do		= 523.25;
const static float O4_Do_Sharp	= 554.36;
const static float O4_Re		= 587.32;
const static float O4_Re_Sharp	= 622.25;
const static float O4_Mi		= 659.25;
const static float O4_Fa		= 698.45;
const static float O4_Fa_Sharp	= 739.98;
const static float O4_Sol		= 783.99;
const static float O4_Sol_Sharp	= 830.60;
const static float O4_Ra		= 880.00;
const static float O4_Ra_Sharp	= 932.32;
const static float O4_Si		= 987.76;

const static float O5_Do		= 1046.50;
const static float O5_Do_Sharp	= 1108.73;
const static float O5_Re		= 1174.65;
const static float O5_Re_Sharp	= 1244.50;
const static float O5_Mi		= 1318.51;
const static float O5_Fa		= 1396.91;
const static float O5_Fa_Sharp	= 1479.91;
const static float O5_Sol		= 1567.98;
const static float O5_Sol_Sharp	= 1661.21;
const static float O5_Ra		= 1760.00;
const static float O5_Ra_Sharp	= 1864.65;
const static float O5_Si		= 1975.53;

const static float O6_Do		= 2093.00;
const static float O6_Do_Sharp	= 2217.46;
const static float O6_Re		= 2349.32;
const static float O6_Re_Sharp	= 2489.02;
const static float O6_Mi		= 2637.02;
const static float O6_Fa		= 2793.83;
//const static float O6_Fa_Sharp	= 1479.91;
//const static float O6_Sol		= 1567.98;
//const static float O6_Sol_Sharp	= 1661.21;
//const static float O6_Ra		= 1760.00;
//const static float O6_Ra_Sharp	= 1864.65;
//const static float O6_Si		= 1975.53;

namespace PWM {
	struct _slalom_param {
			const volatile float angle_velocity;
			const volatile float angle_accel;
			const volatile float distance_before_left;
			const volatile float distance_before_right;
			const volatile float distance_after_left;
			const volatile float distance_after_right;
			const volatile float turn_angle;
			const volatile uint8_t clothoid_angle;
			const volatile uint8_t wall_correction;
	};

	class Buzzer {
		public:
			static void Init();
			static void Enable();
			static void Disable();
			static void SetDuty(float hz);
			static void Melody_M_Coin();
			static void Melody_M_Dead();
			static void Melody_M_UpMush();

			static void Melody_TX1();
			static void Melody_TX3();

			static void Melody_GK1_1();
			static void Melody_GK8_1();
			static void Melody_TE3_1();
			static void Melody_TE10_1();
			static void Melody_TE16_1();
			static void Melody_TE26_1();
			static void Melody_TE32_1();

			static void Melody_Namiki();
			static void Melody_Shibuya_Otogi();

			static void Melody_FoxMovie();

			static void Melody_OsakaLoop();
			static void Melody_Oedo_Door();
			static void Melody_ProtectionRadio();
	};

	class Score {
		public:
			void SetScore_TE32_1();
			void ResetScore();
			void PlayScore(bool background);

		private:
			enum Scale {
				_O1_Ra, _O1_Ra_Sharp, _O1_Si,
				_O2_Do, _O2_Do_Sharp, _O2_Re, _O2_Re_Sharp, _O2_Mi, _O2_Fa, _O2_Fa_Sharp, _O2_Sol, _O2_Sol_Sharp, _O2_Ra, _O2_Ra_Sharp, _O2_Si,
				_O3_Do, _O3_Do_Sharp, _O3_Re, _O3_Re_Sharp, _O3_Mi, _O3_Fa, _O3_Fa_Sharp, _O3_Sol, _O3_Sol_Sharp, _O3_Ra, _O3_Ra_Sharp, _O3_Si,
				_O4_Do, _O4_Do_Sharp, _O4_Re, _O4_Re_Sharp, _O4_Mi, _O4_Fa, _O4_Fa_Sharp, _O4_Sol, _O4_Sol_Sharp, _O4_Ra, _O4_Ra_Sharp, _O4_Si,
				_O5_Do, _O5_Do_Sharp, _O5_Re, _O5_Re_Sharp, _O5_Mi, _O5_Fa, _O5_Fa_Sharp, _O5_Sol, _O5_Sol_Sharp, _O5_Ra, _O5_Ra_Sharp, _O5_Si,
				_O6_Do, _O6_Do_Sharp, _O6_Re, _O6_Re_Sharp, _O6_Mi, _O6_Fa,
			};

			uint8_t tempo, beat, scale[100], rhythm[100], note_num;
			uint16_t elapsed_time;
			bool nowplaying;
	};

	class Motor {
		public:
			enum DriveDir {
				Forward, Back, Turn
			};

			static void Init();
			static void Enable();
			static void Disable();
			static void SetDriveDir(unsigned char dir);
			static void SetDuty();

			static void AccelDecel(float velocity, float accel, bool half_block);
			static void Run(uint8_t block, bool half_block);
			static void AccelRun (uint8_t section, bool slant, float accel_target, float decel_target, float decel_length, float accel);
			static void Turning(bool opposite, int8_t dir);
			static void BackAtBlindAllay();
			static void Return();
			static void Slalom(uint8_t parameter_num, int8_t dir);
			static void Slalom(int8_t dir);

			static void AdjustPosture_Angle();

			static void CtrlAvoidObstacle(bool exit_area);

			static void TestPIDGain();
			static void TestWheelDiameter();
			static void TestDetectEdge(bool slant, int8_t side);
			static void TestSlalom(bool use_fan);

		private:
			static struct _slalom_param slalom_param[NUMBER_SLALOM_PARAM + 1];
	};
}

#endif /* INCLUDE_PWM_HPP_ */
