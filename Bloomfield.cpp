/***************************************************************/
/*                                                             */
/*      PROJECT NAME :  Bloomfield                             */
/*      FILE         :  Bloomfield.cpp                         */
/*      DESCRIPTION  :  Main Program                           */
/*      CPU SERIES   :  RX600                                  */
/*      CPU TYPE     :  RX631                                  */
/*                                                             */
/*      This file is generated by e2 studio.                   */
/*                                                             */
/***************************************************************/                                
                                                                           
                                                                           
/************************************************************************/
/*    File Version: V1.00                                               */
/*    Date Generated: 08/07/2013                                        */
/************************************************************************/

#include "include/common.hpp"

int16_t log_cnt = 0;
float logdata1[LOGSIZE], logdata2[LOGSIZE], logdata3[LOGSIZE];

Status::Value Velocity;
Status::Value2 VelocityDiff;
Status::Value2 TargetVelocity;
Status::Value Accel;

Status::Value A_Velocity;
Status::Value2 A_VelocityDiff;
Status::Value2 TargetA_Velocity;
Status::Value A_Accel;

Status::Value2 Degree;
Status::Value2 Distance;
Status::Value2 GyroRef;

Status::Flag ExecuteFlag;
Status::Flag WallPFlag;

#ifdef CPPAPP
//Initialize global constructors
extern "C" void __main()
{
  static int initialized;
  if (! initialized)
    {
      typedef void (*pfunc) ();
      extern pfunc __ctors[];
      extern pfunc __ctors_end[];
      pfunc *p;

      initialized = 1;
      for (p = __ctors_end; p > __ctors; )
    (*--p) ();

    }
}
#endif 

int main(void) {
	char senddata[127];
	log_cnt = 0;

	System::SetUp::Clock();
	System::SetUp::IO();
	System::SetUp::Functions();
	System::SetUp::CreateClass();

	System::Timer::Switch(0, 1);
	System::Timer::Switch(1, 1);

	System::Interface::InitLED();
	System::SetUp::StartCheck();
	Mystat::Map::Init();

	PWM::Buzzer::Melody_M_Coin();
//	PWM::Buzzer::Melody_FoxMovie();

	while(1) {
		System::Interface::ModeSelect();
		System::Interface::SetLEDColor(0, 0, 255, 0);
		PWM::Buzzer::Melody_TE32_1();
		System::Interface::SetLEDColor(0, 0, 0, 0);

		System::Timer::wait_ms(1000);
		Mystat::Position::Reset();
		Mystat::Map::Search_Adachi(GOAL_X, GOAL_Y, false, false);
		ExecuteFlag.SetValue(false);

		System::Timer::wait_ms(1000);
	}

//	while(SW_PREV == 1);
//	System::Interface::SetLEDColor(0, 0, 0, 255);
//	System::Timer::wait_ms(2000);
//	Status::Calc::SetGyroReference();
//	System::Timer::wait_ms(1000);
//
//	ExecuteFlag.SetValue(true);
//	PWM::Motor::Enable();
//	PWM::Motor::AccelDecel(240.0, 3000.0, true);
//	PWM::Motor::Run(7, false);
//	PWM::Motor::AccelDecel(0.0, -3000.0, true);
//	PWM::Motor::Disable();
//	System::Timer::wait_ms(1000);
//
//	while(SW_NEXT == 1);
//	System::Interface::SetLEDColor(0, 0, 255, 0);
//	for (uint16_t cnt = 0; cnt < LOGSIZE; cnt++) {
//		sprintf(senddata, "%f, %f, %f\n", logdata1[cnt], logdata2[cnt], logdata3[cnt]);
//		System::SCI::SendChar(senddata);
//	}

	return 0;
}
