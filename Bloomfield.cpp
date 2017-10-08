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
	System::Interface::SetLEDColor(0, 0, 255, 0);
	System::Interface::SetLEDColor(1, 0, 0, 0);

//	System::Interface::ModeSelect();

//	while(1) {
//		sprintf(senddata, "%4d, %4d, %4d, %4d, %4d, %4d, %4d\n", Status::Sensor::GetValue(Status::Sensor::LS, false), Status::Sensor::GetValue(Status::Sensor::LC, false), Status::Sensor::GetValue(Status::Sensor::LF, false), Status::Sensor::GetValue(Status::Sensor::F, false), Status::Sensor::GetValue(Status::Sensor::RF, false), Status::Sensor::GetValue(Status::Sensor::RC, false), Status::Sensor::GetValue(Status::Sensor::RS, false));
//		System::SCI::SendChar(senddata);
//		System::Timer::wait_ms(1000);
//	}

	while(1) {
		while(SW_PREV == 1);
		System::Interface::SetLEDColor(0, 0, 0, 255);
		PWM::Buzzer::Melody_TE32_1();

		System::Timer::wait_ms(1000);
		Mystat::Position::Reset();
		Mystat::Map::Search_Adachi(GOAL_X, GOAL_Y, false, false);
		ExecuteFlag.SetValue(false);

		System::Timer::wait_ms(1000);

		while(SW_PREV == 1);
		Mystat::Map::SendData();
		System::Timer::wait_ms(1000);
	}

/*	System::Timer::wait_ms(2000);

	System::Interface::SetLEDColor(0, 0, 255, 0);

	ExecuteFlag.SetValue(true);
	PWM::Motor::Enable();

	PWM::Motor::Turning(false, SLALOM_LEFT);
	System::Timer::wait_ms(100);

	PWM::Motor::Disable();
	ExecuteFlag.SetValue(false);

	System::Interface::SetLEDColor(0, 0, 0, 255);
	while(SW_NEXT == 1);
	System::Interface::SetLEDColor(0, 0, 255, 0);

	for (uint16_t cnt = 0; cnt < LOGSIZE; cnt++) {
		sprintf(senddata, "%f, %f, %f\n", logdata1[cnt], logdata2[cnt], logdata3[cnt]);
		System::SCI::SendChar(senddata);
		}

	System::Interface::SetLEDColor(0, 0, 255, 0);*/

/*	System::Timer::wait_ms(2000);

	System::Interface::SetLEDColor(0, 0, 255, 0);

	ExecuteFlag.SetValue(true);
	PWM::Motor::Enable();

	Accel.SetValue(false, 2000.0F);
	while(Velocity.GetValue(false) < 240.0F);
	Velocity.SetValue(false, 240.0F);

	System::Timer::wait_ms(100);

	Accel.SetValue(false, -2000.0F);
	while(Velocity.GetValue(false) > 0.0F);
	Velocity.SetValue(false, 0.0F);
	Accel.SetValue(false, 0.0F);
	System::Timer::wait_ms(1000);

	PWM::Motor::Disable();
	ExecuteFlag.SetValue(false);

	System::Interface::SetLEDColor(0, 0, 0, 255);
	while(SW_NEXT == 1);
	System::Interface::SetLEDColor(0, 0, 255, 0);

	for (uint16_t cnt = 0; cnt < LOGSIZE; cnt++) {
		sprintf(senddata, "%f, %f, %f\n", logdata1[cnt], logdata2[cnt], logdata3[cnt]);
		System::SCI::SendChar(senddata);
	}

	System::Interface::SetLEDColor(0, 0, 255, 0);*/

/*	System::Timer::wait_ms(2000);
	System::Interface::SetLEDColor(0, 0, 255, 0);

	ExecuteFlag.SetValue(true);

	System::Timer::wait_ms(1000);

	ExecuteFlag.SetValue(false);

	System::Interface::SetLEDColor(0, 0, 0, 255);
	while(SW_NEXT == 1);
	System::Interface::SetLEDColor(0, 0, 255, 0);

	for (uint16_t cnt = 0; cnt < LOGSIZE; cnt++) {
		sprintf(senddata, "%d, %f\n", cnt, logdata1[cnt]);
		System::SCI::SendChar(senddata);
	}

	System::Interface::SetLEDColor(0, 0, 255, 0);*/

	return 0;
}