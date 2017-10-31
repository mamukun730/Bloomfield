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

uint16_t log_cnt = 0;
uint16_t log_sen_ls[LOGSIZE], log_sen_lc[LOGSIZE], log_sen_rc[LOGSIZE], log_sen_rs[LOGSIZE];
float log_v_actual[LOGSIZE], log_v_target[LOGSIZE], log_a_v_actual[LOGSIZE], log_a_v_target[LOGSIZE];

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
Status::Flag WallEdgeFlag;
Status::Flag GyroCtrlFlag;

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

	PWM::Buzzer::Melody_M_Coin();
//	PWM::Buzzer::Melody_FoxMovie();

	while(1) {
		System::Interface::ModeSelect();
		System::Interface::SetLEDColor(0, 0, 255, 0);
		PWM::Buzzer::Melody_TE32_1();
		System::Interface::SetLEDColor(0, 0, 0, 0);

		System::Timer::wait_ms(1000);
		Mystat::Position::Reset();

		switch (System::Interface::GetExecuteMode()) {
			case 1:
				Mystat::Map::Search_Adachi(GOAL_X, GOAL_Y, false, true, false);

				if (ExecuteFlag.GetValue()) {
					Mystat::Map::Search_Adachi(START_X, START_Y, false, true, true);
				}
				break;

			case 2:
				Mystat::Map::Search_Adachi(GOAL_X, GOAL_Y, false, true, true);
				break;

			case 3:
//				Mystat::Map::Search_Adachi(GOAL_X, GOAL_Y, false, false, false);
//				Mystat::Map::SendData();
				Mystat::Map::MakePath(480, 320, 320, 3000);
				Mystat::Map::ReadPath(480, 320, 320, 3000);
				break;

			case 4:
//				PWM::Motor::TestSlalom(false);
//				PWM::Motor::TestDetectEdge(false, SLALOM_RIGHT);
				Mystat::Map::MakePath(640, 320, 320, 3000);
				Mystat::Map::ReadPath(640, 320, 320, 3000);

//				while(1) {
//					sprintf(senddata, "%4d, %4d, %4d, %4d, %4d, %4d, %4d\n", Status::Sensor::GetValue(Status::Sensor::LS, false), Status::Sensor::GetValue(Status::Sensor::LC, false), Status::Sensor::GetValue(Status::Sensor::LF, false), Status::Sensor::GetValue(Status::Sensor::F, false), Status::Sensor::GetValue(Status::Sensor::RF, false), Status::Sensor::GetValue(Status::Sensor::RC, false), Status::Sensor::GetValue(Status::Sensor::RS, false));
//					System::SCI::SendChar(senddata);
//					System::Timer::wait_ms(1000);
//				}
				break;

			case 5:
				if (System::Flash::EraseBlock(BLOCK_DB0)) {
					Mystat::Map::Init();
					System::Interface::SetLEDColor(0, 0, 255, 0);
				} else {
					System::Interface::SetLEDColor(0, 255, 0, 0);
				}

			default:
				break;
		}

//		while(SW_NEXT == 1);
//		System::Interface::SetLEDColor(0, 0, 0, 255);
//		System::Timer::wait_ms(2000);
//		Status::Calc::SetGyroReference();
//		System::Timer::wait_ms(1000);
//
//		ExecuteFlag.SetValue(true);
//		PWM::Motor::Enable();
//		PWM::Motor::AccelDecel(SEARCH_SPEED, SEARCH_ACCEL, false);
//	//	PWM::Motor::Run(4, false);
//		PWM::Motor::AccelDecel(0.0, -SEARCH_ACCEL, false);
//	//	PWM::Motor::AccelDecel(480.0, 4000.0, true);
//	//	PWM::Motor::Slalom(3, SLALOM_LEFT);
//	//	PWM::Motor::AccelRun(4, true, 480.0, 480.0, 0.0, 4000.0);
//	//	System::Timer::wait_ms(1500);
//		PWM::Motor::Disable();
//		ExecuteFlag.SetValue(false);
//		System::Timer::wait_ms(1000);
//
//		while(SW_NEXT == 1);
//		System::Interface::SetLEDColor(0, 0, 255, 0);
//		for (uint16_t cnt = 0; cnt < LOGSIZE; cnt++) {
//			sprintf(senddata, "%d, %d, %d, %d, %f, %f, %f, %f\n",
//					log_sen_ls[cnt], log_sen_lc[cnt], log_sen_rc[cnt], log_sen_rs[cnt],
//					log_v_target[cnt], log_v_actual[cnt], log_a_v_target[cnt], log_a_v_actual[cnt]);
//			System::SCI::SendChar(senddata);
//		}

		ExecuteFlag.SetValue(false);

		System::Timer::wait_ms(1000);
		System::Interface::SetLEDColor(0, 0, 0, 0);

		while(SW_NEXT == 1);
	}

	return 0;
}
