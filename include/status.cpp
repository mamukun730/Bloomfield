/*
 * status.cpp
 *
 *  Created on: 2017/09/16
 *	  Author: mmkn
 */

#include "common.hpp"
#include "status.hpp"

namespace Status {
	void Reset() {
//		Calc::RenewActualVelocity(true);
		Calc::RenewTargetVelocity(true);
		Calc::RenewVelocityDiff(true);
		Calc::RenewAccelTarget(true);
		Calc::RenewDegree(true);
		Calc::RenewDistance(true);

		Accel.SetValue(false, 0.0);
		A_Accel.SetValue(false, 0.0);

		GyroCtrlFlag.SetValue(true);
	}

	bool CheckGoalArrival(unsigned char dest_x, unsigned char dest_y) {
		bool arrive = true;

		if ((Mystat::Position::Get(Mystat::Position::X) == dest_x) && (Mystat::Position::Get(Mystat::Position::Y) == dest_y)) {
			arrive = true;
		} else if (ENABLE_FULLGOAL && ((dest_x != START_X) || (dest_y != START_Y))) {
			arrive = false;

			if ((Mystat::Position::Get(Mystat::Position::X) == (dest_x + 1)) && (Mystat::Position::Get(Mystat::Position::Y) == dest_y)) {
				arrive = true;
			}

			if ((Mystat::Position::Get(Mystat::Position::X) == (dest_x + 1)) && (Mystat::Position::Get(Mystat::Position::Y) == (dest_y + 1))) {
				arrive = true;
			}

			if ((Mystat::Position::Get(Mystat::Position::X) == dest_x) && (Mystat::Position::Get(Mystat::Position::Y) == (dest_y + 1))) {
				arrive = true;
			}
		} else {
			arrive = false;
		}

		return arrive;
	}

	void CheckMachineVelocity() {
		static uint16_t cnt_v = 0, cnt_a_v = 0, cnt_deg = 0;

		if (ExecuteFlag.GetValue()) {
			if ((fabsf(Velocity.GetValue(false) - Velocity.GetValue(true)) > 100.0) && (Velocity.GetValue(false) > 0.0)) {
				cnt_v++;
			} else {
				cnt_v = 0;
			}

			if (fabsf(A_Velocity.GetValue(false) - A_Velocity.GetValue(true)) > 150.0) {
				cnt_a_v++;
			} else {
				cnt_a_v = 0;
			}

			if (cnt_v > 100) {
				ExecuteFlag.SetValue(false);
	 			PWM::Motor::Disable();
	 			System::Interface::SetLEDColor(0, 255, 0, 0);
			}

			if (cnt_a_v > 60) {
				ExecuteFlag.SetValue(false);
	 			PWM::Motor::Disable();
	 			System::Interface::SetLEDColor(0, 255, 255, 0);
			}
		} else {
			cnt_v = 0;
			cnt_a_v = 0;
			cnt_deg = 0;
		}
	}

	void DetectWallEdge() {
		float distance = Distance.GetValue();
		static bool flag_l_01 = false, flag_l_02 = false, flag_l_wall = false,
				flag_r_01 = false, flag_r_02 = false, flag_r_wall = false;

		if ((distance >= 25.0) && (distance <= 65.0) && WallEdgeFlag.GetValue()) {
			if (!flag_l_01 && !flag_l_02 && !flag_r_02) {
				if (Status::Sensor::GetValue(Status::Sensor::LS, false) > WALL_EDGE_THRESHOLD_F_LS) {
					flag_l_01 = true;
					flag_l_wall = true;
				} else if (Status::Sensor::GetValue(Status::Sensor::LC, false) > WALL_EDGE_THRESHOLD_F_LC) {
					flag_l_01 = true;
				}
			} else if (flag_l_01 && !flag_l_02 && !flag_r_02) {
				if ((Status::Sensor::GetValue(Status::Sensor::LS, false) > WALL_EDGE_THRESHOLD_F_LS)
						&& (Status::Sensor::GetValue(Status::Sensor::LC, false) < WALL_EDGE_THRESHOLD2_F_LC)
						&& flag_l_wall) {
					flag_l_02 = true;
					Distance.SetValue(POSITION_EDGE_DETECT_F_L);
					System::Interface::SetLEDColor(0, 255, 0, 0);
				} else if ((Status::Sensor::GetValue(Status::Sensor::LS, false) < WALL_EDGE_THRESHOLD_F_LS)
						&& (Status::Sensor::GetValue(Status::Sensor::LC, false) < POLE_EDGE_THRESHOLD_F_LC)
						&& !flag_l_wall) {
					flag_l_02 = true;
					Distance.SetValue(POSITION_POLE_DETECT_F_L);
					System::Interface::SetLEDColor(0, 255, 0, 0);
					System::Interface::SetLEDColor(1, 255, 255, 255);
				}
			}

			if (!flag_r_01 && !flag_r_02 && !flag_l_02) {
				if (Status::Sensor::GetValue(Status::Sensor::RS, false) > WALL_EDGE_THRESHOLD_F_RS) {
					flag_r_01 = true;
					flag_r_wall = true;
				} else if (Status::Sensor::GetValue(Status::Sensor::RC, false) > WALL_EDGE_THRESHOLD_F_RC) {
					flag_r_01 = true;
				}
			} else if (flag_r_01 && !flag_r_02 && !flag_l_02) {
				if ((Status::Sensor::GetValue(Status::Sensor::RS, false) > WALL_EDGE_THRESHOLD_F_RS)
						&& (Status::Sensor::GetValue(Status::Sensor::RC, false) < WALL_EDGE_THRESHOLD2_F_RC)
						&& flag_r_wall) {
					flag_r_02 = true;
					Distance.SetValue(POSITION_EDGE_DETECT_F_R);
					System::Interface::SetLEDColor(0, 0, 255, 0);
				} else if ((Status::Sensor::GetValue(Status::Sensor::RS, false) < WALL_EDGE_THRESHOLD_F_RS)
						&& (Status::Sensor::GetValue(Status::Sensor::RC, false) < POLE_EDGE_THRESHOLD_F_RC)
						&& !flag_r_wall) {
					flag_r_02 = true;
					Distance.SetValue(POSITION_POLE_DETECT_F_R);
					System::Interface::SetLEDColor(0, 0, 255, 0);
					System::Interface::SetLEDColor(1, 255, 255, 255);
				}
			}
		} else {
			flag_l_01 = false;
			flag_l_02 = false;
			flag_l_wall = false;
			flag_r_01 = false;
			flag_r_02 = false;
			flag_r_wall = false;

			if (WallEdgeFlag.GetValue()) {
				System::Interface::SetLEDColor(0, 0, 0, 0);
				System::Interface::SetLEDColor(1, 0, 0, 0);
			}
		}
	}

	bool DetectWallEdge_Slant(signed char side, bool reset) {
		static bool left_1 = false, left_2 = false, right_1 = false, right_2 = false;
		bool detect = false;

		if (reset) {
			left_1 = false;
			left_2 = false;
			right_1 = false;
			right_2 = false;
		} else {
			if (side == SLALOM_LEFT) {
				if ((!left_1) && (Status::Sensor::GetValue(Status::Sensor::LC, false) > POLE_EDGE_THRESHOLD_SL_START)) {
					left_1 = true;
				}

				if (left_1 && (Status::Sensor::GetValue(Status::Sensor::LC, false) < POLE_EDGE_THRESHOLD_SL_END)) {
					left_2 = true;
					detect = true;
					Distance.SetValue(POSITION_EDGE_DETECT_SL_L);
				}
			} else if (side == SLALOM_RIGHT) {
				if ((!right_1) && (Status::Sensor::GetValue(Status::Sensor::RC, false) > POLE_EDGE_THRESHOLD_SL_START)) {
					right_1 = true;
				}

				if (right_1 && (Status::Sensor::GetValue(Status::Sensor::RC, false) < POLE_EDGE_THRESHOLD_SL_END)) {
					right_2 = true;
					detect = true;
					Distance.SetValue(POSITION_EDGE_DETECT_SL_R);
				}
			}

//			if (!left_2 && !right_2) {
//				if ((!left_1) && (ADC::Sensor::GetValue(ADC::Sensor::LS) > WALL_EDGE_THRESHOLD_F_LS_SL)) {
//					left_1 = true;
//				}
//
//				if ((!right_1) && (ADC::Sensor::GetValue(ADC::Sensor::RS) > WALL_EDGE_THRESHOLD_F_RS_SL)) {
//					right_1 = true;
//				}
//
//				if (left_1 && (ADC::Sensor::GetValue(ADC::Sensor::LS) < WALL_EDGE_THRESHOLD2_F_LS_SL)) {
//					left_2 = true;
//					detect = true;
//					Mystat::Status::SetSectionDistance(false, POSITION_EDGE_DETECT_F_LS_SL);
//					Interface::LED::SetColor(Interface::LED::Yellow, Interface::LED::Left);
//				}
//
//				if (right_1 && (ADC::Sensor::GetValue(ADC::Sensor::RS) < WALL_EDGE_THRESHOLD2_F_RS_SL)) {
//					right_2 = true;
//					detect = true;
//					Mystat::Status::SetSectionDistance(false, POSITION_EDGE_DETECT_F_RS_SL);
//					Interface::LED::SetColor(Interface::LED::Yellow, Interface::LED::Right);
//				}
//			}
		}

		return detect;
	}

// Calc
	float Calc::GetVelocity() {
		int16_t encoder_l = System::Interface::GetEncoderValue(System::Interface::Left), encoder_r = System::Interface::GetEncoderValue(System::Interface::Right);
		float velocity = 0.0;

		velocity = (float)CTRL_INTERVAL * (M_PI * (((float)(encoder_l + encoder_r) / 2.0) / (float)ENCODER_MAX) / GEAR_RATIO) * WHEEL_D;

		return velocity;
	}

	float Calc::GyroOutToA_Velocity() {
		int16_t data = 0;

		data = System::RSPI::ReadData(System::RSPI::Gyro, RSPI_ADDRESS_GYRO_ZOUT_L);
		data |= (System::RSPI::ReadData(System::RSPI::Gyro, RSPI_ADDRESS_GYRO_ZOUT_H) << 8);

		if ((data & 0x8000) == 0x8000) {
			data -= 65536;
		}

		return (((float)data * GYRO_RESOLUTION) - GyroRef.GetValue());
	}

	float Calc::GyroOutToA_Velocity_Y() {
		int16_t data = 0;

		data = System::RSPI::ReadData(System::RSPI::Gyro, RSPI_ADDRESS_GYRO_YOUT_L);
		data |= (System::RSPI::ReadData(System::RSPI::Gyro, RSPI_ADDRESS_GYRO_YOUT_H) << 8);

		if ((data & 0x8000) == 0x8000) {
			data -= 65536;
		}

		return ((float)data * GYRO_RESOLUTION);
	}

	float Calc::ZAccelOutToMetric() {
		int16_t data = 0;

		data = System::RSPI::ReadData(System::RSPI::Gyro, RSPI_ADDRESS_GYRO_ACC_ZOUT_L);
		data |= (System::RSPI::ReadData(System::RSPI::Gyro, RSPI_ADDRESS_GYRO_ACC_ZOUT_H) << 8);

		if ((data & 0x8000) == 0x8000) {
			data -= 65536;
		}

		return ((float)data * ACCEL_RESOLUTION * GRAVITY_METRIC);
	}

	void Calc::SetGyroReference() {
		float data = 0.0;

		GyroRef.SetValue(0.0F);

		for (uint16_t cnt = 0; cnt < GYRO_REFERENCE_SAMPLE; cnt++) {
			data += Calc::GyroOutToA_Velocity();
			System::Timer::wait_ms(1);
		}

		GyroRef.SetValue(data / (float)GYRO_REFERENCE_SAMPLE);
	}

	void Calc::RenewActualVelocity(bool reset) {
		float velocity = 0.0, a_velocity = 0.0;

		if (!reset) {
			velocity = Calc::GetVelocity();
			a_velocity = Calc::GyroOutToA_Velocity();
		}

		Velocity.SetValue(true, velocity);
		A_Velocity.SetValue(true, a_velocity);
	}

	void Calc::RenewTargetVelocity(bool reset) {
		float velocity = 0.0, a_velocity = 0.0, accel = 0.0, a_accel = 0.0;

		if (!reset) {
			velocity = Velocity.GetValue(false);
			a_velocity = A_Velocity.GetValue(false);
			accel = Accel.GetValue(false);
			a_accel = A_Accel.GetValue(false);

			velocity += accel * ((float)CTRL_INTERVAL / 1.0e6);
			a_velocity += a_accel * ((float)CTRL_INTERVAL / 1.0e6);
		}

		Velocity.SetValue(false, velocity);
		A_Velocity.SetValue(false, a_velocity);
	}

	void Calc::RenewVelocityDiff(bool reset) {
		float v_diff = 0.0, a_v_diff = 0.0;

		if (!reset) {
			v_diff = VelocityDiff.GetValue();
			a_v_diff = A_VelocityDiff.GetValue();

			v_diff += Velocity.GetValue(false) - Velocity.GetValue(true);
			a_v_diff += A_Velocity.GetValue(false) - A_Velocity.GetValue(true);
		}

		VelocityDiff.SetValue(v_diff);
		A_VelocityDiff.SetValue(a_v_diff);
	}

	void Calc::RenewAccelTarget(bool reset) {
		float v_target = 0.0, a_v_target = 0.0;

		if (reset) {
			TargetVelocity.SetValue(0.0F);
//		  TargetA_Velocity.SetValue(0.0F);
		} else {
			if ((Accel.GetValue(false) > 0.0) && (Velocity.GetValue(false) >= TargetVelocity.GetValue())) {
				Accel.SetValue(false, 0.0);
				Velocity.SetValue(false, TargetVelocity.GetValue());
			}

			if ((Accel.GetValue(false) < 0.0) && (Velocity.GetValue(false) <= TargetVelocity.GetValue())) {
				Accel.SetValue(false, 0.0);
				Velocity.SetValue(false, TargetVelocity.GetValue());
			}

//		  if ((A_Accel.GetValue(false) > 0.0) && (A_Velocity.GetValue(false) > TargetA_Velocity.GetValue())) {
//			  A_Accel.SetValue(false, 0.0);
//			  A_Velocity.SetValue(false, TargetA_Velocity.GetValue());
//		  }
//
//		  if ((A_Accel.GetValue(false) < 0.0) && (A_Velocity.GetValue(false) < TargetA_Velocity.GetValue())) {
//			  A_Accel.SetValue(false, 0.0);
//			  A_Velocity.SetValue(false, TargetA_Velocity.GetValue());
//		  }
		}
	}

	void Calc::RenewDegree(bool reset) {
		float degree = 0.0;

		if (!reset) {
			degree = Degree.GetValue();
			degree += A_Velocity.GetValue(false) * ((float)CTRL_INTERVAL / 1.0e6);
		}

		Degree.SetValue(degree);
	}

	void Calc::RenewDistance(bool reset) {
		float dist = 0.0;

		if (!reset) {
			dist = Distance.GetValue();
			dist += Velocity.GetValue(false) * ((float)CTRL_INTERVAL / 1.0e6);
		}

		Distance.SetValue(dist);
	}

//  std::array<uint16_t, 2> Calc::WallControlQuantity() {
	int16_t Calc::WallControlQuantity() {
		int16_t error_sensor = 0;

		if ((Status::Sensor::GetDiff(Status::Sensor::LS) > SENSOR_DIFF_THRESHOLD) && (Status::Sensor::GetDiff(Status::Sensor::RS) > SENSOR_DIFF_THRESHOLD)) {
			error_sensor = 0;
		} else if ((Status::Sensor::GetDiff(Status::Sensor::LS) > SENSOR_DIFF_THRESHOLD) && (Status::Sensor::GetValue(Status::Sensor::RS, false) > SENSOR_CTRL_THRESHOLD_R)) {
			error_sensor = -2 * (Status::Sensor::GetValue(Status::Sensor::RS, false) - SENSOR_TARGET_R);
		} else if ((Status::Sensor::GetDiff(Status::Sensor::RS) > SENSOR_DIFF_THRESHOLD) && (Status::Sensor::GetValue(Status::Sensor::LS, false) > SENSOR_CTRL_THRESHOLD_L)) {
			error_sensor = 2 * (Status::Sensor::GetValue(Status::Sensor::LS, false) - SENSOR_TARGET_L);
		} else {
			if ((Status::Sensor::GetValue(Status::Sensor::LS, false) > SENSOR_CTRL_THRESHOLD_L) && (Status::Sensor::GetValue(Status::Sensor::RS, false) > SENSOR_CTRL_THRESHOLD_R)) {
				error_sensor = (Status::Sensor::GetValue(Status::Sensor::LS, false) - SENSOR_TARGET_L) - (Status::Sensor::GetValue(Status::Sensor::RS, false) - SENSOR_TARGET_R);
			} else if ((Status::Sensor::GetValue(Status::Sensor::LS, false) < SENSOR_CTRL_THRESHOLD_L) && (Status::Sensor::GetValue(Status::Sensor::RS, false) < SENSOR_CTRL_THRESHOLD_R)) {
				error_sensor = 0;
			} else if (Status::Sensor::GetValue(Status::Sensor::LS, false) > SENSOR_CTRL_THRESHOLD_L) {
				error_sensor = 2 * (Status::Sensor::GetValue(Status::Sensor::LS, false) - SENSOR_TARGET_L);
			} else if (Status::Sensor::GetValue(Status::Sensor::RS, false) > SENSOR_CTRL_THRESHOLD_R) {
				error_sensor = -2 * (Status::Sensor::GetValue(Status::Sensor::RS, false) - SENSOR_TARGET_R);
			} else {
				error_sensor = 0;
			}
		}

//	  return (float)(-1.0 * GAIN_P_WALL * error_sensor);
		return error_sensor;
	}

// Sensor
	uint16_t Sensor::last_value[SENSOR_AMOUNT], Sensor::past_value[SENSOR_AMOUNT];
	int16_t Sensor::diff[SENSOR_AMOUNT];

	void Sensor::SetValue(uint16_t on[SENSOR_AMOUNT], uint16_t off[SENSOR_AMOUNT]) {
		for (uint8_t cnt = 0; cnt < SENSOR_AMOUNT; cnt++) {
			past_value[cnt] = last_value[cnt];

			if (on[cnt] - off[cnt] > 0) {
				last_value[cnt] = on[cnt] - off[cnt];
			} else {
				last_value[cnt] = 0;
			}

			diff[cnt] = last_value[cnt] - past_value[cnt];
		}
	}

	uint16_t Sensor::GetValue(uint8_t id, bool past) {
		if (id >= SENSOR_AMOUNT) {
			return 0;
		} else if (past) {
			return past_value[id];
		} else {
			return last_value[id];
		}
	}

	int16_t Sensor::GetDiff(uint8_t id) {
		if (id >= SENSOR_AMOUNT) {
			return 0;
		} else {
			return abs(diff[id]);
		}
	}

	bool Sensor::CheckWallExist(int8_t dir) {
		bool return_val = false;

		switch (dir) {
			case SIDE_LEFT:
				if (Sensor::GetValue(Sensor::LC, false) > SENSOR_WALL_EXIST_L) {
					return_val = true;
				}
				break;

			case SIDE_FORWARD:
				if (Sensor::GetValue(Sensor::F, false) > SENSOR_WALL_EXIST_F) {
					return_val = true;
				}
				break;

			case SIDE_RIGHT:
				if (Sensor::GetValue(Sensor::RC, false) > SENSOR_WALL_EXIST_R) {
					return_val = true;
				}
				break;
		}

		return return_val;
	}

/*
センサー値送信用

	while(1) {
		sprintf(senddata, "%4d, %4d, %4d, %4d, %4d, %4d, %4d\n", Status::Sensor::GetValue(Status::Sensor::LS, false), Status::Sensor::GetValue(Status::Sensor::LC, false), Status::Sensor::GetValue(Status::Sensor::LF, false), Status::Sensor::GetValue(Status::Sensor::F, false), Status::Sensor::GetValue(Status::Sensor::RF, false), Status::Sensor::GetValue(Status::Sensor::RC, false), Status::Sensor::GetValue(Status::Sensor::RS, false));
		System::SCI::SendChar(senddata);
		System::Timer::wait_ms(1000);
	}

 */

// Value
	Value::Value() {
		actual_val = 0.0;
		target_val = 0.0;
	}

	Value::~Value() {
		// none
	}

	void Value::SetValue(bool actual, float value) {
		if (actual) {
			actual_val = value;
		} else {
			target_val = value;
		}
	}

	float Value::GetValue(bool actual) {
		if (actual) {
			return actual_val;
		} else {
			return target_val;
		}
	}

	Value2::Value2() {
		val = 0.0;
	}

	Value2::~Value2() {
		// none
	}

	void Value2::SetValue(float value) {
		val = value;
	}

	float Value2::GetValue() {
		return val;
	}

// Flag
	Flag::Flag() {
		flag = false;
	}

	Flag::~Flag() {
		// none
	}

	void Flag::SetValue(bool execute) {
		flag = execute;
	}

	bool Flag::GetValue() {
		return flag;
	}
}

namespace Mystat {
	unsigned char Position::x = 0, Position::y = 0;
	signed char Position::dir = DIR_NORTH;

	signed char Position::Get(unsigned char type){
		signed char return_val = 0;

		switch (type) {
			case Position::X:
				return_val = Position::x;
				break;

			case Position::Y:
				return_val = Position::y;
				break;

			case Position::Dir:
				return_val = Position::dir;
				break;
		}

		return return_val;
	}

	void Position::Reset() {
		x = START_X;
		y = START_Y;
		dir = 0;
	}

	void Position::UpDate (signed char traveling_dir) {
		bool wall_l = false, wall_f = false, wall_r = false;

		dir += traveling_dir;

		if (dir > 4) {		   // (> 7) → (> 4)
			dir -= 8;
		} else if (dir <= -4) {  // (< 0) → (<= -4)
			dir += 8;
		}

		switch (dir) {
			case DIR_NORTH:
				y++;
				break;

			case DIR_EAST:
				x++;
				break;

			case DIR_SOUTH:
				y--;
				break;

			case DIR_WEST:
				x--;
				break;
		}

//	  if (ADC::Sensor::GetValue(ADC::Sensor::LS) > WALL_THRESHOLD_F_LS) {
		if (Status::Sensor::GetValue(Status::Sensor::LS, false) > SENSOR_WALL_EXIST_L) {
			wall_l = true;
		}

		if (Status::Sensor::GetValue(Status::Sensor::F, false) > SENSOR_WALL_EXIST_F) {
			wall_f = true;
		}

		if (Status::Sensor::GetValue(Status::Sensor::RS, false) > SENSOR_WALL_EXIST_R) {
			wall_r = true;
		}

		Map::SetWallData(wall_l, wall_f, wall_r);
	}

	void Position::UpDate (bool move_section, signed char traveling_dir) {
		if (move_section) {
			Position::UpDate(traveling_dir);
		} else {
			dir += traveling_dir;

			if (dir > 4) {		   // (> 7) → (> 4)
				dir -= 8;
			} else if (dir <= -4) {  // (< 0) → (<= -4)
				dir += 8;
			}
		}
	}
	/* ---------------------------------------------------
		Map
	 --------------------------------------------------- */
	unsigned char Map::data[MAPSIZE_X][MAPSIZE_Y], Map::step[MAPSIZE_X][MAPSIZE_Y];
	bool Map::checked[MAPSIZE_X][MAPSIZE_Y];

	struct _dijkstra Map::dijkstra;
	struct _path Map::path;

	float Map::velocity = 0.0, Map::turn_velocity = 0.0, Map::accel = 0.0;

	/*
		MapData
		東西南北順で記述

		  Flag	  Wall
		E W S N   E W S N
		x x x x | x x x x

		Direction
		E W S N
		0 1 2 3

		Wall
		1: Left
		2: Front
		3: Right
	*/

	void Map::Init() {
		for (unsigned char cnt_x = 0; cnt_x < MAPSIZE_X; cnt_x++) {
			for (unsigned char cnt_y = 0; cnt_y < MAPSIZE_Y; cnt_y++) {
				data[cnt_x][cnt_y] = 0;
				step[cnt_x][cnt_y] = 255;
				checked[cnt_x][cnt_y] = false;

				// 迷路外縁部には壁を入れる
				if (cnt_x == 0) {
					data[cnt_x][cnt_y] |= WALL_WEST;
				}

				if (cnt_x == (MAPSIZE_X - 1)) {
					data[cnt_x][cnt_y] |= WALL_EAST;
				}

				if (cnt_y == 0) {
					data[cnt_x][cnt_y] |= WALL_SOUTH;
				}

				if (cnt_y == (MAPSIZE_Y - 1)) {
					data[cnt_x][cnt_y] |= WALL_NORTH;
				}
			}
		}

		step[GOAL_X][GOAL_Y]		= 0;
		checked[START_X][START_Y]   = true;
		data[START_X][START_Y]	  |= (FLAG_EAST | FLAG_NORTH | WALL_EAST);	// 0x98
		data[START_X][START_Y + 1]  |= FLAG_SOUTH;							  // 0x20
		data[START_X + 1][START_Y]  |= (FLAG_WEST | WALL_WEST);				 // 0x44

		if (ENABLE_FULLGOAL) {
			data[GOAL_X][GOAL_Y]		|= (FLAG_NORTH | FLAG_EAST);
			data[GOAL_X + 1][GOAL_Y]	|= (FLAG_NORTH | FLAG_WEST);
			data[GOAL_X][GOAL_Y + 1]	|= (FLAG_SOUTH | FLAG_EAST);
			data[GOAL_X + 1][GOAL_Y + 1]|= (FLAG_SOUTH | FLAG_WEST);
		}

		Position::Reset();
	}

	float Map::CalcAccelStep(bool slant, unsigned char partition) {
		float section_length, velocity;

		if (slant) {
//			section_length = SECTION_SLANT;
			section_length = SECTION_STRAIGHT;
		} else {
			section_length = SECTION_STRAIGHT;
		}

		if (partition > 1) {
			velocity = sqrt(Map::accel * (section_length / 2.0) * (float)partition + Map::turn_velocity * Map::turn_velocity);

			if (velocity > Map::velocity) {
				velocity = Map::velocity;
			}

			return (float)(partition * section_length * 2.0 * (Map::turn_velocity / (velocity + Map::turn_velocity)));
		} else {
			if (slant) {
				return SECTION_STRAIGHT;
			} else {
				return SECTION_STRAIGHT;
			}
		}
	}

	void Map::CalcStep (unsigned char dest_x, unsigned char dest_y, bool enable_neglect) {
		unsigned char searched_x[257], searched_y[257], wallcnt;
		bool search_flag = 1, none_refresh = 1;
		unsigned char cnt_next_contour = 0, cnt_before_contour = 0;
		unsigned int cnt_contour = 0, searched_element = 0;
		unsigned char now_step = 0, cnt_judgechecked = 0;

		for (unsigned char i = 0; i < MAPSIZE_X; i++) {
			for (unsigned char j = 0; j < MAPSIZE_Y; j++) {
				step[i][j] = 255;
				wallcnt = 0;

				if ((i < (MAPSIZE_X - 1)) && (j < (MAPSIZE_Y - 1))) {
					if ((data[i][j] & (FLAG_EAST | WALL_EAST)) == FLAG_EAST) {
						wallcnt++;
					}

					if ((data[i][j + 1] & (FLAG_EAST | WALL_EAST)) == FLAG_EAST) {
						wallcnt++;
					}

					if ((data[i][j] & (FLAG_NORTH | WALL_NORTH)) == FLAG_NORTH) {
						wallcnt++;
					}

					if ((data[i + 1][j] & (FLAG_NORTH | WALL_NORTH)) == FLAG_NORTH) {
						wallcnt++;
					}

					if ((wallcnt == 3) && ((i != GOAL_X) || (j != GOAL_Y))) {
					//if (wallcnt == 3) {
						if (!(data[i][j] & FLAG_EAST)) {
							data[i][j] |= WALL_EAST;
							data[i + 1][j] |= WALL_WEST;
						}

						if (!(data[i][j + 1] & FLAG_EAST)) {
							data[i][j + 1] |= WALL_EAST;
							data[i + 1][j + 1] |= WALL_WEST;
						}

						if (!(data[i][j] & FLAG_NORTH)) {
							data[i][j] |= WALL_NORTH;
							data[i][j + 1] |= WALL_SOUTH;
						}

						if (!(data[i + 1][j] & FLAG_NORTH)) {
							data[i + 1][j] |= WALL_NORTH;
							data[i + 1][j + 1] |= WALL_SOUTH;
						}
					}
				}
			}
		}

		step[dest_x][dest_y] = 0;

		searched_x[0] = dest_x;
		searched_y[0] = dest_y;
		now_step++;

		while (search_flag) {
			for (cnt_contour = (searched_element - cnt_before_contour); cnt_contour <= searched_element; cnt_contour++) {
				if (!(data[searched_x[cnt_contour]][searched_y[cnt_contour]] & WALL_EAST)) {
					if (enable_neglect && !(data[searched_x[cnt_contour]][searched_y[cnt_contour]] & FLAG_EAST)) {
						//break;
					} else {
						if (step[searched_x[cnt_contour] + 1][searched_y[cnt_contour]] > now_step) {
							cnt_next_contour++;
							none_refresh = 0;
							step[searched_x[cnt_contour] + 1][searched_y[cnt_contour]] = now_step;
							searched_x[searched_element + cnt_next_contour] = searched_x[cnt_contour] + 1;
							searched_y[searched_element + cnt_next_contour] = searched_y[cnt_contour];
							//sprintf(sendchar, "%3d: (%2d, %2d)\n", searched_element + cnt_next_contour, searched_x[cnt_contour] + 1, searched_y[cnt_contour]);
							//comm_SCISend_Char(sendchar);
						}
					}
				}

				if (!(data[searched_x[cnt_contour]][searched_y[cnt_contour]] & WALL_WEST)) {
					if (enable_neglect && !(data[searched_x[cnt_contour]][searched_y[cnt_contour]] & FLAG_WEST)) {
						//break;
					} else {
						if (step[searched_x[cnt_contour] - 1][searched_y[cnt_contour]] > now_step) {
							cnt_next_contour++;
							none_refresh = 0;
							step[searched_x[cnt_contour] - 1][searched_y[cnt_contour]] = now_step;
							searched_x[searched_element + cnt_next_contour] = searched_x[cnt_contour] - 1;
							searched_y[searched_element + cnt_next_contour] = searched_y[cnt_contour];
							//sprintf(sendchar, "%3d: (%2d, %2d)\n", searched_element + cnt_next_contour, searched_x[cnt_contour] - 1, searched_y[cnt_contour]);
							//comm_SCISend_Char(sendchar);
						}
					}
				}

				if (!(data[searched_x[cnt_contour]][searched_y[cnt_contour]] & WALL_SOUTH)) {
					if (enable_neglect && !(data[searched_x[cnt_contour]][searched_y[cnt_contour]] & FLAG_SOUTH)) {
						//break;
					} else {
						if (step[searched_x[cnt_contour]][searched_y[cnt_contour] - 1] > now_step) {
							cnt_next_contour++;
							none_refresh = 0;
							step[searched_x[cnt_contour]][searched_y[cnt_contour] - 1] = now_step;
							searched_x[searched_element + cnt_next_contour] = searched_x[cnt_contour];
							searched_y[searched_element + cnt_next_contour] = searched_y[cnt_contour] - 1;
							//sprintf(sendchar, "%3d: (%2d, %2d)\n", searched_element + cnt_next_contour, searched_x[cnt_contour], searched_y[cnt_contour] - 1);
							//comm_SCISend_Char(sendchar);
						}
					}
				}

				if (!(data[searched_x[cnt_contour]][searched_y[cnt_contour]] & WALL_NORTH)) {
					if (enable_neglect && !(data[searched_x[cnt_contour]][searched_y[cnt_contour]] & FLAG_NORTH)) {
						// break;
					} else {
						if (step[searched_x[cnt_contour]][searched_y[cnt_contour] + 1] > now_step) {
							cnt_next_contour++;
							none_refresh = 0;
							step[searched_x[cnt_contour]][searched_y[cnt_contour] + 1] = now_step;
							searched_x[searched_element + cnt_next_contour] = searched_x[cnt_contour];
							searched_y[searched_element + cnt_next_contour] = searched_y[cnt_contour] + 1;
							//sprintf(sendchar, "%3d: (%2d, %2d)\n", searched_element + cnt_next_contour, searched_x[cnt_contour], searched_y[cnt_contour] + 1);
							//comm_SCISend_Char(sendchar);
						}
					}
				}

				// TODO: 壁2枚以上時既知判定
				// 壁枚数3枚以上なら既知区間(袋小路)
				if (Map::GetBlockWallNum(searched_x[cnt_contour], searched_y[cnt_contour], SIDE_NONE) >= 3) {
//				  checked[searched_x[cnt_contour]][searched_y[cnt_contour]] = true;
				} else if (Map::GetBlockWallNum(searched_x[cnt_contour], searched_y[cnt_contour], SIDE_NONE) >= 2) {
					// 北壁と東壁無し
					if ((data[searched_x[cnt_contour]][searched_y[cnt_contour]] & (FLAG_NORTH | FLAG_EAST | WALL_NORTH | WALL_EAST)) == (FLAG_NORTH | FLAG_EAST)) {
						if ((searched_x[cnt_contour] < MAPSIZE_X) && (searched_y[cnt_contour] < MAPSIZE_Y)) {
							/*  Pattern1: 北東柱-北側だけ壁
									 O
									X+X
									MX
							 */
							if ((data[searched_x[cnt_contour] + 1][searched_y[cnt_contour]] & (FLAG_NORTH | WALL_NORTH)) == FLAG_NORTH) {
								data[searched_x[cnt_contour]][searched_y[cnt_contour] + 1] |=	   (FLAG_EAST | WALL_EAST);
								data[searched_x[cnt_contour] + 1][searched_y[cnt_contour] + 1] |=   (FLAG_WEST | WALL_WEST);
							}

							/*  Pattern2: 北東柱-東側だけ壁
									 X
									X+O
									MX
							 */
							if ((data[searched_x[cnt_contour]][searched_y[cnt_contour] + 1] & (FLAG_EAST | WALL_EAST)) == FLAG_EAST) {
								data[searched_x[cnt_contour] + 1][searched_y[cnt_contour]] |=	   (FLAG_NORTH | WALL_NORTH);
								data[searched_x[cnt_contour] + 1][searched_y[cnt_contour] + 1] |=   (FLAG_SOUTH | WALL_SOUTH);
							}
						}
					}

					// 東壁と南壁無し
					if ((data[searched_x[cnt_contour]][searched_y[cnt_contour]] & (FLAG_EAST | FLAG_SOUTH | WALL_EAST | WALL_SOUTH)) == (FLAG_EAST | FLAG_SOUTH)) {
						if ((searched_x[cnt_contour] < MAPSIZE_X) && (searched_y[cnt_contour] > 0)) {
							/*  Pattern3: 南東柱-南側だけ壁
									MX
									X+X
									 O
							 */
							if ((data[searched_x[cnt_contour] + 1][searched_y[cnt_contour] - 1] & (FLAG_NORTH | WALL_NORTH)) == FLAG_NORTH) {
								data[searched_x[cnt_contour]][searched_y[cnt_contour] - 1] |=	   (FLAG_EAST | WALL_EAST);
								data[searched_x[cnt_contour] + 1][searched_y[cnt_contour] - 1] |=   (FLAG_WEST | WALL_WEST);
							}

							/*  Pattern4: 南東柱-東側だけ壁
									MX
									X+O
									 X
							 */
							if ((data[searched_x[cnt_contour]][searched_y[cnt_contour] - 1] & (FLAG_EAST | WALL_EAST)) == FLAG_EAST) {
								data[searched_x[cnt_contour] + 1][searched_y[cnt_contour] - 1] |=   (FLAG_NORTH | WALL_NORTH);
								data[searched_x[cnt_contour] + 1][searched_y[cnt_contour]] |=	   (FLAG_SOUTH | WALL_SOUTH);
							}
						}
					}

					// 南壁と西壁無し
					if ((data[searched_x[cnt_contour]][searched_y[cnt_contour]] & (FLAG_SOUTH | FLAG_WEST | WALL_SOUTH | WALL_WEST)) == (FLAG_SOUTH | FLAG_WEST)) {
						if ((searched_x[cnt_contour] > 0) && (searched_y[cnt_contour] > 0)) {
							/*  Pattern5: 南西柱-南側だけ壁
									 XM
									X+X
									 O
							 */
							if ((data[searched_x[cnt_contour] - 1][searched_y[cnt_contour] - 1] & (FLAG_NORTH | WALL_NORTH)) == FLAG_NORTH) {
								data[searched_x[cnt_contour] - 1][searched_y[cnt_contour] - 1] |=   (FLAG_EAST | WALL_EAST);
								data[searched_x[cnt_contour]][searched_y[cnt_contour] - 1] |=	   (FLAG_WEST | WALL_WEST);
							}

							/*  Pattern6: 南西柱-西側だけ壁
									 XM
									O+X
									 X
							 */
							if ((data[searched_x[cnt_contour] - 1][searched_y[cnt_contour] - 1] & (FLAG_EAST | WALL_EAST)) == FLAG_EAST) {
								data[searched_x[cnt_contour] - 1][searched_y[cnt_contour] - 1] |=   (FLAG_NORTH | WALL_NORTH);
								data[searched_x[cnt_contour] - 1][searched_y[cnt_contour]] |=	   (FLAG_SOUTH | WALL_SOUTH);
							}
						}
					}

					// 西壁と北壁無し
					if ((data[searched_x[cnt_contour]][searched_y[cnt_contour]] & (FLAG_WEST | FLAG_NORTH | WALL_WEST | WALL_NORTH)) == (FLAG_WEST | FLAG_NORTH)) {
						if ((searched_x[cnt_contour] > 0) && (searched_y[cnt_contour] < MAPSIZE_Y)) {
							/*  Pattern7: 北西柱-北側だけ壁
									 O
									X+X
									 XM
							 */
							if ((data[searched_x[cnt_contour] - 1][searched_y[cnt_contour]] & (FLAG_NORTH | WALL_NORTH)) == FLAG_NORTH) {
								data[searched_x[cnt_contour] - 1][searched_y[cnt_contour] + 1] |=   (FLAG_EAST | WALL_EAST);
								data[searched_x[cnt_contour]][searched_y[cnt_contour] + 1] |=	   (FLAG_WEST | WALL_WEST);
							}

							/*  Pattern8: 北西柱-東側だけ壁
									 X
									O+X
									 XM
							 */
							if ((data[searched_x[cnt_contour] - 1][searched_y[cnt_contour] + 1] & (FLAG_EAST | WALL_EAST)) == FLAG_EAST) {
								data[searched_x[cnt_contour] - 1][searched_y[cnt_contour]] |=	   (FLAG_NORTH | WALL_NORTH);
								data[searched_x[cnt_contour] - 1][searched_y[cnt_contour] + 1] |=   (FLAG_SOUTH | WALL_SOUTH);
							}
						}
					}

					// 全て壁チェック済みなら既知区間扱い
					if ((data[searched_x[cnt_contour]][searched_y[cnt_contour]] & 0xf0) == 0xf0) {
//					  checked[searched_x[cnt_contour]][searched_y[cnt_contour]] = true;
					}
				}
			}

			searched_element += cnt_next_contour;
			cnt_before_contour = cnt_next_contour;
			cnt_next_contour = 0;
			now_step++;

			//sprintf(sendchar, "------\n", searched_element);
			//comm_SCISend_Char(sendchar);

			if (none_refresh) {
				search_flag = 0;
			}

			none_refresh = 1;
		}
	}

	unsigned char Map::ReadWallData(unsigned char x, unsigned char y) {
		if ((x < MAPSIZE_X) && (y < MAPSIZE_Y)) {
			return Map::data[x][y];
		} else {
			return 255;
		}
	}

	void Map::WriteWallData(unsigned char x, unsigned char y, unsigned char walldata) {
		if ((x < MAPSIZE_X) && (y < MAPSIZE_Y)) {
			Map::data[x][y] = walldata;
		}
	}

	void Map::SetWallData(bool left, bool front, bool right) {
		unsigned char x = Position::Get(Position::X), y = Position::Get(Position::Y), data_temp = 0;
		bool wall_east = false, wall_west = false, wall_south = false, wall_north = false;

		checked[x][y] = true;

		switch (Position::Get(Position::Dir)) {
			case DIR_EAST:
				wall_east = front;
				wall_west = false;
				wall_south = right;
				wall_north = left;
				break;

			case DIR_WEST:
				wall_east = false;
				wall_west = front;
				wall_south = left;
				wall_north = right;
				break;

			case DIR_SOUTH:
				wall_east = left;
				wall_west = right;
				wall_south = front;
				wall_north = false;
				break;

			case DIR_NORTH:
				wall_east = right;
				wall_west = left;
				wall_south = false;
				wall_north = front;
				break;
		}

		if ((data[x][y] & FLAG_EAST) == 0) {
			if (wall_east) {
				data[x][y] |= (FLAG_EAST | WALL_EAST);

				if (x < (MAPSIZE_X - 1)) {
					data[x + 1][y] |= (FLAG_WEST | WALL_WEST);
				}
			} else {
				data[x][y] |= FLAG_EAST;

				if (x < (MAPSIZE_X - 1)) {
					data[x + 1][y] |= FLAG_WEST;
				}
			}
		}

		if ((data[x][y] & FLAG_WEST) == 0) {
			if (wall_west) {
				data[x][y] |= (FLAG_WEST | WALL_WEST);

				if (x > 0) {
					data[x - 1][y] |= (FLAG_EAST | WALL_EAST);
				}
			} else {
				data[x][y] |= FLAG_WEST;

				if (x > 0) {
					data[x - 1][y] |= FLAG_EAST;
				}
			}
		}

		if ((data[x][y] & FLAG_SOUTH) == 0) {
			if (wall_south) {
				data[x][y] |= (FLAG_SOUTH | WALL_SOUTH);

				if (y > 0) {
					data[x][y - 1] |= (FLAG_NORTH | WALL_NORTH);
				}
			} else {
				data[x][y] |= FLAG_SOUTH;

				if (y > 0) {
					data[x][y - 1] |= FLAG_NORTH;
				}
			}
		}

		if ((data[x][y] & FLAG_NORTH) == 0) {
			if (wall_north) {
				data[x][y] |= (FLAG_NORTH | WALL_NORTH);

				if (y < (MAPSIZE_Y - 1)) {
					data[x][y + 1] |= (FLAG_SOUTH | WALL_SOUTH);
				}
			} else {
				data[x][y] |= FLAG_NORTH;

				if (y < (MAPSIZE_Y - 1)) {
					data[x][y + 1] |= FLAG_SOUTH;
				}
			}
		}
	}

	void Map::SendData () {
		volatile unsigned char send_skip = 0;
		char senddata[127];

	/*
			0 1 2
		0   +---+
		1   | ff|
		2   +---+
	*/

		sprintf(senddata, "[Position] X:%2d, Y:%2d, Direction:%d\n\n", Position::Get(Position::X), Position::Get(Position::Y), Position::Get(Position::Dir));
		System::SCI::SendChar(senddata);

		for (unsigned char i = 0; i < MAPSIZE_X; i++) {
			sprintf(senddata, " %3d ", i);
			System::SCI::SendChar(senddata);
		}

		sprintf(senddata, "\n");
		System::SCI::SendChar(senddata);

		for (signed int j = MAPSIZE_Y * 3 - 1; j >= 0; j--) {
			for (signed int i = 0; i < MAPSIZE_X * 3; i++) {
				// 南側壁情報
				if ((j % 3 == 0) && (i % 3 == 0)) {
					if (data[i / 3][j / 3] & WALL_SOUTH) {
						sprintf(senddata, "+---+");
					} else if (!(data[i / 3][j / 3] & FLAG_SOUTH)) {
						sprintf(senddata, "+ ? +");
					} else {
						sprintf(senddata, "+   +");
					}
				} else if (j % 3 == 1) {
					// 西側壁情報
					if (i % 3 == 0) {
						if (data[i / 3][(j - 1) / 3] & WALL_WEST) {
							sprintf(senddata, "|");
						} else if (!(data[i / 3][(j - 1) / 3] & FLAG_WEST)) {
							sprintf(senddata, "?");
						} else {
							sprintf(senddata, " ");
						}
					// 歩数・既知/未知区間情報
					} else if (i % 3 == 1) {
						if (checked[(i - 1) / 3][(j - 1) / 3]) {
							sprintf(senddata, "*%2x", step[(i - 1) / 3][(j - 1) / 3]);
						} else {
							sprintf(senddata, "%3x", step[(i - 1) / 3][(j - 1) / 3]);
						}
					// 東側壁情報
					} else {
						if (data[i / 3][(j - 1) / 3] & WALL_EAST) {
							sprintf(senddata, "|");
						} else if (!(data[i / 3][(j - 1) / 3] & FLAG_EAST)) {
							sprintf(senddata, "?");
						} else {
							sprintf(senddata, " ");
						}
					}
				// 北側壁情報
				} else if ((j % 3 == 2) && (i % 3 == 0)) {
					if (data[i / 3][j / 3] & WALL_NORTH) {
						sprintf(senddata, "+---+");
					} else if (!(data[i / 3][j / 3] & FLAG_NORTH)) {
						sprintf(senddata, "+ ? +");
					} else {
						sprintf(senddata, "+   +");
					}
				} else {
					send_skip = 1;
				}

				if (!send_skip) {
					System::SCI::SendChar(senddata);
				}

				send_skip = 0;
			}

			if (j % 3 == 1) {
				sprintf(senddata, " %2d\n", (j - 1) / 3);
			} else {
				sprintf(senddata, "\n");
			}

			System::SCI::SendChar(senddata);
		}

		sprintf(senddata, "\n");
		System::SCI::SendChar(senddata);

		for (signed int j = MAPSIZE_Y - 1; j >= 0; j--) {
			for (signed int i = 0; i < MAPSIZE_X; i++) {
				sprintf(senddata, "%3x", data[i][j]);
				System::SCI::SendChar(senddata);
			}

			sprintf(senddata, "\n");
			System::SCI::SendChar(senddata);
		}
	}

	bool Map::GetBlockChecked(unsigned char x, unsigned char y, signed char side) {
		signed char diff_x, diff_y, check_dir = 0;

		check_dir = Position::Get(Position::Dir) + side;

		if (check_dir > 4) {
			check_dir -= 8;
		} else if (check_dir <= -4) {
			check_dir += 8;
		}

		switch (check_dir) {
			case DIR_EAST:
				diff_x = 1;
				diff_y = 0;
				break;

			case DIR_WEST:
				diff_x = -1;
				diff_y = 0;
				break;

			case DIR_SOUTH:
				diff_x = 0;
				diff_y = -1;
				break;

			case DIR_NORTH:
				diff_x = 0;
				diff_y = 1;
				break;

			case DIR_N_EAST:
				diff_x = 1;
				diff_y = 1;
				break;

			case DIR_S_EAST:
				diff_x = 1;
				diff_y = -1;
				break;

			case DIR_S_WEST:
				diff_x = -1;
				diff_y = -1;
				break;

			case DIR_N_WEST:
				diff_x = -1;
				diff_y = 1;
				break;
		}

		if (((check_dir % 2) == 0) && (side == SIDE_F_FORWARD)) {
			diff_x *= 2;
			diff_y *= 2;
		}

		if ((x + diff_x) < 0) {
			return true;
		}

		if ((y + diff_y) < 0) {
			return true;
		}

		if ((x + diff_x) > (MAPSIZE_X - 1)) {
			return true;
		}

		if ((y + diff_y) > (MAPSIZE_Y - 1)) {
			return true;
		}

		return checked[x + diff_x][y + diff_y];
	}

	unsigned char Map::GetBlockStep(unsigned char x, unsigned char y, signed char side) {
		signed char diff_x, diff_y, check_dir = 0;

		if (side != SIDE_F_FORWARD) {
			check_dir = Position::Get(Position::Dir) + side;
		} else {
			check_dir = Position::Get(Position::Dir);
		}

		if (check_dir > 4) {
			check_dir -= 8;
		} else if (check_dir <= -4) {
			check_dir += 8;
		}

		switch (check_dir) {
			case DIR_EAST:
				diff_x = 1;
				diff_y = 0;
				break;

			case DIR_WEST:
				diff_x = -1;
				diff_y = 0;
				break;

			case DIR_SOUTH:
				diff_x = 0;
				diff_y = -1;
				break;

			case DIR_NORTH:
				diff_x = 0;
				diff_y = 1;
				break;

			case DIR_N_EAST:
				diff_x = 1;
				diff_y = 1;
				break;

			case DIR_S_EAST:
				diff_x = 1;
				diff_y = -1;
				break;

			case DIR_S_WEST:
				diff_x = -1;
				diff_y = -1;
				break;

			case DIR_N_WEST:
				diff_x = -1;
				diff_y = 1;
				break;
		}

		if (((check_dir % 2) == 0) && (side == SIDE_F_FORWARD)) {
			diff_x *= 2;
			diff_y *= 2;
		}

		if ((x + diff_x) < 0) {
			return 255;
		}

		if ((y + diff_y) < 0) {
			return 255;
		}

		if ((x + diff_x) > (MAPSIZE_X - 1)) {
			return 255;
		}

		if ((y + diff_y) > (MAPSIZE_Y - 1)) {
			return 255;
		}

		return step[x + diff_x][y + diff_y];
}

	unsigned char Map::GetBlockStep(unsigned char x, unsigned char y) {
		if ((x >= 0) && (x < MAPSIZE_X) && (y >= 0) && (y < MAPSIZE_Y)) {
			return step[x][y];
		} else {
			return 255;
		}
	}

	unsigned char Map::GetBlockWallNum(unsigned char check_x, unsigned char check_y, signed char side) {
		signed char diff_x = 0, diff_y = 0, check_dir = 0;
		unsigned char return_val = 0;

		if (side != SIDE_NONE) {
			if (side != SIDE_F_FORWARD) {
				check_dir = Position::Get(Position::Dir) + side;
			} else {
				check_dir = Position::Get(Position::Dir);
			}

			if (check_dir > 4) {
				check_dir -= 8;
			} else if (check_dir <= -4) {
				check_dir += 8;
			}
		}

		switch (check_dir) {
			case DIR_EAST:
				diff_x = 1;
				diff_y = 0;
				break;

			case DIR_WEST:
				diff_x = -1;
				diff_y = 0;
				break;

			case DIR_SOUTH:
				diff_x = 0;
				diff_y = -1;
				break;

			case DIR_NORTH:
				diff_x = 0;
				diff_y = 1;
				break;

			case DIR_N_EAST:
				diff_x = 1;
				diff_y = 1;
				break;

			case DIR_S_EAST:
				diff_x = 1;
				diff_y = -1;
				break;

			case DIR_S_WEST:
				diff_x = -1;
				diff_y = -1;
				break;

			case DIR_N_WEST:
				diff_x = -1;
				diff_y = 1;
				break;

			case SIDE_NONE:
				diff_x = 0;
				diff_y = 0;
				break;
		}

		if (((check_dir % 2) == 0) && (side == SIDE_F_FORWARD)) {
			diff_x *= 2;
			diff_y *= 2;
		}

		if (check_x + diff_x < 0) {
			return_val = 4;
		} else if (check_x + diff_x >= MAPSIZE_X) {
			return_val = 4;
		} else if (check_y + diff_y < 0) {
			return_val = 4;
		} else if (check_y + diff_y >= MAPSIZE_Y) {
			return_val = 4;
		} else {
			if ((data[check_x + diff_x][check_y + diff_y] & WALL_EAST) == WALL_EAST) {
				return_val++;
			}

			if ((data[check_x + diff_x][check_y + diff_y] & WALL_WEST) == WALL_WEST) {
				return_val++;
			}

			if ((data[check_x + diff_x][check_y + diff_y] & WALL_SOUTH) == WALL_SOUTH) {
				return_val++;
			}

			if ((data[check_x + diff_x][check_y + diff_y] & WALL_NORTH) == WALL_NORTH) {
				return_val++;
			}
		}

		return return_val;
	}

	unsigned char Map::GetWallData(unsigned char x, unsigned char y) {
		if ((x >= 0) && (x < MAPSIZE_X) && (y >= 0) && (y < MAPSIZE_Y)) {
			return data[x][y];
		} else {
			return 255;
		}
	}

	bool Map::CheckAheadWallExist(unsigned char x, unsigned char y, signed char side) {
		signed char diff_x, diff_y, check_dir = 0;
		unsigned char mask = 0, check = 0;

		check_dir = Position::Get(Position::Dir) + side;

		if (check_dir > 4) {
			check_dir -= 8;
		} else if (check_dir <= -4) {
			check_dir += 8;
		}

		switch (Position::Get(Position::Dir)) {
			case DIR_EAST:
				diff_x = 1;
				diff_y = 0;
				break;

			case DIR_WEST:
				diff_x = -1;
				diff_y = 0;
				break;

			case DIR_SOUTH:
				diff_x = 0;
				diff_y = -1;
				break;

			case DIR_NORTH:
				diff_x = 0;
				diff_y = 1;
				break;

			default:
				return true;
				break;
		}

		switch (check_dir) {
			case DIR_EAST:
				mask = FLAG_EAST | WALL_EAST;
				check = FLAG_EAST;
				break;

			case DIR_WEST:
				mask = FLAG_WEST | WALL_WEST;
				check = FLAG_WEST;
				break;

			case DIR_SOUTH:
				mask = FLAG_SOUTH | WALL_SOUTH;
				check = FLAG_SOUTH;
				break;

			case DIR_NORTH:
				mask = FLAG_NORTH | WALL_NORTH;
				check = FLAG_NORTH;
				break;

			default:
				return true;
				break;
		}

		if ((x + diff_x) < 0) {
			return true;
		}

		if ((y + diff_y) < 0) {
			return true;
		}

		if ((x + diff_x) > (MAPSIZE_X - 1)) {
			return true;
		}

		if ((y + diff_y) > (MAPSIZE_Y - 1)) {
			return true;
		}

		if ((Map::GetWallData(x + diff_x, y + diff_y) & mask) == check) {
			return false;
		} else {
			return true;
		}
	}

	void Map::CheckDijikstra_Vertical (unsigned char check_x, unsigned char check_y) {
		float cost;
		unsigned char cnt_x = 0;
		bool refresh = false;

		// 北
		for (unsigned char i = 2; i < (2 * MAPSIZE_Y - check_y); i += 2) {
			cost = CalcAccelStep(false, (i / 2));
			refresh = true;

			if ((dijkstra.flag[check_x][check_y + i] == 0) && (dijkstra.cost[check_x][check_y + i] > (dijkstra.cost[check_x][check_y] + (int)cost))) {
				if ((check_x > 0) && (dijkstra.cost[check_x][check_y + i] != INF)) {
					if (dijkstra.cost[check_x][check_y + i] == dijkstra.cost[check_x - 1][check_y + i]) {
						refresh = false;
					}
				}

				if ((check_x < (MAPSIZE_X - 1)) && (dijkstra.cost[check_x][check_y + i] != INF)) {
					if (dijkstra.cost[check_x][check_y + i] == dijkstra.cost[check_x + 1][check_y + i]) {
						refresh = false;
					}
				}

				if (refresh) {
					dijkstra.cost[check_x][check_y + i] = (dijkstra.cost[check_x][check_y] + (int)cost);
					dijkstra.next_x[check_x][check_y + i] = check_x;
					dijkstra.next_y[check_x][check_y + i] = (check_y + (i - 2));
					//dijkstra.next_x[check_x][check_y + i] = check_x;
					//dijkstra.next_y[check_x][check_y + i] = check_y;
				}
			} else {
				break;
			}
		}

		// 南
		for (unsigned char i = 2; i <= check_y; i += 2) {
			cost = CalcAccelStep(false, (i / 2));
			refresh = true;

			if ((dijkstra.flag[check_x][check_y - i] == 0) && (dijkstra.cost[check_x][check_y - i] > (dijkstra.cost[check_x][check_y] + (int)cost))) {
				if ((check_x > 0) && (dijkstra.cost[check_x][check_y - i] != INF)) {
					if (dijkstra.cost[check_x][check_y - i] == dijkstra.cost[check_x - 1][check_y - i]) {
						refresh = false;
					}
				}

				if ((check_x < (MAPSIZE_X - 1)) && (dijkstra.cost[check_x][check_y - i] != INF)) {
					if (dijkstra.cost[check_x][check_y - i] == dijkstra.cost[check_x + 1][check_y - i]) {
						refresh = false;
					}
				}

				if (refresh) {
					dijkstra.cost[check_x][check_y - i] = (dijkstra.cost[check_x][check_y] + (int)cost);
					dijkstra.next_x[check_x][check_y - i] = check_x;
					dijkstra.next_y[check_x][check_y - i] = (check_y - (i - 2));
					//dijkstra.next_x[check_x][check_y - i] = check_x;
					//dijkstra.next_y[check_x][check_y - i] = check_y;
				}
			} else {
				break;
			}
		}

		// 北西
		for (unsigned char i = 1; i < (MAPSIZE_Y * 2 - check_y); i++) {
			cost = CalcAccelStep(true, i);
			refresh = true;

			if (i % 2 == 1) {
				cnt_x++;
			}

			if ((check_x - cnt_x) < 0) {
				break;
			} else if ((dijkstra.flag[check_x - cnt_x][check_y + i] == 0) && (dijkstra.cost[check_x - cnt_x][check_y + i] > (dijkstra.cost[check_x][check_y] + (int)cost))) {
				//if ((check_y + i + 1) < (MAPSIZE_Y * 2 - 1)) {
				//	if (dijkstra.cost[check_x - (cnt_x - (i % 2))][check_y + (i + 1)] == dijkstra.cost[check_x - (cnt_x - (i % 2)) - 1][check_y + (i + 1)]) {
				//		if ((dijkstra.cost[check_x - (cnt_x - (i % 2))][check_y + (i + 1)] != INF) && (dijkstra.cost[check_x - (cnt_x - (i % 2)) - 1][check_y + (i + 1)] != INF)) {
				//			refresh = false;
				//		}
				//	}
				//}

				//if (refresh) {
				//	dijkstra.cost[check_x - cnt_x][check_y + i] = (dijkstra.cost[check_x][check_y] + (int)cost);
				//	dijkstra.next_x[check_x - cnt_x][check_y + i] = (check_x - (cnt_x - (i % 2)));
				//	dijkstra.next_y[check_x - cnt_x][check_y + i] = (check_y + (i - 1));
				//}

				dijkstra.cost[check_x - cnt_x][check_y + i] = (dijkstra.cost[check_x][check_y] + (int)cost);
				dijkstra.next_x[check_x - cnt_x][check_y + i] = (check_x - (cnt_x - (i % 2)));
				dijkstra.next_y[check_x - cnt_x][check_y + i] = (check_y + (i - 1));
				////dijkstra.next_x[check_x - cnt_x][check_y + i] = check_x;
				////dijkstra.next_y[check_x - cnt_x][check_y + i] = check_y;
			} else if (dijkstra.cost[check_x - cnt_x][check_y + i] == INF) {
				break;
			}
		}

		cnt_x = 0;

		// 南西
		for (unsigned char i = 1; i <= check_y; i++) {
			cost = CalcAccelStep(true, i);
			refresh = true;

			if (i % 2 == 1) {
				cnt_x++;
			}

			if ((check_x - cnt_x) < 0) {
				break;
			} else if ((dijkstra.flag[check_x - cnt_x][check_y - i] == 0) && (dijkstra.cost[check_x - cnt_x][check_y - i] > (dijkstra.cost[check_x][check_y] + (int)cost))) {
				//if ((check_y - i - 1) > 0) {
				//	if (dijkstra.cost[check_x - (cnt_x - (i % 2))][check_y - (i + 1)] == dijkstra.cost[check_x - (cnt_x - (i % 2)) - 1][check_y - (i + 1)]) {
				//		if ((dijkstra.cost[check_x - (cnt_x - (i % 2))][check_y - (i + 1)] != INF) && (dijkstra.cost[check_x - (cnt_x - (i % 2)) - 1][check_y - (i + 1)] != INF)) {
				//			refresh = false;
				//		}
				//	}
				//}

				//if (refresh) {
				//	dijkstra.cost[check_x - cnt_x][check_y - i] = (dijkstra.cost[check_x][check_y] + (int)cost);
				//	dijkstra.next_x[check_x - cnt_x][check_y - i] = (check_x - (cnt_x - (i % 2)));
				//	dijkstra.next_y[check_x - cnt_x][check_y - i] = (check_y - (i - 1));
				//}

				dijkstra.cost[check_x - cnt_x][check_y - i] = (dijkstra.cost[check_x][check_y] + (int)cost);
				dijkstra.next_x[check_x - cnt_x][check_y - i] = (check_x - (cnt_x - (i % 2)));
				dijkstra.next_y[check_x - cnt_x][check_y - i] = (check_y - (i - 1));
				////dijkstra.next_x[check_x - cnt_x][check_y - i] = check_x;
				////dijkstra.next_y[check_x - cnt_x][check_y - i] = check_y;
		} else if (dijkstra.cost[check_x - cnt_x][check_y - i] == INF) {
				break;
			}
		}

		cnt_x = 0;

		// 北東
		for (unsigned char i = 1; i < (MAPSIZE_Y * 2 - check_y); i++) {
			cost = CalcAccelStep(true, i);
			refresh = true;

			if (i % 2 == 0) {
				cnt_x++;
			}

			if ((check_x + cnt_x) >= MAPSIZE_X) {
				break;
			} else if ((dijkstra.flag[check_x + cnt_x][check_y + i] == 0) && (dijkstra.cost[check_x + cnt_x][check_y + i] > (dijkstra.cost[check_x][check_y] + (int)cost))) {
				//if ((check_y + i + 1) < (MAPSIZE_Y * 2 - 1)) {
				//	if (dijkstra.cost[check_x + (cnt_x - ((i + 1) % 2))][check_y + (i + 1)] == dijkstra.cost[check_x + (cnt_x - ((i + 1) % 2)) + 1][check_y + (i + 1)]) {
				//		if ((dijkstra.cost[check_x + (cnt_x - ((i + 1) % 2))][check_y + (i + 1)] != INF) && (dijkstra.cost[check_x + (cnt_x - ((i + 1) % 2)) + 1][check_y + (i + 1)] != INF)) {
				//			refresh = false;
				//		}
				//	}
				//}

				//if (refresh) {
				//	dijkstra.cost[check_x + cnt_x][check_y + i] = (dijkstra.cost[check_x][check_y] + (int)cost);
				//	dijkstra.next_x[check_x + cnt_x][check_y + i] = (check_x + (cnt_x - ((i + 1) % 2)));
				//	dijkstra.next_y[check_x + cnt_x][check_y + i] = (check_y + (i - 1));
				//}

				dijkstra.cost[check_x + cnt_x][check_y + i] = (dijkstra.cost[check_x][check_y] + (int)cost);
				dijkstra.next_x[check_x + cnt_x][check_y + i] = (check_x + (cnt_x - ((i + 1) % 2)));
				dijkstra.next_y[check_x + cnt_x][check_y + i] = (check_y + (i - 1));
				////dijkstra.next_x[check_x + cnt_x][check_y + i] = check_x;
				////dijkstra.next_y[check_x + cnt_x][check_y + i] = check_y;
			} else if (dijkstra.cost[check_x + cnt_x][check_y + i] == INF) {
				break;
			}
		}

		cnt_x = 0;

		// 南東
		for (unsigned char i = 1; i <= check_y; i++) {
			cost = CalcAccelStep(true, i);
			refresh = true;

			if (i % 2 == 0) {
				cnt_x++;
			}

			if ((check_x + cnt_x) >= MAPSIZE_X) {
				break;
			} else if ((dijkstra.flag[check_x + cnt_x][check_y - i] == 0) && (dijkstra.cost[check_x + cnt_x][check_y - i] > (dijkstra.cost[check_x][check_y] + (int)cost))) {
				//if ((check_y - i - 1) > 0) {
				//	if (dijkstra.cost[check_x + (cnt_x - ((i + 1) % 2))][check_y - (i + 1)] == dijkstra.cost[check_x + (cnt_x - ((i + 1) % 2)) + 1][check_y - (i + 1)]) {
				//		if ((dijkstra.cost[check_x + (cnt_x - ((i + 1) % 2))][check_y - (i + 1)] != INF) && (dijkstra.cost[check_x + (cnt_x - ((i + 1) % 2)) + 1][check_y - (i + 1)] != INF)) {
				//			refresh = false;
				//		}
				//	}
				//}


				//if (refresh) {
				//	dijkstra.cost[check_x + cnt_x][check_y - i] = (dijkstra.cost[check_x][check_y] + (int)cost);
				//	dijkstra.next_x[check_x + cnt_x][check_y - i] = (check_x + (cnt_x - ((i + 1) % 2)));
				//	dijkstra.next_y[check_x + cnt_x][check_y - i] = (check_y - (i - 1));
				//}

				dijkstra.cost[check_x + cnt_x][check_y - i] = (dijkstra.cost[check_x][check_y] + (int)cost);
				dijkstra.next_x[check_x + cnt_x][check_y - i] = (check_x + (cnt_x - ((i + 1) % 2)));
				dijkstra.next_y[check_x + cnt_x][check_y - i] = (check_y - (i - 1));

				//dijkstra.next_x[check_x + cnt_x][check_y - i] = check_x;
				//dijkstra.next_y[check_x + cnt_x][check_y - i] = check_y;
			} else if (dijkstra.cost[check_x + cnt_x][check_y - i] == INF) {
				break;
			}
		}

		cnt_x = 0;
	}

	void Map::CheckDijikstra_Horizontal (unsigned char check_x, unsigned char check_y) {
		float cost;
		unsigned char cnt_x = 0;

		// 東
		for (unsigned char i = 1; i < (MAPSIZE_X - check_x); i++) {
			cost = CalcAccelStep(false, i);

			if ((dijkstra.flag[check_x + i][check_y] == 0) && (dijkstra.cost[check_x + i][check_y] > (dijkstra.cost[check_x][check_y] + (int)cost))) {
				dijkstra.cost[check_x + i][check_y] = (dijkstra.cost[check_x][check_y] + (int)cost);
				dijkstra.next_x[check_x + i][check_y] = (check_x + (i - 1));
				dijkstra.next_y[check_x + i][check_y] = check_y;
				//dijkstra.next_x[check_x + i][check_y] = check_x;
				//dijkstra.next_y[check_x + i][check_y] = check_y;
			} else {
				break;
			}
		}

		// 西
		for (unsigned char i = 1; i <= check_x; i++) {
			cost = CalcAccelStep(false, i);

			if ((dijkstra.flag[check_x - i][check_y] == 0) && (dijkstra.cost[check_x - i][check_y] > (dijkstra.cost[check_x][check_y] + (int)cost))) {
				dijkstra.cost[check_x - i][check_y] = (dijkstra.cost[check_x][check_y] + (int)cost);
				dijkstra.next_x[check_x - i][check_y] = (check_x - (i - 1));
				dijkstra.next_y[check_x - i][check_y] = check_y;
				//dijkstra.next_x[check_x - i][check_y] = check_x;
				//dijkstra.next_y[check_x - i][check_y] = check_y;
			} else {
				break;
			}
		}

		// 北西
		for (unsigned char i = 1; i < (MAPSIZE_Y * 2 - check_y); i++) {
			cost = CalcAccelStep(true, i);

			if (i % 2 == 0) {
				cnt_x++;
			}

			if ((check_x - cnt_x) < 0) {
				break;
			} else if ((dijkstra.flag[check_x - cnt_x][check_y + i] == 0) && (dijkstra.cost[check_x - cnt_x][check_y + i] > (dijkstra.cost[check_x][check_y] + (int)cost))) {
				dijkstra.cost[check_x - cnt_x][check_y + i] = (dijkstra.cost[check_x][check_y] + (int)cost);
				dijkstra.next_x[check_x - cnt_x][check_y + i] = (check_x - (cnt_x - ((i + 1) % 2)));
				dijkstra.next_y[check_x - cnt_x][check_y + i] = (check_y + (i - 1));
				//dijkstra.next_x[check_x - cnt_x][check_y + i] = check_x;
				//dijkstra.next_y[check_x - cnt_x][check_y + i] = check_y;
			} else if (dijkstra.cost[check_x - cnt_x][check_y + i] == INF) {
				break;
			}
		}

		cnt_x = 0;

		// 南西
		for (unsigned char i = 1; i <= check_y; i++) {
			cost = CalcAccelStep(true, i);

			if (i % 2 == 0) {
				cnt_x++;
			}

			if ((check_x - cnt_x) < 0) {
				break;
			} else if ((dijkstra.flag[check_x - cnt_x][check_y - i] == 0) && (dijkstra.cost[check_x - cnt_x][check_y - i] > (dijkstra.cost[check_x][check_y] + (int)cost))) {
				dijkstra.cost[check_x - cnt_x][check_y - i] = (dijkstra.cost[check_x][check_y] + (int)cost);
				dijkstra.next_x[check_x - cnt_x][check_y - i] = (check_x - (cnt_x - ((i + 1) % 2)));
				dijkstra.next_y[check_x - cnt_x][check_y - i] = (check_y - (i - 1));
				//dijkstra.next_x[check_x - cnt_x][check_y - i] = check_x;
				//dijkstra.next_y[check_x - cnt_x][check_y - i] = check_y;
			} else if (dijkstra.cost[check_x - cnt_x][check_y - i] == INF) {
				break;
			}
		}

		cnt_x = 0;

		// 北東
		for (unsigned char i = 1; i < (MAPSIZE_Y * 2 - check_y); i++) {
			cost = CalcAccelStep(true, i);

			if (i % 2 == 1) {
				cnt_x++;
			}

			if ((check_x + cnt_x) >= MAPSIZE_X) {
				break;
			} else if ((dijkstra.flag[check_x + cnt_x][check_y + i] == 0) && (dijkstra.cost[check_x + cnt_x][check_y + i] > (dijkstra.cost[check_x][check_y] + (int)cost))) {
				dijkstra.cost[check_x + cnt_x][check_y + i] = (dijkstra.cost[check_x][check_y] + (int)cost);
				dijkstra.next_x[check_x + cnt_x][check_y + i] = (check_x + (cnt_x - (i % 2)));
				dijkstra.next_y[check_x + cnt_x][check_y + i] = (check_y + (i - 1));
				//dijkstra.next_x[check_x + cnt_x][check_y + i] = check_x;
				//dijkstra.next_y[check_x + cnt_x][check_y + i] = check_y;
			} else if (dijkstra.cost[check_x + cnt_x][check_y + i] == INF) {
				break;
			}
		}

		cnt_x = 0;

		// 南西
		for (unsigned char i = 1; i <= check_y; i++) {
			cost = CalcAccelStep(true, i);

			if (i % 2 == 1) {
				cnt_x++;
			}

			if ((check_x + cnt_x) >= MAPSIZE_X) {
				break;
			} else if ((dijkstra.flag[check_x + cnt_x][check_y - i] == 0) && (dijkstra.cost[check_x + cnt_x][check_y - i] > (dijkstra.cost[check_x][check_y] + (int)cost))) {
				dijkstra.cost[check_x + cnt_x][check_y - i] = (dijkstra.cost[check_x][check_y] + (int)cost);
				dijkstra.next_x[check_x + cnt_x][check_y - i] = (check_x + (cnt_x - (i % 2)));
				dijkstra.next_y[check_x + cnt_x][check_y - i] = (check_y - (i - 1));
				//dijkstra.next_x[check_x + cnt_x][check_y - i] = check_x;
				//dijkstra.next_y[check_x + cnt_x][check_y - i] = check_y;
			} else if (dijkstra.cost[check_x + cnt_x][check_y - i] == INF) {
				break;
			}
		}

		cnt_x = 0;
	}

	void Map::Dijkstra (void) {
		unsigned char check_x = 0, check_y = 0;
		unsigned char j, i;

		path.cost_slant = 10;
		path.cost_straight = 14;

		for (j = 0; j < MAPSIZE_Y * 2; j++) {
			for (i = 0; i < MAPSIZE_X; i++) {
				dijkstra.cost[i][j] = INF;
				dijkstra.next_x[i][j] = 0xff;
				dijkstra.next_y[i][j] = 0xff;

				if ((j % 2 == 0) && (i == MAPSIZE_X - 1)) {
					dijkstra.flag[i][j] = 1;
				} else {
					dijkstra.flag[i][j] = 0;
				}
			}

			if (GOAL_X > 0) {
				dijkstra.cost[GOAL_X - 1][2 * GOAL_Y] = 0;
				dijkstra.flag[GOAL_X - 1][2 * GOAL_Y] = 0;
			}

			dijkstra.cost[GOAL_X][2 * GOAL_Y] = 0;
			dijkstra.flag[GOAL_X][2 * GOAL_Y] = 1;

			dijkstra.cost[GOAL_X][2 * GOAL_Y + 1] = 0;
			dijkstra.flag[GOAL_X][2 * GOAL_Y + 1] = 1;

			if (GOAL_Y > 0) {
				dijkstra.cost[GOAL_X][2 * GOAL_Y - 1] = 0;
				dijkstra.flag[GOAL_X][2 * GOAL_Y - 1] = 0;
			}

			if (ENABLE_FULLGOAL) {
				// 上
				dijkstra.cost[GOAL_X - 1][2 * GOAL_Y + 2] = 0;
				dijkstra.flag[GOAL_X - 1][2 * GOAL_Y + 2] = 0;
				dijkstra.cost[GOAL_X][2 * GOAL_Y + 3] = 0;
				dijkstra.flag[GOAL_X][2 * GOAL_Y + 3] = 0;
				dijkstra.cost[GOAL_X][2 * GOAL_Y + 2] = 0;
				dijkstra.flag[GOAL_X][2 * GOAL_Y + 2] = 1;

				// 右上
				dijkstra.cost[GOAL_X + 1][2 * GOAL_Y + 3] = 0;
				dijkstra.flag[GOAL_X + 1][2 * GOAL_Y + 3] = 0;
				dijkstra.cost[GOAL_X + 1][2 * GOAL_Y + 2] = 0;
				dijkstra.flag[GOAL_X + 1][2 * GOAL_Y + 2] = 0;

				// 右横
				dijkstra.cost[GOAL_X + 1][2 * GOAL_Y] = 0;
				dijkstra.flag[GOAL_X + 1][2 * GOAL_Y] = 0;
				dijkstra.cost[GOAL_X + 1][2 * GOAL_Y + 1] = 0;
				dijkstra.flag[GOAL_X + 1][2 * GOAL_Y + 1] = 1;

				if (GOAL_Y > 0) {
					dijkstra.cost[GOAL_X + 1][2 * GOAL_Y - 1] = 0;
					dijkstra.flag[GOAL_X + 1][2 * GOAL_Y - 1] = 0;
				}

				// 左下-左
				if (GOAL_X > 0) {
					dijkstra.next_x[GOAL_X - 1][2 * GOAL_Y] = GOAL_X;
					dijkstra.next_y[GOAL_X - 1][2 * GOAL_Y] = 2 * GOAL_Y;
				}

				// 左下-下
				if (GOAL_Y > 0) {
					dijkstra.next_x[GOAL_X][2 * GOAL_Y - 1] = GOAL_X;
					dijkstra.next_y[GOAL_X][2 * GOAL_Y - 1] = 2 * GOAL_Y + 1;
				}

				// 左上-左
				if (GOAL_X > 0) {
					dijkstra.next_x[GOAL_X - 1][2 * GOAL_Y + 2] = GOAL_X;
					dijkstra.next_y[GOAL_X - 1][2 * GOAL_Y + 2] = 2 * GOAL_Y + 2;
				}

				// 左上-上
				dijkstra.next_x[GOAL_X][2 * GOAL_Y + 3] = GOAL_X;
				dijkstra.next_y[GOAL_X][2 * GOAL_Y + 3] = 2 * GOAL_Y + 1;

				// 右上-上
				dijkstra.next_x[GOAL_X + 1][2 * GOAL_Y + 3] = GOAL_X + 1;
				dijkstra.next_y[GOAL_X + 1][2 * GOAL_Y + 3] = 2 * GOAL_Y + 1;

				// 右上-右
				dijkstra.next_x[GOAL_X + 1][2 * GOAL_Y + 2] = GOAL_X;
				dijkstra.next_y[GOAL_X + 1][2 * GOAL_Y + 2] = 2 * GOAL_Y + 2;

				// 右下-右
				dijkstra.next_x[GOAL_X + 1][2 * GOAL_Y] = GOAL_X;
				dijkstra.next_y[GOAL_X + 1][2 * GOAL_Y] = 2 * GOAL_Y;

				// 右下-下
				if (GOAL_Y > 0) {
					dijkstra.next_x[GOAL_X + 1][2 * GOAL_Y - 1] = GOAL_X + 1;
					dijkstra.next_y[GOAL_X + 1][2 * GOAL_Y - 1] = 2 * GOAL_Y + 1;
				}
			} else {
				dijkstra.flag[GOAL_X][2 * GOAL_Y] = 0;
				dijkstra.flag[GOAL_X][2 * GOAL_Y + 1] = 0;
			}
		}

		for (j = 0; j < MAPSIZE_Y; j++) {
			for (i = 0; i < MAPSIZE_X; i++) {
				if (((data[i][j] & WALL_NORTH) == WALL_NORTH) && (j < 15)) {
					dijkstra.cost[i][2 * j + 1] = INF;
					dijkstra.flag[i][2 * j + 1] = 1;
				}

				if (((data[i][j] & WALL_EAST) == WALL_EAST) && (i < 15)) {
					dijkstra.cost[i][2 * j] = INF;
					dijkstra.flag[i][2 * j] = 1;
				}

				// 動作未検証
				if (((data[i][j] & WALL_NORTH) == 0) && ((data[i][j] & FLAG_NORTH) == 0) && (j < 15)) {
					dijkstra.cost[i][2 * j + 1] = INF;
					dijkstra.flag[i][2 * j + 1] = 1;
				}

				if (((data[i][j] & WALL_EAST) == 0) && ((data[i][j] & FLAG_EAST) == 0) && (i < 15)) {
					dijkstra.cost[i][2 * j] = INF;
					dijkstra.flag[i][2 * j] = 1;
				}
			}
		}

		while (1) {
			dijkstra.cost_min = INF;

			for (j = 0; j < MAPSIZE_Y * 2; j++) {
				for (i = 0; i < MAPSIZE_X; i++) {
					if ((dijkstra.cost_min > dijkstra.cost[i][j]) && (dijkstra.flag[i][j] == 0)) {
						dijkstra.cost_min = dijkstra.cost[i][j];
						check_x = i;
						check_y = j;
					}
				}
			}

			if (dijkstra.cost_min == INF) {
				break;
			} else {
				if ((check_y % 2) == 0) {
					CheckDijikstra_Horizontal(check_x, check_y);
				} else {
					CheckDijikstra_Vertical(check_x, check_y);
				}
			}

			dijkstra.flag[check_x][check_y] = 1;
		}
	}

	void Map::MakePath (unsigned int velocity, unsigned int turn_velocity, unsigned int slant_velocity, unsigned int accel) {
		unsigned char check_x = 0, check_y = 1, next_x = 0, next_y = 0;
		unsigned int path_cnt = 1, path_long = 0;
		char senddata[127];

		/*
		 TODO: 斜め中加速度を別に指定できるようにする(次期目標?)
		 */

		Map::velocity = (float)velocity;
		Map::turn_velocity = (float)turn_velocity;
		Map::accel = (float)accel;

		Map::Dijkstra();

        do {
			next_x = dijkstra.next_x[check_x][check_y];
			next_y = dijkstra.next_y[check_x][check_y];

//			sprintf(senddata, "(%2d, %2d) => (%2d, %2d)\n", check_x, check_y, next_x, next_y);
//			System::SCI::SendChar(senddata);

			if ((check_y % 2) == 0) {
				if ((check_x < next_x) && (next_y == check_y)) {
					// 東
					path.dir[path_cnt] = 2;
				} else if ((check_x > next_x) && (next_y == check_y)) {
					// 西
					path.dir[path_cnt] = -2;
				} else if ((next_x > check_x) && (next_y < check_y)) {
					// 南東
					if ((next_x + 1 < (MAPSIZE_X - 1)) && (path_cnt > 0)) {
						if ((dijkstra.cost[next_x][next_y] == dijkstra.cost[next_x - 1][next_y]) && (path.dir[path_cnt - 1] == -3)) {
							path.dir[path_cnt] = -3;
							next_x--;
						} else {
							path.dir[path_cnt] = 3;
						}
					} else {
						path.dir[path_cnt] = 3;
					}
				} else if ((next_x <= check_x) && (next_y < check_y)) {
					// 南西
					if ((next_x + 1 < (MAPSIZE_X - 1)) && (path_cnt > 0)) {
						if ((dijkstra.cost[next_x][next_y] == dijkstra.cost[next_x + 1][next_y]) && (path.dir[path_cnt - 1] == 3)) {
							path.dir[path_cnt] = 3;
							next_x++;
						} else {
							path.dir[path_cnt] = -3;
						}
					} else {
						path.dir[path_cnt] = -3;
					}
				} else if ((next_x > check_x) && (next_y > check_y)) {
					// 北東
					if ((next_x + 1 < (MAPSIZE_X - 1)) && (path_cnt > 0)) {
						if ((dijkstra.cost[next_x][next_y] == dijkstra.cost[next_x - 1][next_y]) && (path.dir[path_cnt - 1] == -1)) {
							path.dir[path_cnt] = -1;
							next_x--;
						} else {
							path.dir[path_cnt] = 1;
						}
					} else {
						path.dir[path_cnt] = 1;
					}
				} else if ((next_x <= check_x) && (next_y > check_y)) {
					// 北西
					if ((next_x + 1 < (MAPSIZE_X - 1)) && (path_cnt > 0)) {
						if ((dijkstra.cost[next_x][next_y] == dijkstra.cost[next_x + 1][next_y]) && (path.dir[path_cnt - 1] == 1)) {
							path.dir[path_cnt] = 1;
							next_x++;
						} else {
							path.dir[path_cnt] = -1;
						}
					} else {
						path.dir[path_cnt] = -1;
					}
				}
			} else {
				if ((next_x == check_x) && (next_y > (check_y + 1))) {
					// 北
					path.dir[path_cnt] = 0;
				} else if ((next_x == check_x) && (next_y < (check_y - 1))) {
					// 南
					path.dir[path_cnt] = 4;
				} else if ((next_x >= check_x) && (next_y < check_y)) {
					// 南東
					path.dir[path_cnt] = 3;
				} else if ((next_x < check_x) && (next_y < check_y)) {
					// 南西
					path.dir[path_cnt] = -3;
				} else if ((next_x >= check_x) && (next_y > check_y)) {
					// 北東
					path.dir[path_cnt] = 1;
				} else if ((next_x < check_x) && (next_y > check_y)) {
					// 北西
					path.dir[path_cnt] = -1;
				}
			}

			check_x = next_x;
			check_y = next_y;

            path_cnt++;

			if ((next_x >= MAPSIZE_X) || (next_y >= (MAPSIZE_Y * 2))) {
				break;
			}
        } while ((dijkstra.next_x[check_x][check_y] != 255) && (dijkstra.next_y[check_x][check_y] != 255));

        path_long = path_cnt;
		path.length = path_long;

	    path.mode[0] = 1;
	    path.dir[0] = 0;
		path.velocity[0] = (float)velocity;
		path_cnt++;

	    // 読出し時の旋回方向決定
	    // dirの差を取って正負によって判定
	    // dir[cnt + 1] - dir[cnt] (正なら右, 負なら左)
	    // ⇒ 0で判定不能: dir[cnt] - dir[cnt - 1] (正なら右, 負なら左)
	    for (path_cnt = 1; path_cnt < path_long; path_cnt++) {
	        if ((abs(path.dir[path_cnt - 1]) % 2 == 0) && (abs(path.dir[path_cnt]) % 2 == 1) && (abs(path.dir[path_cnt + 1]) % 2 == 0)) {
				path.mode[path_cnt] = 10;   // 大回り90
				path.velocity[path_cnt] = (float)turn_velocity;
				path.mode[path_cnt + 1] = 1;
				path.velocity[path_cnt + 1] = (float)velocity;

	            if ((path_cnt > 1) && (path.mode[path_cnt - 1] <= 1) && (path.mode[path_cnt - 2] >= 10)) {
	                path.mode[path_cnt - 1] = 255;
	            } else {
	                if (path_cnt == 1) {
	                    path.mode[path_cnt - 1] = 255;
	                } else {
	                    path.mode[path_cnt - 1] = 1;
	                }
	            }

	            path_cnt++;
	        } else if ((abs(path.dir[path_cnt - 1]) % 2 == 0) && (abs(path.dir[path_cnt]) % 2 == 1) && (abs(path.dir[path_cnt + 1]) % 2 == 1) && (abs(path.dir[path_cnt + 2]) % 2 == 0) && (path.dir[path_cnt] != path.dir[path_cnt + 1])) {
				path.mode[path_cnt] = 11;   // 大回り180
				path.velocity[path_cnt] = (float)turn_velocity;
				path.mode[path_cnt + 1] = 255;
	            path.mode[path_cnt + 2] = 1;
				path.velocity[path_cnt + 2] = (float)velocity;

	            if ((path.mode[path_cnt - 1] <= 1) && (path.mode[path_cnt - 2] >= 10)) {
	                path.mode[path_cnt - 1] = 255;
	            } else {
	                path.mode[path_cnt - 1] = 1;
	            }

	            path_cnt += 2;
	        } else if ((path.mode[path_cnt - 1] < 10) && (abs(path.dir[path_cnt]) % 2 == 1) && (path.dir[path_cnt] == path.dir[path_cnt + 1])) {
				path.mode[path_cnt] = 20;   // 45 In
				path.velocity[path_cnt] = (float)turn_velocity;

	            if ((path_cnt > 1) && (path.mode[path_cnt - 1] <= 1) && (path.mode[path_cnt - 2] >= 10)) {
	                path.mode[path_cnt - 1] = 255;
	            } else {
	                if (path_cnt == 1) {
	                    path.mode[path_cnt - 1] = 255;
	                } else {
	                    path.mode[path_cnt - 1] = 1;
	                }
	            }
	        } else if ((abs(path.dir[path_cnt + 1]) % 2 == 0) && (abs(path.dir[path_cnt]) % 2 == 1) && (path.dir[path_cnt] == path.dir[path_cnt - 1])) {
				path.mode[path_cnt] = 21;   // 45 Out
				path.velocity[path_cnt] = (float)turn_velocity;
				path.mode[path_cnt + 1] = 1;
				path.velocity[path_cnt + 1] = (float)velocity;
				path_cnt++;
	        } else if ((abs(path.dir[path_cnt]) % 2 == 1) && (path.dir[path_cnt] == path.dir[path_cnt - 1]) && (path.dir[path_cnt + 1] == path.dir[path_cnt + 2]) && (path.dir[path_cnt] != path.dir[path_cnt + 1])) {
				path.mode[path_cnt] = 30;   // 90
				path.velocity[path_cnt] = (float)turn_velocity;
				path.mode[path_cnt + 1] = 255;
	            path_cnt++;
	        } else if ((abs(path.dir[path_cnt - 1]) % 2 == 0) && (abs(path.dir[path_cnt]) % 2 == 1) && (abs(path.dir[path_cnt + 1]) % 2 == 1) && (path.dir[path_cnt] != path.dir[path_cnt + 1])) {
				path.mode[path_cnt] = 40;   // 135 In
				path.velocity[path_cnt] = (float)turn_velocity;
				path.mode[path_cnt + 1] = 255;

	            if ((path_cnt > 1) && (path.mode[path_cnt - 1] <= 1) && (path.mode[path_cnt - 2] >= 10)) {
	                path.mode[path_cnt - 1] = 255;
	            } else {
	                if (path_cnt == 1) {
	                    path.mode[path_cnt - 1] = 255;
	                } else {
	                    path.mode[path_cnt - 1] = 1;
	                }
	            }

	            path_cnt++;
	        } else if ((abs(path.dir[path_cnt]) % 2 == 1) && (abs(path.dir[path_cnt + 1]) % 2 == 1) && (abs(path.dir[path_cnt + 2]) % 2 == 0) && (path.dir[path_cnt] != path.dir[path_cnt + 1])) {
				path.mode[path_cnt] = 41;   // 135 Out
				path.velocity[path_cnt] = (float)turn_velocity;
				path.mode[path_cnt + 1] = 255;
	            path.mode[path_cnt + 2] = 1;
				path.velocity[path_cnt + 2] = (float)velocity;
				path_cnt += 2;
	        } else if ((abs(path.dir[path_cnt - 1]) % 2 == 1) && (abs(path.dir[path_cnt]) % 2 == 1) && (abs(path.dir[path_cnt + 1]) % 2 == 1)) {
				path.mode[path_cnt] = 100;  // 斜め直進
				path.velocity[path_cnt] = (float)slant_velocity;
			} else {
				path.mode[path_cnt] = 0;
				path.velocity[path_cnt] = (float)slant_velocity;
			}
	    }

	    path.mode[path_cnt] = 1;
	    path.dir[path_cnt] = 0;
		path.velocity[path_cnt] = 0;

		path.mode[path_cnt + 1] = 127;
		path.dir[path_cnt + 1] = 0;
		path.velocity[path_cnt + 1] = 0;

	    path.length++;
	}

	void Map::ReadPath(unsigned int velocity, unsigned int turn_velocity, unsigned int slant_velocity, unsigned int accel) {
		unsigned int path_cnt;
		unsigned char i = 0, j = 0, turnparam = 0, turnparam_num = 0, prev_mode = 255;
		signed char path_dir;
		float accel_distance;
		bool failed = false, straight_first = false;

		switch (turn_velocity) {
			default:
				turnparam_num = 0;
				break;
		}

		// 進路情報からPath生成
		// 加減速距離をここで計算しておく? (平方根が重そう)
		// 減速距離だけ求めておいてリミッターに引っかかるor減速開始位置到達まで加速させる?
		// 引数にすると汚いから専用構造体用意 & モード遷移時に値読み出し&代入とか
		for (path_cnt = 0; path_cnt < path.length; path_cnt++) {
			path_dir = path.dir[path_cnt + 1] - path.dir[path_cnt];

			if (path_dir <= -4) {
				path_dir += 8;
			} else if (path_dir > 4) {
				path_dir -= 8;
			}

			if (path.mode[path_cnt] < 10) {
				path.turn_dir[path_cnt] = 0;
			} else if (path.mode[path_cnt] < 100) {
				if (path_dir != 0) {
					if (path_dir > 0) {
						path.turn_dir[path_cnt] = 1;
					} else {
						path.turn_dir[path_cnt] = -1;
					}
				} else if (path_cnt > 0) {
					path_dir = path.dir[path_cnt] - path.dir[path_cnt - 1];

					if (path_dir <= -4) {
						path_dir += 8;
					} else if (path_dir > 4) {
						path_dir -= 8;
					}

					if (path_dir > 0) {
						path.turn_dir[path_cnt] = 1;
					} else {
						path.turn_dir[path_cnt] = -1;
					}
				} else {
					path.turn_dir[path_cnt] = 127;
				}
			} else {
				path.turn_dir[path_cnt] = 0;
			}
		}

		for (path_cnt = 0; path_cnt < path.length; path_cnt++) {
			if (path.mode[path_cnt] != 255) {
				i = 0;
				j = 0;

				switch (path.mode[path_cnt]) {
					case 0:
					case 1:
						i = 0;

						if (path.mode[path_cnt] == 0) {
							j = 2;
						} else {
							j = 1;
						}

						do {
							if ((path.mode[path_cnt + i + 1] >= 10) && (path.mode[path_cnt + i + 1] != 255)) {
								break;
							} else if (path.mode[path_cnt + i + 1] == 0) {
								path.mode[path_cnt + i + 1] = 255;
								i++;
								j += 2;
							} else if (path.mode[path_cnt + i + 1] == 1) {
								path.mode[path_cnt + i + 1] = 255;
								i++;
								j++;
							}
						} while ((path_cnt + i + 1) < path.length);

						path.velocity_next[path_cnt] = path.velocity[path_cnt + i + 1];
						accel_distance = ((float)(path.velocity[path_cnt] * path.velocity[path_cnt] - path.velocity[path_cnt + i + 1] * path.velocity[path_cnt + i + 1]) / (2 * (float)accel));

	                    if (path_cnt == 0) {
	                        accel_distance += ((float)(path.velocity[path_cnt] * path.velocity[path_cnt]) / (2 * (float)accel));

	                        if (accel_distance > (SECTION_STRAIGHT * (float)j)) {
	                            path.velocity[path_cnt] = (short)sqrt(2.0 * (float)accel * SECTION_STRAIGHT * 0.25 * j);
	                            path.decel_length[path_cnt] = (short)((path.velocity[path_cnt] * path.velocity[path_cnt]) / (2 * (float)accel));
	                        } else {
	                            path.velocity[path_cnt] = (short)sqrt((2.0 * (float)accel * SECTION_STRAIGHT * 0.25 * j)  + path.velocity[path_cnt + i + 1] * path.velocity[path_cnt + i + 1]);
	                            path.decel_length[path_cnt] = (short)((path.velocity[path_cnt] * path.velocity[path_cnt] - path.velocity[path_cnt + i + 1] * path.velocity[path_cnt + i + 1]) / (2 * (float)accel));
	                        }
	                    } else if (accel_distance > (SECTION_STRAIGHT * (float)j * 0.25)) {
							path.velocity[path_cnt] = (short)sqrt((2.0 * (float)accel * SECTION_STRAIGHT * 0.25 * j) + path.velocity[path_cnt + i + 1] * path.velocity[path_cnt + i + 1]);
							path.decel_length[path_cnt] = (short)((path.velocity[path_cnt] * path.velocity[path_cnt] - path.velocity[path_cnt + i + 1] * path.velocity[path_cnt + i + 1]) / (2 * (float)accel));
						} else {
							path.decel_length[path_cnt] = (short)accel_distance;
						}

						path.section[path_cnt] = (unsigned char)j;
						path_cnt += i;
						break;

					case 100:	// 斜め中直進
						i = 0;

						while (1) {
							if (path.mode[path_cnt + i + 1] != 100) {
								break;
							} else {
								path.mode[path_cnt + i + 1] = 255;
								i++;
							}
						}

						path.velocity_next[path_cnt] = path.velocity[path_cnt + i + 1];
						accel_distance = ((float)(path.velocity[path_cnt] * path.velocity[path_cnt] - path.velocity[path_cnt + i + 1] * path.velocity[path_cnt + i + 1]) / (2 * (float)accel));

						if (accel_distance > (SECTION_SLANT * (float)(i + 1) * 0.5)) {
							path.velocity[path_cnt] = (short)sqrt((2 * (float)accel * SECTION_SLANT * 0.5 * (i + 1)) + path.velocity[path_cnt + i + 1] * path.velocity[path_cnt + i + 1]);
							path.decel_length[path_cnt] = (short)((path.velocity[path_cnt] * path.velocity[path_cnt] - path.velocity[path_cnt + i + 1] * path.velocity[path_cnt + i + 1]) / (2 * (float)accel));
						} else {
							path.decel_length[path_cnt] = (short)accel_distance;
						}

						path.section[path_cnt] = (unsigned char)(i + 1);
						path_cnt += i;
						i = 0;
						break;

					default:
						path.section[path_cnt] = 0;
						break;
				}
			}
		}

		Status::Calc::SetGyroReference();
		System::Timer::wait_ms(10);

		Status::Reset();
//		Mystat::Status::SetRunMode(false);
//		Mystat::Status::SetTravelDir(false);
		ExecuteFlag.SetValue(true);
		WallPFlag.SetValue(true);
		WallEdgeFlag.SetValue(false);
		PWM::Motor::Enable();

//		PWM::Motor::AdjustPosture_Angle();
		System::Timer::wait_ms(100);

		if (path.mode[0] >= 10) {
			TargetVelocity.SetValue(480.0);
			Accel.SetValue(false, 6500.0);
		} else {
			TargetVelocity.SetValue((float)velocity);
			Accel.SetValue(false, (float)accel);

			turnparam = turnparam_num;
			straight_first = true;
		}

		for (path_cnt = 0; path_cnt < path.length; path_cnt++) {
			if (path.mode[path_cnt] != 255) {
				if ((prev_mode <= 1) && (path.mode[path_cnt] >= 10)) {
//					Mystat::Status::ResetDegreeValue();
				}

				prev_mode = path.mode[path_cnt];

				switch (path.mode[path_cnt]) {
					case 0:
					case 1:
						if (!straight_first) {
							turnparam = turnparam_num;
							straight_first = true;
						}

						PWM::Motor::AccelRun(path.section[path_cnt], false, path.velocity[path_cnt], path.velocity_next[path_cnt], path.decel_length[path_cnt], (float)accel);
						break;

					case 10:	// 大回り90
						PWM::Motor::Slalom(1 + (7 * turnparam), -path.turn_dir[path_cnt]);
						break;

					case 11:	// 大回り180
						PWM::Motor::Slalom(2 + (7 * turnparam), -path.turn_dir[path_cnt]);
						break;

					case 20:	// 45度In
						PWM::Motor::Slalom(3 + (7 * turnparam), -path.turn_dir[path_cnt]);
						break;

					case 21:	// Out
						PWM::Motor::Slalom(4 + (7 * turnparam), -path.turn_dir[path_cnt]);
						break;

					case 30:	// 90度
						PWM::Motor::Slalom(7 + (7 * turnparam), -path.turn_dir[path_cnt]);
						break;

					case 40:	// 135度In
						PWM::Motor::Slalom(5 + (7 * turnparam), -path.turn_dir[path_cnt]);
						break;

					case 41:	// Out
						PWM::Motor::Slalom(6 + (7 * turnparam), -path.turn_dir[path_cnt]);
						break;

					case 100:	// 斜め中直進
						PWM::Motor::AccelRun(path.section[path_cnt], true, path.velocity[path_cnt], path.velocity_next[path_cnt], path.decel_length[path_cnt], (float)accel);

						if (!straight_first) {
							turnparam = turnparam_num;
							straight_first = true;
						}
						break;

					default:
						break;
				}
			}

			if (!ExecuteFlag.GetValue()) {
				failed = true;
				break;
			}
		}

//		PWM::Fan::Disable();
		System::Timer::wait_ms(1000);
		PWM::Motor::Disable();
		ExecuteFlag.SetValue(false);

		if (failed) {
			PWM::Buzzer::Melody_ProtectionRadio();
		} else {
			System::Interface::SetLEDColor(0, 0, 255, 0);
			PWM::Buzzer::Melody_GK1_1();
		}
	}

	void Map::Search_Adachi(unsigned char dest_x, unsigned char dest_y, bool f_method, bool slalom, bool shortcut) {
		bool wall_r = false, wall_l = false, wall_f = false, fast = false, next_known = false, expiration = false;
		bool blockinfo_checked_forward, blockinfo_checked_left, blockinfo_checked_right, flag_goal = false, use_f_method = f_method;
		signed char next_dir = 0;
		unsigned char steps_now = 0, times = 0, now_x = 0, now_y = 0;
		unsigned char blockinfo_step_forward, blockinfo_step_left, blockinfo_step_right;

		Status::Calc::SetGyroReference();

		Status::Reset();
		ExecuteFlag.SetValue(true);
		PWM::Motor::Enable();

		PWM::Motor::AccelDecel(SEARCH_SPEED, 3000.0, true);
		Mystat::Position::UpDate(SIDE_FORWARD);

		while(!Status::CheckGoalArrival(dest_x, dest_y) && ExecuteFlag.GetValue()) {
			Mystat::Map::CalcStep(dest_x, dest_y, false);

			now_x = Mystat::Position::Get(Mystat::Position::X);
			now_y = Mystat::Position::Get(Mystat::Position::Y);
			steps_now = Mystat::Map::GetBlockStep(now_x, now_y);
			next_dir = SIDE_REAR;

			if (steps_now == 255) {
				System::Interface::SetLEDColor(0, 255, 0, 0);
				ExecuteFlag.SetValue(false);
				break;
			}

			blockinfo_checked_forward = Mystat::Map::GetBlockChecked(now_x, now_y, SIDE_FORWARD);
			blockinfo_checked_left = Mystat::Map::GetBlockChecked(now_x, now_y, SIDE_LEFT);
			blockinfo_checked_right = Mystat::Map::GetBlockChecked(now_x, now_y, SIDE_RIGHT);

			blockinfo_step_forward = Mystat::Map::GetBlockStep(now_x, now_y, SIDE_FORWARD);
			blockinfo_step_left = Mystat::Map::GetBlockStep(now_x, now_y, SIDE_LEFT);
			blockinfo_step_right = Mystat::Map::GetBlockStep(now_x, now_y, SIDE_RIGHT);

			next_known = false;

//		  if (!shortcut) {
				if (use_f_method) {
					// 未知区間直進優先
					if (!Status::Sensor::CheckWallExist(SIDE_FORWARD) && !blockinfo_checked_forward) {
						if ((GetBlockWallNum(now_x, now_y, SIDE_FORWARD) < 3) && (steps_now != 0)) {
							steps_now = 0;
							next_dir = SIDE_FORWARD;
							next_known = false;
						}
					}

					if (!Status::Sensor::CheckWallExist(SIDE_RIGHT) && !blockinfo_checked_right) {
						if ((GetBlockWallNum(now_x, now_y, SIDE_RIGHT) < 3) && (steps_now != 0)) {
							steps_now = 0;
							next_dir = SIDE_RIGHT;
							next_known = false;
						}
					}

					if (!Status::Sensor::CheckWallExist(SIDE_LEFT) && !blockinfo_checked_left) {
						if ((GetBlockWallNum(now_x, now_y, SIDE_LEFT) < 3) && (steps_now != 0)) {
							steps_now = 0;
							next_dir = SIDE_LEFT;
							next_known = false;
						}
					}
				}

				// 未知区間歩数基準
				if (!Status::Sensor::CheckWallExist(SIDE_FORWARD) && !blockinfo_checked_forward) {
					if (blockinfo_step_forward < steps_now) {
						steps_now = blockinfo_step_forward;
						next_dir = SIDE_FORWARD;
						next_known = false;
					}
				}

				if (!Status::Sensor::CheckWallExist(SIDE_RIGHT) && !blockinfo_checked_right) {
					if (blockinfo_step_right < steps_now) {
						steps_now = blockinfo_step_right;
						next_dir = SIDE_RIGHT;
						next_known = false;
					}
				}

				if (!Status::Sensor::CheckWallExist(SIDE_LEFT) && !blockinfo_checked_left) {
					if (blockinfo_step_left < steps_now) {
						steps_now = blockinfo_step_left;
						next_dir = SIDE_LEFT;
						next_known = false;
					}
				}
//		}

			// 既知区間歩数基準
			if (!Status::Sensor::CheckWallExist(SIDE_FORWARD) && blockinfo_checked_forward) {
				if (blockinfo_step_forward < steps_now) {
					steps_now = blockinfo_step_forward;
					next_dir = SIDE_FORWARD;
					next_known = true;
				}
			}

			if (!Status::Sensor::CheckWallExist(SIDE_RIGHT) && blockinfo_checked_right) {
				if (blockinfo_step_right < steps_now) {
					steps_now = blockinfo_step_right;
					next_dir = SIDE_RIGHT;
					next_known = true;
				}
			}

			if (!Status::Sensor::CheckWallExist(SIDE_LEFT) && blockinfo_checked_left) {
				if (blockinfo_step_left < steps_now) {
					steps_now = blockinfo_step_left;
					next_dir = SIDE_LEFT;
					next_known = true;
				}
			}

			switch (next_dir) {
				case SIDE_FORWARD:
				  if (shortcut) {
					  steps_now = Mystat::Map::GetBlockStep(now_x, now_y, SIDE_FORWARD);

					  blockinfo_checked_forward = Mystat::Map::GetBlockChecked(now_x, now_y, SIDE_F_FORWARD);
					  blockinfo_checked_left = Mystat::Map::GetBlockChecked(now_x, now_y, SIDE_F_LEFT);
					  blockinfo_checked_right = Mystat::Map::GetBlockChecked(now_x, now_y, SIDE_F_RIGHT);

					  blockinfo_step_forward = Mystat::Map::GetBlockStep(now_x, now_y, SIDE_F_FORWARD);
					  blockinfo_step_left = Mystat::Map::GetBlockStep(now_x, now_y, SIDE_F_LEFT);
					  blockinfo_step_right = Mystat::Map::GetBlockStep(now_x, now_y, SIDE_F_RIGHT);

					  fast = false;

					  if (use_f_method) {
						  if (!Map::CheckAheadWallExist(now_x, now_y, SIDE_FORWARD) && !blockinfo_checked_forward) {
							  if ((GetBlockWallNum(now_x, now_y, SIDE_F_FORWARD) < 3) && (steps_now != 0)) {
								  steps_now = 0;
								  fast = false;
							  }
						  }

						  if (!Map::CheckAheadWallExist(now_x, now_y, SIDE_RIGHT) && !blockinfo_checked_right) {
							  if ((GetBlockWallNum(now_x, now_y, SIDE_F_RIGHT) < 3) && (steps_now != 0)) {
								  steps_now = 0;
								  fast = false;
							  }
						  }

						  if (!Map::CheckAheadWallExist(now_x, now_y, SIDE_LEFT) && !blockinfo_checked_left) {
							  if ((GetBlockWallNum(now_x, now_y, SIDE_F_LEFT) < 3) && (steps_now != 0)) {
								  steps_now = 0;
								  fast = false;
							  }
						  }
					  }

					  // 未知区間歩数基準
					  if (!Map::CheckAheadWallExist(now_x, now_y, SIDE_FORWARD) && !blockinfo_checked_forward) {
						  if (blockinfo_step_forward < steps_now) {
							  steps_now = blockinfo_step_forward;
							  fast = false;
						  }
					  }

					  if (!Map::CheckAheadWallExist(now_x, now_y, SIDE_RIGHT) && !blockinfo_checked_right) {
						  if (blockinfo_step_right < steps_now) {
							  steps_now = blockinfo_step_right;
							  fast = false;
						  }
					  }

					  if (!Map::CheckAheadWallExist(now_x, now_y, SIDE_LEFT) && !blockinfo_checked_left) {
						  if (blockinfo_step_left < steps_now) {
							  steps_now = blockinfo_step_left;
							  fast = false;
						  }
					  }

					  // 既知区間歩数基準
					  if (!Map::CheckAheadWallExist(now_x, now_y, SIDE_FORWARD) && blockinfo_checked_forward) {
						  if (blockinfo_step_forward < steps_now) {
							  steps_now = blockinfo_step_forward;
							  fast = true;
						  }
					  }

					  if (!Map::CheckAheadWallExist(now_x, now_y, SIDE_RIGHT) && blockinfo_checked_right) {
						  if (blockinfo_step_right < steps_now) {
							  steps_now = blockinfo_step_right;
							  fast = false;
						  }
					  }

					  if (!Map::CheckAheadWallExist(now_x, now_y, SIDE_LEFT) && blockinfo_checked_left) {
						  if (blockinfo_step_left < steps_now) {
							  steps_now = blockinfo_step_left;
							  fast = false;
						  }
					  }

					  if (fast && (Velocity.GetValue(false) < 250.0)) {
						  PWM::Motor::AccelDecel(SEARCH_SPEED * 2.0, 4000.0, false);
					  } else if (!fast && (Velocity.GetValue(false) > 250.0)) {
						  PWM::Motor::AccelDecel(SEARCH_SPEED, -4000.0, false);
					  } else if (!fast && next_known) {
						  Accel.SetValue(false, 4000.0);
						  TargetVelocity.SetValue(SEARCH_SPEED * 1.5);
						  PWM::Motor::Run(0, true);

						  Accel.SetValue(false, -4000.0);
						  TargetVelocity.SetValue(SEARCH_SPEED);
						  PWM::Motor::Run(0, true);
					  } else {
						  PWM::Motor::Run(1, false);
					  }
				  } else {
					  PWM::Motor::Run(1, false);
				  }

				  Mystat::Position::UpDate(SIDE_FORWARD);
				  break;

				case SIDE_LEFT:
					if (slalom) {
						PWM::Motor::Slalom(SLALOM_LEFT);
					} else {
						PWM::Motor::AccelDecel(0.0, -2000.0, true);

						System::Timer::wait_ms(50);
						PWM::Motor::Turning(false, SLALOM_LEFT);
						System::Timer::wait_ms(50);

						PWM::Motor::AccelDecel(SEARCH_SPEED, 4000.0, true);
					}

					Mystat::Position::UpDate(SIDE_LEFT);
					break;

				case SIDE_RIGHT:
					if (slalom) {
						PWM::Motor::Slalom(SLALOM_RIGHT);
					} else {
						PWM::Motor::AccelDecel(0.0, -2000.0, true);

						System::Timer::wait_ms(50);
						PWM::Motor::Turning(false, SLALOM_RIGHT);
						System::Timer::wait_ms(50);

						PWM::Motor::AccelDecel(SEARCH_SPEED, 4000.0, true);
					}

					Mystat::Position::UpDate(SIDE_RIGHT);
					break;

				case SIDE_REAR:
					PWM::Motor::AccelDecel(0.0, -2000.0, true);
					System::Timer::wait_ms(50);
				
					if (Status::Sensor::GetValue(Status::Sensor::F, false) > SENSOR_WALL_EXIST_F_NEAR) {
						PWM::Motor::Turning(true, 0);
						System::Timer::wait_ms(50);
						PWM::Motor::BackAtBlindAllay();
						System::Timer::wait_ms(50);
					} else {
						PWM::Motor::Turning(true, 0);
						System::Timer::wait_ms(50);
					}

					PWM::Motor::AccelDecel(SEARCH_SPEED, 4000.0, false);
					Mystat::Position::UpDate(SIDE_REAR);
					break;
			}

//		  if (Interrupt::CMTimer::GetTimerElapsed() > ACT_TIMER) {
//			  if ((dest_x == GOAL_X) && (dest_y == GOAL_Y)) {
//				  if (use_f_method) {
//					  PWM::Buzzer::Enable();
//					  PWM::Buzzer::SetDuty(2000.0);
//
//					  use_f_method = false;
//				  } else if (MTU2.TGRC != MTU2.TGRD) {
//					  PWM::Buzzer::Disable();
//				  }
//			  } else {
//				  expiration = true;
//				  ExecuteFlag.SetValue(false);
//			  }
//		  }
		}

		if (ExecuteFlag.GetValue()) {
			wall_r = Status::Sensor::CheckWallExist(SIDE_RIGHT);
			wall_l = Status::Sensor::CheckWallExist(SIDE_LEFT);
			wall_f  = Status::Sensor::CheckWallExist(SIDE_FORWARD);

			PWM::Motor::AccelDecel(0.0, -14000.0, true);
			System::Timer::wait_ms(10);
			Status::Reset();
			PWM::Motor::Disable();

//		  (void)System::Flash::EraseBlock(BLOCK_DB0);

//		  if (Flash::WriteWallData()) {
//			  Interface::LED::SetColor(Interface::LED::Green, Interface::LED::Both);
//		  } else {
//			  Interface::LED::SetColor(Interface::LED::Red, Interface::LED::Both);
//		  }

			if ((dest_x == GOAL_X) && (dest_y == GOAL_Y)) {
				PWM::Buzzer::Melody_TX1();
			} else if ((dest_x == START_X) && (dest_y == START_Y)) {
				PWM::Buzzer::Melody_TX3();
			}

			System::Timer::wait_ms(100);

			if (!wall_f) {
				// なにもしない
			} else if (!wall_r) {
				PWM::Motor::Turning(false, SLALOM_RIGHT);
				Mystat::Position::UpDate(false, SIDE_RIGHT);
			} else if (!wall_l) {
				PWM::Motor::Turning(false, SLALOM_LEFT);
				Mystat::Position::UpDate(false, SIDE_LEFT);
			} else {
				PWM::Motor::Turning(true, 0);
				Mystat::Position::UpDate(false, SIDE_REAR);
			}

			Status::Reset();
			PWM::Motor::Disable();

			(void)System::Flash::EraseBlock(BLOCK_DB0);

			if (System::Flash::WriteWallData()) {
				System::Interface::SetLEDColor(0, 0, 255, 0);
			} else {
				System::Interface::SetLEDColor(0, 255, 0, 0);
			}

			System::Timer::wait_ms(1000);
			System::Interface::SetLEDColor(0, 0, 0, 0);
		} else {
			Status::Reset();
			PWM::Motor::Disable();

//		  if (expiration) {
//			  Interface::LED::SetColor(Interface::LED::Purple, Interface::LED::Both);
//			  PWM::Buzzer::Melody_Oedo_Door();
//		  }
		}
	}
}
