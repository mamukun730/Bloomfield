/*
 * status.hpp
 *
 *  Created on: 2017/09/16
 *      Author: mmkn
 */

#ifndef INCLUDE_STATUS_HPP_
#define INCLUDE_STATUS_HPP_

namespace Status {
    void Reset();
    bool CheckGoalArrival(unsigned char dest_x, unsigned char dest_y);
    void CheckMachineVelocity();
    void DetectWallEdge();
    bool DetectWallEdge_Slant(signed char side, bool reset);

    class Calc {
        public:
            static float GetVelocity();
            static float GyroOutToA_Velocity();
            static float GyroOutToA_Velocity_Y();
            static float ZAccelOutToMetric();
            static void SetGyroReference();
            static void RenewActualVelocity(bool reset);
            static void RenewTargetVelocity(bool reset);
            static void RenewVelocityDiff(bool reset);
            static void RenewAccelTarget(bool reset);
            static void RenewDegree(bool reset);
            static void RenewDistance(bool reset);
//          static std::array<uint16_t, 2> WallControlQuantity();
            static int16_t WallControlQuantity();
    };

    class Sensor {
        public:
            enum Sensor_ID {
                LS, LC, LF, F, RF, RC, RS
            };

            static void SetValue(uint16_t on[SENSOR_AMOUNT], uint16_t off[SENSOR_AMOUNT]);
            static uint16_t GetValue(uint8_t id, bool past);
            static int16_t GetDiff(uint8_t id);
            static bool CheckWallExist(int8_t dir);

        private:
            static uint16_t last_value[SENSOR_AMOUNT], past_value[SENSOR_AMOUNT];
            static int16_t diff[SENSOR_AMOUNT];
    };

    class Value {
        public:
            Value();
            ~Value();

            float GetValue(bool actual);
            void SetValue(bool actual, float value);

        private:
            float target_val, actual_val;
    };

    class Value2 {
        public:
            Value2();
            ~Value2();

            float GetValue();
            void SetValue(float value);

        private:
            float val;
    };

    class Flag {
        public:
            Flag();
            ~Flag();

            bool GetValue();
            void SetValue(bool execute);

        private:
            bool flag;
    };
}

namespace Mystat {
    struct _dijkstra {
        unsigned long cost[MAPSIZE_X][MAPSIZE_Y * 2];
        unsigned char flag[MAPSIZE_X][MAPSIZE_Y * 2];
        unsigned char next_x[MAPSIZE_X][MAPSIZE_Y * 2];
        unsigned char next_y[MAPSIZE_X][MAPSIZE_Y * 2];
        unsigned long cost_min;
    };

    struct _path {
        signed char dir[255];
        signed char turn_dir[255];
        unsigned char mode[255];
        unsigned char section[255];
        unsigned short velocity[255];
        unsigned short velocity_next[255];
        unsigned short decel_length[255];
        unsigned int length;
        unsigned char cost_straight;
        unsigned char cost_slant;
    };

    class Position {
        private:
            static unsigned char x;
            static unsigned char y;
            static signed char dir;

        public:
            enum type {
                X, Y, Dir
            };

            static signed char Get(unsigned char type);
            static void Reset();
            static void UpDate(signed char traveling_dir);
            static void UpDate(bool move_section, signed char traveling_dir);
    };

    class Map {
        private:
            static unsigned char data[MAPSIZE_X][MAPSIZE_Y];
            static unsigned char step[MAPSIZE_X][MAPSIZE_Y];
            static bool checked[MAPSIZE_X][MAPSIZE_Y];
            static struct _dijkstra dijkstra;
            static struct _path path;
            static float velocity;
            static float turn_velocity;
            static float accel;

        public:
            static void Init();

            static void CalcStep(unsigned char dest_x, unsigned char dest_y, bool enable_neglect, bool searched_element);
			static float CalcAccelStep(bool slant, unsigned char partition);

            static bool CheckAheadWallExist(unsigned char x, unsigned char y, signed char side);

            static void SetWallData(bool left, bool front, bool right);
            static void SendData();

            static void WriteWallData(unsigned char x, unsigned char y, unsigned char walldata);
            static unsigned char ReadWallData(unsigned char x, unsigned char y);

            static bool GetBlockChecked(unsigned char x, unsigned char y, signed char side);
            static unsigned char GetBlockStep(unsigned char x, unsigned char y, signed char side);
            static unsigned char GetBlockStep(unsigned char x, unsigned char y);
            static unsigned char GetBlockWallNum(unsigned char x, unsigned char y, signed char side);
            static unsigned char GetWallData(unsigned char x, unsigned char y);

            static void Search_Adachi(unsigned char dest_x, unsigned char dest_y, bool f_method, bool slalom, bool shortcut);

			static void CheckDijikstra_Vertical(unsigned char check_x, unsigned char check_y);
			static void CheckDijikstra_Horizontal(unsigned char check_x, unsigned char check_y);
			static void Dijkstra(void);
			static void MakePath(unsigned int velocity, unsigned int turn_velocity, unsigned int slant_velocity, unsigned int accel, unsigned int slant_accel);
			static void ReadPath(unsigned int velocity, unsigned int turn_velocity, unsigned int slant_velocity, unsigned int accel, unsigned int slant_accel);
    };
}

#endif /* INCLUDE_STATUS_HPP_ */
