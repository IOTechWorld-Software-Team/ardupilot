#pragma once
#include <inttypes.h>
#include <AP_BattMonitor/AP_BattMonitor.h>

#define CALIBRATION_COMPLETE_CODE       31
#define TANK_FAILSAFE                   32
#define PUMP_NOT_CALIBRATED             33


class AP_UAVCAN_TF {
public:
    AP_UAVCAN_TF();

    void send_pump_status(uint8_t run, float calibration_value = 0.0f, float extra1 = 0.0f, float extra2 = 0.0f);

    bool check_calibration_status(float &cal_value);

    bool check_tank_failsafe();

    float current_flow();

    float current_water();

    uint32_t last_time = 0;

    bool update_time = false;
    bool wat_lvl = true;
    float flow;

private:


    float pow_oc = 0;
    uint32_t level_time = 0;
    float pow_o = 0;
    float water_Level;
    int w_count = 0;
    bool noidea = true;
    float left_pani = 0;
    float flow_milli_litres_pass = 0;
};