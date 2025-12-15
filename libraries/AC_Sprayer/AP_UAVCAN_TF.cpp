#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>

#if HAL_ENABLE_DRONECAN_DRIVERS

#include "AP_UAVCAN_TF.h"
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AC_Sprayer/AC_Sprayer.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

AP_UAVCAN_TF::AP_UAVCAN_TF() {
    // init the tank failsafe
}

void AP_UAVCAN_TF::send_pump_status(uint8_t run, float calibration_value, float extra1, float extra2) {
    uint8_t can_num_drivers = AP::can().get_num_drivers();

    for (uint8_t i = 0; i < can_num_drivers; i++) {
        AP_DroneCAN *uavcan = AP_DroneCAN::get_dronecan(i);
        if (uavcan != nullptr) {
            uavcan->pump_vesc_send(run,calibration_value,extra1,extra2);
        }
    }
}

bool AP_UAVCAN_TF::check_calibration_status(float &cal_value)
{
    AP_ESC_Telem *esc_telem = AP_ESC_Telem::get_singleton();

    if (esc_telem == nullptr)
    {
        return false;
    }
    AP_ESC_Telem::ESC_Status esc_copy = esc_telem->esc_get_pump_data(PUMP_INDEX);

    if (esc_copy.error_count == CALIBRATION_COMPLETE_CODE) {
        // calibration is completed let's take the cal value
        cal_value = esc_copy.temperature;
        return true;
    }
    return false;
}

bool AP_UAVCAN_TF::check_tank_failsafe()
{
    const uint32_t now = AP_HAL::millis();
    // get the pump data
    // check for the tank failsafe
    AP_ESC_Telem *esc_telem = AP_ESC_Telem::get_singleton();

    if (esc_telem == nullptr)
    {
        return false;
    }

    AP_ESC_Telem::ESC_Status esc_copy = esc_telem->esc_get_pump_data(PUMP_INDEX);

    if (esc_copy.error_count == TANK_FAILSAFE) {
        flow = 0;
        return true;
    }

    float rpm_ = esc_telem->esc_arr[PUMP_INDEX].rpm;
                // get_rpm(PUMP_INDEX, rpm_);
    flow = ((0.000000142555f)*rpm_*rpm_*rpm_) + ((-0.001241f)*rpm_*rpm_) +(3.881346f)*rpm_ + (-1614.489895f);

    if(update_time) {
        uint32_t last_time_sec = (now - last_time);

        if(last_time_sec == 0 ) {
            last_time_sec = 303;
        }

        if (flow > 0.1) {
            flow_milli_litres_pass = ((flow / 60.0) * (last_time_sec / 1000.0));
        } else {
            flow_milli_litres_pass = 0;  // or handle as needed for zero flow
        }
    }
    update_time = true;
    last_time = now;

    return false;

}

float AP_UAVCAN_TF::current_flow()
{
    return flow/1000;
}

float AP_UAVCAN_TF:: current_water()
{
    return left_pani;
}
#endif