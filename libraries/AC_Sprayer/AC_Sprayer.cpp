#include "AC_Sprayer_config.h"

#if HAL_SPRAYER_ENABLED

#include "AC_Sprayer.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// ------------------------------

const AP_Param::GroupInfo AC_Sprayer::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Sprayer enable/disable
    // @Description: Allows you to enable (1) or disable (0) the sprayer
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 0, AC_Sprayer, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PUMP_RATE
    // @DisplayName: Pump speed
    // @Description: Desired pump speed when travelling 1m/s expressed as a percentage
    // @Units: %
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("PUMP_RATE",   1, AC_Sprayer, _pump_pct_1ms, AC_SPRAYER_DEFAULT_PUMP_RATE),

    // @Param: SPINNER
    // @DisplayName: Spinner rotation speed
    // @Description: Spinner's rotation speed in PWM (a higher rate will disperse the spray over a wider area horizontally)
    // @Units: ms
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("SPINNER",     2, AC_Sprayer, _spinner_pwm, AC_SPRAYER_DEFAULT_SPINNER_PWM),

    // @Param: SPEED_MIN
    // @DisplayName: Speed minimum
    // @Description: Speed minimum at which we will begin spraying
    // @Units: cm/s
    // @Range: 0 1000
    // @User: Standard
    AP_GROUPINFO("SPEED_MIN",   3, AC_Sprayer, _speed_min, AC_SPRAYER_DEFAULT_SPEED_MIN),

    // @Param: PUMP_MIN
    // @DisplayName: Pump speed minimum
    // @Description: Minimum pump speed expressed as a percentage
    // @Units: %
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("PUMP_MIN",   4, AC_Sprayer, _pump_min_pct, AC_SPRAYER_DEFAULT_PUMP_MIN),

     // @Param: TFS_EN
    // @DisplayName: Tank Failsafe Enable
    // @Description: Enables(1)/Disables(0) Tank Failsafe
    // @Units: Boolean
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("TFSEN",   6, AC_Sprayer, _en_tfs , AC_SPRAYER_DEFAULT_TFS_EN),

    // @Param: SEN_TY
    // @DisplayName: Sensor type for Tank Failsafe
    // @Description: Sensor type for Tank Failsafe
    // @Values: 0:Capacitive,1:FlowMeter
    // @User: Standard
    AP_GROUPINFO("SENS_TYPE",   8, AC_Sprayer, _sensor_type , 0),

    // @Param: RUN_MODE
    // @DisplayName: Define the operation of the pump
    // @Description: Define the operation of the pump
    // @Values: 2:RUN CALIBRATION
    // @User: Standard
    AP_GROUPINFO("RUN_MODE", 9, AC_Sprayer, _run_mode, 1),

    // @Param: CAL_VALUE
    // @DisplayName: Calirated value of the pump
    // @Description: Calirated value of the pump
    // @User: Standard
    AP_GROUPINFO("CAL_VALUE", 10, AC_Sprayer, _cal_value, 0.0f),

    AP_GROUPEND
};

AC_Sprayer::AC_Sprayer()
{
    if (_singleton) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many sprayers");
#endif
        return;
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);

    // check for silly parameter values
    if (_pump_pct_1ms < 0.0f || _pump_pct_1ms > 100.0f) {
        _pump_pct_1ms.set_and_save(AC_SPRAYER_DEFAULT_PUMP_RATE);
    }
    if (_spinner_pwm < 0) {
        _spinner_pwm.set_and_save(AC_SPRAYER_DEFAULT_SPINNER_PWM);
    }

    // To-Do: ensure that the pump and spinner servo channels are enabled
}

/*
 * Get the AP_Sprayer singleton
 */
AC_Sprayer *AC_Sprayer::_singleton;
AC_Sprayer *AC_Sprayer::get_singleton()
{
    return _singleton;
}

void AC_Sprayer::run(const bool activate)
{
    // return immediately if no change
    if (_flags.running == activate) {
        return;
    }

    // set flag indicate whether spraying is permitted:
    // do not allow running to be set to true if we are currently not enabled
    _flags.running = _enabled && activate;

    // turn off the pump and spinner servos if necessary
    if (!_flags.running) {
        stop_spraying();
    }
}

void AC_Sprayer::stop_spraying()
{
    SRV_Channels::set_output_limit(SRV_Channel::k_sprayer_pump, SRV_Channel::Limit::MIN);
    SRV_Channels::set_output_limit(SRV_Channel::k_sprayer_spinner, SRV_Channel::Limit::MIN);

    _flags.spraying = false;
}

/// update - adjust pwm of servo controlling pump speed according to the desired quantity and our horizontal speed
void AC_Sprayer::update()
{
    // exit immediately if we are disabled or shouldn't be running
    if (!_enabled || !running()) {
        run(false);
        return;
    }

    // exit immediately if the pump function has not been set-up for any servo
    if (!SRV_Channels::function_assigned(SRV_Channel::k_sprayer_pump)) {
        return;
    }

    // get horizontal velocity
    Vector3f velocity;
    if (!AP::ahrs().get_velocity_NED(velocity)) {
        // treat unknown velocity as zero which should lead to pump stopping
        // velocity will already be zero but this avoids a coverity warning
        velocity.zero();
    }

    float ground_speed = velocity.xy().length() * 100.0;

    // get the current time
    const uint32_t now = AP_HAL::millis();

    bool should_be_spraying = _flags.spraying;
    // check our speed vs the minimum
    if (ground_speed >= _speed_min) {
        // if we are not already spraying
        if (!_flags.spraying) {
            // set the timer if this is the first time we've surpassed the min speed
            if (_speed_over_min_time == 0) {
                _speed_over_min_time = now;
            }else{
                // check if we've been over the speed long enough to engage the sprayer
                if((now - _speed_over_min_time) > AC_SPRAYER_DEFAULT_TURN_ON_DELAY) {
                    should_be_spraying = true;
                    _speed_over_min_time = 0;
                }
            }
        }
        // reset the speed under timer
        _speed_under_min_time = 0;
    } else {
        // we are under the min speed.
        if (_flags.spraying) {
            // set the timer if this is the first time we've dropped below the min speed
            if (_speed_under_min_time == 0) {
                _speed_under_min_time = now;
            }else{
                // check if we've been over the speed long enough to engage the sprayer
                if((now - _speed_under_min_time) > AC_SPRAYER_DEFAULT_SHUT_OFF_DELAY) {
                    should_be_spraying = false;
                    _speed_under_min_time = 0;
                }
            }
        }
        // reset the speed over timer
        _speed_over_min_time = 0;
    }

    // if testing pump output speed as if travelling at 1m/s
    if (_flags.testing) {
        ground_speed = 100.0f;
        should_be_spraying = true;
    }

    // if spraying or testing update the pump servo position
    if (should_be_spraying) {
        float pos = ground_speed * _pump_pct_1ms;
        pos = MAX(pos, 100 *_pump_min_pct); // ensure min pump speed
        pos = MIN(pos,10000); // clamp to range
        SRV_Channels::move_servo(SRV_Channel::k_sprayer_pump, pos, 0, 10000);
        SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_spinner, _spinner_pwm);
        _flags.spraying = true;
    } else {
        stop_spraying();
    }
}

void AC_Sprayer::set_master(bool true_false){
    _master = true_false;
    if(true_false==true){
        run(true);
    }
    if(_enabled){
        if(true_false){
            gcs().send_text(MAV_SEVERITY_WARNING,"Sprayer Master Switch On");
        }
        else{
            gcs().send_text(MAV_SEVERITY_WARNING,"Sprayer Master Switch Off");
        }
    }
}

bool AC_Sprayer::en_tfs_status(void)
{
    if (_en_tfs >= 1) {
        return true;
    } else {
        return false;
    }
}

bool AC_Sprayer::failsafe_check_pump()
{
#if HAL_ENABLE_DRONECAN_DRIVERS
    // check for tank failsafe
    // sprayer should be on for this
    if (pump_found) {
        _sensor_type.set_and_save_ifchanged(2);
    }
    
    if(first_time){
        if ((_run_mode == 1) && is_zero(_cal_value)) {
            tf_uavcan.send_pump_status(0);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Pump calibration needed");
            first_time = false;
            return true;
            }
    }
    // check if calibration is get called by the user
    if (_run_mode == 2) {
        // calibration need to run so let's send the packet for the calibration
        tf_uavcan.send_pump_status(2);
        // check for the calibration is completed or not
        float cal_value;
        if (tf_uavcan.check_calibration_status(cal_value)) {
            // calibration is done, let's reset the run mode, save the cal value and notify the user that pump is calibrated
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Pump is calibrated!! Reboot the Drone");
            _run_mode.set_and_save(1);
            _cal_value.set_and_save(cal_value);
            return true;

        }
    } else {
        // in any case if run_mode is not 2 it's mean normal operation from the tank side
        tf_uavcan.send_pump_status(1, _cal_value);
        // check for failsafe of the tank
        if (tf_uavcan.check_tank_failsafe()) {
            return true;
        }

    }
#endif
    return false;
    // treat this as the pump is running // send the calibrated value
    // _flags.spraying ;
}

namespace AP {

AC_Sprayer *sprayer()
{
    return AC_Sprayer::get_singleton();
}

};
#endif // HAL_SPRAYER_ENABLED
