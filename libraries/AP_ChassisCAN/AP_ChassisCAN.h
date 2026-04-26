#pragma once

#include <AP_ChassisCAN/AP_ChassisCAN_config.h>

#if AP_CHASSISCAN_ENABLED

#include <AP_CANManager/AP_CANDriver.h>
#include <AP_Param/AP_Param.h>

class AP_ChassisCAN : public AP_CANDriver {
public:
    AP_ChassisCAN();
    CLASS_NO_COPY(AP_ChassisCAN);

    static const struct AP_Param::GroupInfo var_info[];
    static AP_ChassisCAN *get_singleton();

    bool add_interface(AP_HAL::CANIface* can_iface) override;
    void init(uint8_t driver_index) override;

    // called from SRV_Channels to refresh actuator setpoints
    void update();

    bool pre_arm_check(char* reason, uint8_t reason_len) const;
    bool failsafe_triggered() const;
    bool manual_override_active() const;

private:
    void loop();
    bool write_frame(AP_HAL::CANFrame &frame, uint32_t timeout_us);
    bool read_frame(AP_HAL::CANFrame &frame, uint32_t timeout_us);

    void send_eps_command(uint32_t now_ms);
    void send_ehb_command(uint32_t now_ms);
    void handle_rx_frame(const AP_HAL::CANFrame &frame, uint32_t now_ms);

    struct {
        bool dbw_mode;
        bool fault_flag;
        bool center_uncalibrated;
        uint8_t fault_level;
        uint8_t fault_code;
        int16_t angle_deg;
        int16_t rate_dps;
        int8_t motor_current_a;
        int8_t temperature_c;
        uint32_t last_rx_ms;
    } _eps;

    struct {
        uint8_t pressure_req_raw;
        uint8_t pressure_fb_raw;
        uint8_t work_status;
        bool brake_light;
        bool driver_intervention;
        bool alarm_light;
        bool pedal_depressed;
        bool pedal_effective;
        uint16_t fault_bits;
        uint8_t angle_feedback;
        uint8_t vital_signs;
        uint32_t last_rx_ms;
    } _ehb;

    bool _initialized;
    uint8_t _driver_index;
    AP_HAL::CANIface *_can_iface;
    HAL_BinarySemaphore _sem;

    float _target_steering_deg;
    float _target_brake_mpa;

    uint32_t _last_eps_tx_ms;
    uint32_t _last_ehb_tx_ms;

    // parameters
    AP_Int16 _eps_ang_max_deg;
    AP_Int16 _eps_rate_dps;
    AP_Int8 _eps_fs_level;

    AP_Int8 _ehb_enable;
    AP_Int8 _ehb_src;
    AP_Float _ehb_max_mpa;
    AP_Int8 _ehb_selflearn;
    AP_Int8 _ehb_fs_on_fault;

    static AP_ChassisCAN *_singleton;
};

namespace AP {
    AP_ChassisCAN *chassis_can();
}

#endif // AP_CHASSISCAN_ENABLED
