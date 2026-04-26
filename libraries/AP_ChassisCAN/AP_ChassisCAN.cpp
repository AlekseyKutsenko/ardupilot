#include "AP_ChassisCAN.h"

#if AP_CHASSISCAN_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CAN.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

#define CHCAN_EPS_TX_ID 0x314
#define CHCAN_EPS_RX_ID 0x18F
#define CHCAN_EHB_TX_ID 0x364
#define CHCAN_EHB_RX_ID 0x289

#define CHCAN_EPS_TX_PERIOD_MS 20U
#define CHCAN_EHB_TX_PERIOD_MS 20U
#define CHCAN_EPS_RX_TIMEOUT_MS 300U
#define CHCAN_EHB_RX_TIMEOUT_MS 150U

AP_ChassisCAN *AP_ChassisCAN::_singleton;

const AP_Param::GroupInfo AP_ChassisCAN::var_info[] = {
    // @Param: EPS_ANG
    // @DisplayName: EPS max rack angle
    // @Description: Maximum absolute steering angle command to EPS in degrees
    // @Range: 100 650
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("EPS_ANG", 1, AP_ChassisCAN, _eps_ang_max_deg, 650),

    // @Param: EPS_RATE
    // @DisplayName: EPS angular rate
    // @Description: Steering angular velocity command for EPS
    // @Range: 50 520
    // @Units: deg/s
    // @User: Advanced
    AP_GROUPINFO("EPS_RATE", 2, AP_ChassisCAN, _eps_rate_dps, 360),

    // @Param: EPS_FSLV
    // @DisplayName: EPS failsafe level
    // @Description: Minimum EPS fault level that triggers Rover failsafe
    // @Range: 1 3
    // @Values: 1:Minor,2:Severe,3:Critical
    // @User: Advanced
    AP_GROUPINFO("EPS_FSLV", 3, AP_ChassisCAN, _eps_fs_level, 2),

    // @Param: EHB_EN
    // @DisplayName: EHB enable
    // @Description: Enable EHB M201 command/telemetry on chassis CAN bus
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("EHB_EN", 4, AP_ChassisCAN, _ehb_enable, 0),

    // @Param: EHB_SRC
    // @DisplayName: EHB command source
    // @Description: Source for brake pressure command
    // @Values: 0:None,1:ThrottleNegative
    // @User: Advanced
    AP_GROUPINFO("EHB_SRC", 5, AP_ChassisCAN, _ehb_src, 1),

    // @Param: EHB_MAX
    // @DisplayName: EHB max pressure
    // @Description: Maximum brake pressure command sent to EHB
    // @Range: 0 10
    // @Units: MPa
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("EHB_MAX", 6, AP_ChassisCAN, _ehb_max_mpa, 10.0f),

    // @Param: EHB_LEARN
    // @DisplayName: EHB self learning request
    // @Description: Set to 1 to request EHB self-learning (one-shot command byte value 0x03)
    // @Values: 0:Disabled,1:Request
    // @User: Advanced
    AP_GROUPINFO("EHB_LEARN", 7, AP_ChassisCAN, _ehb_selflearn, 0),

    // @Param: EHB_FSFLT
    // @DisplayName: EHB failsafe on fault
    // @Description: Trigger Rover failsafe when EHB reports fault bits or failed state
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("EHB_FSFLT", 8, AP_ChassisCAN, _ehb_fs_on_fault, 1),

    AP_GROUPEND
};

AP_ChassisCAN::AP_ChassisCAN()
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_ChassisCAN must be singleton");
    }
    _singleton = this;
}

AP_ChassisCAN *AP_ChassisCAN::get_singleton()
{
    return _singleton;
}

bool AP_ChassisCAN::add_interface(AP_HAL::CANIface *can_iface)
{
    if (_can_iface != nullptr) {
        return false;
    }
    _can_iface = can_iface;
    if (_can_iface == nullptr || !_can_iface->is_initialized()) {
        _can_iface = nullptr;
        return false;
    }
    if (!_can_iface->set_event_handle(&_sem)) {
        _can_iface = nullptr;
        return false;
    }
    return true;
}

void AP_ChassisCAN::init(uint8_t driver_index)
{
    _driver_index = driver_index;
    if (_initialized || _can_iface == nullptr) {
        return;
    }
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ChassisCAN::loop, void),
                                      "chassis_can",
                                      3072,
                                      AP_HAL::Scheduler::PRIORITY_CAN,
                                      0)) {
        return;
    }
    _initialized = true;
}

bool AP_ChassisCAN::write_frame(AP_HAL::CANFrame &frame, uint32_t timeout_us)
{
    if (!_initialized || _can_iface == nullptr) {
        return false;
    }

    bool read_select = false;
    bool write_select = true;
    if (!_can_iface->select(read_select, write_select, &frame, AP_HAL::micros64() + timeout_us) || !write_select) {
        return false;
    }

    return _can_iface->send(frame, AP_HAL::micros64() + timeout_us, AP_HAL::CANIface::AbortOnError) == 1;
}

bool AP_ChassisCAN::read_frame(AP_HAL::CANFrame &frame, uint32_t timeout_us)
{
    if (!_initialized || _can_iface == nullptr) {
        return false;
    }

    bool read_select = true;
    bool write_select = false;
    if (!_can_iface->select(read_select, write_select, nullptr, AP_HAL::micros64() + timeout_us) || !read_select) {
        return false;
    }

    uint64_t ts_us;
    AP_HAL::CANIface::CanIOFlags flags{};
    return _can_iface->receive(frame, ts_us, flags) == 1;
}

void AP_ChassisCAN::update()
{
    if (!_initialized) {
        return;
    }

    const float steering_deg = SRV_Channels::get_output_scaled(SRV_Channel::k_steering);
    _target_steering_deg = constrain_float(steering_deg / 45.0f, -1.0f, 1.0f) * _eps_ang_max_deg;

    if (_ehb_enable != 0 && _ehb_src == 1) {
        const float throttle_pct = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
        const float brake_norm = constrain_float(-throttle_pct / 100.0f, 0.0f, 1.0f);
        _target_brake_mpa = brake_norm * _ehb_max_mpa;
    }
}

void AP_ChassisCAN::send_eps_command(uint32_t now_ms)
{
    if ((now_ms - _last_eps_tx_ms) < CHCAN_EPS_TX_PERIOD_MS) {
        return;
    }
    _last_eps_tx_ms = now_ms;

    uint8_t data[8] {};
    const bool request_dbw = hal.util->get_soft_armed();
    if (request_dbw && !_eps.fault_flag) {
        data[0] |= 0x01;
    }

    const int16_t angle_cmd = constrain_int16((int16_t)lrintf(_target_steering_deg), -650, 650);
    data[1] = uint8_t((uint16_t(angle_cmd) >> 8) & 0xFF);
    data[2] = uint8_t(uint16_t(angle_cmd) & 0xFF);

    uint16_t rate_dps = constrain_int16(_eps_rate_dps, 50, 520);
    data[3] = uint8_t((rate_dps >> 8) & 0xFF);
    data[4] = uint8_t(rate_dps & 0xFF);

    AP_HAL::CANFrame frame(CHCAN_EPS_TX_ID, data, 8, false);
    write_frame(frame, 2000);
}

void AP_ChassisCAN::send_ehb_command(uint32_t now_ms)
{
    if (_ehb_enable == 0 || (now_ms - _last_ehb_tx_ms) < CHCAN_EHB_TX_PERIOD_MS) {
        return;
    }
    _last_ehb_tx_ms = now_ms;

    uint8_t data[8] {};
    const float clamped_mpa = constrain_float(_target_brake_mpa, 0.0f, _ehb_max_mpa);
    data[0] = uint8_t(constrain_int16((int16_t)lrintf(clamped_mpa * 10.0f), 0, 100));
    if (_ehb_selflearn != 0) {
        data[7] = 0x03;
        _ehb_selflearn.set_and_save_ifchanged(0);
    }

    AP_HAL::CANFrame frame(CHCAN_EHB_TX_ID, data, 8, false);
    write_frame(frame, 2000);
}

void AP_ChassisCAN::handle_rx_frame(const AP_HAL::CANFrame &frame, uint32_t now_ms)
{
    const uint32_t sid = frame.id & AP_HAL::CANFrame::MaskStdID;

    if (sid == CHCAN_EPS_RX_ID && frame.dlc >= 8) {
        const uint8_t status = frame.data[0];
        _eps.dbw_mode = (status & 0x01U) != 0U;
        _eps.fault_flag = (status & 0x02U) != 0U;
        _eps.center_uncalibrated = (status & 0x04U) != 0U;
        _eps.angle_deg = int16_t((uint16_t(frame.data[1]) << 8) | frame.data[2]);
        _eps.rate_dps = int16_t((uint16_t(frame.data[3]) << 8) | frame.data[4]);
        _eps.motor_current_a = int8_t(frame.data[5]);
        _eps.temperature_c = int8_t(frame.data[6]);

        const uint8_t fault = frame.data[7];
        _eps.fault_code = (fault >> 4) & 0x0F;
        if ((fault & 0x01U) != 0U) {
            _eps.fault_level = 3;
        } else if ((fault & 0x02U) != 0U) {
            _eps.fault_level = 2;
        } else if ((fault & 0x04U) != 0U) {
            _eps.fault_level = 1;
        } else {
            _eps.fault_level = 0;
        }
        _eps.last_rx_ms = now_ms;
        return;
    }

    if (sid == CHCAN_EHB_RX_ID && frame.dlc >= 8) {
        _ehb.pressure_fb_raw = frame.data[0];
        _ehb.brake_light = (frame.data[1] & (1U << 2)) != 0U;
        _ehb.work_status = (frame.data[1] >> 4) & 0x0F;
        _ehb.driver_intervention = (frame.data[3] & (1U << 4)) != 0U;
        _ehb.alarm_light = (frame.data[3] & (1U << 5)) != 0U;
        _ehb.pedal_depressed = (frame.data[3] & (1U << 6)) != 0U;
        _ehb.pedal_effective = (frame.data[3] & (1U << 7)) != 0U;
        _ehb.fault_bits = uint16_t(frame.data[4]) | (uint16_t(frame.data[5]) << 8);
        _ehb.angle_feedback = frame.data[6];
        _ehb.vital_signs = frame.data[7] & 0x0F;
        _ehb.last_rx_ms = now_ms;
    }
}

bool AP_ChassisCAN::failsafe_triggered() const
{
    const uint32_t now_ms = AP_HAL::millis();

    const bool eps_timeout = (_eps.last_rx_ms != 0U) && ((now_ms - _eps.last_rx_ms) > CHCAN_EPS_RX_TIMEOUT_MS);
    if (eps_timeout) {
        return true;
    }
    if (_eps.fault_level >= uint8_t(_eps_fs_level.get())) {
        return true;
    }

    if (_ehb_enable != 0 && _ehb_fs_on_fault != 0) {
        const bool ehb_timeout = (_ehb.last_rx_ms != 0U) && ((now_ms - _ehb.last_rx_ms) > CHCAN_EHB_RX_TIMEOUT_MS);
        if (ehb_timeout) {
            return true;
        }
        if (_ehb.work_status == 0x07 || _ehb.fault_bits != 0) {
            return true;
        }
    }

    return false;
}

bool AP_ChassisCAN::pre_arm_check(char* reason, uint8_t reason_len) const
{
    if (!_initialized || _can_iface == nullptr) {
        hal.util->snprintf(reason, reason_len, "CHCAN not initialized");
        return false;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (_eps.last_rx_ms == 0U || (now_ms - _eps.last_rx_ms) > CHCAN_EPS_RX_TIMEOUT_MS) {
        hal.util->snprintf(reason, reason_len, "CHCAN EPS no data");
        return false;
    }

    if (_eps.center_uncalibrated) {
        hal.util->snprintf(reason, reason_len, "CHCAN EPS center uncalibrated");
        return false;
    }

    if (_eps.fault_level >= uint8_t(_eps_fs_level.get())) {
        hal.util->snprintf(reason, reason_len, "CHCAN EPS fault L%u C%u", (unsigned)_eps.fault_level, (unsigned)_eps.fault_code);
        return false;
    }

    if (!_eps.dbw_mode) {
        hal.util->snprintf(reason, reason_len, "CHCAN EPS not in DBW mode");
        return false;
    }

    if (_ehb_enable != 0) {
        if (_ehb.last_rx_ms == 0U || (now_ms - _ehb.last_rx_ms) > CHCAN_EHB_RX_TIMEOUT_MS) {
            hal.util->snprintf(reason, reason_len, "CHCAN EHB no data");
            return false;
        }

        if (_ehb.work_status == 0x01) {
            hal.util->snprintf(reason, reason_len, "CHCAN EHB init state");
            return false;
        }

        if (_ehb.work_status == 0x07) {
            hal.util->snprintf(reason, reason_len, "CHCAN EHB failed state");
            return false;
        }

        if (_ehb.driver_intervention) {
            hal.util->snprintf(reason, reason_len, "CHCAN EHB driver intervention");
            return false;
        }

        if (_ehb.fault_bits != 0) {
            hal.util->snprintf(reason, reason_len, "CHCAN EHB fault 0x%04X", (unsigned)_ehb.fault_bits);
            return false;
        }
    }

    return true;
}

bool AP_ChassisCAN::manual_override_active() const
{
    const bool eps_override = (_eps.last_rx_ms != 0U) && !_eps.dbw_mode;
    const bool ehb_override = (_ehb.last_rx_ms != 0U) && _ehb.driver_intervention;
    return eps_override || ehb_override;
}

void AP_ChassisCAN::loop()
{
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay(1);
    }

    while (true) {
        const uint32_t now_ms = AP_HAL::millis();

        AP_HAL::CANFrame frame;
        while (read_frame(frame, 1000)) {
            handle_rx_frame(frame, now_ms);
        }

        send_eps_command(now_ms);
        send_ehb_command(now_ms);

        hal.scheduler->delay_microseconds(1000);
    }
}

namespace AP {
AP_ChassisCAN *chassis_can()
{
    return AP_ChassisCAN::get_singleton();
}
}

#endif // AP_CHASSISCAN_ENABLED
