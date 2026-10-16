/*
   AP_Camera_P6UC.cpp -- see AP_Camera_P6UC.h for an overview.
*/

#include "AP_Camera_P6UC.h"

#if AP_CAMERA_P6UC_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <cstring>

extern const AP_HAL::HAL& hal;

// how often (ms) we ping the camera with a version-read to confirm the
// link is still alive
static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 5000;
// how long (ms) without a valid response before we call the link unhealthy
static constexpr uint32_t HEALTHY_TIMEOUT_MS = 10000;

static const char *const PALETTE_NAMES[] = {
    "White Hot", "Black Hot", "Rainbow", "Green Hot", "Red Hot", "Iron Red"
};

const AP_Param::GroupInfo AP_Camera_P6UC::var_info[] = {

    // @Param: PALETTE
    // @DisplayName: P6-UC pseudo-color palette
    // @Description: Palette shown by the P6-UC. Changing it in flight applies immediately.
    // @Values: 0:White Hot,1:Black Hot,2:Rainbow,3:Green Hot,4:Red Hot,5:Iron Red
    // @User: Standard
    AP_GROUPINFO("PALETTE", 1, AP_Camera_P6UC, _palette, 0),

    // @Param: AUTOCAL
    // @DisplayName: P6-UC automatic NUC calibration at boot
    // @Description: Enables the camera's automatic shutter (NUC) calibration when the link is established.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("AUTOCAL", 2, AP_Camera_P6UC, _default_auto_cal, 1),

    // @Param: MIRROR
    // @DisplayName: P6-UC mirror mode for the record-video switch
    // @Description: Mirror mode applied when the RC "Camera Record Video" switch is HIGH. LOW returns to no mirror.
    // @Values: 1:Horizontal,2:Vertical,3:Diagonal
    // @User: Standard
    AP_GROUPINFO("MIRROR", 3, AP_Camera_P6UC, _mirror_mode, 1),

    // @Param: ZOOM_STEP
    // @DisplayName: P6-UC zoom step
    // @Description: Zoom change each time the RC Camera Zoom switch is moved up or down
    // @Units: x
    // @Range: 0.1 3.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("ZOOM_STEP", 4, AP_Camera_P6UC, _zoom_step, 1.0f),

    AP_GROUPEND
};

AP_Camera_P6UC::AP_Camera_P6UC(AP_Camera &frontend, AP_Camera_Params &params, uint8_t instance) :
    AP_Camera_Backend(frontend, params, instance),
    _uart(nullptr),
    _initialised(false),
    _rx_len(0),
    _last_heartbeat_ms(0),
    _last_response_ms(0),
    _sw_version(0), _hw_version(0), _version_valid(false),
    _serial_number_mask(0), _serial_number_valid(false),
    _antiburn_switch_state(false), _antiburn_switch_valid(false),
    _antiburn_recovery_ms(0), _antiburn_recovery_valid(false),
    _palette_index(0)
{
    memset(_serial_number, 0, sizeof(_serial_number));
    AP_Param::setup_object_defaults(this, var_info);
}

// ---------------------------------------------------------------------
// init / update / healthy
// ---------------------------------------------------------------------

void AP_Camera_P6UC::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // NOTE: this assumes a single P6-UC instance using serial index 0
    // of its protocol slot. If you need more than one P6-UC camera,
    // give this class an extra static instance-counter constructor
    // argument, the same way AP_RunCam handles multiple RunCams.
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_P6UC, 0);
    if (_uart == nullptr) {
        return;
    }

    // the camera's UART is fixed at 115200 8N1 by hardware, so we set
    // this explicitly rather than trusting SERIALx_BAUD
    _uart->begin(BAUD_RATE);
    _initialised = true;

    // apply persisted boot-time defaults
    apply_settings();

    // kick off a heartbeat immediately so healthy() has something to go on
    request_version();
    _last_heartbeat_ms = AP_HAL::millis();

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "P6-UC camera: UART connected");
}

// push our settings to the camera; called at boot and again whenever the
// camera starts answering (e.g. it powered up after the autopilot)
void AP_Camera_P6UC::apply_settings()
{
    const int8_t p = _palette.get();
    set_palette((p >= 0 && p <= 5) ? (Palette)p : Palette::WHITE_HOT);
    set_auto_calibration(_default_auto_cal.get() != 0);
    set_digital_zoom(_zoom_x);
}

void AP_Camera_P6UC::update()
{
    if (!_initialised) {
        return;
    }

    read_incoming();
    update_rate_controls();

    // camera just started answering: resend our settings in case it missed them at boot
    const bool link_up = healthy();
    if (link_up && !_link_up) {
        apply_settings();
    }
    _link_up = link_up;

    // CAMx_PALETTE changed from the GCS: apply it now
    const int8_t p = _palette.get();
    if (p != _palette_index && p >= 0 && p <= 5) {
        set_palette((Palette)p);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "P6-UC: palette %s", PALETTE_NAMES[p]);
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_heartbeat_ms >= HEARTBEAT_INTERVAL_MS) {
        _last_heartbeat_ms = now_ms;
        request_version();
    }
}

bool AP_Camera_P6UC::healthy() const
{
    if (!_initialised) {
        return false;
    }
    return _last_response_ms != 0 && (AP_HAL::millis() - _last_response_ms) < HEALTHY_TIMEOUT_MS;
}

// ---------------------------------------------------------------------
// AP_Camera_Backend virtuals
// ---------------------------------------------------------------------

bool AP_Camera_P6UC::trigger_pic()
{
    // no onboard storage -- nearest useful equivalent is a manual NUC
    if (!trigger_manual_calibration()) {
        return false;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "P6-UC: NUC");
    return true;
}

bool AP_Camera_P6UC::record_video(bool start_recording)
{
    // no onboard recording -- the record switch toggles mirror instead
    if (!_initialised) {
        return false;
    }
    Mirror mode = Mirror::NONE;
    if (start_recording) {
        const int8_t m = _mirror_mode.get();
        mode = (m >= 1 && m <= 3) ? (Mirror)m : Mirror::HORIZONTAL;
    }
    set_mirror(mode);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "P6-UC: mirror %s", start_recording ? "on" : "off");
    return true;
}

bool AP_Camera_P6UC::set_zoom(ZoomType zoom_type, float zoom_value)
{
    switch (zoom_type) {
    case ZoomType::PCT: {
        // 0-100% maps onto the camera's native 1.0x-4.0x range
        const float pct = constrain_float(zoom_value, 0.0f, 100.0f);
        return set_digital_zoom(1.0f + (pct / 100.0f) * 3.0f);
    }
    case ZoomType::RATE: {
        // RC 3-pos switch: one step each time it is moved to UP (+) or DOWN (-).
        // It must leave that position (e.g. back to middle) before stepping again.
        const int8_t dir = (zoom_value > 0.5f) ? 1 : (zoom_value < -0.5f) ? -1 : 0;
        const bool moved = (dir != _zoom_switch);
        _zoom_switch = dir;
        if (!_initialised) {
            return false;
        }
        if (dir == 0 || !moved) {
            return true;
        }
        const float step = constrain_float(_zoom_step.get(), 0.1f, 3.0f);
        set_digital_zoom(_zoom_x + dir * step);   // clamps to 1.0x-4.0x
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "P6-UC: zoom %.1fx", (double)_zoom_x);
        return true;
    }
    default:
        return false;
    }
}

SetFocusResult AP_Camera_P6UC::set_focus(FocusType focus_type, float focus_value)
{
    if (!_initialised) {
        return SetFocusResult::FAILED;
    }
    switch (focus_type) {
    case FocusType::RATE: {
        // RC "Camera Manual Focus" 3-pos switch -> brightness
        const int8_t rate = (focus_value > 0.5f) ? 1 : (focus_value < -0.5f) ? -1 : 0;
        if (rate == 0 && _brightness_rate != 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "P6-UC: brightness %u", (unsigned)_brightness);
        }
        _brightness_rate = rate;
        return SetFocusResult::ACCEPTED;
    }
    case FocusType::AUTO:
        // RC "Camera Auto Focus" switch HIGH -> persist settings on the camera
        save_parameters();
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "P6-UC: settings saved");
        return SetFocusResult::ACCEPTED;
    default:
        return SetFocusResult::UNSUPPORTED;
    }
}

bool AP_Camera_P6UC::change_setting(CameraSetting setting, float value)
{
    switch (setting) {
    case CameraSetting::THERMAL_PALETTE: {
        const int16_t p = (int16_t)roundf(value);
        if (p < 0 || p > 5) {
            return false;
        }
        if (!set_palette((Palette)p)) {
            return false;
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "P6-UC: palette %s", PALETTE_NAMES[p]);
        return true;
    }
    default:
        return false;
    }
}

void AP_Camera_P6UC::update_rate_controls()
{
    const uint32_t now_ms = AP_HAL::millis();

    if (_brightness_rate != 0 && now_ms - _last_brightness_step_ms >= RATE_STEP_MS) {
        _last_brightness_step_ms = now_ms;
        int16_t target = (int16_t)_brightness + BRIGHTNESS_STEP * _brightness_rate;
        target = constrain_int16(target, 0, BRIGHTNESS_MAX);
        if (target != _brightness) {
            set_brightness((uint8_t)target);
        }
    }
}

void AP_Camera_P6UC::cam_mode_toggle()
{
    _palette_index = (_palette_index + 1) % 6;
    if (set_palette((Palette)_palette_index)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "P6-UC: palette %s", PALETTE_NAMES[_palette_index]);
    }
}

// ---------------------------------------------------------------------
// native ICD commands
// ---------------------------------------------------------------------

bool AP_Camera_P6UC::set_brightness(uint8_t value)
{
    if (!_initialised) {
        return false;
    }
    _brightness = (uint8_t)constrain_int16(value, 0, BRIGHTNESS_MAX);
    send_u16(0x2A, 0x01, _brightness);
    return true;
}

bool AP_Camera_P6UC::set_contrast(uint8_t value)
{
    if (!_initialised) {
        return false;
    }
    send_u16(0x2A, 0x02, value);
    return true;
}

bool AP_Camera_P6UC::set_enhancement(uint8_t value)
{
    if (!_initialised) {
        return false;
    }
    send_u16(0x2A, 0x03, value);
    return true;
}

bool AP_Camera_P6UC::set_denoise(uint8_t value)
{
    if (!_initialised) {
        return false;
    }
    send_u16(0x2A, 0x04, value);
    return true;
}

bool AP_Camera_P6UC::set_digital_zoom(float zoom_x)
{
    if (!_initialised) {
        return false;
    }
    zoom_x = constrain_float(zoom_x, 1.0f, 4.0f);
    const uint8_t raw = (uint8_t)roundf(zoom_x * 10.0f);  // 10=1.0x ... 40=4.0x
    _zoom_x = raw * 0.1f;   // remember what the camera actually got
    send_u8(0x2B, 0x00, raw);
    return true;
}

bool AP_Camera_P6UC::set_mirror(Mirror mode)
{
    if (!_initialised) {
        return false;
    }
    send_frame(0x2C, (uint8_t)mode);
    return true;
}

bool AP_Camera_P6UC::set_palette(Palette palette)
{
    if (!_initialised) {
        return false;
    }
    send_frame(0x2D, (uint8_t)palette);
    _palette_index = (uint8_t)palette;
    _palette.set((int8_t)palette);
    return true;
}

bool AP_Camera_P6UC::set_auto_calibration(bool enable)
{
    if (!_initialised) {
        return false;
    }
    send_u8(0x26, 0x01, enable ? 1 : 0);
    return true;
}

bool AP_Camera_P6UC::trigger_manual_calibration()
{
    if (!_initialised) {
        return false;
    }
    send_frame(0x26, 0x02);
    return true;
}

bool AP_Camera_P6UC::trigger_background_calibration()
{
    if (!_initialised) {
        return false;
    }
    send_frame(0x26, 0x03);
    return true;
}

bool AP_Camera_P6UC::set_auto_calibration_interval_ms(uint32_t ms)
{
    if (!_initialised) {
        return false;
    }
    send_u32(0x26, 0x04, ms);
    return true;
}

bool AP_Camera_P6UC::set_shutter_always_open()
{
    if (!_initialised) {
        return false;
    }
    send_frame(0x26, 0x05);
    return true;
}

bool AP_Camera_P6UC::set_shutter_always_closed()
{
    if (!_initialised) {
        return false;
    }
    send_frame(0x26, 0x06);
    return true;
}

bool AP_Camera_P6UC::save_parameters()
{
    if (!_initialised) {
        return false;
    }
    send_frame(0x29, 0x00);
    return true;
}

bool AP_Camera_P6UC::restore_factory_defaults()
{
    if (!_initialised) {
        return false;
    }
    send_frame(0x2E, 0x00);
    return true;
}

bool AP_Camera_P6UC::set_antiburn_protection(bool enable)
{
    if (!_initialised) {
        return false;
    }
    send_u8(0x37, 0x01, enable ? 1 : 0);
    return true;
}

bool AP_Camera_P6UC::set_antiburn_recovery_time_ms(uint32_t ms)
{
    if (!_initialised) {
        return false;
    }
    send_u32(0x36, 0x01, ms);
    return true;
}

// ---------------------------------------------------------------------
// asynchronous read-backs
// ---------------------------------------------------------------------

void AP_Camera_P6UC::request_version()
{
    if (!_initialised) {
        return;
    }
    send_frame(0x24, 0x00);
}

void AP_Camera_P6UC::request_serial_number()
{
    if (!_initialised) {
        return;
    }
    _serial_number_mask = 0;
    _serial_number_valid = false;
    send_frame(0x22, 0xF1);
    send_frame(0x22, 0xF2);
    send_frame(0x22, 0xF3);
    send_frame(0x22, 0xF4);
}

void AP_Camera_P6UC::request_antiburn_switch()
{
    if (!_initialised) {
        return;
    }
    send_frame(0x37, 0xF1);
}

void AP_Camera_P6UC::request_antiburn_recovery_time()
{
    if (!_initialised) {
        return;
    }
    send_frame(0x36, 0xF1);
}

bool AP_Camera_P6UC::get_version(uint16_t &sw_version, uint16_t &hw_version) const
{
    if (!_version_valid) {
        return false;
    }
    sw_version = _sw_version;
    hw_version = _hw_version;
    return true;
}

bool AP_Camera_P6UC::get_serial_number(uint8_t buf[32]) const
{
    if (!_serial_number_valid) {
        return false;
    }
    memcpy(buf, _serial_number, 32);
    return true;
}

bool AP_Camera_P6UC::get_antiburn_switch(bool &enabled) const
{
    if (!_antiburn_switch_valid) {
        return false;
    }
    enabled = _antiburn_switch_state;
    return true;
}

bool AP_Camera_P6UC::get_antiburn_recovery_time_ms(uint32_t &ms) const
{
    if (!_antiburn_recovery_valid) {
        return false;
    }
    ms = _antiburn_recovery_ms;
    return true;
}

// ---------------------------------------------------------------------
// low level frame TX
// ---------------------------------------------------------------------

void AP_Camera_P6UC::send_frame(uint8_t cmd1, uint8_t cmd2, const uint8_t data[8])
{
    if (_uart == nullptr) {
        return;
    }

    uint8_t frame[FRAME_LEN];
    frame[0] = FRAME_HEADER;
    frame[1] = cmd1;
    frame[2] = cmd2;
    if (data != nullptr) {
        memcpy(&frame[3], data, 8);
    } else {
        memset(&frame[3], 0, 8);
    }

    uint8_t checksum = 0;
    for (uint8_t i = 0; i < FRAME_LEN - 2; i++) {
        checksum += frame[i];
    }
    frame[FRAME_LEN - 2] = checksum;
    frame[FRAME_LEN - 1] = FRAME_TAIL;

    _uart->write(frame, FRAME_LEN);
}

void AP_Camera_P6UC::send_u8(uint8_t cmd1, uint8_t cmd2, uint8_t value)
{
    uint8_t data[8] = {};
    data[0] = value;
    send_frame(cmd1, cmd2, data);
}

void AP_Camera_P6UC::send_u16(uint8_t cmd1, uint8_t cmd2, uint16_t value)
{
    uint8_t data[8] = {};
    data[0] = (uint8_t)(value & 0xFF);
    data[1] = (uint8_t)((value >> 8) & 0xFF);
    send_frame(cmd1, cmd2, data);
}

void AP_Camera_P6UC::send_u32(uint8_t cmd1, uint8_t cmd2, uint32_t value)
{
    uint8_t data[8] = {};
    data[0] = (uint8_t)(value & 0xFF);
    data[1] = (uint8_t)((value >> 8) & 0xFF);
    data[2] = (uint8_t)((value >> 16) & 0xFF);
    data[3] = (uint8_t)((value >> 24) & 0xFF);
    send_frame(cmd1, cmd2, data);
}

// ---------------------------------------------------------------------
// low level frame RX (non-blocking)
// ---------------------------------------------------------------------

void AP_Camera_P6UC::read_incoming()
{
    if (_uart == nullptr) {
        return;
    }

    int32_t available = _uart->available();
    while (available-- > 0) {
        const int16_t b = _uart->read();
        if (b < 0) {
            break;
        }
        if (_rx_len == 0 && (uint8_t)b != FRAME_HEADER) {
            // resync: discard bytes until we see a header
            continue;
        }
        _rx_buf[_rx_len++] = (uint8_t)b;
        if (_rx_len == FRAME_LEN) {
            handle_frame(_rx_buf);
            _rx_len = 0;
        }
    }
}

void AP_Camera_P6UC::handle_frame(const uint8_t frame[FRAME_LEN])
{
    if (frame[FRAME_LEN - 1] != FRAME_TAIL) {
        return;  // malformed, drop
    }

    uint8_t checksum = 0;
    for (uint8_t i = 0; i < FRAME_LEN - 2; i++) {
        checksum += frame[i];
    }
    if (checksum != frame[FRAME_LEN - 2]) {
        return;  // checksum failed, drop
    }

    const uint8_t cmd1 = frame[1];
    const uint8_t cmd2 = frame[2];
    const uint8_t *data = &frame[3];

    switch (cmd1) {
    case 0x24:  // version read-back
        _sw_version = data[0] | ((uint16_t)data[1] << 8);
        _hw_version = data[2] | ((uint16_t)data[3] << 8);
        _version_valid = true;
        break;

    case 0x22: {  // serial number chunk read-back
        uint8_t chunk_index;
        switch (cmd2) {
        case 0xF1: chunk_index = 0; break;
        case 0xF2: chunk_index = 1; break;
        case 0xF3: chunk_index = 2; break;
        case 0xF4: chunk_index = 3; break;
        default:
            return;
        }
        memcpy(&_serial_number[chunk_index * 8], data, 8);
        _serial_number_mask |= (1U << chunk_index);
        if (_serial_number_mask == 0x0F) {
            _serial_number_valid = true;
        }
        break;
    }

    case 0x37:  // anti-burn-in switch read-back
        if (cmd2 == 0xF1) {
            _antiburn_switch_state = (data[0] != 0);
            _antiburn_switch_valid = true;
        }
        break;

    case 0x36:  // anti-burn-in recovery time read-back
        if (cmd2 == 0xF1) {
            _antiburn_recovery_ms = data[0] | ((uint32_t)data[1] << 8) |
                                     ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
            _antiburn_recovery_valid = true;
        }
        break;

    default:
        // set-commands generally echo the frame back; nothing to cache
        break;
    }

    _last_response_ms = AP_HAL::millis();
}

#endif  // AP_CAMERA_P6UC_ENABLED