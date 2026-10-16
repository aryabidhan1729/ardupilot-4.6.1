/*
   AP_Camera_P6UC.h

   AP_Camera backend driver for the P6-UC uncooled infrared camera,
   controlled over a dedicated UART using the vendor's binary ICD
   (P6-UC UART ICD V1.0):

     Frame:  0x55 | CMD1 | CMD2 | DATA[8] | CHECKSUM | 0xAA   (13 bytes)
     Baud:   115200, 8N1
     Checksum = low byte of the sum of all preceding bytes.

   This class follows the same shape as AP_RunCam (also in this
   directory): a single AP_Camera_Backend implementation that owns its
   own UART, found via AP_SerialManager, and drives it from update()
   at the scheduler's camera rate (~50Hz).

   The P6-UC has no onboard picture/video storage (video leaves the
   aircraft over analog CVBS or USB only), so the handful of
   AP_Camera_Backend virtuals that assume a "picture" are deliberately
   re-mapped to the nearest sensible P6-UC action (see comments below).
   Every other ICD command (palette, mirror, brightness/contrast/
   enhance/denoise, NUC calibration, anti-burn-in) is exposed as a
   plain public method that can be called directly by other C++ code
   (e.g. a custom MAVLink handler, a mission/aux-switch hook, or Lua
   scripting bindings you add later).

   See INTEGRATION.md alongside this file for the (small) set of edits
   needed elsewhere in ArduPilot to register this backend.
*/
#pragma once

#include "AP_Camera_Backend.h"

#ifndef AP_CAMERA_P6UC_ENABLED
#define AP_CAMERA_P6UC_ENABLED AP_CAMERA_BACKEND_DEFAULT_ENABLED
#endif

#if AP_CAMERA_P6UC_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

class AP_Camera_P6UC : public AP_Camera_Backend
{
public:

    // constructor
    AP_Camera_P6UC(AP_Camera &frontend, AP_Camera_Params &params, uint8_t instance);

    CLASS_NO_COPY(AP_Camera_P6UC);

    // backend-specific persisted parameters (small, self-contained --
    // see INTEGRATION.md for how these get wired into the param tree)
    static const struct AP_Param::GroupInfo var_info[];

    // init - locates and opens the UART, called once at boot
    void init() override;

    // update - should be called at ~50Hz by the AP_Camera scheduler
    void update() override;

    // return true if we have a UART and have heard back from the camera recently
    bool healthy() const override;

    //
    // AP_Camera_Backend virtuals, remapped onto the P6-UC's own features
    //
    // The P6-UC cannot capture/store a picture itself, so "take a
    // picture" is remapped to a manual NUC (shutter) calibration --
    // the nearest equivalent one-shot action a GCS "take picture"
    // button/mission item/RC "Camera Trigger" switch can usefully trigger.
    bool trigger_pic() override;

    // no onboard recording, so the RC "Camera Record Video" switch is
    // remapped to mirror: HIGH applies P6UC_MIRROR, otherwise no mirror
    bool record_video(bool start_recording) override;

    // PCT:  MAV_CMD_SET_CAMERA_ZOOM 0-100% -> 1.0x-4.0x
    // RATE: RC "Camera Zoom" 3-pos switch, +1 zoom in / 0 hold / -1 zoom out,
    //       stepped in update() while held
    bool set_zoom(ZoomType zoom_type, float zoom_value) override;

    // no focus on a thermal camera, so:
    // RATE: RC "Camera Manual Focus" 3-pos switch -> brightness up/hold/down
    // AUTO: RC "Camera Auto Focus" switch HIGH  -> save settings on the camera
    SetFocusResult set_focus(FocusType focus_type, float focus_value) override;

    // CameraSetting::THERMAL_PALETTE -> select palette 0-5 directly
    // (reachable from Lua / MAVLink camera settings)
    bool change_setting(CameraSetting setting, float value) override;

    // cycles through the 6 pseudo-color palettes; wired to whatever
    // RC aux switch / GCS button triggers "camera mode toggle"
    void cam_mode_toggle() override;

    //
    // native P6-UC ICD commands -- call these directly for full control
    //
    enum class Palette : uint8_t {
        WHITE_HOT = 0x00,
        BLACK_HOT = 0x01,
        RAINBOW   = 0x02,
        GREEN_HOT = 0x03,
        RED_HOT   = 0x04,
        IRON_RED  = 0x05,
    };

    enum class Mirror : uint8_t {
        NONE       = 0x00,
        HORIZONTAL = 0x01,
        VERTICAL   = 0x02,
        DIAGONAL   = 0x03,
    };

    // section 4.5 -- image effect adjustment
    bool set_brightness(uint8_t value);        // 0-128, default 64
    bool set_contrast(uint8_t value);          // 0-255, default 64
    bool set_enhancement(uint8_t value);       // 0-255
    bool set_denoise(uint8_t value);           // 0-255

    // section 4.6 -- digital zoom, 1.0x - 4.0x, 0.1x steps
    bool set_digital_zoom(float zoom_x);

    // section 4.7 -- mirror
    bool set_mirror(Mirror mode);

    // section 4.8 -- pseudo-color palette
    bool set_palette(Palette palette);

    // section 4.3 -- shutter / NUC calibration
    bool set_auto_calibration(bool enable);
    bool trigger_manual_calibration();
    bool trigger_background_calibration();
    bool set_auto_calibration_interval_ms(uint32_t ms);
    bool set_shutter_always_open();
    bool set_shutter_always_closed();

    // section 4.4 / 4.9 -- parameter persistence
    bool save_parameters();
    bool restore_factory_defaults();

    // section 4.10 / 4.11 -- anti-burn-in protection
    bool set_antiburn_protection(bool enable);
    bool set_antiburn_recovery_time_ms(uint32_t ms);

    // asynchronous read-backs (sections 4.1/4.2/4.10.2/4.11.2): each
    // "request_*" sends the query; the answer (if any) is parsed out
    // of the UART in update() and cached. This class never blocks
    // waiting for a reply -- that would stall the flight scheduler.
    void request_version();
    void request_serial_number();
    void request_antiburn_switch();
    void request_antiburn_recovery_time();

    bool get_version(uint16_t &sw_version, uint16_t &hw_version) const;
    bool get_serial_number(uint8_t buf[32]) const;
    bool get_antiburn_switch(bool &enabled) const;
    bool get_antiburn_recovery_time_ms(uint32_t &ms) const;

private:

    // ---- low level frame handling (P6-UC UART ICD V1.0, section 4) ----
    static constexpr uint8_t FRAME_HEADER = 0x55;
    static constexpr uint8_t FRAME_TAIL   = 0xAA;
    static constexpr uint8_t FRAME_LEN    = 13;
    static constexpr uint32_t BAUD_RATE   = 115200;

    void send_frame(uint8_t cmd1, uint8_t cmd2, const uint8_t data[8] = nullptr);
    void send_u8(uint8_t cmd1, uint8_t cmd2, uint8_t value);
    void send_u16(uint8_t cmd1, uint8_t cmd2, uint16_t value);
    void send_u32(uint8_t cmd1, uint8_t cmd2, uint32_t value);

    // non-blocking receive: called every update() to drain the UART
    // and parse any complete, checksum-valid frames
    void read_incoming();
    void handle_frame(const uint8_t frame[FRAME_LEN]);

    AP_HAL::UARTDriver *_uart;
    bool _initialised;

    // receive state machine
    uint8_t _rx_buf[FRAME_LEN];
    uint8_t _rx_len;

    // heartbeat / health tracking
    uint32_t _last_heartbeat_ms;   // last time we *sent* a version query
    uint32_t _last_response_ms;    // last time we *received* any valid frame

    // cached async read-back results
    uint16_t _sw_version, _hw_version;
    bool     _version_valid;

    uint8_t _serial_number[32];
    uint8_t _serial_number_mask;   // bit per 8-byte chunk (F1..F4) received
    bool    _serial_number_valid;

    bool     _antiburn_switch_state;
    bool     _antiburn_switch_valid;
    uint32_t _antiburn_recovery_ms;
    bool     _antiburn_recovery_valid;

    // state for cam_mode_toggle()
    uint8_t _palette_index;

    // ---- RC rate controls (zoom / brightness held on a 3-pos switch) ----
    static constexpr uint32_t RATE_STEP_MS      = 100;   // one step per 100 ms while held
    //static constexpr float    ZOOM_STEP_X       = 0.1f;  // camera's native zoom resolution
    static constexpr uint8_t  BRIGHTNESS_STEP   = 4;     // 0-128 range -> ~3 s end to end
    static constexpr uint8_t  BRIGHTNESS_MAX    = 128;

    void update_rate_controls();

    float    _zoom_x = 1.0f;               // last zoom sent to the camera
    int8_t   _zoom_switch = 0;             // last RC zoom switch position (-1/0/+1)
    bool     _link_up = false;             // camera was answering last update()

    void apply_settings();                 // push palette/auto-cal/zoom to the camera

    uint8_t  _brightness = 64;             // camera default; tracked from here on
    int8_t   _brightness_rate = 0;         // -1, 0, +1 from the RC switch
    uint32_t _last_brightness_step_ms = 0;

    // backend-specific persisted settings, applied once at init()
    AP_Int8  _palette;             // CAMx_PALETTE: 0=White Hot ... 5=Iron Red, applied live
    AP_Int8  _default_auto_cal;    // CAMx_AUTOCAL
    AP_Int8  _mirror_mode;         // CAMx_MIRROR
    AP_Float _zoom_step;           // CAMx_ZOOM_STEP: zoom change per RC switch flick
};

#endif  // AP_CAMERA_P6UC_ENABLED