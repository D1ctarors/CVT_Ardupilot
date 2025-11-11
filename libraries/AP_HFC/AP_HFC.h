
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <stdint.h>

class AP_HFC
{
public:
    struct Data
    {
        uint8_t identifikator;
        float current;
        float battery_voltage;
        float output_voltage;
        float te_voltage;
        uint32_t runtime;
        int16_t pressure;
        int16_t temp1;
        int16_t temp2;
        uint8_t pwm;
        uint8_t status;
        uint8_t errors;
        uint32_t counter;
    };

    AP_HFC();

    void init(AP_HAL::UARTDriver *uart);

    void update();

    void send_start_command(uint8_t id);
    void send_stop_command(uint8_t id);

    const Data &data() const { return data_; }
    bool healthy() const { return healthy_; }
    uint32_t last_update_ms() const { return last_update_ms_; }

    void bind(AP_HAL::UARTDriver *uart);
    void request_switch(uint8_t id, bool enable);

    bool is_bound() const { return uart_ != nullptr; }

private:
    static constexpr uint8_t ID_MIN = 0x50;
    static constexpr uint8_t ID_MAX = 0x55;

    static constexpr float SCALE_0P1 = 0.1f;
    static constexpr int TEMP_OFFS = 75;

    static constexpr uint16_t READ_BUDGET_BYTES = 20 * 4;
    static constexpr uint32_t READ_BUDGET_TIME_US = 2000;

    enum class PState : uint8_t
    {
        WAIT_ID = 0,
        READ
    };
    PState state_ = PState::WAIT_ID;
    uint8_t pkt_[20]{};
    uint8_t idx_ = 0;

    AP_HAL::UARTDriver *uart_ = nullptr;

    Data data_{};
    bool healthy_ = false;
    uint32_t last_update_ms_ = 0;

    void parse_packet();

    static inline bool crc_is_valid(const uint8_t *p)
    {
        uint16_t s = 0;
        for (uint8_t i = 1; i < 20; i++)
        {
            s += p[i];
        }
        return (uint8_t)s == 0;
    }

    void handle_byte(uint8_t b);
    static inline bool id_is_valid(uint8_t id) { return id >= ID_MIN && id <= ID_MAX; }

    struct PendingCmd
    {
        uint8_t id = 0;
        bool enable = false;
        bool pending = false;
    } cmd_;

    void send_command_now_(uint8_t id, bool on);
};
