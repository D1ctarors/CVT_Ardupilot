

#include "AP_HFC.h"
#include <AP_HAL/AP_HAL.h>
#include <cstring>

extern const AP_HAL::HAL &hal;

#define AP_HFC_DEBUG 0

AP_HFC::AP_HFC() = default;

void AP_HFC::init(AP_HAL::UARTDriver *uart)
{
    uart_ = uart;
    if (!uart_)
    {
#if AP_HFC_DEBUG
        hal.console->printf("ERROR: Hydrogen Fuel Cell: UART not initialized\n");
#endif
        return;
    }

    state_ = PState::WAIT_ID;
    idx_ = 0;
    healthy_ = false;
    last_update_ms_ = 0;
}

void AP_HFC::request_switch(uint8_t id, bool enable)
{
    cmd_.id = id;
    cmd_.enable = enable;
    cmd_.pending = true;
}

void AP_HFC::send_command_now_(uint8_t id, bool on)
{
    if (!uart_)
        return;

    uint8_t cmd[7]{};
    cmd[0] = 25; // CMD_PREFIX
    cmd[1] = id;
    for (uint8_t i = 2; i < 7; i++)
    {
        cmd[i] = on ? 96 : 69; // ON / OFF
    }
    (void)uart_->write(cmd, sizeof(cmd));
}

void AP_HFC::update()
{
    if (!uart_)
    {
        return;
    }

    if (cmd_.pending && uart_)
    {
        send_command_now_(cmd_.id, cmd_.enable);
        cmd_.pending = false;
    }

    uint16_t budget = READ_BUDGET_BYTES;
    const uint32_t t0 = AP_HAL::micros();

    while (budget && uart_->available() > 0)
    {
        handle_byte(uart_->read());
        budget--;

        if ((AP_HAL::micros() - t0) > READ_BUDGET_TIME_US)
        {
            break;
        }
    }
}

void AP_HFC::handle_byte(uint8_t b)
{
    switch (state_)
    {
    case PState::WAIT_ID:
        if (id_is_valid(b))
        {
            pkt_[0] = b;
            idx_ = 1;
            state_ = PState::READ;
        }
        break;

    case PState::READ:
        pkt_[idx_++] = b;
        if (idx_ == 20)
        {
            if (crc_is_valid(pkt_))
            {
                parse_packet();
                healthy_ = true;
                last_update_ms_ = AP_HAL::millis();
            }
            else
            {
#if AP_HFC_DEBUG
                hal.console->printf("DEBUG: Hydrogen Fuel Cell: bad CRC (id=0x%02X)\n", pkt_[0]);
#endif
            }
            state_ = PState::WAIT_ID;
            idx_ = 0;
        }
        break;
    }
}

void AP_HFC::parse_packet()
{
    const uint8_t *p = pkt_;
    data_.identifikator = p[0];

    const uint16_t u16_current = uint16_t(p[1] | (uint16_t(p[2]) << 8));
    const uint16_t u16_batt = uint16_t(p[3] | (uint16_t(p[4]) << 8));
    const uint16_t u16_out = uint16_t(p[5] | (uint16_t(p[6]) << 8));
    const uint16_t u16_te = uint16_t(p[7] | (uint16_t(p[8]) << 8));
    const uint16_t u16_runtime = uint16_t(p[9] | (uint16_t(p[10]) << 8));
    const int16_t i16_press = int16_t(p[11] | (uint16_t(p[12]) << 8));

    data_.current = float(u16_current) * SCALE_0P1;
    data_.battery_voltage = float(u16_batt) * SCALE_0P1;
    data_.output_voltage = float(u16_out) * SCALE_0P1;
    data_.te_voltage = float(u16_te) * SCALE_0P1;
    data_.runtime = uint32_t(u16_runtime);
    data_.pressure = i16_press;
    data_.temp1 = int16_t(int8_t(p[13])) + TEMP_OFFS;
    data_.temp2 = int16_t(int8_t(p[14])) + TEMP_OFFS;
    data_.pwm = p[15];
    data_.status = p[16];
    data_.errors = p[17];
    data_.counter = uint32_t(p[18]);
#if AP_HFC_DEBUG
    hal.console->printf("DEBUG: Hydrogen Fuel Cell: parsed I=%.1fA Ubat=%.1fV Uout=%.1fV TE=%.1fV T1=%d T2=%d PWM=%u\n",
                        (double)data_.current,
                        (double)data_.battery_voltage,
                        (double)data_.output_voltage,
                        (double)data_.te_voltage,
                        (int)data_.temp1, (int)data_.temp2, (unsigned)data_.pwm);
#endif
}

void AP_HFC::bind(AP_HAL::UARTDriver *uart)
{
    uart_ = uart;
    state_ = PState::WAIT_ID;
    idx_ = 0;
    healthy_ = false;
    last_update_ms_ = 0;
}

void AP_HFC::send_start_command(uint8_t id) { request_switch(id, true); }

void AP_HFC::send_stop_command(uint8_t id) { request_switch(id, false); }
