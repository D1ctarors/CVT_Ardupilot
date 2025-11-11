#include "AP_HFC_CN.h"
#include <cstring>

extern const AP_HAL::HAL &hal;

// -------- CRC16 Modbus (poly 0xA001), init 0xFFFF --------
uint16_t AP_HFC_CN::modbus_crc16_(const uint8_t *p, uint16_t n)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < n; i++)
    {
        crc ^= uint16_t(p[i]);
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc = (crc >> 1);
        }
    }
    return crc;
}

// --------------------------- Init ---------------------------
void AP_HFC_CN::init(AP_HAL::UARTDriver *uart, uint8_t device_addr)
{
    uart_ = uart;
    dev_addr_ = device_addr;

    rx_state_ = RxState::WAIT_ADDR;
    rx_func_ = 0;
    rx_len_ = 0;
    rx_pos_ = 0;
    healthy_ = false;
    last_update_ms_ = 0;
    last_poll_ms_ = 0;
    waiting_read_resp_ = false;

    // UART уже сконфигурирован SerialManager-ом под 57600 8N1
}

// --------------------------- Polling + RX ---------------------------
void AP_HFC_CN::update()
{
    if (!uart_)
        return;

    // 1) Периодический опрос 0x03: 5 Гц как безопасная частота
    const uint32_t now_ms = AP_HAL::millis();
    if (!waiting_read_resp_ && (now_ms - last_poll_ms_ >= 200))
    {
        poll_once_();
        last_poll_ms_ = now_ms;
        waiting_read_resp_ = true;
    }

    // 2) Чтение с бюджетом по времени, чтобы не блокировать loop()
    const uint32_t t0 = AP_HAL::micros();
    while (uart_->available() > 0)
    {
        const uint8_t b = uart_->read();
        handle_byte_(b);

        if ((AP_HAL::micros() - t0) > 2000) // ~2 ms бюджет
            break;
    }
}

void AP_HFC_CN::poll_once_()
{
    // Read Holding Registers: start=0x0000, count=0x0013 (19)
    send_read_03_(REG_START, REG_COUNT);
}

void AP_HFC_CN::send_read_03_(uint16_t start_reg, uint16_t n_regs)
{
    uint8_t frm[8];
    frm[0] = dev_addr_;
    frm[1] = FUNC_READ_HOLDING;
    frm[2] = uint8_t(start_reg >> 8);
    frm[3] = uint8_t(start_reg & 0xFF);
    frm[4] = uint8_t(n_regs >> 8);
    frm[5] = uint8_t(n_regs & 0xFF);
    const uint16_t crc = modbus_crc16_(frm, 6);
    frm[6] = uint8_t(crc & 0xFF); // CRC low
    frm[7] = uint8_t(crc >> 8);   // CRC high
    (void)uart_->write(frm, sizeof(frm));
}

void AP_HFC_CN::send_cmd_06_(uint16_t reg, uint16_t value)
{
    uint8_t frm[8];
    frm[0] = dev_addr_;
    frm[1] = FUNC_WRITE_SINGLE;
    frm[2] = uint8_t(reg >> 8);
    frm[3] = uint8_t(reg & 0xFF);
    frm[4] = uint8_t(value >> 8);
    frm[5] = uint8_t(value & 0xFF);
    const uint16_t crc = modbus_crc16_(frm, 6);
    frm[6] = uint8_t(crc & 0xFF);
    frm[7] = uint8_t(crc >> 8);
    (void)uart_->write(frm, sizeof(frm));
}

// --------------------------- Commands ---------------------------
void AP_HFC_CN::send_start()
{
    send_cmd_06_(REG_STARTSTOP, START_VAL);
}

void AP_HFC_CN::send_stop()
{
    send_cmd_06_(REG_STARTSTOP, STOP_VAL);
}

// --------------------------- RX State Machine ---------------------------
void AP_HFC_CN::handle_byte_(uint8_t b)
{
    switch (rx_state_)
    {
    case RxState::WAIT_ADDR:
        if (b == dev_addr_)
        {
            rx_crc_calc_ = 0xFFFF;
            // Крутим CRC по байтам по мере накопления
            uint8_t t[1] = {b};
            rx_crc_calc_ = modbus_crc16_(t, 1);
            rx_state_ = RxState::WAIT_FUNC;
        }
        break;

    case RxState::WAIT_FUNC:
    {
        rx_func_ = b;
        uint8_t t[1] = {b};
        rx_crc_calc_ = modbus_crc16_(&t[0], 1) ^ (rx_crc_calc_ ^ 0xFFFF); // аккуратно добавим байт
        // Проще — пересчитать на лету всё сообщение: соберём в локальный буфер
        // но для ясности пересчитаем далее целиком в handle_frame_.

        // Переходим по формату:
        if (rx_func_ == FUNC_READ_HOLDING)
        {
            rx_state_ = RxState::WAIT_03_LEN;
        }
        else if (rx_func_ == FUNC_WRITE_SINGLE)
        {
            // Для 0x06 далее идут 4 байта данных (адрес рег + значение) => len=4 + 2? (на самом деле 4)
            rx_len_ = 4; // 2 байта регистр + 2 байта значение
            rx_pos_ = 0;
            rx_state_ = RxState::READ_DATA;
        }
        else
        {
            // неизвестная функция — сброс
            rx_state_ = RxState::WAIT_ADDR;
        }
        break;
    }

    case RxState::WAIT_03_LEN:
        // byte count для 0x03; ожидаем строго 0x26 (38)
        rx_len_ = b;
        rx_pos_ = 0;
        rx_state_ = RxState::READ_DATA;
        break;

    case RxState::READ_DATA:
        if (rx_pos_ < sizeof(rx_buf_))
        {
            rx_buf_[rx_pos_++] = b;
            if (rx_pos_ == rx_len_)
            {
                rx_state_ = RxState::READ_CRC_LO;
            }
        }
        else
        {
            // переполнение — сброс
            rx_state_ = RxState::WAIT_ADDR;
        }
        break;

    case RxState::READ_CRC_LO:
        rx_crc_recv_ = b; // low
        rx_state_ = RxState::READ_CRC_HI;
        break;

    case RxState::READ_CRC_HI:
        rx_crc_recv_ |= (uint16_t(b) << 8); // high
        // Соберём весь кадр заново и проверим CRC:
        // frame = [addr][func][(opt)len][data...]
        {
            uint8_t hdr[2] = {dev_addr_, rx_func_};
            uint16_t crc = modbus_crc16_(hdr, 2);

            if (rx_func_ == FUNC_READ_HOLDING)
            {
                uint8_t lenb[1] = {rx_len_};
                crc = modbus_crc16_(lenb, 1) ^ (crc ^ 0xFFFF); // корректно добавить байт — проще посчитать на полном временном буфере:
                // Для простоты — посчитаем на большом временном буфере
                uint8_t temp[2 + 1 + 64];
                uint16_t n = 0;
                temp[n++] = dev_addr_;
                temp[n++] = rx_func_;
                temp[n++] = rx_len_;
                memcpy(&temp[n], rx_buf_, rx_len_);
                n += rx_len_;
                const uint16_t calc = modbus_crc16_(temp, n);
                if (calc == rx_crc_recv_)
                {
                    handle_frame_();
                }
            }
            else if (rx_func_ == FUNC_WRITE_SINGLE)
            {
                uint8_t temp[2 + 4];
                uint16_t n = 0;
                temp[n++] = dev_addr_;
                temp[n++] = rx_func_;
                memcpy(&temp[n], rx_buf_, rx_len_);
                n += rx_len_;
                const uint16_t calc = modbus_crc16_(temp, n);
                if (calc == rx_crc_recv_)
                {
                    handle_frame_();
                }
            }
        }
        // В любом случае вернёмся к ожиданию нового кадра
        rx_state_ = RxState::WAIT_ADDR;
        break;
    }
}

void AP_HFC_CN::handle_frame_()
{
    if (rx_func_ == FUNC_READ_HOLDING)
    {
        // Ждём ровно 0x26 байт (38)
        if (rx_len_ == 0x26)
        {
            parse_read_03_(rx_buf_, rx_len_);
            waiting_read_resp_ = false;
            healthy_ = true;
            last_update_ms_ = AP_HAL::millis();
        }
        // иначе — игнорировать
    }
    else if (rx_func_ == FUNC_WRITE_SINGLE)
    {
        // Эхо 0x06: [reg_hi][reg_lo][val_hi][val_lo]
        handle_ack_06_(rx_buf_, rx_len_);
    }
}

void AP_HFC_CN::handle_ack_06_(const uint8_t *data, uint8_t len)
{
    if (len != 4)
        return;
    const uint16_t reg = (uint16_t(data[0]) << 8) | data[1];
    const uint16_t val = (uint16_t(data[2]) << 8) | data[3];
    // Можно при желании логировать подтверждения:
    // hal.console->printf("HFC_CN: ACK 0x06 reg=0x%04X val=0x%04X\n", reg, val);
}

// --------------------------- Parse 0x03 ---------------------------
void AP_HFC_CN::parse_read_03_(const uint8_t *data, uint8_t len)
{
    // len должен быть 38 = 19 * 2
    if (len != 38)
        return;

    // Соберём массив 19 регистров (BE)
    uint16_t R[19]{};
    for (uint8_t i = 0; i < 19; i++)
    {
        const uint8_t hi = data[2 * i + 0];
        const uint8_t lo = data[2 * i + 1];
        R[i] = (uint16_t(hi) << 8) | lo;
    }

    // Используем ТОЛЬКО те поля, что указаны в PDF:
    data_.humidity_RH = float(R[REG_RH]) * SCALE_0P1;
    data_.fc_voltage_V = float(R[REG_FC_V]) * SCALE_0P1;
    data_.fc_current_A = float(R[REG_FC_I]) * SCALE_0P1;
    data_.fc_temp1_C = float(R[REG_TEMP1]) * SCALE_0P1;
    data_.fc_power_W = float(R[REG_FC_P]) * SCALE_0P1;
    data_.lb_voltage_V = float(R[REG_LB_V]) * SCALE_0P1;
    data_.lb_current_A = float(R[REG_LB_I]) * SCALE_0P1;
    data_.fc_temp2_C = float(R[REG_TEMP2]) * SCALE_0P1;
    data_.lb_power_W = float(R[REG_LB_P]) * SCALE_0P1;
    data_.total_power_W = float(R[REG_TOTAL_P]) * SCALE_0P1;

    // Давление: два регистра (hi/lo) => 32-битное значение, делим на 10 (MPa)
    const uint32_t h2_raw = (uint32_t(R[REG_H2_HI]) << 16) | uint32_t(R[REG_H2_LO]);
    data_.hydrogen_pressure_MPa = float(h2_raw) * SCALE_0P1;

    data_.ambient_temp_C = float(R[REG_AMBIENT_T]) * SCALE_0P1;
}
