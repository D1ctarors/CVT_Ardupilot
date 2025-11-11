#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <stdint.h>

/*
  Телеметрия строго по мануалу "Fuel Cell User Manual - FCCU Version":
  - Протокол: Modbus RTU 57600 8N1, адрес по умолчанию 0x01
  - Чтение:  Function 0x03, Start=0x0000, Count=0x0013 (19 регистров => 0x26 байт)
  - Ответ:   [addr][0x03][0x26][38 байт данных][CRC lo][CRC hi]
  - Команды: Function 0x06 в регистр 0x0013: 0x0001 (Start) / 0x0000 (Stop)
*/

class AP_HFC_CN
{
public:
    struct Data
    {
        // Масштабирование всех *_SCALE = 0.1f (из PDF)
        float humidity_RH = 0.0f;           // рег. 3  (/10)
        float fc_voltage_V = 0.0f;          // рег. 4  (/10)
        float fc_current_A = 0.0f;          // рег. 5  (/10)
        float fc_temp1_C = 0.0f;            // рег. 6  (/10)
        float fc_power_W = 0.0f;            // рег. 7  (/10)
        float lb_voltage_V = 0.0f;          // рег. 8  (/10)
        float lb_current_A = 0.0f;          // рег. 9  (/10)
        float fc_temp2_C = 0.0f;            // рег. 10 (/10)
        float lb_power_W = 0.0f;            // рег. 11 (/10)
        float total_power_W = 0.0f;         // рег. 12 (/10)
        float hydrogen_pressure_MPa = 0.0f; // рег. 13 (hi) + 14 (lo) -> /10
        float ambient_temp_C = 0.0f;        // рег. 18 (/10)
    };

    AP_HFC_CN() = default;

    // Инициализация: передать UART и (опционально) адрес устройства Modbus
    void init(AP_HAL::UARTDriver *uart, uint8_t device_addr = 0x01);

    // Вызывать часто из scheduler: опрос + приём/парсинг
    void update();

    // Команды пуска/остановки по Modbus 0x06 в регистр 0x0013
    void send_start();
    void send_stop();

    // Доступ к состоянию
    const Data &data() const { return data_; }
    bool healthy() const { return healthy_; }
    uint32_t last_update_ms() const { return last_update_ms_; }
    bool is_bound() const { return uart_ != nullptr; }

private:
    // --- Константы протокола (из PDF) ---
    static constexpr uint32_t BAUDRATE = 57600; // 8N1
    static constexpr uint8_t FUNC_READ_HOLDING = 0x03;
    static constexpr uint8_t FUNC_WRITE_SINGLE = 0x06;
    static constexpr uint16_t REG_START = 0x0000;
    static constexpr uint16_t REG_COUNT = 0x0013; // 19 регистров
    static constexpr uint16_t REG_STARTSTOP = 0x0013;
    static constexpr uint16_t START_VAL = 0x0001;
    static constexpr uint16_t STOP_VAL = 0x0000;

    static constexpr float SCALE_0P1 = 0.1f;

    // Индексы регистров ответа (0..18)
    enum : uint8_t
    {
        REG_BAUD = 0,    // ignore
        REG_DEVADDR = 1, // ignore
        REG_CALIB_V = 2, // ignore
        REG_RH = 3,
        REG_FC_V = 4,
        REG_FC_I = 5,
        REG_TEMP1 = 6,
        REG_FC_P = 7,
        REG_LB_V = 8,
        REG_LB_I = 9,
        REG_TEMP2 = 10,
        REG_LB_P = 11,
        REG_TOTAL_P = 12,
        REG_H2_HI = 13,
        REG_H2_LO = 14,
        REG_RSVD15 = 15, // ignore
        REG_RSVD16 = 16, // ignore
        REG_RSVD17 = 17, // ignore
        REG_AMBIENT_T = 18
    };

    // --- Приёмный автомат Modbus RTU ---
    enum class RxState : uint8_t
    {
        WAIT_ADDR = 0,
        WAIT_FUNC,
        WAIT_03_LEN,
        READ_DATA, // generic: читаем N data-байт для 0x03/0x06
        READ_CRC_LO,
        READ_CRC_HI
    };

    AP_HAL::UARTDriver *uart_ = nullptr;
    uint8_t dev_addr_ = 0x01;

    RxState rx_state_ = RxState::WAIT_ADDR;
    uint8_t rx_func_ = 0;
    uint8_t rx_len_ = 0; // для 0x03: byte count; для 0x06: длина данных (6 байт)
    uint8_t rx_buf_[64]{};
    uint8_t rx_pos_ = 0;
    uint16_t rx_crc_recv_ = 0;
    uint16_t rx_crc_calc_ = 0;

    Data data_{};
    bool healthy_ = false;
    uint32_t last_update_ms_ = 0;

    // Опрос
    uint32_t last_poll_ms_ = 0;
    bool waiting_read_resp_ = false;

    // Внутренние функции
    void poll_once_();
    void handle_byte_(uint8_t b);
    void handle_frame_();
    void parse_read_03_(const uint8_t *data, uint8_t len);
    void handle_ack_06_(const uint8_t *data, uint8_t len);

    void send_cmd_06_(uint16_t reg, uint16_t value);
    void send_read_03_(uint16_t start_reg, uint16_t n_regs);

    static uint16_t modbus_crc16_(const uint8_t *p, uint16_t n);
};
