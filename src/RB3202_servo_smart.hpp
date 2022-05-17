#pragma once

#include <esp_err.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/queue.h>

#include "lx16a/angle.hpp"

#include <vector>
#include <mutex>

namespace rb3202 {

class ServoSmartBus {
public:
    ServoSmartBus();
    ServoSmartBus(const ServoSmartBus&) = delete;
    ~ServoSmartBus();

    esp_err_t install(uint8_t servo_count, uart_port_t uart, gpio_num_t pin);

    void setAngle(uint8_t id, Angle angle, float speedDegPerSecond = 180.f, float speedRaise = 0.0015f);

    Angle getAngleOnline(uint8_t id);
    Angle getAngle(uint8_t id);

private:
    static constexpr int REGULATOR_TIME_SLICE_MS = 30;
    struct tx_item {
        uint8_t data[16];
        QueueHandle_t response_queue;
        bool expect_response;
        uint8_t size;
    };

    struct rx_response {
        uint8_t data[16];
        uint8_t size;
    };

    struct servo_info {
        uint16_t current;
        uint16_t target;
        float speed_target;
        float speed_raise;
        float speed_coef;

        servo_info() {
            current = 0xFFFF;
            target = 0xFFFF;
            speed_target = 0;
            speed_raise = 0;
            speed_coef = 0;
        }
    };

    void send(const uint8_t *data, size_t size);
    struct rx_response sendWithResponseBlocking(uint8_t *data, size_t size, TickType_t timeout = portMAX_DELAY);

    size_t receive(uint8_t *buffer, size_t buffer_capacity);

    static void uartRoutine(void *selfVoid);
    static void regulatorRoutine(void *selfVoid);
    void regulateServo(uint8_t id);

    uart_port_t m_uart;
    gpio_num_t m_uart_pin;
    QueueHandle_t m_tx_queue;

    std::mutex m_mutex;
    std::vector<servo_info> m_servos;
};

};
