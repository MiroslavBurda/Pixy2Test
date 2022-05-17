
#include "RB3202_servo_smart.hpp"
#include "lx16a/half_duplex_uart.hpp"
#include "lx16a/lx16a.hpp"

#include <string.h>

#define TAG "rbsmartservo"
#define MS_TO_TICKS(ms) ((portTICK_PERIOD_MS <= ms) ? (ms / portTICK_PERIOD_MS) : 1)

namespace rb3202 {


ServoSmartBus::ServoSmartBus() {

}

ServoSmartBus::~ServoSmartBus() {

}


esp_err_t ServoSmartBus::install(uint8_t servo_count, uart_port_t uart, gpio_num_t pin) {
    m_uart = uart;
    m_uart_pin = pin;
    m_tx_queue = xQueueCreate(8, sizeof(struct tx_item));

    xTaskCreate(ServoSmartBus::uartRoutine,"rb3202_servo_tx", 4096, this, 1, NULL);

    {
        std::lock_guard<std::mutex> lo(m_mutex);
        m_servos.resize(servo_count);

        for(uint8_t i = 0; i < servo_count; ++i) {
            auto pos = getAngle(i);

            if(pos.isNaN()) {
                uint16_t posInt = pos.deg() * 100;
                m_servos[i].current = posInt;
                m_servos[i].target = posInt;
            } else {
                ESP_LOGE("smartservo", "failed to get servo %d position", i);
            }
        }
    }

    xTaskCreate(ServoSmartBus::regulatorRoutine,"rb3202_servo_reg", 4096, this, 2, NULL);

    return ESP_OK;
}

void ServoSmartBus::send(const uint8_t *data, size_t size) {
    struct tx_item it = { };
    if(size > sizeof(it.data)) {
        ESP_LOGE("smartservo", "send size too big: %d > %d", size, sizeof(it.data));
        return;
    }

    memcpy(it.data, data, size);
    it.size = size;

    if(xQueueSend(m_tx_queue, &it, pdMS_TO_TICKS(300)) == pdFALSE) {
        ESP_LOGE("smartservo", "m_tx_queue is full");
    }
}

ServoSmartBus::rx_response ServoSmartBus::sendWithResponseBlocking(uint8_t *data, size_t size, TickType_t timeout) {
    struct rx_response resp = {};
    struct tx_item it = { };
    if(size > sizeof(it.data)) {
        ESP_LOGE("smartservo", "send size too big: %d > %d", size, sizeof(it.data));
        return resp;
    }

    memcpy(it.data, data, size);
    it.size = size;
    it.expect_response = true;
    it.response_queue = xQueueCreate(1, sizeof(struct rx_response));

    if(xQueueSend(m_tx_queue, &it, pdMS_TO_TICKS(300)) == pdFALSE) {
        ESP_LOGE("smartservo", "m_tx_queue is full");
    }

    xQueueReceive(it.response_queue, &resp, timeout);
    vQueueDelete(it.response_queue);
    return resp;
}

void ServoSmartBus::setAngle(uint8_t id, Angle angle, float speedDegPerSecond, float speedRaise) {
    float speedPerMs = std::max(1.f, std::min(240.f, speedDegPerSecond)) / 10.f;
    const uint16_t angleInt = std::max(0.f, std::min(360.f, (float)angle.deg())) * 100;

    std::lock_guard<std::mutex> lock(m_mutex);

    auto& s = m_servos[id];

    if(s.current == angleInt) {
        return;
    }

    if((s.current > s.target) != (s.current > angleInt)) {
        s.speed_coef = 0.f;
    }

    s.target = angleInt;
    s.speed_raise = speedRaise;
    s.speed_target = speedPerMs;
}

Angle ServoSmartBus::getAngleOnline(uint8_t id) {
    lw::Packet pkt(id, lw::Command::SERVO_POS_READ);

    auto resp = sendWithResponseBlocking(pkt._data.data(), pkt._data.size());
    if (resp.size != 0x08) {
        return Angle::nan();
    }

    uint16_t val = ((resp.data[6] << 8) | resp.data[5]);

    // The servo's angle counter can underflow when it moves
    // to the 0 position. Handle this case by reseting it to 0.
    if (val > 32767) {
        val = 0;
    } else if (val > 1000) {
        val = 1000;
    }

    return Angle::deg((float(val) / 1000.f) * 240.f);
}

Angle ServoSmartBus::getAngle(uint8_t id) {
    std::lock_guard<std::mutex> lock(m_mutex);
    return Angle::deg(float(m_servos[id].current) * 100);
}

void ServoSmartBus::regulatorRoutine(void *selfVoid) {
    ServoSmartBus *self = (ServoSmartBus*)selfVoid;

    const auto servosCount = self->m_servos.size();

    const auto ticksPerServo = MS_TO_TICKS(REGULATOR_TIME_SLICE_MS);
    const auto ticksPerIter = ticksPerServo * servosCount;

    while(true) {
        const auto tmIterStart = xTaskGetTickCount();

        for(size_t i = 0; i < servosCount; ++i) {
            const auto tmServoStart = xTaskGetTickCount();
            self->regulateServo(i);
            const auto diff = xTaskGetTickCount() - tmServoStart;
            if (diff < ticksPerServo) {
                vTaskDelay(ticksPerServo - diff);
            }
        }

        const auto diff = xTaskGetTickCount() - tmIterStart;
        if (diff < ticksPerIter) {
            vTaskDelay(ticksPerIter - diff);
        }
    }
}

void ServoSmartBus::regulateServo(uint8_t id) {
    auto& s = m_servos[id];

    float moveDeg;

    {
        std::lock_guard<std::mutex> lock(m_mutex);

        if(s.current == s.target) {
            return;
        }

        float speed = s.speed_target;
        if(s.speed_coef < 1.f) {
            s.speed_coef = std::min(1.f, s.speed_coef + s.speed_raise * REGULATOR_TIME_SLICE_MS);
            speed = speed * (s.speed_coef * s.speed_coef);
        }

        int32_t dist = abs(int32_t(s.target) - int32_t(s.current));
        dist = std::max(1, std::min(dist, int32_t(speed * REGULATOR_TIME_SLICE_MS)));

        if(s.target < s.current) {
            s.current -= dist;
        } else {
            s.current += dist;
        }

        if(s.current == s.target) {
            s.speed_coef = 0.f;
        }

        moveDeg = float(s.current) / 100.f;
    }

    const auto pkt = lw::Servo::move(id, Angle::deg(moveDeg),
        std::chrono::milliseconds(REGULATOR_TIME_SLICE_MS - 5));
    send(pkt._data.data(), pkt._data.size());
}

void ServoSmartBus::uartRoutine(void *selfVoid) {
    ServoSmartBus *self = (ServoSmartBus*)selfVoid;

    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(half_duplex::uart_param_config(self->m_uart, &uart_config));
    ESP_ERROR_CHECK(half_duplex::uart_driver_install(self->m_uart, 256, 0, 0, NULL, 0));

    half_duplex::uart_set_half_duplex_pin(self->m_uart, self->m_uart_pin);

    struct tx_item item;
    struct rx_response resp;
    auto tm_last = xTaskGetTickCount();
    constexpr auto min_delay = MS_TO_TICKS(15);
    while(true) {
        if(xQueueReceive(self->m_tx_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        const auto diff = xTaskGetTickCount() - tm_last;
        if (diff < min_delay) {
            vTaskDelay(min_delay - diff);
        }

        half_duplex::uart_tx_chars(self->m_uart, (char*)item.data, item.size);
        tm_last = xTaskGetTickCount();

        // discard copy of the transmitted data
        self->receive(resp.data, sizeof(resp.data));

        if(item.expect_response) {
            resp.size = self->receive(resp.data, sizeof(resp.data));
            if(item.response_queue) {
                xQueueSend(item.response_queue, &resp, 10);
            }
        }
    }
}

size_t ServoSmartBus::receive(uint8_t *buff, size_t bufcap) {
    constexpr TickType_t wait_period = MS_TO_TICKS(4);
    constexpr TickType_t timeout = MS_TO_TICKS(20);

    size_t bufsize = 0;

    while (true) {
        size_t avail = 0;
        size_t need = 0;
        const size_t oldsize = bufsize;
        switch (oldsize) {
        case 0:
        case 1:
            need = 1;
            break;
        case 2:
            need = 3;
            break;
        case 5:
            need = buff[3] - 2;
            break;
        default:
            return bufsize;
        }

        if (need + oldsize > bufcap) {
            ESP_LOGE(TAG, "invalid packet size received: %d.\n", (int)buff[3]);
            return 0;
        }

        TickType_t waiting = 0;
        while (half_duplex::uart_get_buffered_data_len(m_uart, &avail) != ESP_OK || avail < need) {
            if (waiting >= timeout) {
                ESP_LOGE(TAG, "timeout when waiting for data!");
                return 0;
            }
            vTaskDelay(wait_period);
            waiting += wait_period;
        }

        int res = half_duplex::uart_read_bytes(m_uart, buff + oldsize, need, 0);
        if (res < 0 || ((size_t)res) != need) {
            ESP_LOGE(TAG, "invalid packet read: %d, aborting.\n", res);
            abort();
            return 0;
        }
        bufsize += need;

        if (oldsize < 2 && buff[oldsize] != 0x55)
            bufsize = 0;
    }
    return 0;
}

};
