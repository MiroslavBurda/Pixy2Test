#include <esp_log.h>
#include <driver/spi_master.h>
#include <freertos/task.h>
#include <vector>
#include <driver/gpio.h>
#include <driver/uart.h>

#include "pixy2/pixy2.hpp"
#include "pixy2/spi.hpp"
#include "pixy2/i2c.hpp"

using namespace pixy2;

extern "C" void app_main()
{
   uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 256, 0, NULL, 0));


    const spi_bus_config_t busCfg = {
        .mosi_io_num = GPIO_NUM_25, //13 
        .miso_io_num = GPIO_NUM_26, //12
        .sclk_io_num = GPIO_NUM_27, //14
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        // .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_IOMUX_PINS,
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &busCfg, 0));

    auto linkRes = LinkSpi::addSpiDevice(SPI2_HOST);
    ESP_ERROR_CHECK(std::get<1>(linkRes));

    LinkSpi link = std::move(std::get<0>(linkRes));
    auto pixy = Pixy2<LinkSpi>(std::move(link));

    // pixy2::VersionResponse ver;
    // ESP_ERROR_CHECK(pixy.waitForStartup( *ver, pdMS_TO_TICKS(1000)));
    ESP_ERROR_CHECK(pixy.waitForStartup());
    // GetBlocksContext blocksCtx;
    GetBlocksContext blocksCtxRed, blocksCtxBlue;
    while(true) {
        //auto err = pixy.getColorBlocks(1, 4, blocksCtx); // cervena
        auto err = pixy.getColorBlocks(1, 4, blocksCtxRed); // cervena
        auto err2 = pixy.getColorBlocks(2, 4, blocksCtxBlue); // modra
        if(err == pixy.ERR_PIXY_BUSY) {
            vTaskDelay(1);
            continue;
        } else if(err != ESP_OK) {
            printf("Error1: %d\n", err);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if(err2 == pixy.ERR_PIXY_BUSY) {
            vTaskDelay(1);
            continue;
        } else if(err2 != ESP_OK) {
            printf("Error2: %d\n", err);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // const uint8_t lorrisHeader[] = { 0xFF, 0x01, (uint8_t)(blocksCtx.blocks.size() * sizeof(ColorBlock)) };
        // uart_write_bytes(UART_NUM_0, lorrisHeader, sizeof(lorrisHeader));
        // if(blocksCtx.blocks.size() > 0) {
        //     uart_write_bytes(UART_NUM_0, blocksCtx.blocks.data(), sizeof(ColorBlock)*blocksCtx.blocks.size());
        // }

        const uint8_t lorrisHeader[] = { 0xFF, 0x01, (uint8_t)(blocksCtxRed.blocks.size() * sizeof(ColorBlock)) };
        uart_write_bytes(UART_NUM_0, lorrisHeader, sizeof(lorrisHeader));
        if(blocksCtxRed.blocks.size() > 0) {
            uart_write_bytes(UART_NUM_0, blocksCtxRed.blocks.data(), sizeof(ColorBlock)*blocksCtxRed.blocks.size());
        }
        if(blocksCtxBlue.blocks.size() > 0) {
            uart_write_bytes(UART_NUM_0, blocksCtxBlue.blocks.data(), sizeof(ColorBlock)*blocksCtxBlue.blocks.size());
        }


        // int poziceX = blocksCtxRed.blocks[0]->x;
        // int poziceY = blocksCtxRed.blocks[0]->y;
        // int poziceX2 = blocksCtxBlue.blocks[1]->x;
        // int poziceY2 = blocksCtxBlue.blocks[1]->y;

        // int poziceX = blocksCtx.blocks[0]->x;
        // int poziceY = blocksCtx.blocks[0]->y;
        // int poziceX2 = blocksCtx.blocks[1]->x;
        // int poziceY2 = blocksCtx.blocks[1]->y;

        // int i;
        // for(i =0; i<6; i++)
        //    printf("i: %i, %i, %i, %i\n", poziceX, poziceY, poziceX2, poziceY2);
        // vTaskDelay(pdMS_TO_TICKS(1000));

    } 
}

