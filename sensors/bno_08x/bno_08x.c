/*
    bno_08x.c
    Author: Diego
    Created on: 10, April 2026
 */

#include "bno_08x.h"

static sh2_Hal_t sh2_hal;
static imu_ctrl_t *g_imu; // Make imu global so hal_read can access it for GPIO state

//Open/Close stubs (You already initialize hardware in bno_08x_init)
static int hal_open(sh2_Hal_t *self) { return SH2_OK; }
static void hal_close(sh2_Hal_t *self) {}

// Open/Close stubs (You already initialize hardware in bno_08x_init)
static uint32_t hal_getTimeUs(sh2_Hal_t *self) {
    // FreeRTOS tick count converted to microseconds
    return xTaskGetTickCount() * 1000; 
}

// SPI Write Bridge
static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    // Note: Assuming 'g_imu' is accessible here
    status_t status = spi_master_transfer(&g_imu->spi_ctrl, pBuffer, NULL, len);
    return (status == kStatus_Success) ? len : 0;
}


// SPI Read Bridge (The tricky part)
static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    // The CEVA library polls this function. We only read if the BNO085 has pulled HINT low.
    if (GPIO_PinRead(g_imu->gpio_event.gpio_base, g_imu->gpio_event.pin) != 0) {
        return 0; // HINT is high, no data ready
    }
    
    *t_us = hal_getTimeUs(self);

    // BNO085 requires reading the 4-byte header to determine packet length.
    // To avoid toggling Chip Select (PCS) between the header read and body read,
    // which breaks the BNO085 SPI protocol, we read a fixed chunk large enough 
    // for standard reports (like Quaternions) in a single transaction.
    uint8_t temp_buf[SH2_HAL_MAX_TRANSFER_IN] = {0}; 
    spi_master_transfer(&g_imu->spi_ctrl, NULL, temp_buf, sizeof(temp_buf));

    // Extract length from SHTP header
    uint16_t packet_len = (temp_buf[0] | (temp_buf[1] << 8)) & 0x7FFF;
    
    if (packet_len > 0 && packet_len <= sizeof(temp_buf) && packet_len <= len) {
        memcpy(pBuffer, temp_buf, packet_len);
        return packet_len;
    }
    return 0;
}

status_t bno_08x_init(imu_ctrl_t *imu, void* spi_callback, void* gpio_callback, sh2_SensorCallback_t sh2_callback)
{
    gpio_ctrl_t gpio_event ={
        .gpio_base = GPIO1,
        .port_base = PORT1,
        .dir = gpio_input,
        .pin = 17
    };

    gpio_ctrl_t gpio_reset = {
        .gpio_base = GPIO1,
        .port_base = PORT1,
        .dir = gpio_output,
        .pin = 16
    };


    imu->gpio_event = gpio_event;
    imu->gpio_reset = gpio_reset;


    NVIC_SetPriority(GPIO10_IRQn, 5);
    NVIC_SetPriority(GPIO11_IRQn, 5);

#ifdef MCXN947
    spi_get_defaultconfig_imu(&imu->spi_ctrl, spi_callback);
    imu->spi_ctrl.enable_dma = false;
#endif

    g_imu = imu; // Store in global for hal_read access

    
    gpio_init(&imu->gpio_reset);
    gpio_set_output(&imu->gpio_reset, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_output(&imu->gpio_reset, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    gpio_init(&imu->gpio_event);
    gpio_attach_interrupt(&imu->gpio_event, gpio_callback);

    // Mount the HAL to the CEVA Library
    sh2_hal.open = hal_open;
    sh2_hal.close = hal_close;
    sh2_hal.read = hal_read;
    sh2_hal.write = hal_write;
    sh2_hal.getTimeUs = hal_getTimeUs;

    // Open the SHTP session
    if (sh2_open(&sh2_hal, NULL, NULL) != SH2_OK) {
        PRINTF("Failed to open SH2 session\r\n");
        return kStatus_Fail;
    }

    // Register your sensor data callback
    sh2_setSensorCallback(sh2_callback, NULL);

    // Request the Rotation Vector (Quaternions) at 100Hz
    sh2_SensorConfig_t config = {0};
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.batchInterval_us = 0;
    config.sensorSpecific = 0;
    config.reportInterval_us = 10000; // 10ms = 100Hz

    sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config);

    return spi_init(&imu->spi_ctrl);

}

