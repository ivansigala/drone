/*
    bno_08x.c
    Author: Diego
    Created on: 10, April 2026
 */

#include "bno_08x.h"

#define SHTP_HDR_LEN 4

static sh2_Hal_t sh2_hal;
static imu_ctrl_t *g_imu; // Make imu global so hal_read can access it for GPIO state
static volatile bool g_reset_occurred = false; // Flag set by SH2 event callback
static volatile bool g_ps0_asserted = false;

// Open/Close stubs (hardware is initialized in bno_08x_init)
static int hal_open(sh2_Hal_t *self) { return SH2_OK; }
static void hal_close(sh2_Hal_t *self) {}

static uint32_t hal_getTimeUs(sh2_Hal_t *self) {
    // FreeRTOS tick count converted to microseconds.
    return (uint32_t)(xTaskGetTickCount() * (1000000U / configTICK_RATE_HZ));
}

status_t bno_08x_tare(void) {
    int rc = sh2_setTareNow(SH2_TARE_X | SH2_TARE_Y | SH2_TARE_Z,
                            SH2_TARE_BASIS_ROTATION_VECTOR);
    return (rc == SH2_OK) ? kStatus_Success : kStatus_Fail;
}

// SPI Write Bridge
static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    // While waiting for PS0 wake acknowledgment, do not attempt a read.
    // The HINT low in this state belongs to the write handshake, not incoming data.
    if (g_ps0_asserted) {
        return 0;
    }

    if (gpio_read_input(&g_imu->gpio_event) != 0) {
        return 0;
    }

    *t_us = hal_getTimeUs(self);
    static uint8_t temp_buf[SH2_HAL_MAX_TRANSFER_IN_APP] = {0};
    memset(temp_buf, 0, sizeof(temp_buf));
    spi_master_transfer(&g_imu->spi_ctrl, NULL, temp_buf, sizeof(temp_buf));

    uint16_t packet_len = (temp_buf[0] | (temp_buf[1] << 8)) & 0x7FFF;
    if (packet_len >= SHTP_HDR_LEN) {
        uint16_t copy_len = packet_len;
        if (copy_len > len) copy_len = len;
        if (copy_len > sizeof(temp_buf)) copy_len = sizeof(temp_buf);
        memcpy(pBuffer, temp_buf, copy_len);
        return copy_len;
    }
    return 0;
}

static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    if (!g_ps0_asserted) {
        // First call: assert nWAKE and return busy.
        // txProcess will call shtp_service() then retry hal_write.
        gpio_set_output(&g_imu->gpio_ps0, 0);
        g_ps0_asserted = true;
        return 0;
    }

    // Subsequent calls: wait for BNO085 to acknowledge by pulling HINT low.
    if (gpio_read_input(&g_imu->gpio_event) != 0) {
        return 0;  // HINT still high — BNO085 not yet ready, keep retrying
    }

    // HINT is low — BNO085 acknowledged the wake. Perform the write now.
    status_t status = spi_master_transfer(&g_imu->spi_ctrl, pBuffer, NULL, len);

    // Release nWAKE after the transfer completes.
    gpio_set_output(&g_imu->gpio_ps0, 1);
    g_ps0_asserted = false;

    return (status == kStatus_Success) ? len : 0;
}

// Internal event callback to detect reset-complete during init
static void hal_event_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
    if (pEvent->eventId == SH2_RESET) {
        g_reset_occurred = true;
    }
}

status_t bno_08x_init(imu_ctrl_t *imu, void* gpio_callback)
{
    status_t spi_status;
    gpio_ctrl_t gpio_event = {
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

    gpio_ctrl_t gpio_ps0 = {
        .gpio_base = GPIO1,
        .port_base = PORT1,
        .dir = gpio_output,
        .pin = 18
    };

    imu->gpio_event = gpio_event;
    imu->gpio_reset = gpio_reset;
    imu->gpio_ps0 = gpio_ps0;

    NVIC_SetPriority(GPIO10_IRQn, 5);
    NVIC_SetPriority(EDMA_0_CH7_IRQn, 5);
    NVIC_SetPriority(EDMA_0_CH8_IRQn, 5);

#ifdef MCXN947
    spi_get_defaultconfig_imu(&imu->spi_ctrl);
#endif

    g_imu = imu; // Store in global for hal_read access

    spi_status = spi_init(&imu->spi_ctrl);
    if (spi_status != kStatus_Success) {
        return spi_status;
    }

    // Hardware reset: pull RSTN low, wait, release
    gpio_init(&imu->gpio_reset);
    gpio_init(&imu->gpio_event);
    gpio_init(&imu->gpio_ps0);
    gpio_set_output(&imu->gpio_ps0, 1);
    gpio_set_output(&imu->gpio_reset, 0);
    SDK_DelayAtLeastUs(10000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY); // 10ms reset pulse
    gpio_set_output(&imu->gpio_reset, 1);
    
    gpio_attach_interrupt(&imu->gpio_event, gpio_callback);

    // Mount the HAL to the CEVA Library (but do NOT open yet —
    // sh2_open has a blocking poll loop that requires working timestamps,
    // so it must be called after the FreeRTOS scheduler starts).
    sh2_hal.open = hal_open;
    sh2_hal.close = hal_close;
    sh2_hal.read = hal_read;
    sh2_hal.write = hal_write;
    sh2_hal.getTimeUs = hal_getTimeUs;

    return spi_status;
}

status_t bno_08x_start(sh2_SensorCallback_t sh2_callback)
{
    // Must be called from a FreeRTOS task context (scheduler running)
    // so that hal_getTimeUs works and the sh2_open timeout is functional.
    if (sh2_open(&sh2_hal, hal_event_callback, NULL) != SH2_OK) {
        PRINTF("Failed to open SH2 session\r\n");
        return kStatus_Fail;
    }

    sh2_setSensorCallback(sh2_callback, NULL);
    return kStatus_Success;
}

bool bno_08x_reset_occurred(void)
{
    if (g_reset_occurred) {
        g_reset_occurred = false;
        return true;
    }
    return false;
}

status_t bno_08x_configure_sensors(void)
{
    sh2_SensorConfig_t config = {0};
    config.changeSensitivityEnabled  = false;
    config.wakeupEnabled             = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled           = false;
    config.changeSensitivity         = 0;
    config.batchInterval_us          = 0;
    config.sensorSpecific            = 0;

    config.reportInterval_us = 10000; // 100 Hz
    if (sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config) != SH2_OK) {
        PRINTF("Failed to configure rotation vector\r\n");
        return kStatus_Fail;
    }

    config.reportInterval_us = 10000; // 100 Hz
    if (sh2_setSensorConfig(SH2_LINEAR_ACCELERATION, &config) != SH2_OK) {
        PRINTF("Failed to configure linear acceleration\r\n");
        return kStatus_Fail;
    }

    PRINTF("Sensors configured at 100Hz\r\n");
    return kStatus_Success;
}