/*
    i2c_driver_mcxn947.h
    Author: Diego
    Created on: 5, May 2026

    Thin LPI2C wrapper that mirrors spi_driver_mcxn947.h.  It exists so that
    sensor drivers (vl53l0x, ...) talk to a small project-local API instead
    of the raw NXP SDK calls.

    NON-BLOCKING MODEL
    ------------------
    When ctrl->enable_dma is true (the default for the VL53L0X config) every
    register transfer goes through eDMA + LPI2C.  The calling task is then
    parked on a binary semaphore until the DMA-completion ISR signals it,
    so other RTOS tasks run while the bytes are clocking on the wire.  This
    is "non-blocking from the CPU's perspective" — exactly the pattern the
    SPI wrapper uses for the BMP581 / BNO085 drivers.

    EDMA INITIALISATION
    -------------------
    EDMA_Init() resets the entire DMA controller and is destructive.  The
    SPI wrapper already calls it once via its own guard, and this project
    always brings up the IMU (SPI) before any I²C device, so this wrapper
    deliberately does NOT call EDMA_Init().  If you ever build a project
    that uses I²C without SPI, call EDMA_Init(DMA0, ...) yourself before
    invoking i2c_init().
 */

#ifndef I2C_DRIVER_MCXN947_H_
#define I2C_DRIVER_MCXN947_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "fsl_device_registers.h"
#include "fsl_lpi2c.h"
#include "fsl_lpi2c_edma.h"
#include "fsl_edma.h"

/* FreeRTOS — needed for the DMA completion semaphore */
#include "FreeRTOS.h"
#include "semphr.h"


/* ---- VL53L0X (Time-of-Flight) on FlexComm 2 = LPI2C2 ---------------------
 * Pins are already muxed by pin_mux.c (re-using the BME280 SPI pins now
 * reconfigured for LPI2C SCL/SDA).  400 kHz fast-mode is the maximum the
 * VL53L0X supports per its datasheet §4.1.                                  */
#define TOF_I2C_BAUDRATE              400000U
#define TOF_I2C_BASEADDR              (LPI2C2)
#define TOF_I2C_INSTANCE              (LPI2C_GetInstance(TOF_I2C_BASEADDR))
#define TOF_I2C_CLK_FREQ              CLOCK_GetLPFlexCommClkFreq(TOF_I2C_INSTANCE)
#define TOF_I2C_DMA_BASE              DMA0
#define TOF_I2C_DMA_RX_CHANNEL        11U     /* free — IMU uses 7/8, BMP used 9/10 */
#define TOF_I2C_DMA_TX_CHANNEL        12U
#define TOF_I2C_TX_EDMA_REQUEST       kDma0RequestMuxLpFlexcomm2Tx
#define TOF_I2C_RX_EDMA_REQUEST       kDma0RequestMuxLpFlexcomm2Rx


/*!
 * @brief Per-instance control / state block for the I²C wrapper.
 *
 * Populate via i2c_get_defaultconfig_*() then pass to i2c_init().  Once
 * initialised the struct is owned by the driver (it stores the DMA
 * semaphore handle here) and must outlive every transfer.
 */
typedef struct i2c_ctrl_s {
    LPI2C_Type           *i2c_base;
    DMA_Type             *dma_base;
    uint32_t              source_clock;
    uint32_t              dma_rx_channel;
    uint32_t              dma_tx_channel;
    uint32_t              baudrate;
    uint32_t              instance;
    dma_request_source_t  edma_rx_channel;
    dma_request_source_t  edma_tx_channel;
    bool                  enable_dma;
    /* Binary semaphore signalled by the DMA completion ISR so that
     * i2c_*() can park the calling task instead of busy-waiting.
     * Created in i2c_init() when enable_dma == true.                       */
    SemaphoreHandle_t     dma_semaphore;
} i2c_ctrl_t;


/*!
 * @brief Fills @p tof with the default configuration for the VL53L0X
 *        ToF sensor on LPI2C2 / FlexComm 2.  Same convention as
 *        spi_get_defaultconfig_imu/bar.
 */
void i2c_get_defaultconfig_tof(i2c_ctrl_t *tof);

/*!
 * @brief Initialises the LPI2C peripheral, the eDMA channel mux, the
 *        per-instance eDMA handles, and creates the FreeRTOS binary
 *        semaphore used to wait for DMA-completion.  Does NOT call
 *        EDMA_Init() — see file header.
 */
status_t i2c_init(i2c_ctrl_t *ctrl);

/* --- Register-style accessors (write subaddress, then read/write data) --
 *
 * These are the routines sensor drivers should reach for first; they cover
 * 99 % of sensor traffic (single reg writes, burst reads at an auto-
 * incrementing address).  All blocking-from-task-perspective: the task
 * sleeps on the DMA semaphore until the transfer finishes.
 * ----------------------------------------------------------------------- */
status_t i2c_read_reg (i2c_ctrl_t *ctrl, uint8_t addr7, uint8_t reg, uint8_t *data, size_t size);
status_t i2c_write_reg(i2c_ctrl_t *ctrl, uint8_t addr7, uint8_t reg, const uint8_t *data, size_t size);

/* --- Convenience scalars: byte / 16-bit big-endian (datasheet §4.2 says
 * VL53L0X is MSB-first on the wire, like most ToF / pressure sensors).    */
status_t i2c_read_u8  (i2c_ctrl_t *ctrl, uint8_t addr7, uint8_t reg, uint8_t  *out);
status_t i2c_read_u16 (i2c_ctrl_t *ctrl, uint8_t addr7, uint8_t reg, uint16_t *out);
status_t i2c_write_u8 (i2c_ctrl_t *ctrl, uint8_t addr7, uint8_t reg, uint8_t  val);
status_t i2c_write_u16(i2c_ctrl_t *ctrl, uint8_t addr7, uint8_t reg, uint16_t val);


#endif /* I2C_DRIVER_MCXN947_H_ */
