/*
    bme280.c
    Author: Diego
    Created on: 13, April 2026
 */

#include "bme280.h"
#include "bme280_reg.h"
#include "fsl_debug_console.h"
#include "task.h"


/*************************************
 Global Variables
*************************************/
bme280_ctrl_t *g_barometer;
int32_t g_t_fine_32;



AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_bme_tx[32], 4);
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_bme_rx[32], 4);

/*************************************
 Private helpers
*************************************/

/*!
 * @brief Writes a single byte to a BME280 register over SPI.
 *        The BME280 SPI write protocol: send (reg & 0x7F) followed by the value.
 */
static status_t bme280_write_reg(bme280_ctrl_t *bme, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg & BME280_SPI_WR_MASK, val };
    return spi_master_transfer(&bme->spi_ctrl, tx, NULL, sizeof(tx));
}

/*!
 * @brief Reads a single byte from a BME280 register over SPI.
 */
static status_t bme280_read_reg(bme280_ctrl_t *bme, uint8_t reg, uint8_t *out)
{
    s_bme_tx[0] = reg | BME280_SPI_RD_MASK;
    s_bme_tx[1] = 0x00;
    s_bme_rx[0] = 0;
    s_bme_rx[1] = 0;

    status_t status = spi_master_transfer(&bme->spi_ctrl, s_bme_tx, s_bme_rx, 2);
    if (status == kStatus_Success) {
        *out = s_bme_rx[1];
    }
    return status;
}


status_t bme280_init(bme280_ctrl_t *bme_ctl)
{
    status_t status;
    uint8_t chip_id = 0;

#ifdef MCXN947
    spi_get_defaultconfig_bar(&bme_ctl->spi_ctrl);
#endif

    g_barometer = bme_ctl;
    g_t_fine_32 = 0;

    NVIC_SetPriority(EDMA_0_CH9_IRQn, 5);
    NVIC_SetPriority(EDMA_0_CH10_IRQn, 5);

    status = spi_init(&(bme_ctl->spi_ctrl));
    if(status != kStatus_Success)
    {
        return status;
    }

    /* --- Verify SPI communication by reading the chip ID ---
     * The BME280 always returns 0x60 at register 0xD0.
     */
    status = bme280_read_reg(bme_ctl, ID_REG, &chip_id);
    if(status != kStatus_Success)
    {
        return status;
    }
    if(chip_id != BME280_CHIP_ID)
    {
        PRINTF("BME280: unexpected chip ID 0x%02X (expected 0x60)\r\n", chip_id);
        return kStatus_Fail;
    }
    PRINTF("BME280: chip ID OK (0x60)\r\n");

    /* --- Soft reset — brings the sensor to its power-on state --- */
    status = bme280_write_reg(bme_ctl, RESET_REG, BME280_SOFT_RESET_CMD);
    if(status != kStatus_Success)
    {
        return status;
    }
    /* Datasheet §4.3: device needs up to 2 ms after reset to be ready */
    vTaskDelay(pdMS_TO_TICKS(3));

    /* --- Configure oversampling and operating mode ---
     *
     * The BME280 powers on in SLEEP mode (mode[1:0] = 00).
     * With no oversampling set (osrs = 000 = skipped) the output registers
     * hold sentinel values (0x80000 / 0x8000) and no measurement ever runs.
     *
     * IMPORTANT: CTRL_HUM must be written BEFORE CTRL_MEAS.
     * The humidity setting only takes effect on the next write to CTRL_MEAS.  */
    status = bme280_write_reg(bme_ctl, CTRL_HUM_REG,  BME280_CTRL_HUM_VAL);
    if(status != kStatus_Success)
    {
        return status;
    }
    status = bme280_write_reg(bme_ctl, CONFIG_REG,    BME280_CONFIG_VAL);
    if(status != kStatus_Success)
    {
        return status;
    }
    /* Writing CTRL_MEAS with mode=NORMAL starts continuous measurements */
    status = bme280_write_reg(bme_ctl, CTRL_MEAS_REG, BME280_CTRL_MEAS_VAL);
    if(status != kStatus_Success)
    {
        return status;
    }

    //PRINTF("BME280: normal mode started (osrs T/P/H = x1, standby 0.5 ms)\r\n");
    return kStatus_Success;
}


/*!
 * @brief Reads all factory calibration trim parameters from the sensor's NVM.
 *
 * Two burst SPI reads:
 *   Block 1 — 0x88..0x9F  (24 bytes): dig_T1..T3, dig_P1..P9
 *   Block 2 — 0xE1..0xE7  (7 bytes) : dig_H2..H6
 * Plus one single-byte read at 0xA1 for dig_H1.
 *
 * The BME280 SPI read protocol: send (reg | 0x80) as the address byte,
 * then clock out as many dummy bytes as needed. The driver fills rxData[0]
 * with the echo of the address — actual data starts at rxData[1].
 */
status_t bme280_read_calibration(bme280_ctrl_t *bme_ctl)
{
    status_t status;

    /* -----------------------------------------------------------
     * Block 1: dig_T1 (0x88) … dig_P9 (0x9F) — 24 calibration bytes
     * tx[0] = address with read bit set; tx[1..24] = 0x00 dummy bytes
     * rx[0] = echoed address (discard); rx[1..24] = calibration data
     * ----------------------------------------------------------- */
    uint8_t tx1[BME280_CALIB_T_P_LEN + 1U] = {0};
    uint8_t rx1[BME280_CALIB_T_P_LEN + 1U] = {0};

    tx1[0] = BME280_CALIB_T_P_START | BME280_SPI_RD_MASK;

    status = spi_master_transfer(&bme_ctl->spi_ctrl, tx1, rx1, sizeof(tx1));
    if(status != kStatus_Success)
    {
        return status;
    }

    /* All T/P trim words are stored little-endian (LSB at lower address) */
    bme_ctl->cal_data.dig_T1 = (uint16_t)(rx1[2]  << 8) | rx1[1];
    bme_ctl->cal_data.dig_T2 = (int16_t) ((rx1[4]  << 8) | rx1[3]);
    bme_ctl->cal_data.dig_T3 = (int16_t) ((rx1[6]  << 8) | rx1[5]);
    bme_ctl->cal_data.dig_P1 = (uint16_t)(rx1[8]  << 8) | rx1[7];
    bme_ctl->cal_data.dig_P2 = (int16_t) ((rx1[10] << 8) | rx1[9]);
    bme_ctl->cal_data.dig_P3 = (int16_t) ((rx1[12] << 8) | rx1[11]);
    bme_ctl->cal_data.dig_P4 = (int16_t) ((rx1[14] << 8) | rx1[13]);
    bme_ctl->cal_data.dig_P5 = (int16_t) ((rx1[16] << 8) | rx1[15]);
    bme_ctl->cal_data.dig_P6 = (int16_t) ((rx1[18] << 8) | rx1[17]);
    bme_ctl->cal_data.dig_P7 = (int16_t) ((rx1[20] << 8) | rx1[19]);
    bme_ctl->cal_data.dig_P8 = (int16_t) ((rx1[22] << 8) | rx1[21]);
    bme_ctl->cal_data.dig_P9 = (int16_t) ((rx1[24] << 8) | rx1[23]);

    /* -----------------------------------------------------------
     * dig_H1 — single byte at 0xA1
     * ----------------------------------------------------------- */
    uint8_t txH1[2] = { BME280_CALIB_H1_REG | BME280_SPI_RD_MASK, 0x00 };
    uint8_t rxH1[2] = {0};

    status = spi_master_transfer(&bme_ctl->spi_ctrl, txH1, rxH1, sizeof(txH1));
    if(status != kStatus_Success)
    {
        return status;
    }
    bme_ctl->cal_data.dig_H1 = rxH1[1];

    /* -----------------------------------------------------------
     * Block 2: dig_H2 (0xE1) … dig_H6 (0xE7) — 7 bytes
     *
     * Register map (datasheet §4.2.2):
     *   0xE1 = dig_H2 LSB
     *   0xE2 = dig_H2 MSB
     *   0xE3 = dig_H3
     *   0xE4 = dig_H4 [11:4]
     *   0xE5 = dig_H5 [3:0] | dig_H4 [3:0]   (shared byte!)
     *   0xE6 = dig_H5 [11:4]
     *   0xE7 = dig_H6
     * ----------------------------------------------------------- */
    uint8_t tx2[BME280_CALIB_H_LEN + 1U] = {0};
    uint8_t rx2[BME280_CALIB_H_LEN + 1U] = {0};

    tx2[0] = BME280_CALIB_H_START | BME280_SPI_RD_MASK;

    status = spi_master_transfer(&bme_ctl->spi_ctrl, tx2, rx2, sizeof(tx2));
    if(status != kStatus_Success)
    {
        return status;
    }

    bme_ctl->cal_data.dig_H2 = (int16_t)((rx2[2] << 8) | rx2[1]);
    bme_ctl->cal_data.dig_H3 = rx2[3];
    /* dig_H4 is 12-bit: MSB in rx2[4][7:4], LSB in rx2[5][3:0] */
    bme_ctl->cal_data.dig_H4 = (int16_t)(((int16_t)rx2[4] << 4) | (rx2[5] & 0x0F));
    /* dig_H5 is 12-bit: MSB in rx2[6][7:4..0], LSB in rx2[5][7:4] */
    bme_ctl->cal_data.dig_H5 = (int16_t)(((int16_t)rx2[6] << 4) | (rx2[5] >> 4));
    bme_ctl->cal_data.dig_H6 = (int8_t)rx2[7];

    return kStatus_Success;
}


void bme280_parse_data(bme280_ctrl_t *bme_ctl)
{
    /* -----------------------------------------------------------
     * Burst-read 8 output bytes starting at 0xF7:
     *   rx[1] = press_msb   0xF7  bits [7:0]
     *   rx[2] = press_lsb   0xF8  bits [7:0]
     *   rx[3] = press_xlsb  0xF9  bits [7:4] (bits [3:0] unused)
     *   rx[4] = temp_msb    0xFA  bits [7:0]
     *   rx[5] = temp_lsb    0xFB  bits [7:0]
     *   rx[6] = temp_xlsb   0xFC  bits [7:4] (bits [3:0] unused)
     *   rx[7] = hum_msb     0xFD  bits [7:0]
     *   rx[8] = hum_lsb     0xFE  bits [7:0]
     * ----------------------------------------------------------- */
    uint8_t tx[BME280_DATA_BURST_LEN + 1U] = {0};
    uint8_t rx[BME280_DATA_BURST_LEN + 1U] = {0};

    tx[0] = BME280_DATA_BURST_START | BME280_SPI_RD_MASK;

    if(spi_master_transfer(&bme_ctl->spi_ctrl, tx, rx, sizeof(tx)) != kStatus_Success)
    {
        return;
    }

    /* Reconstruct raw ADC values */
    int32_t adc_press = ((int32_t)rx[1] << 12) | ((int32_t)rx[2] << 4) | (rx[3] >> 4);
    int32_t adc_temp  = ((int32_t)rx[4] << 12) | ((int32_t)rx[5] << 4) | (rx[6] >> 4);
    int32_t adc_hum   = ((int32_t)rx[7] << 8)  |  rx[8];

    /* Run compensation — temperature must be first (updates g_t_fine_32) */
    bme280_temperature_int32(bme_ctl, adc_temp);
    bme280_pressure_int64(bme_ctl, adc_press);
    bme280_humidity_int32(bme_ctl, adc_hum);
}

/*!
 * @brief Calculates the temperature in °C with a resolution of 0.01 °C
 * @param adc_val adc value related to the temperature
 * @return output value of 5123 equals 51.23 °C
 */
void bme280_temperature_int32(bme280_ctrl_t *bme, int32_t adc_val)
{
    int32_t var1, var2;

    var1 = ( ( (adc_val>>3) - ( (int32_t)bme->cal_data.dig_T1<<1 ) ) * ( (int32_t)bme->cal_data.dig_T2) ) >> 11;
    var2 = ( ( ( ( (adc_val>>4) - ( (int32_t)bme->cal_data.dig_T1) ) * 
           ( ( adc_val >> 4) - ((int32_t)bme->cal_data.dig_T1)) ) >> 12 ) * ((int32_t)bme->cal_data.dig_T3) ) >> 14;

    g_t_fine_32 = var1 + var2;

    bme->data.temperature = (g_t_fine_32 * 5 + 128) >> 8;

}

/*!
 * @brief Calculates the pressure in Pa as unsigned integer in Q24.8 format (24 integer and 8 fractional bits)
 * @param adc_val adc value related to the temperature
 * @return output value of 24674867 represents 96386.2 Pa pressure/256
 */
void bme280_pressure_int64(bme280_ctrl_t *bme, int32_t adc_val)
{
    int64_t var1, var2, pressure;

    var1 = ((int64_t)g_t_fine_32) - 128000;
    var2 = var1 * var1 * (int64_t)bme->cal_data.dig_P6;
    var2 = var2 + ( ( var1 * (int64_t)bme->cal_data.dig_P5) << 17);
    var2 = var2 + ( ( (int64_t)bme->cal_data.dig_P4) << 35 );
    var1 = ( (var1 * var1 * (int64_t)bme->cal_data.dig_P3) >> 8 ) + ( (var1 * (int64_t)bme->cal_data.dig_P2) << 12);
    var1 = ( ( ((int64_t)1) << 47) + var1 ) * ((int64_t)bme->cal_data.dig_P1) >> 33;
    
    if(var1 == 0)
    {
        return;
    }

    pressure = 1048576 - adc_val;
    pressure = (((pressure << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)bme->cal_data.dig_P9) * (pressure>>13) * (pressure>>13) >> 25;
    var2 = ( ((int64_t)bme->cal_data.dig_P8) * pressure ) >> 19;
    bme->data.pressure = ((pressure + var1 + var2) >> 8) + (((int64_t)bme->cal_data.dig_P7) << 4);

}

/*!
 * @brief Calculates the humidity in %rH as an unsigned integer in Q22.10 format
 *        (22 integer and 10 fractional bits).
 * @param adc_val Raw ADC humidity value from registers 0xFD–0xFE
 * @return Output stored in bme->data.humidity; divide by 1024 to obtain %rH.
 *         E.g. a stored value of 47445 → 47445 / 1024 = 46.33 %rH.
 * @note   bme280_temperature_int32() must be called before this function to
 *         ensure g_t_fine_32 is up to date.
 */
void bme280_humidity_int32(bme280_ctrl_t *bme, int32_t adc_val)
{
    int32_t v_x1_u32r;

    v_x1_u32r = g_t_fine_32 - ((int32_t)76800);

    v_x1_u32r = (((((adc_val << 14) - (((int32_t)bme->cal_data.dig_H4) << 20) -
                    (((int32_t)bme->cal_data.dig_H5) * v_x1_u32r)) +
                   ((int32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)bme->cal_data.dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)bme->cal_data.dig_H3)) >> 11) +
                       ((int32_t)32768))) >> 10) +
                    ((int32_t)2097152)) *
                   ((int32_t)bme->cal_data.dig_H2) + 8192) >> 14));

    v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               ((int32_t)bme->cal_data.dig_H1)) >> 4);

    /* Clamp to valid range [0, 419430400] which corresponds to 0..100 %rH */
    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;

    /* Store as Q22.10 — caller divides by 1024 to obtain %rH as a float */
    bme->data.humidity = (uint32_t)(v_x1_u32r >> 12);
}