/*
    vl53l0x.c
    Author: Diego
    Created on: 5, May 2026

    VL53L0X driver — see vl53l0x.h for the API contract.

    INIT-SEQUENCE NOTE
    ------------------
    ST keeps the full register map confidential and ships a ~3000-line
    closed-but-source-available API.  This driver runs the open-source-
    port-equivalent of that API, in two flavours selected by the
    VL53L0X_PERFORM_CALIBRATION macro in vl53l0x.h:

    Without VL53L0X_PERFORM_CALIBRATION (smallest code):
      * Captures the stop variable.
      * Disables MSRC / PRE_RANGE rate-limit checks.
      * Sets default 0.25 MCPS signal-rate limit.
      * Configures GPIO1 → DRDY, starts continuous back-to-back ranging.
      * In practice the chip emits status = 6 / distance = 8190 mm because
        the timing PLL is never locked.  Use this only to verify wiring.

    With VL53L0X_PERFORM_CALIBRATION (recommended, ST-API equivalent):
      * Reads SPAD info from NVM (which detectors are good on this die).
      * Applies the SPAD enable map.
      * Loads ST's "tuning settings" block (~80 register writes).
      * Runs reference VHV + phase calibration.
      * After this the chip ranges accurately to ~3 % per the datasheet.

    All register addresses come from the open-source ST API ports
    (Pololu / Adafruit) — they're stable across silicon revisions.
 */

#include "vl53l0x.h"
#include "vl53l0x_reg.h"
#include "fsl_debug_console.h"

#include "FreeRTOS.h"
#include "task.h"


/* Convenience macros so the init body reads cleanly.  All return on
 * first failure — typical bring-up problem (wrong address, missing
 * pull-ups, wrong baud) surfaces here as kStatus_Fail.                     */
#define I2C_W8(REG, VAL)                                                     \
    do { status_t _st = i2c_write_u8(&vl->i2c_ctrl, vl->i2c_addr,            \
                                     (REG), (VAL));                          \
         if (_st != kStatus_Success) return _st; } while (0)

#define I2C_R8(REG, OUT)                                                     \
    do { status_t _st = i2c_read_u8(&vl->i2c_ctrl, vl->i2c_addr,             \
                                    (REG), (OUT));                           \
         if (_st != kStatus_Success) return _st; } while (0)

#define I2C_W16(REG, VAL)                                                    \
    do { status_t _st = i2c_write_u16(&vl->i2c_ctrl, vl->i2c_addr,           \
                                      (REG), (VAL));                         \
         if (_st != kStatus_Success) return _st; } while (0)


/* ------------------------------------------------------------------ *
 *  Static helpers                                                    *
 * ------------------------------------------------------------------ */

/*!
 * @brief Pololu-compatible "data init" — captures the so-called stop
 *        variable that the ST API references in several other paths.
 *        Without it the ST-API helpers (which we may add later) refuse
 *        to run, so we capture it now even though this minimal driver
 *        doesn't strictly need it for ranging.
 */
static status_t vl53l0x_data_init(vl53l0x_ctrl_t *vl)
{
    /* Use I²C standard mode at 2.8 V VDD (this register only matters on
     * 1.8 V parts, but writing 0 here is harmless).                       */
    I2C_W8(0x88, 0x00);

    /* Magic incantation that captures the stop variable used by the      */
    /* ST API in several elsewhere paths.                                 */
    I2C_W8(0x80, 0x01);
    I2C_W8(0xFF, 0x01);
    I2C_W8(0x00, 0x00);

    I2C_R8(0x91, &vl->stop_variable);

    I2C_W8(0x00, 0x01);
    I2C_W8(0xFF, 0x00);
    I2C_W8(0x80, 0x00);

    /* Disable SIGNAL_RATE_MSRC and SIGNAL_RATE_PRE_RANGE limit checks —
     * the bare-minimum tweak Pololu / Adafruit make so the chip will
     * actually return a range without a full reference calibration.       */
    uint8_t msrc_cfg = 0;
    I2C_R8(VL53L0X_REG_MSRC_CONFIG_CONTROL, &msrc_cfg);
    I2C_W8(VL53L0X_REG_MSRC_CONFIG_CONTROL, msrc_cfg | 0x12U);

    /* Set signal-rate limit to 0.25 MCPS in Q9.7 fixed-point.  This is
     * the ST-API default; lowering it improves long-range performance at
     * the cost of false readings.                                         */
    I2C_W16(VL53L0X_REG_FINAL_RANGE_CONFIG_RATE_LIMIT_LO, 0x0020U);

    /* Enable the full measurement state-machine sequence (TCC | DSS |
     * MSRC | PRE_RANGE | FINAL_RANGE).                                    */
    I2C_W8(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG,
           VL53L0X_SYSTEM_SEQUENCE_CONFIG_DEFAULT);

    return kStatus_Success;
}


#ifdef VL53L0X_PERFORM_CALIBRATION
/* ====================================================================== *
 *  Optional ST-API equivalent calibration block                          *
 *                                                                        *
 *  Compiled in only when VL53L0X_PERFORM_CALIBRATION is defined (see     *
 *  vl53l0x.h).  Implements the four init steps that turn a "chip alive"  *
 *  bring-up into actually-ranges-correctly behaviour:                    *
 *                                                                        *
 *    1. vl53l0x_get_spad_info()         – which SPADs are good on this   *
 *                                          die (read from NVM).          *
 *    2. vl53l0x_set_spad_map()          – apply that SPAD enable map.    *
 *    3. vl53l0x_load_tuning_settings()  – ST's ~80-register magic block. *
 *    4. vl53l0x_perform_ref_calibration – VHV + phase calibration runs.  *
 *                                                                        *
 *  All of the magic register addresses below come from the open-source   *
 *  ST API ports (Pololu / Adafruit) — ST does not document them, but     *
 *  the values are stable across silicon revisions.                        *
 * ====================================================================== */

/*!
 * @brief  Reads the SPAD count and aperture-vs-standard flag stored in
 *         the chip's NVM.  Used to know how many of the 192 SPADs in
 *         the receive array are usable on *this* particular die.
 */
static status_t vl53l0x_get_spad_info(vl53l0x_ctrl_t *vl,
                                      uint8_t *spad_count,
                                      bool    *type_is_aperture)
{
    uint8_t  tmp = 0;
    uint32_t budget;

    I2C_W8(0x80, 0x01);
    I2C_W8(0xFF, 0x01);
    I2C_W8(0x00, 0x00);

    I2C_W8(0xFF, 0x06);
    I2C_R8(0x83, &tmp);
    I2C_W8(0x83, tmp | 0x04U);
    I2C_W8(0xFF, 0x07);
    I2C_W8(0x81, 0x01);

    I2C_W8(0x80, 0x01);

    I2C_W8(0x94, 0x6B);
    I2C_W8(0x83, 0x00);

    /* Poll 0x83 until it self-clears, indicating the NVM read is done.    */
    budget = 200U;   /* up to ~200 ms — datasheet doesn't quote this */
    while (budget-- > 0U)
    {
        I2C_R8(0x83, &tmp);
        if (tmp != 0x00U) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (tmp == 0x00U)
    {
        return kStatus_Timeout;
    }

    I2C_W8(0x83, 0x01);
    I2C_R8(0x92, &tmp);

    *spad_count       = (uint8_t)(tmp & 0x7FU);
    *type_is_aperture = (bool)((tmp >> 7) & 0x01U);

    I2C_W8(0x81, 0x00);
    I2C_W8(0xFF, 0x06);
    I2C_R8(0x83, &tmp);
    I2C_W8(0x83, tmp & ~0x04U);
    I2C_W8(0xFF, 0x01);
    I2C_W8(0x00, 0x01);

    I2C_W8(0xFF, 0x00);
    I2C_W8(0x80, 0x00);

    return kStatus_Success;
}


/*!
 * @brief  Reads the default SPAD enable map from
 *         GLOBAL_CONFIG_SPAD_ENABLES_REF_0 (0xB0..0xB5), masks it down to
 *         only the @p spad_count "good" SPADs (skipping the first 12 if
 *         this die uses aperture SPADs), and writes the result back.
 */
static status_t vl53l0x_set_spad_map(vl53l0x_ctrl_t *vl,
                                     uint8_t spad_count,
                                     bool    type_is_aperture)
{
    uint8_t  ref_spad_map[6] = {0};
    status_t status;

    status = i2c_read_reg(&vl->i2c_ctrl, vl->i2c_addr,
                          0xB0U, ref_spad_map, 6U);
    if (status != kStatus_Success) return status;

    I2C_W8(0xFF, 0x01);
    I2C_W8(0x4F, 0x00);   /* DYNAMIC_SPAD_REF_EN_START_OFFSET            */
    I2C_W8(0x4E, 0x2C);   /* DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD          */
    I2C_W8(0xFF, 0x00);
    I2C_W8(0xB6, 0xB4);   /* GLOBAL_CONFIG_REF_EN_START_SELECT            */

    /* Aperture SPADs are at indices 12..47; standard SPADs at 0..11.       */
    uint8_t first_spad_to_enable = type_is_aperture ? 12U : 0U;
    uint8_t spads_enabled        = 0U;

    for (uint8_t i = 0U; i < 48U; ++i)
    {
        if (i < first_spad_to_enable || spads_enabled == spad_count)
        {
            ref_spad_map[i / 8U] &= (uint8_t)~(1U << (i % 8U));
        }
        else if ((ref_spad_map[i / 8U] >> (i % 8U)) & 0x01U)
        {
            ++spads_enabled;
        }
    }

    return i2c_write_reg(&vl->i2c_ctrl, vl->i2c_addr,
                         0xB0U, ref_spad_map, 6U);
}


/*!
 * @brief  Writes ST's "tuning settings" block — ~80 (reg, value) pairs
 *         that tune the analog front-end and timing.  Verbatim from
 *         VL53L0X_load_tuning_settings() in the ST API.
 */
static status_t vl53l0x_load_tuning_settings(vl53l0x_ctrl_t *vl)
{
    /* ST's tuning settings — do NOT reorder, do NOT delete entries. They
     * include several page switches (writes to 0xFF) that gate which
     * register bank the next writes hit.                                  */
    I2C_W8(0xFF, 0x01); I2C_W8(0x00, 0x00);

    I2C_W8(0xFF, 0x00); I2C_W8(0x09, 0x00); I2C_W8(0x10, 0x00); I2C_W8(0x11, 0x00);

    I2C_W8(0x24, 0x01); I2C_W8(0x25, 0xFF); I2C_W8(0x75, 0x00);

    I2C_W8(0xFF, 0x01); I2C_W8(0x4E, 0x2C); I2C_W8(0x48, 0x00); I2C_W8(0x30, 0x20);

    I2C_W8(0xFF, 0x00); I2C_W8(0x30, 0x09); I2C_W8(0x54, 0x00); I2C_W8(0x31, 0x04);
    I2C_W8(0x32, 0x03); I2C_W8(0x40, 0x83); I2C_W8(0x46, 0x25); I2C_W8(0x60, 0x00);
    I2C_W8(0x27, 0x00); I2C_W8(0x50, 0x06); I2C_W8(0x51, 0x00); I2C_W8(0x52, 0x96);
    I2C_W8(0x56, 0x08); I2C_W8(0x57, 0x30); I2C_W8(0x61, 0x00); I2C_W8(0x62, 0x00);
    I2C_W8(0x64, 0x00); I2C_W8(0x65, 0x00); I2C_W8(0x66, 0xA0);

    I2C_W8(0xFF, 0x01); I2C_W8(0x22, 0x32); I2C_W8(0x47, 0x14); I2C_W8(0x49, 0xFF);
    I2C_W8(0x4A, 0x00);

    I2C_W8(0xFF, 0x00); I2C_W8(0x7A, 0x0A); I2C_W8(0x7B, 0x00); I2C_W8(0x78, 0x21);

    I2C_W8(0xFF, 0x01); I2C_W8(0x23, 0x34); I2C_W8(0x42, 0x00); I2C_W8(0x44, 0xFF);
    I2C_W8(0x45, 0x26); I2C_W8(0x46, 0x05); I2C_W8(0x40, 0x40); I2C_W8(0x0E, 0x06);
    I2C_W8(0x20, 0x1A); I2C_W8(0x43, 0x40);

    I2C_W8(0xFF, 0x00); I2C_W8(0x34, 0x03); I2C_W8(0x35, 0x44);

    I2C_W8(0xFF, 0x01); I2C_W8(0x31, 0x04); I2C_W8(0x4B, 0x09); I2C_W8(0x4C, 0x05);
    I2C_W8(0x4D, 0x04);

    I2C_W8(0xFF, 0x00); I2C_W8(0x44, 0x00); I2C_W8(0x45, 0x20); I2C_W8(0x47, 0x08);
    I2C_W8(0x48, 0x28); I2C_W8(0x67, 0x00); I2C_W8(0x70, 0x04); I2C_W8(0x71, 0x01);
    I2C_W8(0x72, 0xFE); I2C_W8(0x76, 0x00); I2C_W8(0x77, 0x00);

    I2C_W8(0xFF, 0x01); I2C_W8(0x0D, 0x01);

    I2C_W8(0xFF, 0x00); I2C_W8(0x80, 0x01); I2C_W8(0x01, 0xF8);

    I2C_W8(0xFF, 0x01); I2C_W8(0x8E, 0x01); I2C_W8(0x00, 0x01);
    I2C_W8(0xFF, 0x00); I2C_W8(0x80, 0x00);

    return kStatus_Success;
}


/*!
 * @brief  Runs a single calibration "step": forces SYSTEM_SEQUENCE_CONFIG
 *         to a single phase (VHV or PHASE), kicks off a single-shot
 *         range with bit 0x40 set (= calibrate), polls
 *         RESULT_INTERRUPT_STATUS until done, clears the interrupt and
 *         returns.  Helper used by perform_ref_calibration().
 */
static status_t vl53l0x_run_single_calibration(vl53l0x_ctrl_t *vl,
                                               uint8_t sequence_config)
{
    uint8_t  tmp;
    uint32_t budget;

    I2C_W8(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, sequence_config);

    /* 0x01 = single-shot start, 0x40 = "this is a calibration run".       */
    I2C_W8(VL53L0X_REG_SYSRANGE_START, 0x01U | 0x40U);

    /* Wait for the chip to assert any of the interrupt-status bits.        */
    budget = 200U;   /* ~200 ms */
    do {
        I2C_R8(VL53L0X_REG_RESULT_INTERRUPT_STATUS, &tmp);
        if ((tmp & 0x07U) != 0U) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    } while (--budget > 0U);
    if (budget == 0U)
    {
        return kStatus_Timeout;
    }

    I2C_W8(VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01U);
    I2C_W8(VL53L0X_REG_SYSRANGE_START,         0x00U);

    return kStatus_Success;
}


/*!
 * @brief  Reference VHV + phase calibration.  Both passes are required:
 *         VHV sets the avalanche-bias high-voltage to the right level,
 *         phase locks the timing PLL.  Without phase calibration the
 *         chip rejects every range with status = 6 (RANGE_PHASE_CHECK).
 */
static status_t vl53l0x_perform_ref_calibration(vl53l0x_ctrl_t *vl)
{
    status_t status;

    status = vl53l0x_run_single_calibration(vl, 0x01U); /* VHV   */
    if (status != kStatus_Success) return status;

    status = vl53l0x_run_single_calibration(vl, 0x02U); /* PHASE */
    if (status != kStatus_Success) return status;

    /* Restore the full sequence config the rest of the driver assumes.     */
    I2C_W8(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG,
           VL53L0X_SYSTEM_SEQUENCE_CONFIG_DEFAULT);

    return kStatus_Success;
}
#endif /* VL53L0X_PERFORM_CALIBRATION */


/*!
 * @brief Configures GPIO1 as "new-sample-ready", active-low, and clears
 *        any pending interrupt so the very first sample will produce a
 *        clean falling edge.
 */
static status_t vl53l0x_configure_interrupt(vl53l0x_ctrl_t *vl)
{
    /* GPIO1 → assert on every new sample.                                 */
    I2C_W8(VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_CONFIG,
           VL53L0X_GPIO_FUNC_NEW_SAMPLE_READY);

    /* Force GPIO1 polarity to active-low (clear bit 4 of HV mux).         *
     * Active-low + external pull-up + falling-edge ISR = standard wiring. */
    uint8_t mux = 0;
    I2C_R8(VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH, &mux);
    I2C_W8(VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH, mux & ~0x10U);

    /* Clear any latched interrupt flag.                                   */
    I2C_W8(VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01U);

    return kStatus_Success;
}


/* ------------------------------------------------------------------ *
 *  Public API                                                        *
 * ------------------------------------------------------------------ */

status_t vl53l0x_init(vl53l0x_ctrl_t *vl, void *gpio_callback)
{
    status_t status;
    uint8_t  model_id = 0;

    gpio_ctrl_t gpio_event = {
        .gpio_base = GPIO4,
        .port_base = PORT4,
        .dir = gpio_input,
        .pin = 5
    };

    if (vl == NULL)
    {
        return kStatus_InvalidArgument;
    }

    /* Default address if the caller left it zeroed.                       */
    if (vl->i2c_addr == 0U)
    {
        vl->i2c_addr = VL53L0X_I2C_ADDR_DEFAULT;
    }

    NVIC_SetPriority(GPIO40_IRQn, 5);
    NVIC_SetPriority(GPIO41_IRQn, 5);
    NVIC_SetPriority(EDMA_0_CH11_IRQn, 5);
    NVIC_SetPriority(EDMA_0_CH12_IRQn, 5);

#ifdef MCXN947
    /* Bring up LPI2C2 / FlexComm 2.  Caller may have pre-populated
     * vl->i2c_ctrl with a custom config; if not, fill it with the ToF
     * default.  enable_dma == false is treated as "caller knows what
     * they want" and left alone.                                          */
    if (vl->i2c_ctrl.i2c_base == NULL)
    {
        i2c_get_defaultconfig_tof(&vl->i2c_ctrl);
    }

    vl->gpio_event = gpio_event;

#endif

    status = i2c_init(&vl->i2c_ctrl);
    if (status != kStatus_Success)
    {
        return status;
    }

    /* --- Verify presence ------------------------------------------------
     * Datasheet §4.2: register 0xC0 always reads 0xEE on a healthy part.
     * Reading it BEFORE any reset is intentional — it confirms the bus
     * is wired and the slave acks at our address before we touch other
     * registers.                                                          */
    status = i2c_read_u8(&vl->i2c_ctrl, vl->i2c_addr,
                         VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model_id);
    if (status != kStatus_Success)
    {
        return status;
    }
    if (model_id != VL53L0X_MODEL_ID_EXPECTED)
    {
        PRINTF("VL53L0X: unexpected model ID 0x%02X (expected 0xEE)\r\n",
               model_id);
        return kStatus_Fail;
    }

    /* --- Minimal init sequence ----------------------------------------- */
    status = vl53l0x_data_init(vl);
    if (status != kStatus_Success)
    {
        return status;
    }

#ifdef VL53L0X_PERFORM_CALIBRATION
    /* --- ST-API equivalent calibration --------------------------------- *
     * Without these four steps the chip ranges at default settings — which
     * in practice means status = 6 / distance = 8190 mm forever, because
     * the timing PLL isn't locked.  Compiled out by undefining
     * VL53L0X_PERFORM_CALIBRATION in vl53l0x.h.                          */
    {
        uint8_t spad_count        = 0;
        bool    type_is_aperture  = false;

        status = vl53l0x_get_spad_info(vl, &spad_count, &type_is_aperture);
        if (status != kStatus_Success)
        {
            PRINTF("VL53L0X: SPAD info read failed (st=%d)\r\n", status);
            return status;
        }

        status = vl53l0x_set_spad_map(vl, spad_count, type_is_aperture);
        if (status != kStatus_Success)
        {
            PRINTF("VL53L0X: SPAD map write failed (st=%d)\r\n", status);
            return status;
        }

        status = vl53l0x_load_tuning_settings(vl);
        if (status != kStatus_Success)
        {
            PRINTF("VL53L0X: tuning settings load failed (st=%d)\r\n", status);
            return status;
        }

        /* Configure GPIO1 → DRDY *before* the calibration runs so the
         * status interrupt the calibration polls on is wired correctly. */
        status = vl53l0x_configure_interrupt(vl);
        if (status != kStatus_Success)
        {
            return status;
        }

        status = vl53l0x_perform_ref_calibration(vl);
        if (status != kStatus_Success)
        {
            PRINTF("VL53L0X: ref calibration failed (st=%d) — chip silicon "
                   "may be damaged or supply is unstable\r\n", status);
            return status;
        }

        PRINTF("VL53L0X: calibrated (SPADs=%u %s)\r\n",
               spad_count, type_is_aperture ? "aperture" : "standard");
    }
#else
    /* --- Interrupt + GPIO1 wiring (no calibration) --------------------- */
    status = vl53l0x_configure_interrupt(vl);
    if (status != kStatus_Success)
    {
        return status;
    }
#endif /* VL53L0X_PERFORM_CALIBRATION */

    /* --- Start continuous back-to-back ranging ------------------------- *
     * This puts the device into "fire as fast as you can" mode (~30 Hz at
     * the default sequence config).  Each completed sample asserts GPIO1.*/
    I2C_W8(VL53L0X_REG_SYSRANGE_START, VL53L0X_SYSRANGE_MODE_BACKTOBACK);

    /* --- Wire up the data-ready GPIO interrupt (optional) -------------- */
    if (gpio_callback != NULL)
    {
        vl->gpio_event.dir = gpio_input;
        gpio_init(&vl->gpio_event);
        gpio_attach_interrupt(&vl->gpio_event, gpio_callback);
    }

    return kStatus_Success;
}


status_t vl53l0x_read_distance(vl53l0x_ctrl_t *vl)
{
    status_t status;

    if (vl == NULL)
    {
        return kStatus_InvalidArgument;
    }

    /* Burst-read 2 bytes starting at RESULT_RANGE_STATUS + 10 → distance.
     * The register reads big-endian per datasheet §4.2, and i2c_read_u16
     * already swaps for us.                                               */
    status = i2c_read_u16(&vl->i2c_ctrl, vl->i2c_addr,
                         VL53L0X_REG_RESULT_RANGE_MM,
                         &vl->data.distance_mm);
    if (status != kStatus_Success)
    {
        return status;
    }

    /* Lower nibble of RESULT_RANGE_STATUS is the per-sample status
     * code (0 = no error, 11 = signal-strength fail, 13 = phase-fail,
     * etc.).  Cheap to grab and useful to know in flight logs.            */
    uint8_t range_status_byte = 0;
    status = i2c_read_u8(&vl->i2c_ctrl, vl->i2c_addr,
                         VL53L0X_REG_RESULT_RANGE_STATUS,
                         &range_status_byte);
    if (status == kStatus_Success)
    {
        vl->data.range_status = (uint8_t)((range_status_byte >> 3) & 0x0FU);
    }

    /* Clear GPIO1 / interrupt latch so the next sample can fire.          */
    return i2c_write_u8(&vl->i2c_ctrl, vl->i2c_addr,
                        VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01U);
}
