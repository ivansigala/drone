/*
 * debug_uart_mcxn947.c
 *
 * See debug_uart_mcxn947.h for the public API and wire protocol.
 *
 * Implementation notes
 * --------------------
 *
 *  - Storage is a single ring buffer drained by the LPUART4 TX-empty
 *    ISR. Producers write into it; the ISR is the only consumer.
 *
 *  - "head" and "tail" are free-running 32-bit counters; the buffer
 *    index is (counter & MASK). The "used" count is (head - tail),
 *    which is correct across uint32_t wrap-around as long as the
 *    buffer size is < 2^31.
 *
 *  - The producer side (debug_uart_write / debug_uart_send_frame) is
 *    callable from any context -- task or ISR. Multi-producer races
 *    on head, drops, the SEQ counter, and the TIE-enable bit are
 *    avoided with a short BASEPRI critical section that masks every
 *    interrupt at or below configMAX_SYSCALL_INTERRUPT_PRIORITY.
 *
 *    LPUART4 IRQ runs at priority 6 (set in debug_uart_init). The
 *    timer ISR runs at priority 5. configMAX_SYSCALL_INTERRUPT_PRIORITY
 *    is 2 (see FreeRTOSConfig_Gen.h), so BASEPRI=2 masks both. Higher-
 *    priority NMI/HardFault don't touch this driver.
 *
 *  - Frames are streamed straight into the ring -- no scratch buffer
 *    on the caller's stack. CRC8 is folded byte-by-byte during the
 *    copy. This keeps the ISR/task footprint tiny.
 *
 *  - Author: Diego
 *  - Created: 2026-05-03
 *  - Revised: 2026-05-04
 */

#include <string.h>

#include "fsl_lpuart.h"
#include "fsl_clock.h"
#include "fsl_common.h"

#include "FreeRTOS.h"  /* for configMAX_SYSCALL_INTERRUPT_PRIORITY */

#include "uart_driver_mcxn947.h"      /* uart_attach_interrupt(), UARTn_HANDLE */
#include "debug_uart_mcxn947.h"

/*******************************************************************************
 * Configuration
 ******************************************************************************/

#define DEBUG_UART_BASE       LPUART4
#define DEBUG_UART_INSTANCE   4U
#define DEBUG_UART_NVIC_PRIO  6U   /* lower than RC/ESC EDMA (priority 2..4) */

#if (DEBUG_UART_TX_BUF_SIZE == 0U) || \
    ((DEBUG_UART_TX_BUF_SIZE & (DEBUG_UART_TX_BUF_SIZE - 1U)) != 0U)
#error "DEBUG_UART_TX_BUF_SIZE must be a non-zero power of two"
#endif

#define DEBUG_UART_BUF_MASK   (DEBUG_UART_TX_BUF_SIZE - 1U)

/*******************************************************************************
 * Module state
 ******************************************************************************/

static volatile uint8_t  s_tx_buf[DEBUG_UART_TX_BUF_SIZE];
static volatile uint32_t s_tx_head;       /* next write index   (producer) */
static volatile uint32_t s_tx_tail;       /* next read index    (consumer) */
static volatile uint32_t s_drops;         /* total bytes dropped           */
static volatile uint32_t s_frame_drops;   /* total frames that didn't fit  */
static volatile uint8_t  s_seq;           /* monotonic frame counter       */
static volatile bool     s_initialised;

/*******************************************************************************
 * Critical-section helpers (BASEPRI-based, callable from any context once
 * the LPUART IRQ priorities are >= configMAX_SYSCALL_INTERRUPT_PRIORITY).
 ******************************************************************************/

static inline uint32_t debug_uart_enter_critical(void)
{
    uint32_t prev = __get_BASEPRI();
    __set_BASEPRI(configMAX_SYSCALL_INTERRUPT_PRIORITY);
    __DSB();
    __ISB();
    return prev;
}

static inline void debug_uart_exit_critical(uint32_t prev)
{
    __set_BASEPRI(prev);
}

/*******************************************************************************
 * Helpers
 ******************************************************************************/

static inline uint32_t buf_used(void)
{
    return (uint32_t)(s_tx_head - s_tx_tail);
}

static inline uint32_t buf_free(void)
{
    return DEBUG_UART_TX_BUF_SIZE - buf_used();
}

/* Single-byte CRC-8 / SMBus update (poly 0x07, init 0x00). */
static inline uint8_t crc8_step(uint8_t crc, uint8_t b)
{
    crc ^= b;
    for (uint8_t i = 0U; i < 8U; ++i)
    {
        crc = (uint8_t)((crc & 0x80U) ? ((crc << 1) ^ 0x07U) : (crc << 1));
    }
    return crc;
}

/*
 * Push a single byte into the ring buffer. The caller MUST hold the
 * critical section; no bounds checking is done here -- the caller is
 * expected to have reserved enough free space upfront.
 */
static inline void ring_push(uint32_t *head, uint8_t b)
{
    s_tx_buf[(*head) & DEBUG_UART_BUF_MASK] = b;
    (*head)++;
}

/*******************************************************************************
 * ISR (consumer)
 ******************************************************************************/

/*
 * Invoked from LP_FLEXCOMM4_IRQHandler via the UART4_HANDLE hook
 * registered by uart_attach_interrupt(). Drains as many bytes as the
 * LPUART will accept (it has a small TX FIFO) and exits.
 *
 * When the ring buffer goes empty we MUST disable the TIE interrupt,
 * otherwise the ISR re-fires immediately because TDRE stays asserted
 * as long as there is room in the FIFO.
 */
static void debug_uart_irq_handler(void)
{
    while ((LPUART_GetStatusFlags(DEBUG_UART_BASE) &
            (uint32_t)kLPUART_TxDataRegEmptyFlag) != 0U)
    {
        if (s_tx_head == s_tx_tail)
        {
            /* Nothing to send -- mute the interrupt. */
            LPUART_DisableInterrupts(DEBUG_UART_BASE,
                                     (uint32_t)kLPUART_TxDataRegEmptyInterruptEnable);
            break;
        }

        uint8_t b = s_tx_buf[s_tx_tail & DEBUG_UART_BUF_MASK];
        /* Publish the new tail AFTER reading the byte. */
        __DMB();
        s_tx_tail++;

        LPUART_WriteByte(DEBUG_UART_BASE, b);
    }
}

/*******************************************************************************
 * Public API
 ******************************************************************************/

void debug_uart_init(uint32_t baudrate)
{
    lpuart_config_t cfg;

    LPUART_GetDefaultConfig(&cfg);
    cfg.baudRate_Bps = baudrate;
    cfg.enableTx     = true;
    cfg.enableRx     = false;   /* TX-only debug stream */

    /* (Re)initialise the LPUART. This wipes anything the SDK debug
     * console may have left behind. */
    uint32_t clk = CLOCK_GetLPFlexCommClkFreq(DEBUG_UART_INSTANCE);
    LPUART_Init(DEBUG_UART_BASE, &cfg, clk);

    /* Reset module state. */
    s_tx_head     = 0U;
    s_tx_tail     = 0U;
    s_drops       = 0U;
    s_frame_drops = 0U;
    s_seq         = 0U;
    s_initialised = true;

    /*
     * Hook our handler into the existing UARTx_HANDLE chain. The shared
     * driver's LP_FLEXCOMM4_IRQHandler calls this before any SDK
     * dispatch, which is exactly what we want.
     *
     * uart_attach_interrupt() also enables the RX-data-ready interrupt
     * unconditionally; we don't use RX, so disable it right after.
     */
    uart_attach_interrupt(DEBUG_UART_BASE, debug_uart_irq_handler);
    LPUART_DisableInterrupts(DEBUG_UART_BASE,
                             (uint32_t)kLPUART_RxDataRegFullInterruptEnable);

    /* TIE stays off until there's something to send. */
    LPUART_DisableInterrupts(DEBUG_UART_BASE,
                             (uint32_t)kLPUART_TxDataRegEmptyInterruptEnable);

    NVIC_SetPriority(LP_FLEXCOMM4_IRQn, DEBUG_UART_NVIC_PRIO);
}

size_t debug_uart_write(const uint8_t *data, size_t len)
{
    if (!s_initialised || data == NULL || len == 0U)
    {
        return 0U;
    }

    size_t   queued;
    uint32_t basepri = debug_uart_enter_critical();
    {
        uint32_t free_now = buf_free();
        size_t   to_copy  = (len > free_now) ? (size_t)free_now : len;

        if (to_copy < len)
        {
            s_drops += (uint32_t)(len - to_copy);
        }

        uint32_t head = s_tx_head;
        for (size_t i = 0U; i < to_copy; ++i)
        {
            s_tx_buf[(head + i) & DEBUG_UART_BUF_MASK] = data[i];
        }

        /* Publish data BEFORE moving the head; the ISR must never read
         * past a head that points to bytes we haven't written yet. */
        __DMB();
        s_tx_head = head + (uint32_t)to_copy;
        queued = to_copy;
    }
    debug_uart_exit_critical(basepri);

    if (queued > 0U)
    {
        /* Make sure TIE is on so the ISR drains what we just queued.
         * Safe to enable even if it was already on. */
        LPUART_EnableInterrupts(DEBUG_UART_BASE,
                                (uint32_t)kLPUART_TxDataRegEmptyInterruptEnable);
    }

    return queued;
}

size_t debug_uart_print(const char *str)
{
    if (str == NULL) return 0U;
    return debug_uart_write((const uint8_t *)str, strlen(str));
}

bool debug_uart_send_frame(uint8_t id, const void *payload, uint8_t len)
{
    if (!s_initialised) return false;
    if (len > DEBUG_UART_MAX_PAYLOAD) return false;

    const uint8_t *p = (const uint8_t *)payload;
    const size_t   total = (size_t)DEBUG_UART_FRAME_OVERHEAD + (size_t)len;
    bool ok;

    uint32_t basepri = debug_uart_enter_critical();
    {
        if (buf_free() < total)
        {
            /* Atomic publish: if the whole frame doesn't fit, we drop
             * the whole frame. The receiver will see a SEQ skip and
             * know exactly how many frames were lost. */
            s_drops       += (uint32_t)total;
            s_frame_drops += 1U;
            ok = false;
        }
        else
        {
            uint8_t  seq  = s_seq;
            uint32_t head = s_tx_head;
            uint8_t  crc  = 0U;

            ring_push(&head, DEBUG_UART_FRAME_SYNC1);
            ring_push(&head, DEBUG_UART_FRAME_SYNC2);

            ring_push(&head, seq);
            crc = crc8_step(crc, seq);

            ring_push(&head, id);
            crc = crc8_step(crc, id);

            ring_push(&head, len);
            crc = crc8_step(crc, len);

            for (uint8_t i = 0U; i < len; ++i)
            {
                uint8_t b = (p != NULL) ? p[i] : 0U;
                ring_push(&head, b);
                crc = crc8_step(crc, b);
            }

            ring_push(&head, crc);

            __DMB();
            s_tx_head = head;
            s_seq     = (uint8_t)(seq + 1U);
            ok = true;
        }
    }
    debug_uart_exit_critical(basepri);

    if (ok)
    {
        LPUART_EnableInterrupts(DEBUG_UART_BASE,
                                (uint32_t)kLPUART_TxDataRegEmptyInterruptEnable);
    }

    return ok;
}

size_t debug_uart_pending(void)
{
    return (size_t)buf_used();
}

size_t debug_uart_free(void)
{
    return (size_t)buf_free();
}

uint32_t debug_uart_drops(void)
{
    return s_drops;
}

uint32_t debug_uart_frame_drops(void)
{
    return s_frame_drops;
}

uint8_t debug_uart_last_seq(void)
{
    /* SEQ is the *next* number to send; the last one actually sent is
     * one less (mod 256). Return that for parity with the host's view. */
    return (uint8_t)(s_seq - 1U);
}
