/*
 * dshot.c
 *
 *  Created on: Feb 28, 2026
 *      Author: diego
 */


#include "dshot.h"



// Buffers for 4 motors (Must be non-cacheable for DMA)
AT_NONCACHEABLE_SECTION(uint16_t dma_buf[DSHOT_DMA_BUFFER_SIZE]);
AT_NONCACHEABLE_SECTION(uint16_t dma_buf_all[MAX_SUPPORTED_MOTORS][DSHOT_DMA_BUFFER_SIZE]);
static uart_ctrl_t esc_ctrl;

/*******************************************************************************
 * Initialization
 ******************************************************************************/

status_t dshot_init(dshotSystem_t *sys, void* telemetry_callback_ptr){

    dshotTelemetry_t telem = {0};
    dshotControl_t control = { .throttle_u16 = 0, .requestTelemetry_b = false };
    dshotMotor_t *motors[MAX_SUPPORTED_MOTORS] = { &sys->motor0, &sys->motor1, &sys->motor2, &sys->motor3 };

    for(int i = 0; i < MAX_SUPPORTED_MOTORS; i++){
        motors[i]->motor_id = i;
        motors[i]->dshot_telemtry = telem;
        motors[i]->dshot_control = control;
    }

#ifdef MCXN947

    NVIC_SetPriority(EDMA_0_CH3_IRQn, 2);
    NVIC_SetPriority(EDMA_0_CH4_IRQn, 2);
    NVIC_SetPriority(EDMA_0_CH5_IRQn, 2);
    NVIC_SetPriority(EDMA_0_CH6_IRQn, 2);

    uart_get_default_esc_config(&(sys->telemetry_uart), telemetry_callback_ptr);
    
    pwm_get_default_dshot_config(&(sys->motor0.pwm), 0);
    pwm_get_default_dshot_config(&(sys->motor1.pwm), 1);
    pwm_get_default_dshot_config(&(sys->motor2.pwm), 2);
    pwm_get_default_dshot_config(&(sys->motor3.pwm), 3);

    dma_get_default_dshot_config(&(sys->motor0.dma), 0);
    sys->motor0.dma_id= 3;
    dma_get_default_dshot_config(&(sys->motor1.dma), 1);
    sys->motor1.dma_id = 4;
    dma_get_default_dshot_config(&(sys->motor2.dma), 2);
    sys->motor2.dma_id = 5;
    dma_get_default_dshot_config(&(sys->motor3.dma), 3);
    sys->motor3.dma_id = 6;
#endif

    dshot_telemetry_init(&(sys->telemetry_uart));

    dshot_motor_init(&(sys->motor0));
    dshot_motor_init(&(sys->motor1));
    dshot_motor_init(&(sys->motor2));
    dshot_motor_init(&(sys->motor3));

    __disable_irq();
    PWM_StartTimer(sys->motor0.pwm.pwm_base, 
                   sys->motor0.pwm.submodule_ctrl | 
                   sys->motor1.pwm.submodule_ctrl | 
                   sys->motor2.pwm.submodule_ctrl | 
                   sys->motor3.pwm.submodule_ctrl);
    __enable_irq();

    return kStatus_Success;
}

status_t dshot_motor_init(dshotMotor_t *motor) {

    // 1. Initialize el PWM
    // If PWM fails, we return
    if (pwm_init(&motor->pwm) != kStatus_Success) {
        return kStatus_Fail;
    }

    // 2. Initialize the DMA cahnnel
    dshot_dma_init(&(motor->dma));

    return kStatus_Success;
}


void dshot_dma_init(dma_ctrl_t *dma) {

	dma_init_channel(dma);

}

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief Generates the 16-bit DShot packet
 * @param throttle_u16: The throttle value (0-2047, where 48-2047 are valid throttle values and 1-47 are special commands)
 * @param request_telemetry_b: Whether to set the telemetry request bit (true or false)
 * @return The 16-bit DShot packet ready to be sent to the ESC
 */
uint16_t dshot_prepare_packet(uint16_t throttle_u16, bool request_telemetry_b) {

    uint16_t packet_u16 = 0;

    // 1. Clamp throttle to safe limits (48 is min throttle, 2047 is max)
    // Values 1-47 are reserved for special ESC commands (like changing 3D direction)
    if (throttle_u16 < 48 && throttle_u16 != 0) throttle_u16 = 48;
    if (throttle_u16 > 2047) throttle_u16 = 2047;

    // 2. Shift throttle into the top 11 bits
    packet_u16 = (throttle_u16 << 1);

    // 3. Add the telemetry request bit
    if (request_telemetry_b) {
        packet_u16 |= 1;
    }

    // 4. Calculate the 4-bit CRC
    // DShot CRC is calculated by XORing the 3 nibbles of the 12-bit data
    uint16_t csum = packet_u16 ^ (packet_u16 >> 4) ^ (packet_u16 >> 8);
    csum &= 0x0F; // Keep only the bottom 4 bits

    // 5. Shift the 12 bits up by 4 and attach the CRC at the bottom
    packet_u16 = (packet_u16 << 4) | csum;

    return packet_u16;
}



void dshot_send_frame(dshotMotor_t *motor) {
    
    uint16_t packet_u16 = dshot_prepare_packet(motor->dshot_control.throttle_u16, motor->dshot_control.requestTelemetry_b);
    uint32_t destAddr_u16 = (uint32_t)&motor->pwm.pwm_base->SM[motor->pwm.submodule].VAL3;

    // 1. Fill the 16 bits
    for (int i = 0; i < 16; i++) {
        dma_buf[i] = (packet_u16 & (0x8000 >> i)) ? DSHOT_600_BIT_1 : DSHOT_600_BIT_0;
    }
    
    // 2. Add the crucial DShot reset period (duty cycle = 0)
    dma_buf[16] = 0;
    dma_buf[17] = 0;

    // Target the specific VAL register

    dma_transfer_submit(motor->dma_id,
                            (uint32_t)dma_buf,
                            destAddr_u16,
                            sizeof(uint16_t),
                            DSHOT_DMA_BUFFER_SIZE * sizeof(uint16_t));

    pwm_set_ldok(&motor->pwm);

}



void dshot_send_frame_all(dshotSystem_t *sys) {
    
    uint8_t ldok_mask = 0;
    uint32_t destAddr_u16[MAX_SUPPORTED_MOTORS] = {0};
    uint16_t packets[MAX_SUPPORTED_MOTORS];
    uint32_t dma_buf_list[MAX_SUPPORTED_MOTORS] = {(uint32_t)dma_buf_all[0], (uint32_t)dma_buf_all[1], (uint32_t)dma_buf_all[2], (uint32_t)dma_buf_all[3]};
    dshotMotor_t  *motors[MAX_SUPPORTED_MOTORS] = { &sys->motor0, &sys->motor1, &sys->motor2, &sys->motor3 };
    uint32_t dma_channels[MAX_SUPPORTED_MOTORS] = {motors[0]->dma_id, motors[1]->dma_id, motors[2]->dma_id, motors[3]->dma_id};

    for(int i = 0; i < MAX_SUPPORTED_MOTORS; i++){
        packets[i] = dshot_prepare_packet(motors[i]->dshot_control.throttle_u16, motors[i]->dshot_control.requestTelemetry_b);
        for (int bit = 0; bit < 16; bit++) {
            dma_buf_all[i][bit] = (packets[i] & (0x8000 >> bit)) ? DSHOT_600_BIT_1 : DSHOT_600_BIT_0;
        }
        dma_buf_all[i][16] = 0;
        dma_buf_all[i][17] = 0;
    }


    for(int i = 0; i < MAX_SUPPORTED_MOTORS; i++){
        destAddr_u16[i] = (uint32_t)&motors[i]->pwm.pwm_base->SM[motors[i]->pwm.submodule].VAL3;
    }

    for(int i = 0; i < MAX_SUPPORTED_MOTORS; i++){
        ldok_mask |= (1U << motors[i]->pwm.submodule);
    }

    // Call the DMA channels
    dma_transfer_submit_channels(dma_channels,
                                dma_buf_list,
                                destAddr_u16,
                                sizeof(uint16_t),
                                DSHOT_DMA_BUFFER_SIZE * sizeof(uint16_t),
                                MAX_SUPPORTED_MOTORS);
    
    
    pwm_set_ldok_mask(motors[0]->pwm.pwm_base, ldok_mask);
    
    
}

/*******************************************************************************
 * Telemetry
 ******************************************************************************/

void dshot_telemetry_init(uart_ctrl_t* uart_config_ptr) {
    
    if(uart_config_ptr) {
        esc_ctrl = *uart_config_ptr;
        uart_init(&esc_ctrl);
    }
}

status_t dshot_process_telemetry(dshotMotor_t *current_motor, uint8_t *buffer) {
  
    dshotTelemetry_t *telem = &(current_motor->dshot_telemtry);
    uint8_t crc = 0;
    
    crc = get_crc8(buffer, 9); // Calculate CRC of the first 9 bytes (data)

    // Validar CRC y asignar al motor correspondiente
    if (crc == buffer[9] && current_motor != NULL) {
        telem->temperature_u8 = buffer[0];
        telem->voltage_cv_u16  = (buffer[1] << 8) | buffer[2];
        telem->current_ca_u16  = (buffer[3] << 8) | buffer[4];
        telem->consumption_mah_u16 = (buffer[5] << 8) | buffer[6];
        telem->erpm_u16 = (buffer[7] << 8) | buffer[8];
        telem->valid_b = true;
    } else {
        if (current_motor != NULL) {
            telem->valid_b = false;
            return kStatus_Fail;
        }
    }
    
    return kStatus_Success;
}

status_t esc_start_dma_rx(uint8_t *buffer, uint32_t length){
    
    if (LPUART_ClearStatusFlags(esc_ctrl.uart_base, kLPUART_RxOverrunFlag | kLPUART_NoiseErrorFlag | kLPUART_FramingErrorFlag | kLPUART_ParityErrorFlag) == kStatus_Fail) {
        return kStatus_Fail;
    }
    uart_read_dma(&esc_ctrl, buffer, length);

    return kStatus_Success;
}



uint8_t update_crc8(uint8_t crc, uint8_t crc_seed) {
    uint8_t crc_u, i;
    crc_u = crc;
    crc_u ^= crc_seed;
    for (i = 0; i < 8; i++) {
        crc_u = (crc_u & 0x80) ? 0x7 ^ (crc_u << 1) : (crc_u << 1);
    }
    return crc_u;
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen) {
    uint8_t crc = 0, i;
    for (i = 0; i < BufLen; i++) {
        crc = update_crc8(Buf[i], crc);
    }
    return crc;
}
