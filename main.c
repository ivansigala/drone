/*
 * Author: Diego 
 * Created on: 6, April 2026
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "app.h"

/* user includes. */
#include "rc_fsia6B.h"
#include "dshot.h"
#include "timer_driver_mcxn947.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Task priorities. */
#define RC_task_PRIORITY 4
#define ESC_task_PRIORITY 4
#define MOTOR_task_PRIORITY 4
#define CONTROL_task_PRIORITY 4
#define SENSOR_task_PRIORITY 4

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void RCParserTask(void *pvParameters);
static void ESCTelemetryTask(void *pvParameters);
static void DSHOTGeneratorTask(void *pvParameters);
// static void ControlLoopTask(void *pvParameters);
// static void SensorTask(void *pvParameters);

void RC_Callback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData);
void ESC_Callback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData);


/*******************************************************************************
 * Variables
 ******************************************************************************/
// lpuart_edma_handle_t g_lpuartEdmaHandle;
AT_NONCACHEABLE_SECTION_INIT(uint8_t g_rc_rxBuffer[FS_IA6B_FRAME_SIZE]) = {0};
AT_NONCACHEABLE_SECTION_INIT(uint8_t g_esc_rxBuffer[DSHOT_TELEMETRY_FRAME_SIZE]) = {0};

/* Task handle for notifications */
TaskHandle_t rcParserTaskHandle = NULL;
TaskHandle_t escParserTaskHandle = NULL;
TaskHandle_t dshotGeneratorTaskHandle = NULL;

/* Queues */
QueueHandle_t rcChannelQueue = NULL;
QueueHandle_t escChannelQueue = NULL;
QueueHandle_t escTelemetryMotorIdQueue = NULL;

/* ESC Handle */
dshotSystem_t esc;

/*******************************************************************************
 * Code
 ******************************************************************************/

void RC_Callback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (kStatus_LPUART_RxIdle == status)
    {
        /* Notify the parsing task that data is ready */
        vTaskNotifyGiveFromISR(rcParserTaskHandle, &xHigherPriorityTaskWoken);
    }
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ESC_Callback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (kStatus_LPUART_RxIdle == status)
    {
        /* Notify the parsing task that data is ready */
        vTaskNotifyGiveFromISR(escParserTaskHandle, &xHigherPriorityTaskWoken);
    }
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void timer_0_callback(void *args) {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Notify the motor task to send the next DShot command */
    vTaskNotifyGiveFromISR(dshotGeneratorTaskHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

/*!
 * @brief Application entry point.
 */
int main(void)
{
    timer_ctrl_t timer_0 ={
        .timer_id = 0,
        .frequency = 800
    };

    /* Init board hardware. */
    BOARD_InitHardware();

    /* FreeRTOS interrupt priority fix */
    NVIC_SetPriority(LPTMR0_IRQn, 5);

    rc_init(RC_Callback);
    
    dshot_init(&esc, ESC_Callback);

    timer_init(&timer_0);
    timer_attach_callback(&timer_0, timer_0_callback);

    /* Create Queue to hold the parsed channels data (buffer size of 5 frames) */
    rcChannelQueue = xQueueCreate(5, sizeof(fs_ia6b_channels_t));
    escChannelQueue = xQueueCreate(5, sizeof(dshotTelemetry_t));
    escTelemetryMotorIdQueue = xQueueCreate(8, sizeof(uint8_t));
    if (rcChannelQueue == NULL || escChannelQueue == NULL || escTelemetryMotorIdQueue == NULL)
    {
        PRINTF("Queue creation failed!.\r\n");
        while (1);
    }

    if (xTaskCreate(RCParserTask, "rc_task", configMINIMAL_STACK_SIZE + 50, NULL, RC_task_PRIORITY, &rcParserTaskHandle) !=
        pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }

    if (xTaskCreate(ESCTelemetryTask , "esc_task", configMINIMAL_STACK_SIZE + 50, &esc, RC_task_PRIORITY, &escParserTaskHandle) !=
        pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }

    if (xTaskCreate(DSHOTGeneratorTask , "motor_task", configMINIMAL_STACK_SIZE + 50, &esc, MOTOR_task_PRIORITY, &dshotGeneratorTaskHandle) !=
        pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }

    // if (xTaskCreate(ControlLoopTask , "control_task", configMINIMAL_STACK_SIZE + 50, &esc, CONTROL_task_PRIORITY, &escParserTaskHandle) !=
    //     pdPASS)
    // {
    //     PRINTF("Task creation failed!.\r\n");
    //     while (1)
    //         ;
    // }

    // if (xTaskCreate(SensorTask , "sensor_task", configMINIMAL_STACK_SIZE + 50, &esc, SENSOR_task_PRIORITY, &dshotGeneratorTaskHandle) !=
    //     pdPASS)
    // {
    //     PRINTF("Task creation failed!.\r\n");
    //     while (1)
    //         ;
    // }

    

    timer_start(&timer_0);

    vTaskStartScheduler();
    for (;;)
        ;
}

// /*!
//  * @brief Task responsible for calculating the control aoutputs based on the RC input and ESC telemetry.
//  */
// static void ControlLoopTask(void *pvParameters){

//     vTaskSuspend(NULL);
    
// }


// /*!
//  * @brief Task responsible for reading sensors the IMU and barometer.
//  */
// static void SensorTask(void *pvParameters){
    
//     vTaskSuspend(NULL);

// }


/*!
 * @brief Task responsible for sending dshot commands.
 */
static void DSHOTGeneratorTask(void *pvParameters)
{

    dshotSystem_t *esc = (dshotSystem_t *)pvParameters;
    uint8_t motor_id = 0;

    for (;;)
    {

        /* Wait to be notified by the timer 0 ISR */
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == pdTRUE){

            /* Track which motor requested telemetry so ESC task can store it correctly. */
            (void)xQueueSendToBack(escTelemetryMotorIdQueue, &motor_id, 0);

            switch (motor_id)
            {
            case 0:
                esc->motor0.dshot_control.requestTelemetry_b = true;
                dshot_send_frame_all(esc);
                // dshot_send_frame(&(esc->motor0));
                // dshot_send_frame(&(esc->motor1));
                // dshot_send_frame(&(esc->motor2));
                // dshot_send_frame(&(esc->motor3));
                esc->motor0.dshot_control.requestTelemetry_b = false;
                break;
            case 1:
                esc->motor1.dshot_control.requestTelemetry_b = true;
                dshot_send_frame(&(esc->motor1));
                esc->motor1.dshot_control.requestTelemetry_b = false;
                break;
            case 2:
                esc->motor2.dshot_control.requestTelemetry_b = true;
                dshot_send_frame(&(esc->motor2));
                esc->motor2.dshot_control.requestTelemetry_b = false;
                break;
            case 3:
                esc->motor3.dshot_control.requestTelemetry_b = true;
                dshot_send_frame(&(esc->motor3));
                esc->motor3.dshot_control.requestTelemetry_b = false;
                break;
            
            default:
                PRINTF("Invalid motor index in DSHOTGeneratorTask.\r\n");
                break;
            }
            motor_id = (motor_id + 1) % MAX_SUPPORTED_MOTORS;

        }
       
    }
}

/*!
 * @brief Task responsible for parsing the received RC frame.
 */
static void RCParserTask(void *pvParameters)
{
    fs_ia6b_frame_t current_rc_frame;

    rc_start_dma_rx(g_rc_rxBuffer, FS_IA6B_FRAME_SIZE);

    for (;;)
    {
        /* Wait to be notified by the EDMA RC ISR */
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == pdTRUE)
        {
            /* Check if the frame passes CRC and is successfully parsed */
            if (rc_parse_frame(g_rc_rxBuffer, &current_rc_frame) == kRC_StatusSucces) 
            {
                PRINTF("%d\r\n", 
                        current_rc_frame.channels.CH1.u16);
            } 
            else 
            {
                //PRINTF("Corrupt Frame or CRC mismatch.\r\n");
                //rc_sync();
            }

            /* Clean up and restart DMA reception for the next frame */
            
            rc_start_dma_rx(g_rc_rxBuffer, FS_IA6B_FRAME_SIZE);
        }
    }
}


/*!
 * @brief Task responsible for parsing the received ESC telemetry frame.
 */
static void ESCTelemetryTask(void *pvParameters)
{
    
    dshotSystem_t *esc = (dshotSystem_t *)pvParameters;
    dshotMotor_t *motor = NULL;
    uint8_t motor_id = 0;

    esc_start_dma_rx(g_esc_rxBuffer, DSHOT_TELEMETRY_FRAME_SIZE);

    for (;;)
    {
        /* Wait to be notified by the EDMA ESC ISR */
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == pdTRUE)
        {   
            if (xQueueReceive(escTelemetryMotorIdQueue, &motor_id, 0) == pdTRUE)
            {
                switch (motor_id)
                {
                case 0:
                    motor = &esc->motor0;
                    break;
                case 1:
                    motor = &esc->motor1;
                    break;
                case 2:
                    motor = &esc->motor2;
                    break;
                case 3:
                    motor = &esc->motor3;
                    break;
                default:
                    motor = NULL;
                    break;
                }
            }

            if ((motor != NULL) && (dshot_process_telemetry(motor, g_esc_rxBuffer) == kStatus_Success)) {
                // PRINTF("ESC Telemetry - Temp: %d C, Voltage: %d cV, Current: %d cA, Consumption: %d mAh, ERPM: %d\r\n",
                //         motor->dshot_telemtry.temperature_u8,
                //         motor->dshot_telemtry.voltage_cv_u16,
                //         motor->dshot_telemtry.current_ca_u16,
                //         motor->dshot_telemtry.consumption_mah_u16,
                //         motor->dshot_telemtry.erpm_u16);
            } else {
                // PRINTF("Invalid ESC Telemetry Frame or CRC mismatch.\r\n");
            }

            esc_start_dma_rx(g_esc_rxBuffer, DSHOT_TELEMETRY_FRAME_SIZE);
        }
    }
}

